#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sys/time.h>

// For ROS2RTF
#include <unistd.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/syscall.h>
#include <mutex>

#include "trace_picas/trace.hpp"

#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/string.hpp"
#include "test_interfaces/msg/test_string.hpp"

using std::placeholders::_1;
//std::mutex mtx;

#define gettid() syscall(__NR_gettid)

//#define USE_INTRA_PROCESS_COMMS false
#define USE_INTRA_PROCESS_COMMS true

#define DUMMY_LOAD_ITER	1000
int dummy_load_calib = 1;

void dummy_load(int load_ms) {
    int i, j;
    for (j = 0; j < dummy_load_calib * load_ms; j++)
        for (i = 0 ; i < DUMMY_LOAD_ITER; i++) 
            __asm__ volatile ("nop");
}

using namespace std::chrono_literals;

class StartNode : public rclcpp::Node
{
public:
    StartNode(const std::string node_name, const std::string pub_topic, std::shared_ptr<trace::Trace> trace_ptr, int exe_time, int period, bool end_flag) 
        : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(USE_INTRA_PROCESS_COMMS)), count_(0), trace_callbacks_(trace_ptr), exe_time_(exe_time), period_(period), end_flag_(end_flag)
    {
        //publisher_ = this->create_publisher<std_msgs::msg::String>(pub_topic, 1);
        publisher_ = this->create_publisher<test_interfaces::msg::TestString>(pub_topic, 1);

        if (period_ == 70)
            timer_ = this->create_wall_timer(70ms, std::bind(&StartNode::timer_callback, this));
        else if (period_ == 80)
            timer_ = this->create_wall_timer(80ms, std::bind(&StartNode::timer_callback, this));
        else if (period_ == 100)
            timer_ = this->create_wall_timer(100ms, std::bind(&StartNode::timer_callback, this));
        else if (period_ == 120)
            timer_ = this->create_wall_timer(120ms, std::bind(&StartNode::timer_callback, this));
        else if (period_ == 160)
            timer_ = this->create_wall_timer(160ms, std::bind(&StartNode::timer_callback, this));
        else if (period_ == 200)
            timer_ = this->create_wall_timer(200ms, std::bind(&StartNode::timer_callback, this));
        else if (period_ == 1000)
            timer_ = this->create_wall_timer(1000ms, std::bind(&StartNode::timer_callback, this));        
        else
            timer_ = this->create_wall_timer(10000ms, std::bind(&StartNode::timer_callback, this));

        gettimeofday(&create_timer, NULL);
        RCLCPP_INFO(this->get_logger(), "Create wall timer at %ld", create_timer.tv_sec*1000+create_timer.tv_usec/1000);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<test_interfaces::msg::TestString>::SharedPtr publisher_;
private:
    size_t count_;
    int exe_time_;
    int period_;
    timeval ctime, ftime, create_timer, latency_time;
    bool end_flag_;
    std::shared_ptr<trace::Trace> trace_callbacks_;

    void dummy_task(int load) {
        int i;
        for (i = 0 ; i < load; i++) 
            __asm__ volatile ("nop");
    }

    void timer_callback()
    {
        std::string name = this->get_name();            
        RCLCPP_INFO(this->get_logger(), ("callback: " + name).c_str());
        gettimeofday(&ctime, NULL);
        //trace_callbacks_->trace_write(name+"_in",std::to_string(ctime.tv_sec*1000+ctime.tv_usec/1000));
        //trace_callbacks_->trace_write_count(name+"_in",std::to_string(ctime.tv_sec*1000+ctime.tv_usec/1000),std::to_string(count_));

        dummy_load(exe_time_);

        auto message = test_interfaces::msg::TestString();
        message.data = std::to_string(count_++);
        message.stamp.sec = ctime.tv_sec;
        message.stamp.usec = ctime.tv_usec;

        //gettimeofday(&ftime, NULL);
        //trace_callbacks_->trace_write(name+"_out",std::to_string(ftime.tv_sec*1000+ftime.tv_usec/1000));         
        if (end_flag_) {
            gettimeofday(&ftime, NULL);
            latency_time.tv_sec = (ftime.tv_sec - message.stamp.sec);
            latency_time.tv_usec = (ftime.tv_usec - message.stamp.usec);                
            //trace_callbacks_->trace_write_count(name+"_latency",std::to_string(latency_time.tv_sec*1000000+latency_time.tv_usec), message.data);  
        }   
        publisher_->publish(message);
    }        
};

class IntermediateNode : public rclcpp::Node
{
public:
    IntermediateNode(const std::string node_name, const std::string sub_topic, const std::string pub_topic, std::shared_ptr<trace::Trace> trace_ptr, int exe_time, bool end_flag) 
        : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(USE_INTRA_PROCESS_COMMS)), count_(0), trace_callbacks_(trace_ptr), exe_time_(exe_time), end_flag_(end_flag)
    {                        
        subscription_ = this->create_subscription<test_interfaces::msg::TestString>(sub_topic, 1, std::bind(&IntermediateNode::callback, this, _1));
        if (pub_topic != "") publisher_ = this->create_publisher<test_interfaces::msg::TestString>(pub_topic, 1);
    }

    rclcpp::Publisher<test_interfaces::msg::TestString>::SharedPtr publisher_;
    rclcpp::Subscription<test_interfaces::msg::TestString>::SharedPtr subscription_;
private:
    size_t count_;
    int exe_time_;
    timeval ctime, ftime, latency_time;
    double latency;
    std::shared_ptr<trace::Trace> trace_callbacks_;
    bool end_flag_;

    void dummy_task(int load) {
        int i;
        for (i = 0 ; i < load; i++) 
            __asm__ volatile ("nop");
    }

    void callback(const test_interfaces::msg::TestString::SharedPtr msg) {
        std::string name = this->get_name();
        RCLCPP_INFO(this->get_logger(), ("callback: " + name).c_str());
        //gettimeofday(&ctime, NULL);            
        //trace_callbacks_->trace_write(name+"_in",std::to_string(ctime.tv_sec*1000+ctime.tv_usec/1000));            
        dummy_load(exe_time_);

        auto message = test_interfaces::msg::TestString();
        message.data = msg->data;
        message.stamp = msg->stamp;

        if (end_flag_) {
            gettimeofday(&ftime, NULL);
            latency_time.tv_sec = (ftime.tv_sec - msg->stamp.sec);
            latency_time.tv_usec = (ftime.tv_usec - msg->stamp.usec);
            trace_callbacks_->trace_write_count(name+"_latency",std::to_string(latency_time.tv_sec*1000000+latency_time.tv_usec), message.data);  
        }

        if (publisher_) publisher_->publish(message);
    }        
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PID: %ld run in ROS2.", gettid());

    // Naive way to calibrate dummy workload for current system
    while (1) {
        timeval ctime, ftime;
        int duration_us;
        gettimeofday(&ctime, NULL);
        dummy_load(100); // 100ms
        gettimeofday(&ftime, NULL);
        duration_us = (ftime.tv_sec - ctime.tv_sec) * 1000000 + (ftime.tv_usec - ctime.tv_usec);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "dummy_load_calib: %d (duration_us: %d ns)", dummy_load_calib, duration_us);
        if (abs(duration_us - 100 * 1000) < 500) { // error margin: 500us
            break;
        }
        dummy_load_calib = 100 * 1000 * dummy_load_calib / duration_us;
        if (dummy_load_calib <= 0) dummy_load_calib = 1;
    }

    timeval ctime;
    gettimeofday(&ctime, NULL);
    std::shared_ptr<trace::Trace> trace_callbacks = std::make_shared<trace::Trace>("data/trace.txt");
    trace_callbacks->trace_write("init",std::to_string(ctime.tv_sec*1000+ctime.tv_usec/1000));

    // Create callbacks
    auto task1 = std::make_shared<StartNode>("C1T_12_26", "task1", trace_callbacks, 2, 80, false);
    auto task2 = std::make_shared<IntermediateNode>("C1R1_12_30", "task1", "task2", trace_callbacks, 16, true);
    
    auto task3 = std::make_shared<IntermediateNode>("C2R1_11_27", "task1", "task3", trace_callbacks, 2, false);
    auto task4 = std::make_shared<IntermediateNode>("C2R2_11_28", "task3", "task4", trace_callbacks, 18, false);
    auto task5 = std::make_shared<IntermediateNode>("C2R3_11_29", "task4", "task5", trace_callbacks, 9, true);

    auto task6 = std::make_shared<StartNode>("C3T_10_22", "task6", trace_callbacks, 23, 100, false);
    auto task7 = std::make_shared<IntermediateNode>("C3R1_10_23", "task6", "task7", trace_callbacks, 8, false);
    auto task8 = std::make_shared<IntermediateNode>("C3R2_10_24", "task7", "task8", trace_callbacks, 14, false);
    auto task9 = std::make_shared<IntermediateNode>("C3R3_10_25", "task8", "task9", trace_callbacks, 18, true);

    auto task10 = std::make_shared<StartNode>("C4T_9_19", "task10", trace_callbacks, 21, 100, false);
    auto task11 = std::make_shared<IntermediateNode>("C4R1_9_20", "task10", "task11", trace_callbacks, 18, false);
    auto task12 = std::make_shared<IntermediateNode>("C4R2_9_21", "task11", "task12", trace_callbacks, 7, true);

    auto task13 = std::make_shared<StartNode>("C5T_8_15", "task13", trace_callbacks, 2, 160, false);
    auto task14 = std::make_shared<IntermediateNode>("C5R1_8_16", "task13", "task14", trace_callbacks, 11, false);
    auto task15 = std::make_shared<IntermediateNode>("C5R2_8_17", "task14", "task15", trace_callbacks, 7, false);
    auto task16 = std::make_shared<IntermediateNode>("C5R3_8_18", "task15", "task16", trace_callbacks, 8, true);

    auto task17 = std::make_shared<StartNode>("C6T_7_13", "task17", trace_callbacks, 2, 1000, false);
    auto task18 = std::make_shared<IntermediateNode>("C6R1_7_14", "task17", "task18", trace_callbacks, 196, true);

    auto task19 = std::make_shared<StartNode>("C7T_6_11", "task19", trace_callbacks, 33, 120, false);
    auto task20 = std::make_shared<IntermediateNode>("C7R1_6_12", "task19", "task20", trace_callbacks, 2, true);
    
    auto task21 = std::make_shared<StartNode>("C8T_5_9", "task21", trace_callbacks, 33, 120, false);
    auto task22 = std::make_shared<IntermediateNode>("C8R1_5_10", "task21", "task22", trace_callbacks, 7, true);

    auto task23 = std::make_shared<StartNode>("C9T_4_7", "task23", trace_callbacks, 33, 120, false);
    auto task24 = std::make_shared<IntermediateNode>("C9R1_4_8", "task23", "task24", trace_callbacks, 7, true);

    auto task25 = std::make_shared<StartNode>("C10T_3_5", "task25", trace_callbacks, 33, 120, false);
    auto task26 = std::make_shared<IntermediateNode>("C10R1_3_6", "task25", "task26", trace_callbacks, 2, true);

    auto task27 = std::make_shared<StartNode>("C11T_2_3", "task27", trace_callbacks, 33, 120, false);
    auto task28 = std::make_shared<IntermediateNode>("C11R1_2_4", "task27", "task28", trace_callbacks, 2, true);

    auto task29 = std::make_shared<StartNode>("C12T_1_1", "task29", trace_callbacks, 33, 120, false);
    auto task30 = std::make_shared<IntermediateNode>("C12R1_1_2", "task29", "task30", trace_callbacks, 2, true);

    // Create executors
    rclcpp::executors::SingleThreadedExecutor exec1;rclcpp::executors::SingleThreadedExecutor exec2;
    rclcpp::executors::SingleThreadedExecutor exec3;
    rclcpp::executors::SingleThreadedExecutor exec4;
    
#ifdef PICAS
    rclcpp::executors::SingleThreadedExecutor exec5;
    rclcpp::executors::SingleThreadedExecutor exec6;
    rclcpp::executors::SingleThreadedExecutor exec7;
    rclcpp::executors::SingleThreadedExecutor exec8;
    rclcpp::executors::SingleThreadedExecutor exec9;
    rclcpp::executors::SingleThreadedExecutor exec10;
    rclcpp::executors::SingleThreadedExecutor exec11;
    rclcpp::executors::SingleThreadedExecutor exec12;
    rclcpp::executors::SingleThreadedExecutor exec13;
    rclcpp::executors::SingleThreadedExecutor exec14;
    rclcpp::executors::SingleThreadedExecutor exec15;
    rclcpp::executors::SingleThreadedExecutor exec16;
    rclcpp::executors::SingleThreadedExecutor exec17;
    rclcpp::executors::SingleThreadedExecutor exec18;
    
    // Enable priority-based callback scheduling
    exec1.enable_callback_priority();
    exec2.enable_callback_priority();
    exec3.enable_callback_priority();
    exec4.enable_callback_priority();
    exec5.enable_callback_priority();
    exec6.enable_callback_priority();
    exec7.enable_callback_priority();
    exec8.enable_callback_priority();
    exec9.enable_callback_priority();
    exec10.enable_callback_priority();
    exec11.enable_callback_priority();
    exec12.enable_callback_priority();
    exec13.enable_callback_priority();
    exec14.enable_callback_priority();
    exec15.enable_callback_priority();
    exec16.enable_callback_priority();
    exec17.enable_callback_priority();
    exec18.enable_callback_priority();
    
    // Set executor's RT priority and CPU allocation
    exec1.set_executor_priority_cpu(90, 2);
    exec2.set_executor_priority_cpu(89, 3);
    exec3.set_executor_priority_cpu(88, 4);
    exec4.set_executor_priority_cpu(87, 5);
    exec5.set_executor_priority_cpu(86, 2);
    exec6.set_executor_priority_cpu(85, 5);
    exec7.set_executor_priority_cpu(84, 4);
    exec8.set_executor_priority_cpu(83, 3);
    exec9.set_executor_priority_cpu(82, 3);
    exec10.set_executor_priority_cpu(81, 5);
    exec11.set_executor_priority_cpu(80, 2);
    exec12.set_executor_priority_cpu(79, 4);
    exec13.set_executor_priority_cpu(78, 5);
    exec14.set_executor_priority_cpu(77, 2);
    exec15.set_executor_priority_cpu(76, 2);
    exec16.set_executor_priority_cpu(75, 3);
    exec17.set_executor_priority_cpu(74, 3);
    exec18.set_executor_priority_cpu(73, 4);
#endif

    
#ifdef PICAS
    // Allocate callbacks to executors
    exec1.add_node(task1); exec1.add_node(task2);
    exec2.add_node(task3); exec2.add_node(task4); exec2.add_node(task5);
    exec3.add_node(task6); exec3.add_node(task7); exec3.add_node(task8); exec3.add_node(task9);
    exec4.add_node(task10); exec4.add_node(task11); exec4.add_node(task12);
    exec5.add_node(task13); exec5.add_node(task14); exec5.add_node(task15); exec5.add_node(task16);
    exec6.add_node(task17); exec6.add_node(task18);
    exec7.add_node(task19); 
    exec8.add_node(task20); 
    exec9.add_node(task21);
    exec10.add_node(task22); 
    exec11.add_node(task23); 
    exec12.add_node(task24);
    exec13.add_node(task25); 
    exec14.add_node(task26);
    exec15.add_node(task27); 
    exec16.add_node(task28); 
    exec17.add_node(task29);
    exec18.add_node(task30);

    // Assign callbacks' priority
    exec1.set_callback_priority(task1->timer_, 26);
    exec1.set_callback_priority(task2->subscription_, 30);
    exec2.set_callback_priority(task3->subscription_, 27);
    exec2.set_callback_priority(task4->subscription_, 28);
    exec2.set_callback_priority(task5->subscription_, 29);
    exec3.set_callback_priority(task6->timer_, 22);
    exec3.set_callback_priority(task7->subscription_, 23);
    exec3.set_callback_priority(task8->subscription_, 24);
    exec3.set_callback_priority(task9->subscription_, 25);
    exec4.set_callback_priority(task10->timer_, 19);
    exec4.set_callback_priority(task11->subscription_, 20);
    exec4.set_callback_priority(task12->subscription_, 21);
    exec5.set_callback_priority(task13->timer_, 15);
    exec5.set_callback_priority(task14->subscription_, 16);
    exec5.set_callback_priority(task15->subscription_, 17);
    exec5.set_callback_priority(task16->subscription_, 18);
    exec6.set_callback_priority(task17->timer_, 13);
    exec6.set_callback_priority(task18->subscription_, 14);
    exec7.set_callback_priority(task19->timer_, 11);
    exec8.set_callback_priority(task20->subscription_, 12);
    exec9.set_callback_priority(task21->timer_, 9);
    exec10.set_callback_priority(task22->subscription_, 10);
    exec11.set_callback_priority(task23->timer_, 7);
    exec12.set_callback_priority(task24->subscription_, 8);
    exec13.set_callback_priority(task25->timer_, 5);
    exec14.set_callback_priority(task26->subscription_, 6);
    exec15.set_callback_priority(task27->timer_, 3);
    exec16.set_callback_priority(task28->subscription_, 4);
    exec17.set_callback_priority(task29->timer_, 1);
    exec18.set_callback_priority(task30->subscription_, 2);

    std::thread spinThread1(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &exec1);
    std::thread spinThread2(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &exec2);
    std::thread spinThread3(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &exec3);
    std::thread spinThread4(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &exec4);
    std::thread spinThread5(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &exec5);
    std::thread spinThread6(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &exec6);
    std::thread spinThread7(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &exec7);
    std::thread spinThread8(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &exec8);
    std::thread spinThread9(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &exec9);
    std::thread spinThread10(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &exec10);
    std::thread spinThread11(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &exec11);
    std::thread spinThread12(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &exec12);
    std::thread spinThread13(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &exec13);
    std::thread spinThread14(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &exec14);
    std::thread spinThread15(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &exec15);
    std::thread spinThread16(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &exec16);
    std::thread spinThread17(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &exec17);
    std::thread spinThread18(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &exec18);

    spinThread1.join();
    spinThread2.join();
    spinThread3.join();
    spinThread4.join();
    spinThread5.join();
    spinThread6.join();
    spinThread7.join();
    spinThread8.join();
    spinThread9.join();
    spinThread10.join();
    spinThread11.join();
    spinThread12.join();
    spinThread13.join();
    spinThread14.join();
    spinThread15.join();
    spinThread16.join();
    spinThread17.join();
    spinThread18.join();

    exec1.remove_node(task1); exec1.remove_node(task2);
    exec2.remove_node(task3); exec2.remove_node(task4); exec2.remove_node(task5); 
    exec3.remove_node(task6); exec3.remove_node(task7); exec3.remove_node(task8); exec3.remove_node(task9);
    exec4.remove_node(task10); exec4.remove_node(task11); exec4.remove_node(task12);
    exec5.remove_node(task13); exec5.remove_node(task14); exec5.remove_node(task15); exec5.remove_node(task16);
    exec6.remove_node(task17); exec6.remove_node(task18);
    
    exec7.remove_node(task19); exec8.remove_node(task20); exec9.remove_node(task21); exec10.remove_node(task22);
    exec11.remove_node(task23); exec12.remove_node(task24); exec13.remove_node(task25); exec14.remove_node(task26);
    exec15.remove_node(task27); exec16.remove_node(task28); exec17.remove_node(task29); exec18.remove_node(task30);
#else
    // Allocate callbacks to executors
    exec1.add_node(task1); exec1.add_node(task2);
    exec1.add_node(task13); exec1.add_node(task14); exec1.add_node(task15); exec1.add_node(task16);
    exec1.add_node(task23); exec1.add_node(task26); exec1.add_node(task27);
    
    exec2.add_node(task3); exec2.add_node(task4); exec2.add_node(task5); 
    exec2.add_node(task21); exec2.add_node(task20); exec2.add_node(task28); exec2.add_node(task29);
    
    exec3.add_node(task6); exec3.add_node(task7); exec3.add_node(task8); exec3.add_node(task9);
    exec3.add_node(task19); exec3.add_node(task24); exec3.add_node(task30);
    
    exec4.add_node(task10); exec4.add_node(task11); exec4.add_node(task12);
    exec4.add_node(task17); exec4.add_node(task18);
    exec4.add_node(task25); exec4.add_node(task22);
        
    //std::thread spinThread1(&rclcpp::executors::SingleThreadedExecutor::spin_cpu, &exec1, 2);
    //std::thread spinThread2(&rclcpp::executors::SingleThreadedExecutor::spin_cpu, &exec2, 3);
    //std::thread spinThread3(&rclcpp::executors::SingleThreadedExecutor::spin_cpu, &exec3, 4);
    //std::thread spinThread4(&rclcpp::executors::SingleThreadedExecutor::spin_cpu, &exec4, 5);
    std::thread spinThread1(&rclcpp::executors::SingleThreadedExecutor::spin, &exec1);
    std::thread spinThread2(&rclcpp::executors::SingleThreadedExecutor::spin, &exec2);
    std::thread spinThread3(&rclcpp::executors::SingleThreadedExecutor::spin, &exec3);
    std::thread spinThread4(&rclcpp::executors::SingleThreadedExecutor::spin, &exec4);

    spinThread1.join();
    spinThread2.join();
    spinThread3.join();
    spinThread4.join();

    exec1.remove_node(task1); exec1.remove_node(task2);
    exec1.remove_node(task13); exec1.remove_node(task14); exec1.remove_node(task15); exec1.remove_node(task16);
    exec1.remove_node(task23); exec1.remove_node(task26); exec1.remove_node(task27);
    
    exec2.remove_node(task3); exec2.remove_node(task4); exec2.remove_node(task5); 
    exec2.remove_node(task21); exec2.remove_node(task20); exec2.remove_node(task28); exec2.remove_node(task29);
    
    exec3.remove_node(task6); exec3.remove_node(task7); exec3.remove_node(task8); exec3.remove_node(task9);
    exec3.remove_node(task19); exec3.remove_node(task24); exec3.remove_node(task30);
    
    exec4.remove_node(task10); exec4.remove_node(task11); exec4.remove_node(task12);
    exec4.remove_node(task17); exec4.remove_node(task18);
    exec4.remove_node(task25); exec4.remove_node(task22);
    
#endif

    rclcpp::shutdown();
    return 0;
}
