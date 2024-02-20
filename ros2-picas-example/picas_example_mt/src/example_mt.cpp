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
#include "test_msgs/msg/detail/test_string__struct.hpp"

using std::placeholders::_1;
//std::mutex mtx;

#define gettid() syscall(__NR_gettid)

//#define USE_INTRA_PROCESS_COMMS false
#define USE_INTRA_PROCESS_COMMS true // Note: if this doesn't work with ROS2 Galactic, update Galactic version to 0.9.3 (20221208) or higher

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
        publisher_ = this->create_publisher<test_msgs::msg::TestString>(pub_topic, 1);

        if (period_ == 10000)
            timer_ = this->create_wall_timer(10000ms, std::bind(&StartNode::timer_callback, this));
        else
            timer_ = this->create_wall_timer(1000ms, std::bind(&StartNode::timer_callback, this));

        gettimeofday(&create_timer, NULL);
        RCLCPP_INFO(this->get_logger(), "Create wall timer at %ld", create_timer.tv_sec*1000+create_timer.tv_usec/1000);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<test_msgs::msg::TestString>::SharedPtr publisher_;
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

        auto message = test_msgs::msg::TestString();
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
        subscription_ = this->create_subscription<test_msgs::msg::TestString>(sub_topic, 1, std::bind(&IntermediateNode::callback, this, _1));
        if (pub_topic != "") publisher_ = this->create_publisher<test_msgs::msg::TestString>(pub_topic, 1);
    }

    rclcpp::Publisher<test_msgs::msg::TestString>::SharedPtr publisher_;
    rclcpp::Subscription<test_msgs::msg::TestString>::SharedPtr subscription_;
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

    void callback(const test_msgs::msg::TestString::SharedPtr msg) {
        std::string name = this->get_name();
        RCLCPP_INFO(this->get_logger(), ("callback: " + name).c_str());
        //gettimeofday(&ctime, NULL);            
        //trace_callbacks_->trace_write(name+"_in",std::to_string(ctime.tv_sec*1000+ctime.tv_usec/1000));            
        dummy_load(exe_time_);

        auto message = test_msgs::msg::TestString();
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
    auto c1_t_cb = std::make_shared<StartNode>("Timer_callback", "c1", trace_callbacks, 100, 1000, false);
    auto c1_r_cb_1 = std::make_shared<IntermediateNode>("Regular_callback1", "c1", "", trace_callbacks, 100, true);
    auto c1_r_cb_2 = std::make_shared<IntermediateNode>("Regular_callback2", "c1", "", trace_callbacks, 100, true);
    auto c1_r_cb_3 = std::make_shared<IntermediateNode>("Regular_callback3", "c1", "", trace_callbacks, 100, true);    
    //auto c1_r_cb_1 = std::make_shared<IntermediateNode>("Regular_callback1", "c1", "c2", trace_callbacks, 1000, true);
    //auto c1_r_cb_2 = std::make_shared<IntermediateNode>("Regular_callback2", "c2", "c3", trace_callbacks, 1000, true);
    //auto c1_r_cb_3 = std::make_shared<IntermediateNode>("Regular_callback3", "c3", "c4", trace_callbacks, 1000, true);

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Manual debug pub_topic - sub_topic : %s - %s", c1_r_cb_2::subscription_, c1_r_cb_2::publisher_);

    // Create executors
    int number_of_threads = 3;
    rclcpp::executors::MultiThreadedExecutor exec1(rclcpp::ExecutorOptions(), number_of_threads, true); 

#ifdef PICAS
    // Enable priority-based callback scheduling
    exec1.enable_callback_priority();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PiCAS priority-based callback scheduling: %s", exec1.callback_priority_enabled ? "Enabled" : "Disabled");

    // Set executor's RT priority and CPU allocation
    //exec1.set_executor_priority_cpu(90, 5);

    // SCHED_FIFO + CPU affinity
    exec1.cpus = {1, 2, 3}; // CPU1, 2, 3
    //exec1.rt_attr.sched_policy = SCHED_FIFO;
    //exec1.rt_attr.sched_priority = 80;
    
    // SCHED_DEADLINE (by default, Linux doesn't allow SCHED_DEADLINE with CPU affinity)
    //exec1.cpus.clear();
    //exec1.rt_attr.sched_policy = SCHED_DEADLINE;
    //exec1.rt_attr.sched_priority = 0;
    //exec1.rt_attr.sched_runtime = 100 * 1000 * 1000; // 100ms budget
    //exec1.rt_attr.sched_period  = 100 * 1000 * 1000; // 100ms period
    //exec1.rt_attr.sched_deadline= 100 * 1000 * 1000; // 100ms deadline

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PiCAS executor 1's rt-priority %d and CPU %d", exec1.executor_priority, exec1.executor_cpu);
#endif

    // Allocate callbacks to executors
    exec1.add_node(c1_t_cb);
    exec1.add_node(c1_r_cb_1);
    exec1.add_node(c1_r_cb_2);    
    exec1.add_node(c1_r_cb_3);    

#ifdef PICAS
    // Assign callbacks' priority (higher value = higher priority)
    exec1.set_callback_priority(c1_t_cb->timer_, 10);
    exec1.set_callback_priority(c1_r_cb_1->subscription_, 11);
    exec1.set_callback_priority(c1_r_cb_2->subscription_, 12);
    exec1.set_callback_priority(c1_r_cb_3->subscription_, 13);
    //try to set thread affinity
    int affinity_threads_timer[] = {1, 2};
    exec1.set_thread_affinity(c1_t_cb->timer_, affinity_threads_timer, (sizeof(affinity_threads_timer) / sizeof(affinity_threads_timer[0])));

    int affinity_threads_sub[] = {2, 3};
    exec1.set_thread_affinity(c1_r_cb_1->subscription_, affinity_threads_sub, (sizeof(affinity_threads_sub) / sizeof(affinity_threads_sub[0])));
    exec1.set_thread_affinity(c1_r_cb_2->subscription_, affinity_threads_sub, (sizeof(affinity_threads_sub) / sizeof(affinity_threads_sub[0])));
    exec1.set_thread_affinity(c1_r_cb_3->subscription_, affinity_threads_sub, (sizeof(affinity_threads_sub) / sizeof(affinity_threads_sub[0])));
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Timer_callback->priority: %d", c1_t_cb->timer_->callback_priority);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Regular_callback1->priority: %d", c1_r_cb_1->subscription_->callback_priority);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Regular_callback2->priority: %d", c1_r_cb_2->subscription_->callback_priority);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Regular_callback3->priority: %d", c1_r_cb_3->subscription_->callback_priority);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Timer_callback->affinity: %d", c1_r_cb_1->subscription_->threadAffinity);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Regular_callback1->affinity: %d", c1_r_cb_1->subscription_->threadAffinity);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Regular_callback2->affinity: %d", c1_r_cb_2->subscription_->threadAffinity);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Regular_callback3->affinity: %d", c1_r_cb_3->subscription_->threadAffinity);
    
#endif

    exec1.spin();

    exec1.remove_node(c1_t_cb);
    exec1.remove_node(c1_r_cb_1);
    exec1.remove_node(c1_r_cb_2);    
    exec1.remove_node(c1_r_cb_3);    

    rclcpp::shutdown();
    return 0;
}
