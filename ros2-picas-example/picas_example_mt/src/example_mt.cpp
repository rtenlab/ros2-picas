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
#include "test_msgs/msg/detail/test_string__struct.hpp"

using std::placeholders::_1;

#define gettid() syscall(__NR_gettid)
#define USE_INTRA_PROCESS_COMMS true
#define DUMMY_LOAD_ITER 10000
#define JITTER 5

int dummy_load_calib = 1;

void dummy_load(int load_ms)
{
    int i, j;
    for (j = 0; j < dummy_load_calib * load_ms; j++)
        for (i = 0; i < DUMMY_LOAD_ITER; i++)
            __asm__ volatile("nop");
}

using namespace std::chrono_literals;

class StartNode : public rclcpp::Node
{
public:
    StartNode(const std::string node_name, const std::string pub_topic, std::shared_ptr<trace::Trace> trace_ptr1, std::string filepath, int exe_time, int period, bool end_flag)
        : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(USE_INTRA_PROCESS_COMMS)), count_(0), trace_latency_(trace_ptr1), exe_time_(exe_time), period_(period), end_flag_(end_flag)
    {
        publisher_ = this->create_publisher<test_msgs::msg::TestString>(pub_topic, 1);

        trace_exectime_ = std::make_shared<trace::Trace>((filepath + node_name + "_E.txt").c_str());

        if (period_ == 50)
            timer_ = this->create_wall_timer(50ms, std::bind(&StartNode::timer_callback, this));
        else if (period_ == 100)
            timer_ = this->create_wall_timer(100ms, std::bind(&StartNode::timer_callback, this));
        else
            timer_ = this->create_wall_timer(200ms, std::bind(&StartNode::timer_callback, this));

        gettimeofday(&create_timer, NULL);
        RCLCPP_INFO(this->get_logger(), "Create wall timer at %ld", create_timer.tv_sec * 1000 + create_timer.tv_usec / 1000);

        last_rtime.tv_sec = last_rtime.tv_usec = 0;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<test_msgs::msg::TestString>::SharedPtr publisher_;

private:
    size_t count_;
    int exe_time_;
    int period_;
    timeval ctime, ftime, create_timer, latency_time;
    bool end_flag_;
    std::shared_ptr<trace::Trace> trace_exectime_;
    std::shared_ptr<trace::Trace> trace_latency_;
    timeval last_rtime; // previous release time

    void timer_callback()
    {
        // find actual execution time
        timeval t1, t2, AET;
        gettimeofday(&t1, NULL);
        ctime = t1;

        // find actual release time
        int elapsed = period_ * 1000 - timer_->time_until_trigger().count() / 1000;
        if (elapsed < 0)
        {
            RCLCPP_INFO(this->get_logger(), "Elapsed time error: elapsed %d, time_until_trigger %ld, period %d (ms)",
                        elapsed / 1000, timer_->time_until_trigger().count() / 1000000, period_);
        }
        if (ctime.tv_usec < elapsed)
        {
            ctime.tv_sec--;
            ctime.tv_usec += 1000000;
        }
        ctime.tv_usec -= elapsed;

        if (last_rtime.tv_sec)
        {
            int interval = (ctime.tv_sec - last_rtime.tv_sec) * 1000000 + ctime.tv_usec - last_rtime.tv_usec;
            interval /= 1000;
            if (interval > period_ + JITTER)
            {
                RCLCPP_INFO(this->get_logger(), "Job Skipped: last release %ld, this %ld, period %d, gap %d (ms)",
                            last_rtime.tv_sec * 1000 + last_rtime.tv_usec / 1000,
                            ctime.tv_sec * 1000 + ctime.tv_usec / 1000,
                            period_,
                            interval);
            }
        }
        last_rtime = ctime;

        std::string name = this->get_name();
        RCLCPP_INFO(this->get_logger(), ("callback: " + name).c_str());

        dummy_load(exe_time_);

        auto message = test_msgs::msg::TestString();
        message.data = std::to_string(count_++);
        message.stamp.sec = ctime.tv_sec;
        message.stamp.usec = ctime.tv_usec;

        if (end_flag_)
        {
            gettimeofday(&ftime, NULL);
            latency_time.tv_sec = (ftime.tv_sec - message.stamp.sec);
            latency_time.tv_usec = (ftime.tv_usec - message.stamp.usec);
            trace_latency_->trace_write_count(name + "_latency", std::to_string(latency_time.tv_sec * 1000000 + latency_time.tv_usec), message.data);
        }
        publisher_->publish(message);

        gettimeofday(&t2, NULL);
        AET.tv_sec = t2.tv_sec - t1.tv_sec;
        AET.tv_usec = t2.tv_usec - t1.tv_usec;
        trace_exectime_->trace_write(name + "_exec_time ", std::to_string(AET.tv_sec * 1000000 + AET.tv_usec)); // microsecond
    }
};

class IntermediateNode : public rclcpp::Node
{
public:
    IntermediateNode(const std::string node_name, const std::string sub_topic, const std::string pub_topic, std::shared_ptr<trace::Trace> trace_ptr1, std::string filepath, int exe_time, bool end_flag)
        : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(USE_INTRA_PROCESS_COMMS)), count_(0), trace_latency_(trace_ptr1), exe_time_(exe_time), end_flag_(end_flag)
    {

        subscription_ = this->create_subscription<test_msgs::msg::TestString>(sub_topic, 1, std::bind(&IntermediateNode::callback, this, _1));
        if (pub_topic != "")
            publisher_ = this->create_publisher<test_msgs::msg::TestString>(pub_topic, 1);

        trace_exectime_ = std::make_shared<trace::Trace>(filepath + node_name + "_E.txt");
    }

    rclcpp::Publisher<test_msgs::msg::TestString>::SharedPtr publisher_;
    rclcpp::Subscription<test_msgs::msg::TestString>::SharedPtr subscription_;

private:
    size_t count_;
    int exe_time_;
    timeval ctime, ftime, latency_time;
    double latency;
    std::shared_ptr<trace::Trace> trace_exectime_;
    std::shared_ptr<trace::Trace> trace_latency_;
    bool end_flag_;

    void callback(const test_msgs::msg::TestString::SharedPtr msg)
    {

        // find actual execution time
        timeval t1, t2, AET;
        gettimeofday(&t1, NULL);

        size_t new_count = std::stoi(msg->data);
        if (count_ != 0 && count_ + 1 != new_count)
        {
            RCLCPP_INFO(this->get_logger(), "Job Skipped: last count %lu, this %lu, missed %lu jobs",
                        count_, new_count, new_count - count_ - 1);
        }
        count_ = new_count;

        std::string name = this->get_name();
        RCLCPP_INFO(this->get_logger(), ("callback: " + name).c_str());

        dummy_load(exe_time_);

        auto message = test_msgs::msg::TestString();
        message.data = msg->data;
        message.stamp = msg->stamp;

        if (end_flag_)
        {
            gettimeofday(&ftime, NULL);
            latency_time.tv_sec = (ftime.tv_sec - msg->stamp.sec);
            latency_time.tv_usec = (ftime.tv_usec - msg->stamp.usec);
            trace_latency_->trace_write_count(name + "_latency", std::to_string(latency_time.tv_sec * 1000000 + latency_time.tv_usec), message.data);
        }

        if (publisher_)
            publisher_->publish(message);

        gettimeofday(&t2, NULL);
        AET.tv_sec = t2.tv_sec - t1.tv_sec;
        AET.tv_usec = t2.tv_usec - t1.tv_usec;
        trace_exectime_->trace_write(name + "_exec_time ", std::to_string(AET.tv_sec * 1000000 + AET.tv_usec));
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PID: %ld run in ROS2.", gettid());

    // Naive way to calibrate dummy workload for current system
    while (1)
    {
        timeval ctime, ftime;
        int duration_us;
        gettimeofday(&ctime, NULL);
        dummy_load(100); // 100ms
        gettimeofday(&ftime, NULL);
        duration_us = (ftime.tv_sec - ctime.tv_sec) * 1000000 + (ftime.tv_usec - ctime.tv_usec);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "dummy_load_calib: %d (duration_us: %d ns)", dummy_load_calib, duration_us);
        if (abs(duration_us - 100 * 1000) < 500)
        { // error margin: 500us
            break;
        }
        dummy_load_calib = 100 * 1000 * dummy_load_calib / duration_us;
        if (dummy_load_calib <= 0)
            dummy_load_calib = 1;
    }

    timeval ctime;
    gettimeofday(&ctime, NULL);

    std::string filepath = "data/case_study/mt4/S-Re/";
    std::shared_ptr<trace::Trace> trace_latency = std::make_shared<trace::Trace>("data/case_study/mt4/S-Re/R.txt");

    // Create callbacks
    auto task9 = std::make_shared<StartNode>("C4T_1", "task9", trace_latency, filepath, 21, 200, false);
    auto task6 = std::make_shared<StartNode>("C3T_5", "task6", trace_latency, filepath, 11, 100, false);
    auto task1 = std::make_shared<StartNode>("C1T_8", "task1", trace_latency, filepath, 3, 50, false);

    auto task2 = std::make_shared<IntermediateNode>("C1R1_12", "task1", "task2", trace_latency, filepath, 3, true);

    auto task3 = std::make_shared<IntermediateNode>("C2R1_9", "task1", "task3", trace_latency, filepath, 4, false);
    auto task4 = std::make_shared<IntermediateNode>("C2R2_10", "task3", "task4", trace_latency, filepath, 3, false);
    auto task5 = std::make_shared<IntermediateNode>("C2R3_11", "task4", "task5", trace_latency, filepath, 4, true);

    auto task7 = std::make_shared<IntermediateNode>("C3R1_6", "task6", "task7", trace_latency, filepath, 12, false);
    auto task8 = std::make_shared<IntermediateNode>("C3R2_7", "task7", "task8", trace_latency, filepath, 12, true);

    auto task10 = std::make_shared<IntermediateNode>("C4R1_2", "task9", "task10", trace_latency, filepath, 41, false);
    auto task11 = std::make_shared<IntermediateNode>("C4R2_3", "task10", "task11", trace_latency, filepath, 42, false);
    auto task12 = std::make_shared<IntermediateNode>("C4R3_4", "task11", "task12", trace_latency, filepath, 22, true);

    // Create executors
    int number_of_threads = 4;
    rclcpp::executors::MultiThreadedExecutor exec1(rclcpp::ExecutorOptions(), number_of_threads, true);

#ifdef PICAS

    // Enable priority-based callback scheduling
    exec1.enable_callback_priority();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PiCAS priority-based callback scheduling: %s", exec1.callback_priority_enabled ? "Enabled" : "Disabled");

    // set executor's attributes
    std::vector<int> assigned_cpus = {0, 1, 2, 3};
    exec1.set_executor_priority_cpu(SCHED_FIFO, 90, assigned_cpus);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PiCAS executor 1's rt-priority %d and CPU:", exec1.rt_attr.sched_priority);
    for (int x:exec1.cpus)     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CPU %d", x);


#endif

    // Allocate callbacks to executors (Reverse-priority order)

    exec1.add_node(task9);
    exec1.add_node(task10);
    exec1.add_node(task11);
    exec1.add_node(task12);

    exec1.add_node(task6);
    exec1.add_node(task7);
    exec1.add_node(task8);

    exec1.add_node(task1);
    exec1.add_node(task3);
    exec1.add_node(task4);
    exec1.add_node(task5);

    exec1.add_node(task2);

#ifdef PICAS
    // Assign callbacks' priority
    exec1.set_callback_priority(task1->timer_, 8);
    exec1.set_callback_priority(task2->subscription_, 12);

    exec1.set_callback_priority(task3->subscription_, 9);
    exec1.set_callback_priority(task4->subscription_, 10);
    exec1.set_callback_priority(task5->subscription_, 11);

    exec1.set_callback_priority(task6->timer_, 5);
    exec1.set_callback_priority(task7->subscription_, 6);
    exec1.set_callback_priority(task8->subscription_, 7);

    exec1.set_callback_priority(task9->timer_, 1);
    exec1.set_callback_priority(task10->subscription_, 2);
    exec1.set_callback_priority(task11->subscription_, 3);
    exec1.set_callback_priority(task12->subscription_, 4);

#endif

    exec1.spin();

    exec1.remove_node(task1);
    exec1.remove_node(task2);
    exec1.remove_node(task3);
    exec1.remove_node(task4);
    exec1.remove_node(task5);
    exec1.remove_node(task6);
    exec1.remove_node(task7);
    exec1.remove_node(task8);
    exec1.remove_node(task9);
    exec1.remove_node(task10);
    exec1.remove_node(task11);
    exec1.remove_node(task12);

    rclcpp::shutdown();
    return 0;
}
