#include <ball_simulator/ClockConfig.h>
#include <chrono>
#include <dynamic_reconfigure/server.h>
#include <iostream>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <thread>

struct ClockConfig
{
    ClockConfig(ros::NodeHandle nh) : use_step(false), ratio(1.0)
    {
        double step = 0.06;
        nh.param("use_step", use_step, use_step);
        nh.param("step", step, step);
        nh.param("ratio", ratio, ratio);
        step_duration = ros::Duration(step);

        dynamic_reconfigure::Server<ball_simulator::ClockConfig>::CallbackType
            reconfigure_callback =
                boost::bind(&ClockConfig::reconfigure, this, _1, _2);
        reconfiguration_server.setCallback(reconfigure_callback);
    }

    void reconfigure(ball_simulator::ClockConfig& config, uint32_t level)
    {
        use_step = config.use_step;
        step_duration = ros::Duration(config.step);
        ratio = config.ratio;
    }

    dynamic_reconfigure::Server<ball_simulator::ClockConfig>
        reconfiguration_server;
    bool use_step;
    ros::Duration step_duration;
    double ratio;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "clock");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");
    ros::Publisher clock_pub =
        nh.advertise<rosgraph_msgs::Clock>("clock", 1, true);
    rosgraph_msgs::Clock clock_msg;

    double rate = 100.0;
    pn.param("rate", rate, rate);

    ClockConfig config(pn);

    std::chrono::milliseconds period_ms(int(1000.0 / (config.ratio * rate)));
    auto last_tick = std::chrono::steady_clock::now();

    clock_msg.clock = ros::Time(
        std::chrono::duration<double>(last_tick.time_since_epoch()).count());

    while (nh.ok())
    {
        ros::spinOnce();
        clock_pub.publish(clock_msg);

        if (config.use_step)
        {
            std::cout << "press enter to step clock" << std::endl;
            std::cin.get();
            clock_msg.clock += config.step_duration;
        }
        else
        {
            std::this_thread::sleep_for(period_ms);
            auto tick = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_time = tick - last_tick;
            last_tick = tick;
            clock_msg.clock +=
                ros::Duration(config.ratio * elapsed_time.count());
        }
    }
}