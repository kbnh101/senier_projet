#ifndef STATE_H
#define STATE_H
#include<iostream>
#include<ros/ros.h>
#include<std_msgs/String.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/PoseStamped.h>

class state
{
public:
    state();
    //void statecallback(const std_msgs::String msg);
    void statecallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void TargetPointtCallback(const geometry_msgs::PoseStamped& msg);

    bool flag;
    void process();

    std::string state_msgs;
    std_msgs::String msg;

    ros::Publisher state_pub;
    ros::Subscriber state_sub;
    ros::Subscriber targetPoint_sub;
};

#endif // STATE_H
