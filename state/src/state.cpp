#include "state.h"

state::state()
{
    ros::NodeHandle nh;

    state_sub = nh.subscribe("initialpose",10,&state::statecallback,this);
    targetPoint_sub = nh.subscribe("move_base_simple/goal", 10, &state::TargetPointtCallback,this);
    state_pub = nh.advertise<std_msgs::String>("state",10);

    flag = true;
}

void state::statecallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    flag = false;
}

void state::TargetPointtCallback(const geometry_msgs::PoseStamped& msg)
{
    flag = true;
}


void state::process()
{
    if(flag == true)
    {
        state_msgs = "green";
    }
    else if(flag == false)
    {
        state_msgs = "red";
    }
    std::stringstream ss;
    ss<<state_msgs;
    msg.data = ss.str();
    state_pub.publish(msg);
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "state");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    state tt;

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        tt.process();
        loop_rate.sleep();
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}
