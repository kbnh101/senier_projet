#include<ros/ros.h>
#include<iostream>
#include<opencv2/opencv.hpp>
#include<math.h>
#include<cmath>
#include<vector>
#include<queue>
#include<tf/tf.h>
#include "pure_pursuit.h"

#include<nav_msgs/Path.h>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/OccupancyGrid.h>

#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>

#include<visualization_msgs/MarkerArray.h>

#include<ackermann_msgs/AckermannDriveStamped.h>
#include<ackermann_msgs/AckermannDrive.h>

#include<sensor_msgs/PointCloud.h>

#include<std_msgs/String.h>

#include<csv.h>

struct Node
{
    int id;
    double x;
    double y;
};

class local_path : public pure_pursuit{
    int num1;
    int prev_temp_num;
public:
    local_path();
    double vehicle_yaw;
    double vehicle_pitch;
    double vehicle_roll;
    double rate = 10;
    int x_offset, y_offset, height, resolution;
    int speed;
    int num;
    bool flag;
    bool stop, GO;

    vector<Node> node;

    nav_msgs::Path path;
    nav_msgs::Path trajectory_path;
    nav_msgs::Path tracking_path;
    nav_msgs::Odometry odometry;

    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped temp_pose;
    geometry_msgs::PoseStamped end_pose;
    geometry_msgs::PoseWithCovarianceStamped start_point;

    ackermann_msgs::AckermannDriveStamped cmd_vel;
    ackermann_msgs::AckermannDriveStamped cmd_vel_;

    sensor_msgs::PointCloud obstacle;

    std_msgs::String state;

    //Publish
    ros::Publisher cmd_pub;
    ros::Publisher path_pub;
    ros::Publisher pose_pub;
    ros::Publisher cmd_pub_;
    ros::Publisher marker_vel;
    //Subscriber
    ros::Subscriber pose_sub;
    ros::Subscriber path_sub;
    ros::Subscriber trajectory;
    ros::Subscriber sim_pose;
    ros::Subscriber speed_sub;
    ros::Subscriber obstacle_sub;
    ros::Subscriber state_sub;
    ros::Subscriber initial_pose;
    ros::Subscriber end_pose_sub;
    ros::Subscriber GO_sub;
    //Callback
    void pathcallback(const nav_msgs::Path& msg);
    void trajectorycallback(const nav_msgs::Path& msg);
    void posecallback(const geometry_msgs::PoseStamped& msg);
    void sim_pose_callback(const nav_msgs::Odometry& msg);
    void speed_callback(const std_msgs::Int32 &msg);
    void obstacle_callback(const sensor_msgs::PointCloud &msg);
    void state_callback(const std_msgs::String &msg);
    void initial_pose_callback(const std_msgs::String& msg);
    void end_pose_callback(const std_msgs::String& msg);
    void go_sub_callback(const std_msgs::String& msg);
    //Function
    double velocity();
    double obstacle_distance(sensor_msgs::PointCloud obstacle);
    double get_passenger(geometry_msgs::PoseStamped pose);
    double get_off_passenger(geometry_msgs::PoseStamped pose);
    void node_parse(istream& file);
    void path_tracking(nav_msgs::Path global_path);
    void process();
};
