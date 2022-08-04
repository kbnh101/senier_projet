#include "pure_pursuit.h"

pure_pursuit::pure_pursuit()
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    pnh.param("L", L, 0.3);
    pnh.param("VL",VL,0.325);
    pnh.param("max_lfd",max_lfd,1.5);
    pnh.param("min_lfd",min_lfd,1.0);

    marker_lfd = nh.advertise<visualization_msgs::MarkerArray>("/Look_Forward_Distance",10);
}

void pure_pursuit::lfd_visualiztion(geometry_msgs::Pose index)
{
    visualization_msgs::MarkerArray node_arr;
    visualization_msgs::Marker node1;
    node1.header.frame_id = "/map"; // map frame 기준
    node1.header.stamp = ros::Time::now();
    node1.type = visualization_msgs::Marker::SPHERE;
    node1.id = 0;
    node1.action = visualization_msgs::Marker::ADD;
    node1.pose.orientation.w = 1.0;
    node1.pose.position.x = index.position.x; //노드의 x 좌표
    node1.pose.position.y = index.position.y; //노드의 y 좌표 // Points are green
    node1.color.g = 0.5;
    node1.color.a = 1.0;
    node1.scale.x = 0.2;
    node1.scale.y = 0.2;
    node_arr.markers.push_back(node1);

    marker_lfd.publish(node_arr);
}

double pure_pursuit::steering_angle(geometry_msgs::PoseStamped pose, nav_msgs::Path tracking_path, double vehicle_yaw, double velocity)
{
    geometry_msgs::Pose index;

    is_look_foward_point = false;

    double back_x = pose.pose.position.x - L*cos(vehicle_yaw);
    double back_y = pose.pose.position.y - L*sin(vehicle_yaw);

    double dis = 0;

    double lfd = 1;
    double max_lfd = this->max_lfd;
    double min_lfd = this->min_lfd;
    double rotated_x = 0;
    double rotated_y = 0;

    lfd = velocity/10.0;

    if(lfd < min_lfd)
    {
        lfd = min_lfd;
    }
    else if(lfd > max_lfd)
    {
        lfd = max_lfd;
    }

    for(int i = 0; i<tracking_path.poses.size(); i++)
    {
        double dx = tracking_path.poses.at(i).pose.position.x - back_x;
        double dy = tracking_path.poses.at(i).pose.position.y - back_y;

        rotated_x = cos(vehicle_yaw)*dx + sin(vehicle_yaw)*dy;
        rotated_y = sin(vehicle_yaw)*dx - cos(vehicle_yaw)*dy;

        if(rotated_x > 0)
        {
            dis = sqrt(pow(rotated_x,2) + pow(rotated_y,2));
            if(dis>=lfd)
            {
                index.position.x = tracking_path.poses.at(i).pose.position.x;
                index.position.y = tracking_path.poses.at(i).pose.position.y;
                is_look_foward_point = true;
                break;
            }
        }
    }

    double theta = atan2(rotated_y,rotated_x);
    double steering = 0;
    if(is_look_foward_point == true)
    {
        double eta = atan2((2*VL*sin(theta)),lfd);
        steering = PID(-eta,10);
        lfd_visualiztion(index);
    }
    else
    {
        ROS_INFO("no found forwad point");
    }

    return steering;
}
