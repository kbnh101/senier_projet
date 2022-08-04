#include "local_path.h"

local_path::local_path()
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    pnh.param("num",num,10);
    pnh.param("x_offset",x_offset,851);
    pnh.param("y_offset",y_offset,200);

    //publish
    cmd_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd",10);
    path_pub = nh.advertise<nav_msgs::Path>("tracking_path",10);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("current_pose",10);
    cmd_pub_ = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/drive",10);
    marker_vel = nh.advertise<visualization_msgs::MarkerArray>("/marker_vel",10);
    //subscibe
    obstacle_sub = nh.subscribe("center_points",10,&local_path::obstacle_callback,this);
    pose_sub = nh.subscribe("/slam_out_pose",10,&local_path::posecallback,this);
    sim_pose = nh.subscribe("/odom",10,&local_path::sim_pose_callback,this);
    path_sub = nh.subscribe("/path",10,&local_path::pathcallback,this);
    trajectory = nh.subscribe("/trajectory",10,&local_path::trajectorycallback,this);
    speed_sub = nh.subscribe("/speed",10,&local_path::speed_callback,this);
    state_sub = nh.subscribe("/state",10,&local_path::state_callback,this);
    initial_pose = nh.subscribe("/Start",10,&local_path::initial_pose_callback,this);
    end_pose_sub = nh.subscribe("/Receive", 10, &local_path::end_pose_callback,this);
    GO_sub = nh.subscribe("/State",10,&local_path::go_sub_callback,this);

    height = 1080;
    resolution = 153;
    stop = false;
    num1 = 20;
    GO = false;
}

void local_path::go_sub_callback(const std_msgs::String &msg)
{
    GO = true;
}

void local_path::end_pose_callback(const std_msgs::String &msg)
{
    string star;
    star = msg.data;
    int num = atoi(star.c_str()) - 1;
    end_pose.pose.position.x = node.at(num).x/resolution;
    end_pose.pose.position.y = (-node.at(num).y + height)/resolution;
}

void local_path::initial_pose_callback(const std_msgs::String &msg)
{
    string star;
    star = msg.data;
    int num = atoi(star.c_str()) - 1;
    start_point.pose.pose.position.x = node.at(num).x/resolution;
    start_point.pose.pose.position.y = (-node.at(num).y + height)/resolution;
}

void local_path::state_callback(const std_msgs::String &msg)
{
    state = msg;
}

void local_path::obstacle_callback(const sensor_msgs::PointCloud &msg)
{
    obstacle = msg;
}

void local_path::trajectorycallback(const nav_msgs::Path& msg)
{
    trajectory_path = msg;
}

void local_path::pathcallback(const nav_msgs::Path& msg)
{
    path = msg;
    flag = true;
}

void local_path::sim_pose_callback(const nav_msgs::Odometry &msg)
{
    odometry = msg;
    pose.pose = msg.pose.pose;
    pose.header = msg.header;
    tf::Quaternion car_orientation(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    tf::Matrix3x3 matrix(car_orientation);
    matrix.getRPY(vehicle_roll, vehicle_pitch, vehicle_yaw);

    temp_pose.header.stamp = ros::Time::now();
    temp_pose.header.frame_id = "odom";
    temp_pose.pose.position.x = (pose.pose.position.x);
    temp_pose.pose.position.y = (pose.pose.position.y);
    temp_pose.pose.position.z = 0;
    temp_pose.pose.orientation.x = pose.pose.orientation.x;
    temp_pose.pose.orientation.y = pose.pose.orientation.y;
    temp_pose.pose.orientation.z = pose.pose.orientation.z;
    temp_pose.pose.orientation.w = pose.pose.orientation.w;

    pose_pub.publish(temp_pose);
}

void local_path::posecallback(const geometry_msgs::PoseStamped& msg)
{
    pose = msg;
    tf::Quaternion car_orientation(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    tf::Matrix3x3 matrix(car_orientation);
    matrix.getRPY(vehicle_roll, vehicle_pitch, vehicle_yaw);

    temp_pose.header.stamp = ros::Time::now();
    temp_pose.header.frame_id = "map";
    temp_pose.pose.position.x = (msg.pose.position.x);
    temp_pose.pose.position.y = (msg.pose.position.y);
    temp_pose.pose.position.z = 0;
    temp_pose.pose.orientation.x = msg.pose.orientation.x;
    temp_pose.pose.orientation.y = msg.pose.orientation.y;
    temp_pose.pose.orientation.z = msg.pose.orientation.z;
    temp_pose.pose.orientation.w = msg.pose.orientation.w;

    //std::cout<<vehicle_yaw<<std::endl;

    pose_pub.publish(temp_pose);
}

void local_path::speed_callback(const std_msgs::Int32 &msg)
{
    speed = msg.data;
}

void local_path::node_parse(istream &file)
{
    Node info;
    if (file.fail())
    {
      std::cout << "해당 경로에 위치하는 파일이 존재하지 않습니다." << std::endl;
    }
    while (file.good())
    {
       vector<string> row = csv_read_row(file, ',');
       info.id = atoi(row[0].c_str());
       double x = atoi(row[1].c_str());
       double y = atoi(row[2].c_str());
       info.x = x - x_offset;
       info.y = y + y_offset;

       node.push_back(info);
    }
}

void local_path::path_tracking(nav_msgs::Path global_path)
{
    if(flag == true)
    {
        double least_dist = 10;
        int temp_num;

        for(int i = 0; i<global_path.poses.size(); i++)
        {
            double dx = pose.pose.position.x - global_path.poses.at(i).pose.position.x;
            double dy = pose.pose.position.y - global_path.poses.at(i).pose.position.y;

            double dist = sqrt(dx*dx + dy*dy);
            if(dist < least_dist)
            {
                least_dist = dist;
                temp_num = i;
            }
        }

        if(prev_temp_num < 0)
        {
            temp_num = temp_num;
        }
        else if(temp_num >= prev_temp_num + 7 || temp_num <= prev_temp_num - 7)
        {
            temp_num = prev_temp_num;
        }

        std::cout<<"temp_num : "<<temp_num <<std::endl;
        std::cout<<"prev_temp_num : "<<prev_temp_num <<std::endl;
        tracking_path.header.stamp = ros::Time::now();
        tracking_path.header.frame_id = "map";
        tracking_path.poses.clear();

        geometry_msgs::PoseStamped temp_pose;
        temp_pose.header.stamp = ros::Time::now();
        temp_pose.header.frame_id = "map";

        int local_path_size = temp_num + num1;

        if(local_path_size >= global_path.poses.size()-1)
        {
            num1 -= 1;
        }

        if(temp_num + num1 <= path.poses.size())
        {
            for(int i = temp_num; i< local_path_size; i++)
            {
                if(state.data == "no")
                {
                    if(get_passenger(pose) < 1.0 && GO == false)
                    {
                        temp_pose.pose.position.x = global_path.poses.at(i).pose.position.x + 0.2*sin(vehicle_yaw);
                        temp_pose.pose.position.y = global_path.poses.at(i).pose.position.y - 0.2*cos(vehicle_yaw);
                        temp_pose.pose.position.z = 0;
                        tracking_path.poses.push_back(temp_pose);
                    }
                    else if(GO == true && get_off_passenger(pose) < 1.25)
                    {
                        temp_pose.pose.position.x = global_path.poses.at(i).pose.position.x + 0.2*sin(vehicle_yaw);
                        temp_pose.pose.position.y = global_path.poses.at(i).pose.position.y - 0.2*cos(vehicle_yaw);
                        temp_pose.pose.position.z = 0;
                        tracking_path.poses.push_back(temp_pose);
                    }
                    else
                    {
                        temp_pose.pose.position.x = global_path.poses.at(i).pose.position.x;
                        temp_pose.pose.position.y = global_path.poses.at(i).pose.position.y;
                        temp_pose.pose.position.z = 0;
                        tracking_path.poses.push_back(temp_pose);
                    }
                }
            }
        }
        else
        {
            flag = false;
        }
        path_pub.publish(tracking_path);
        prev_temp_num = temp_num;
    }
    else
    {
        ROS_INFO("DONE");
    }
}

double local_path::velocity()
{
   double vel = 0.5;

    return vel;
}

double local_path::obstacle_distance(sensor_msgs::PointCloud obstacle)
{
    if(obstacle.points.size() > 0)
    {
        double min_dis = 20;
        int path_index = 0;
        int lidar_index = 0;

        for(int i = 0; i<obstacle.points.size(); i++)
        {
            for(int j = 0; j<tracking_path.poses.size(); j++)
            {
                double tx = obstacle.points.at(i).x;
                double ty = obstacle.points.at(i).y;
                double rotated_x = cos(vehicle_yaw)*tx - sin(vehicle_yaw)*ty;
                double rotated_y = sin(vehicle_yaw)*tx + cos(vehicle_yaw)*ty;

                double x = rotated_x + pose.pose.position.x;
                double y = rotated_y + pose.pose.position.y;

                double dx = x - tracking_path.poses.at(j).pose.position.x;
                double dy = y - tracking_path.poses.at(j).pose.position.y;
                double dis = sqrt(dx*dx + dy*dy);

                if(min_dis > dis)
                {
                    min_dis = dis;
                    path_index = j;
                    lidar_index = i;
                }
            }
        }

        return min_dis;
    }
    return 10;
}

double local_path::get_passenger(geometry_msgs::PoseStamped pose)
{
    double dx = start_point.pose.pose.position.x - pose.pose.position.x;
    double dy = start_point.pose.pose.position.y - pose.pose.position.y;

    double dist = sqrt(dx*dx + dy*dy);

    return dist;
}

double local_path::get_off_passenger(geometry_msgs::PoseStamped pose)
{
    double dx = end_pose.pose.position.x - pose.pose.position.x;
    double dy = end_pose.pose.position.y - pose.pose.position.y;

    double dist = sqrt(dx*dx + dy*dy);

    return dist;
}

void local_path::process()
{
    ros::Duration wait(0.5);

    if(flag == true)
    {
        path_tracking(path);

        cmd_vel.drive.steering_angle = steering_angle(pose,tracking_path,vehicle_yaw,velocity());
        cmd_vel_.drive.steering_angle = steering_angle(pose,tracking_path,vehicle_yaw,velocity());;
        if(is_look_foward_point == true)
        {
            cmd_vel.drive.speed = velocity();
            cmd_vel_.drive.speed = velocity();
            if(get_passenger(pose) < 0.35 && GO == false)
            {
                cmd_vel.drive.speed = 0;
                cmd_vel_.drive.speed = 0;
            }
            else if(GO == true)
            {
                double least_dist = 10000.0;
                int temp_num = 0;

                for(int i = 0; i < path.poses.size(); i++)
                {
                    double dx = start_point.pose.pose.position.x - path.poses.at(i).pose.position.x;
                    double dy = start_point.pose.pose.position.y - path.poses.at(i).pose.position.y;

                    double dist = sqrt(dx*dx + dy*dy);
                    if(dist < least_dist)
                    {
                        least_dist = dist;
                        temp_num = i;
                    }
                }
                for(int i = 0; i < temp_num; i++)
                {
                    path.poses.at(i).pose.position.x = 100;
                    path.poses.at(i).pose.position.y = 100;
                }

                start_point.pose.pose.position.x = 100;
                start_point.pose.pose.position.y = 100;
                cmd_vel.drive.speed = velocity();
                cmd_vel_.drive.speed = velocity();
            }
        }
        else
        {
            num1 = 20;
            cmd_vel.drive.speed = 0;
            cmd_vel_.drive.speed = 0;
            GO = false;
            flag = false;
        }
    }
    else if(flag==false)
    {
        cmd_vel.drive.steering_angle = 0;
        cmd_vel.drive.speed = 0;
        cmd_vel_.drive.speed = 0;
        cmd_vel_.drive.steering_angle = 0;
        prev_temp_num = 0;
    }
    cmd_pub.publish(cmd_vel);
    cmd_pub_.publish(cmd_vel_);
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "local_path");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    local_path path;
    path.flag = false;
    ifstream file_node("/home/a/hand_made_ws/src/hand_made/Dijkstra/path/node.csv");
    path.node_parse(file_node);

    ros::Rate loop_rate(path.rate);
    while(ros::ok())
    {
        path.process();
        loop_rate.sleep();
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}
