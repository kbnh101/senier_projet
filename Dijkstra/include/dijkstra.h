#ifndef DIJKSTRA_H
#define DIJKSTRA_H
#include<ros/ros.h>
#include<nav_msgs/Path.h>
#include<nav_msgs/OccupancyGrid.h>
#include<nav_msgs/GetMap.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<visualization_msgs/MarkerArray.h>
#include<iostream>
#include<string.h>
#include<opencv2/opencv.hpp>
#include<math.h>
#include<cmath>
#include"csv.h"
#include<queue>
#include<bitset>
#include<vector>
#include<std_msgs/Int32.h>
#include<std_msgs/String.h>
#include<nav_msgs/Odometry.h>

struct Node
{
    int id;
    int x;
    int y;
};

struct Edge
{
    int link_id;
    vector<double> way_point_x;
    vector<double> way_point_y;
    int from_node;
    int to_node;
    int cost;
    double speed;
    string state;
};

struct Graph
{
    int node_id;
    vector<Edge> E;
};

struct path_info
{
    double x;
    double y;
    double vel;
    string state;
};

class dijkstra
{
public:
    dijkstra();
    cv::Point2d start_point, dst_point, pose_point;
    vector<double> speedv;
    vector<path_info> pathinfo;
    vector<Node> node;
    vector<Edge> edge;
    vector<Graph> G;
    int width, height;
    int exstart, exend;
    int near_start, near_end;
    int x_offset, y_offset;
    bool flag;
    bool Done;
    double resolution;
    queue<int> Queue;
    vector<Edge> spath;
    vector<int> shortest_path;

    vector<int>* dijkstra1(int start, int V, vector<pair<int,int> > adj[]);

    nav_msgs::OccupancyGrid map;

    geometry_msgs::Pose tt;
    geometry_msgs::PoseWithCovariance pose;

    std_msgs::Int32 speed;
    std_msgs::String state;

    //Function
    void node_parse(istream& file);
    void edge_parse(istream& file);
    void draw_path();
    void make_graph();
    void find_path(cv::Point2d start, cv::Point2d dst);
    void trace_path(int s, int e, vector<int>* from);
    void print_path(int s, int e, vector<int>* from);

    //Callback
    void MapCallback(const nav_msgs::OccupancyGrid& msg);
    void TargetPointtCallback(const std_msgs::String& msg);
    void StartPointCallback(const std_msgs::String& msg);
    void posecallback(const nav_msgs::Odometry& msg);

    //Publish
    ros::Publisher path_pub;
    ros::Publisher marker_pub;
    ros::Publisher speed_pub;
    ros::Publisher state_pub;

    //Subscribe
    ros::Subscriber map_sub;
    ros::Subscriber startPoint_sub;
    ros::Subscriber targetPoint_sub;
    ros::Subscriber pose_sub;
};

#endif // DIJKSTRA_H
