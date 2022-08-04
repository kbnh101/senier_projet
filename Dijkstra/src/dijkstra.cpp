#include "dijkstra.h"

dijkstra::dijkstra()
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    pnh.param("x_offset",x_offset,851);
    pnh.param("y_offset",y_offset,200);
    pnh.param("resolution",resolution,153.0);
    //subscribe
    map_sub = nh.subscribe("map",10,&dijkstra::MapCallback,this);
    startPoint_sub = nh.subscribe("/Start", 10, &dijkstra::StartPointCallback,this);
    targetPoint_sub = nh.subscribe("/Receive", 10, &dijkstra::TargetPointtCallback,this);
    pose_sub = nh.subscribe("/odom",10,&dijkstra::posecallback,this);
    //publish
    state_pub = nh.advertise<std_msgs::String>("/state",10);
    path_pub = nh.advertise<nav_msgs::Path>("path",10);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("marker_node", 1);
    speed_pub = nh.advertise<std_msgs::Int32>("/speed",10);
}

void dijkstra::posecallback(const nav_msgs::Odometry& msg)
{
    pose = msg.pose;
    tt = msg.pose.pose;
    pose_point = cv::Point2d(msg.pose.pose.position.x, msg.pose.pose.position.y);
}

void dijkstra::MapCallback(const nav_msgs::OccupancyGrid& msg)
{
    map = msg;
    width = 1920;
    height = 1080;
}

void dijkstra::StartPointCallback(const std_msgs::String& msg)
{
    string star;
    star = msg.data;
    int num = atoi(star.c_str()) - 1;
    start_point = cv::Point2d(node.at(num).x/resolution, (-node.at(num).y + height)/resolution);
    Done = false;
}

void dijkstra::TargetPointtCallback(const std_msgs::String& msg)
{
    string dst;
    dst = msg.data;
    int num = atoi(dst.c_str()) - 1;
    dst_point = cv::Point2d(node.at(num).x/resolution, (-node.at(num).y + height)/resolution);
    flag = true;
}

void dijkstra::node_parse(istream& file)
{
    Node info;
    if (file.fail())
    {
      cout << "해당 경로에 위치하는 파일이 존재하지 않습니다." << endl;
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

void dijkstra::edge_parse(istream& file)
{
    Edge info;
    if (file.fail())
    {
       std::cout << "해당 경로에 위치하는 파일이 존재하지 않습니다." << endl;
    }
    while (file.good())
    {
       vector<string> row = csv_read_row(file, ',');
       istringstream x(row[1]),y(row[2]);
       string wp_x,wp_y;
       vector<string> list_x, list_y;
       int temp_point_x, temp_point_y;

       info.link_id = atoi(row[0].c_str());
       while(getline(x,wp_x,' '))
       {
           list_x.push_back(wp_x);
       }
       while(getline(y,wp_y,' '))
       {
           list_y.push_back(wp_y);
       }
       for(int i = 0; i < list_x.size(); i++)
       {
           temp_point_x = atoi(list_x.at(i).c_str()) - x_offset;
           temp_point_y = atoi(list_y.at(i).c_str()) + y_offset;
           info.way_point_x.push_back(temp_point_x);
           info.way_point_y.push_back(temp_point_y);
       }
       list_x.clear();
       list_y.clear();

       info.from_node = atoi(row[3].c_str());
       info.to_node = atoi(row[4].c_str());
       info.cost = 1;
       info.speed = atoi(row[5].c_str());
       info.state = row[6].c_str();
       for(int i = 0; i < list_x.size(); i++)
       {
           speedv.push_back(info.speed);
       }
       edge.push_back(info);
       info.way_point_x.clear();
       info.way_point_y.clear();
    }
    for(int i = 0; i < node.size(); i++)
    {
        for(int j = 0; j < edge.size(); j++)
        {
            if(node.at(i).id == edge.at(j).to_node)
            {
                edge.at(j).way_point_x.push_back(node.at(i).x);
                edge.at(j).way_point_y.push_back(node.at(i).y);
            }
        }
    }
}

void dijkstra::make_graph()
{
    Graph temp;
    for(int i = 0; i<node.size(); i++)
    {
        for(int j = 0; j<edge.size(); j++)
        {
            if(node.at(i).id == edge.at(j).from_node)
            {
                temp.node_id = node.at(i).id;
                temp.E.push_back(edge.at(j));
            }
        }
        G.push_back(temp);
        temp.E.clear();
    }
}

void dijkstra::find_path(cv::Point2d start_point, cv::Point2d dst_point)
{
    double short_dist = 10;
    double short_dist1 = 10;
    double Wend_num, Wstart_num, start_num, end_num = 0;
    int start_node, end_node = 0;
    vector<pair<int, int> > adj[node.size() + 1];

    for(int i = 0; i<edge.size(); i++)
    {
        for(int j = 0; j<edge.at(i).way_point_x.size(); j++)
        {
            double dis = (start_point.x - edge.at(i).way_point_x.at(j)/resolution)*(start_point.x - edge.at(i).way_point_x.at(j)/resolution) + (start_point.y - (-edge.at(i).way_point_y.at(j)+height)/resolution)*(start_point.y - (-edge.at(i).way_point_y.at(j)+height)/resolution);
            double dist = std::sqrt(dis);
            if(dist<short_dist)
            {
                short_dist = dist;
                start_num = i;
                Wstart_num = j;
            }
        }
    }
    start_node = edge.at(start_num).to_node;
    near_start = edge.at(start_num).link_id - 1;
    exstart = Wstart_num;

    for(int i = 0; i<edge.size(); i++)
    {
        for(int j = 0; j<edge.at(i).way_point_x.size(); j++)
        {
            double dis = (dst_point.x - edge.at(i).way_point_x.at(j)/resolution)*(dst_point.x - edge.at(i).way_point_x.at(j)/resolution) + (dst_point.y - (-edge.at(i).way_point_y.at(j)+height)/resolution)*(dst_point.y - (-edge.at(i).way_point_y.at(j)+height)/resolution);
            double dist = std::sqrt(dis);
            if(dist<short_dist1)
            {
                short_dist1 = dist;
                end_num = i;
                Wend_num = j;
            }
        }
    }
    end_node = edge.at(end_num).from_node;
    near_end = edge.at(end_num).link_id - 1;
    exend = Wend_num;

    for (int i = 0; i < edge.size(); i++)
    {
        adj[edge.at(i).from_node].push_back(make_pair(edge.at(i).to_node, edge.at(i).cost));
    }

    auto from = dijkstra1(start_node, node.size() + 1, adj);
    if(start_node != end_node)
    {
        print_path(start_node, end_node, from);
    }
    shortest_path.push_back(end_node);
    std::cout<<std::endl<<"Done"<<std::endl;
}

void dijkstra::print_path(int s, int e, vector<int>* from) {
    // 위의 trace_path를 호출하여 최단 경로를 출력한후,
    trace_path(s, e, from);

    // 목적지의 정점 번호도 출력한다.
    std::cout << e;
}

vector<int>* dijkstra::dijkstra1(int start, int V, vector<pair<int,int> > adj[]) {
    int INF = INT32_MAX;

    vector<bool> found(V, false);//최단거리가 발견되면 true
    vector<int> dist(V, INF);    // 전부 INF로 초기화
    priority_queue<pair<int, int> > pq;
    vector<int>* from = new vector<int>(V);//최단거리가 업데이트 될때 바로 이전에 방문하게 되는 정점

    dist[start] = 0;
    //adj[edge.at(i).to_node].push_back(make_pair(edge.at(i).from_node, edge.at(i).cost));
    pq.push(make_pair(0, start));    // 시작 정점 방문

    while (!pq.empty())
    {
        int cost = -pq.top().first;    // 방문한 정점의 dist 값
        int cur = pq.top().second;    // 현재 방문한 정점
        pq.pop();

        found[cur] = true;

        for (int i = 0; i < adj[cur].size(); i++) // 현재 방문한 정점의 주변 정점 모두 조사
        {
            int next = adj[cur][i].first;    // 조사할 다음 정점
            int nCost = cost + adj[cur][i].second;    // 현재 방문한 정점을 거쳐서 다음 정점을 갈때의 비용
            if (nCost < dist[next] )     // 기존 비용보다 현재 방문한 정점을 거친 비용이 더 싸다면
            {
                dist[next] = nCost;    // 갱신
                (*from)[next] = cur;

                pq.push(make_pair(-nCost, next));    // pq에 저장
            }
        }
    }

    return from;
}

void dijkstra::trace_path(int s, int e, vector<int>* from) {
    // 기저 조건 : 시작점과 목적지가 같은 경우
    if ((*from)[e] == s) {
        cout << s << " -> ";
        shortest_path.push_back((*from)[e]);
        return;
    }
    // 재귀호출을 통해 정점 e전의 정점에 대한 경로를 출력한다..
    trace_path(s, (*from)[e], from);

    // 최단경로에서 정점 e 바로 이전의 정점를 화면에 출력한다.
    shortest_path.push_back((*from)[e]);
    cout << (*from)[e] << " - > ";
}

void dijkstra::draw_path()
{
    if(flag == true)
    {
        vector<int> temp_path1, temp_path2, final_path, for_money;
        int first_near, first_start, second_end, end_point;
        if(Done == false)
        {
            find_path(pose_point, start_point);
            for(int i = 0; i < shortest_path.size(); i++)
                temp_path1.push_back(shortest_path.at(i));
            shortest_path.clear();
            first_near = near_start;
            first_start = exstart;

            find_path(start_point, dst_point);
            for(int i = 0; i < shortest_path.size(); i++)
                temp_path2.push_back(shortest_path.at(i));
            shortest_path.clear();
            second_end = near_end;
            end_point = exend;

            for(int i = 0; i < temp_path1.size(); i++)
                final_path.push_back(temp_path1.at(i));
            for(int i = 0; i < temp_path2.size(); i++)
            {
                final_path.push_back(temp_path2.at(i));
                for_money.push_back(temp_path2.at(i));
            }

            temp_path1.clear();
            temp_path2.clear();
        }

        visualization_msgs::MarkerArray node_arr;
        for (size_t i = 0; i < node.size(); i++)
        {
            visualization_msgs::Marker node1;
            node1.header.frame_id = "/map"; // map frame 기준
            node1.header.stamp = ros::Time::now();
            node1.type = visualization_msgs::Marker::SPHERE;
            node1.id = i;
            node1.action = visualization_msgs::Marker::ADD;
            node1.pose.orientation.w = 1.0;
            node1.pose.position.x = node.at(i).x/resolution; //노드의 x 좌표
            node1.pose.position.y = (-node.at(i).y + height)/resolution; //노드의 y 좌표 // Points are green
            node1.color.g = 1.0;
            node1.color.a = 1.0;
            node1.scale.x = 50.0/resolution;
            node1.scale.y = 50.0/resolution;
            node_arr.markers.push_back(node1);
        }
        visualization_msgs::Marker node1;
        node1.header.frame_id = "/map"; // map frame 기준
        node1.header.stamp = ros::Time::now();
        node1.type = visualization_msgs::Marker::SPHERE;
        node1.id = node.size();
        node1.action = visualization_msgs::Marker::ADD;
        node1.pose.orientation.w = 1.0;
        node1.pose.position.x = start_point.x; //노드의 x 좌표
        node1.pose.position.y = start_point.y; //노드의 y 좌표 // Points are green
        node1.color.r = 1.0;
        node1.color.a = 1.0;
        node1.scale.x = 50.0/resolution;
        node1.scale.y = 50.0/resolution;
        node_arr.markers.push_back(node1);
        visualization_msgs::Marker node2;
        node2.header.frame_id = "/map"; // map frame 기준
        node2.header.stamp = ros::Time::now();
        node2.type = visualization_msgs::Marker::SPHERE;
        node2.id = node.size() + 1;
        node2.action = visualization_msgs::Marker::ADD;
        node2.pose.orientation.w = 1.0;
        node2.pose.position.x = dst_point.x; //노드의 x 좌표
        node2.pose.position.y = dst_point.y; //노드의 y 좌표 // Points are green
        node2.color.r = 1.0;
        node2.color.a = 1.0;
        node2.scale.x = 50.0/resolution;
        node2.scale.y = 50.0/resolution;
        node_arr.markers.push_back(node2);
        marker_pub.publish(node_arr);

        nav_msgs::Path rpath;
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.header.stamp = ros::Time::now();
        temp_pose.header.frame_id = "map";
        rpath.header.stamp = ros::Time::now();
        rpath.header.frame_id = "map";
        rpath.poses.clear();
        for(int j = 1; j<final_path.size(); j++)
        {
            for(int i = 0; i<edge.size(); i++)
            {
                int to = final_path.at(j);
                int from = final_path.at(j - 1);

                if(edge.at(i).to_node == to && edge.at(i).from_node == from)
                {
                    path_info cpPath;
                    for(int k = 0; k<edge.at(i).way_point_x.size();k++)
                    {
                        double x = edge.at(i).way_point_x.at(k)/resolution;
                        double y = (-edge.at(i).way_point_y.at(k) + height)/resolution;
                        double vel;
                        temp_pose.pose.position.x = x;
                        temp_pose.pose.position.y = y;
                        temp_pose.pose.position.z = 0;
                        cpPath.x = x;
                        cpPath.y = y;
                        cpPath.vel = vel;
                        cpPath.state = edge.at(i).state;
                        rpath.poses.push_back(temp_pose);
                        pathinfo.push_back(cpPath);
                    }
                }
            }
        }
        int dist_money = 0;
        for(int j = 1; j<for_money.size(); j++)//money_check
        {
            for(int i = 0; i<edge.size(); i++)
            {
                int to = for_money.at(j);
                int from = for_money.at(j - 1);

                if(edge.at(i).to_node == to && edge.at(i).from_node == from)
                {
                    for(int k = 0; k<edge.at(i).way_point_x.size();k++)
                    {
                        double x = edge.at(i).way_point_x.at(k)/resolution;
                        double y = (-edge.at(i).way_point_y.at(k) + height)/resolution;
                        temp_pose.pose.position.x = x;
                        temp_pose.pose.position.y = y;
                        temp_pose.pose.position.z = 0;
                        double temp_dist = sqrt(x*x + y*y);
                        dist_money += temp_dist;
                    }
                }
            }
        }
        cout<<"Driving Distance = "<<dist_money<<endl;
        for_money.clear();

        for(int i = edge.at(first_near).way_point_x.size() - 1; i > first_start; i--)
        {
            double x = edge.at(first_near).way_point_x.at(i)/resolution ;
            double y = (-edge.at(first_near).way_point_y.at(i) + height)/resolution;
            temp_pose.pose.position.x = x;
            temp_pose.pose.position.y = y;
            temp_pose.pose.position.z = 0;
            rpath.poses.insert(rpath.poses.begin(),temp_pose);
        }
        for(int i = 0; i < end_point; i++)
        {
            double x = edge.at(second_end).way_point_x.at(i)/resolution;
            double y = (-edge.at(second_end).way_point_y.at(i) + height)/resolution;

            temp_pose.pose.position.x = x;
            temp_pose.pose.position.y = y;
            temp_pose.pose.position.z = 0;
            rpath.poses.push_back(temp_pose);
        }
        if(Done == false)
        {
            double x = rpath.poses.at(1).pose.position.x - rpath.poses.at(0).pose.position.x;
            double y = rpath.poses.at(1).pose.position.y - rpath.poses.at(0).pose.position.y;

            double cyaw = atan2(y,x);

            std::cout<<cyaw<<std::endl;
            path_pub.publish(rpath);
            rpath.poses.clear();
            final_path.clear();
        }
        Done = true;
        flag = false;
    }
    if(pathinfo.size() != 0)
    {
        double index = 0;
        double min_dist = 20;

        for(int i = 0; i < pathinfo.size(); i++)
        {
            double x = tt.position.x - pathinfo.at(i).x;
            double y = tt.position.y - pathinfo.at(i).y;
            double dist = sqrt((x * x)+(y * y));
            if(min_dist > dist)
            {
                index = i;
                min_dist = dist;
            }
        }

        speed.data = pathinfo.at(index).vel;

        if(speed != speed)
        {
            ROS_INFO_STREAM("speed = "<<speed);
        }
        std::stringstream state_stream;

        state_stream <<pathinfo.at(index).state;
        state.data = state_stream.str();
        speed_pub.publish(speed);
        state_pub.publish(state);
    }
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "dijkstra");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    dijkstra path;

    path.flag = false;
    path.Done = false;

    ifstream file_edge("/home/a/hand_made_ws/src/hand_made/Dijkstra/path/edge.csv");
    ifstream file_node("/home/a/hand_made_ws/src/hand_made/Dijkstra/path/node.csv");

    path.node_parse(file_node);
    path.edge_parse(file_edge);

    file_edge.close();
    file_node.close();

    ros::Rate  loop_rate(10);

    while(ros::ok())
    {
        path.draw_path();        
        loop_rate.sleep();
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}
