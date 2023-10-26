#include "pure_pursuit_controller/pure_pursuit_node.hpp"

namespace pure_pursuit_controller
{
void PurePursuitNode::OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    
    odom_callback_flag = true;
    
    tf::pointMsgToTF(odom_msg->pose.pose.position, odom_pos);
    odom_yaw = tf::getYaw(odom_msg->pose.pose.orientation);
    odom_linear_v = odom_msg->twist.twist.linear.x;
    odom_angular_v = odom_msg->twist.twist.angular.z;
    ROS_INFO("Position:(%f, %f); Yaw: %f", odom_pos.x(), odom_pos.y(), odom_yaw);
}

std::vector<path_type> PurePursuitNode::
                CreatePath(visualization_msgs::Marker& path) {
    
    std::vector<path_type> path_vector;

    // path data info
    path.type = visualization_msgs::Marker::LINE_STRIP;
    path.header.frame_id = "odom";
    path.header.stamp = ros::Time::now();
    path.ns = "odom";
    path.id = 0;
    path.action = visualization_msgs::Marker::ADD;
    path.lifetime = ros::Duration();
    path.color.b = 1.0;
    path.color.a = 1.0;
    path.scale.x = 0.02;
    path.pose.orientation.w = 1.0;

    // create a circle path 
    int circle_slice = 50;
    double circle_radius = 4;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> offset(-0, 0);
    for (int i = 0; i <= circle_slice; i++) {
        geometry_msgs::Point path_point;
        float angle = i * 2 * M_PI / circle_slice;
        path_point.x = circle_radius * cos(angle) + offset(gen);
        path_point.y = circle_radius * sin(angle) + offset(gen);
        path_point.z = 0;
        path.points.push_back(path_point);
        path_vector.push_back(std::make_pair(std::make_pair
                (path_point.x, path_point.y), angle + M_PI / 2));
    }

    return path_vector;
}

void PurePursuitNode::PurePursuitRun() {
    ROS_INFO("Pure-pursuit-control algorithm is running!");

    odom_callback_flag = false;

    cmd_vel_pub = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    marker_pub = n_.advertise<visualization_msgs::Marker>
            ("visualization_marker", 1);
    odom_sub = n_.subscribe<nav_msgs::Odometry>("odom", 10, 
            std::bind(&PurePursuitNode::OdomCallback, this, std::placeholders::_1));

    // ros spins 10 frames per seconds
    ros::Rate loop_rate(10);

    // control turtlebot3 by linear and angular speeds
    geometry_msgs::Twist twist_msg;

    path_type init_pose = std::make_pair(std::make_pair(0, 0), 0);
    path_type current_pose = init_pose;

    int last_index;
    double ld = k * speed + lf;

    while (ros::ok()) {

        // keep publishing path 
        visualization_msgs::Marker path;
        std::vector<path_type> path_vector = CreatePath(path);  
        marker_pub.publish(path);

        current_pose.first.first = odom_pos.x();
        current_pose.first.second = odom_pos.y();
        current_pose.second = odom_yaw;
        // ROS_INFO_STREAM("current_pose:" << current_pose.first.first << " "
        //         << current_pose.first.second << " " << current_pose.second);

        path_type ahead_pose = current_pose;
        ahead_pose.first.first = current_pose.first.first + 
                ld * current_pose.second;
        ahead_pose.first.second = current_pose.first.second + 
                ld * current_pose.second;
        
        // ROS_INFO_STREAM("ahead_pose:" << ahead_pose.first.first << " "
        //         << ahead_pose.first.second << " " << ahead_pose.second);

        int ahead_index = math_tool_.GetMinDisIndex(ahead_pose, path_vector);
        double ld_new;
        while (1) {
            double dis = math_tool_.GetDistance(current_pose, path_vector[ahead_index]);  
            if (dis >= ld) {
                ld_new = dis;
                break;
            }         
            ahead_index += 1;
            if (ahead_index >= path_vector.size()) {
                ahead_index = 0;
            }
        }
        ROS_INFO_STREAM("ahead_index:" << ahead_index);
        path_type ahead_path_point = path_vector[ahead_index];
        // ROS_INFO_STREAM("ahead_path_point:" << ahead_path_point.first.first 
        //         << " " << ahead_path_point.first.second);

        // delta = atan2(2Lsin(alpha) / ld)(L = 1)
        double alpha = std::atan2
                (ahead_path_point.first.second - current_pose.first.second, 
                ahead_path_point.first.first - current_pose.first.first) - 
                current_pose.second;
        // ROS_INFO_STREAM("alpha:" << alpha);
        double delta = std::atan2(2 * 0.5 * sin(alpha), ld_new);        
        // ROS_INFO_STREAM("delta:" << delta);

        twist_msg.linear.x = speed;
        twist_msg.angular.z = delta;
        if (odom_callback_flag) {
            cmd_vel_pub.publish(twist_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Stanley-control algorithm has calculated the result!");

}
}