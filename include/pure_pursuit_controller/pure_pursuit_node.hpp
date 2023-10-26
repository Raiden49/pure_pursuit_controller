#ifndef PURE_PURSUIT_HPP_
#define PURE_PURSUIT_HPP_

#include <sstream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include "pure_pursuit_controller/math_tool.hpp"

namespace pure_pursuit_controller
{
class PurePursuitNode {
    public:
        // ld = k * v + lf;
        double k = 1;
        double speed = 0.5;
        double lf = 0.1;
        /**
         * @brief Construction of PurePursuitNode class
         * 
         * @param n node handle
        */
        PurePursuitNode(ros::NodeHandle& n) :n_(n) {};
        /**
         * @brief Callback function for subscribing odom pose
         * 
         * @param odom_msg odom point
        */
        void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
        /**
         * @brief Create a path for simulation
         * 
         * @param path path data
         * @return path vector
        */
        std::vector<path_type> CreatePath(visualization_msgs::Marker& path);
        /**
         * @brief Core function
        */
        void PurePursuitRun();
        ~PurePursuitNode() {};

    public:
        tf::Point odom_pos;
        double odom_yaw;
        double odom_linear_v, odom_angular_v;
        bool odom_callback_flag;
        ros::Publisher cmd_vel_pub, marker_pub;
        ros::Subscriber odom_sub;
    
    private:
        ros::NodeHandle& n_;
        MathTool math_tool_;

};
}

#endif // PURE_PURSUIT_HPP_