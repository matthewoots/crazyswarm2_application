/*
* common.h
*
* ---------------------------------------------------------------------
* Copyright (C) 2023 Matthew (matthewoots at gmail.com)
*
*  This program is free software; you can redistribute it and/or
*  modify it under the terms of the GNU General Public License
*  as published by the Free Software Foundation; either version 2
*  of the License, or (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
* ---------------------------------------------------------------------
*/

#ifndef COMMON_H
#define COMMON_H

#include <memory>
#include <vector>
#include <regex>
#include <mutex>
#include <queue>
#include <string>
#include "math.h"

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "std_srvs/srv/empty.hpp"

#include "crazyflie_interfaces/srv/land.hpp"
#include "crazyflie_interfaces/srv/go_to.hpp"
#include "crazyflie_interfaces/srv/set_group_mask.hpp"
#include "crazyflie_interfaces/msg/velocity_world.hpp"

#include <gtsam/geometry/Pose3.h>

#include <Eigen/SVD>

#include "visibility.h"

using std_srvs::srv::Empty;
using crazyflie_interfaces::srv::Land;
using crazyflie_interfaces::srv::GoTo;
using crazyflie_interfaces::srv::SetGroupMask;
using crazyflie_interfaces::msg::VelocityWorld;

namespace common
{
    struct agent_struct
    {
        rclcpp::Client<Empty>::SharedPtr emergency;
        rclcpp::Client<SetGroupMask>::SharedPtr set_group;
        rclcpp::Client<GoTo>::SharedPtr go_to;
        rclcpp::Client<Land>::SharedPtr land;
        rclcpp::Publisher<VelocityWorld>::SharedPtr vel_world_publisher;
    };

    struct agent_state
    {
        rclcpp::Time t;
        Eigen::Affine3d transform;
        Eigen::Vector3d velocity;
        std::queue<Eigen::Vector3d> target_queue;
        double target_yaw;
        Eigen::Vector3d previous_target;
        double previous_yaw;
        size_t flight_state;
        bool radio_connection;
        bool completed;
        bool mission_capable;

        gtsam::Pose3 transformEigen2Gtsam()
        {
            Eigen::Matrix3d rot_eigen = transform.linear();
            gtsam::Rot3 rot(
                rot_eigen(0,0), rot_eigen(0,1), rot_eigen(0,2),
                rot_eigen(1,0), rot_eigen(1,1), rot_eigen(1,2),
                rot_eigen(2,0), rot_eigen(2,1), rot_eigen(2,2));
        
            return gtsam::Pose3(rot, 
                gtsam::Point3{transform.translation().x(), 
                transform.translation().y(), 
                transform.translation().z()});
        }
    };

    struct tag
    {
        rclcpp::Time t;
        uint8_t id;
        Eigen::Affine3d transform;
        Eigen::Vector2i pixel_center;
        std::string type;
        bool found = false;

        gtsam::Pose3 transformEigen2Gtsam()
        {
            Eigen::Matrix3d rot_eigen = transform.linear();
            gtsam::Rot3 rot(
                rot_eigen(0,0), rot_eigen(0,1), rot_eigen(0,2),
                rot_eigen(1,0), rot_eigen(1,1), rot_eigen(1,2),
                rot_eigen(2,0), rot_eigen(2,1), rot_eigen(2,2));
        
            return gtsam::Pose3(rot, 
                gtsam::Point3{transform.translation().x(), 
                transform.translation().y(), 
                transform.translation().z()});
        }
    };

    struct tag_queue
    {
        std::queue<tag> t_queue;
        std::queue<agent_state> s_queue;
    };

    struct observation
    {
        gtsam::Pose3 pose;
        std::queue<tag> marker;
    };

    struct factor_graph
    {
        std::map<long, observation> observations;
    };

    enum fsm
    {
        IDLE, // Have not taken off
        TAKEOFF, // Taking off sequence
        MOVE, // Move according to high level command (change target)
        MOVE_VELOCITY, // Move according to velocity command (change target)
        INTERNAL_TRACKING, // Internal command logic takes precedence over external
        HOVER, // Stop and hover
        LAND, // Landing sequence
        EMERGENCY // Emergency
    };

    class string_dictionary
    {
        public:
            const std::string takeoff = "takeoff";
            const std::string takeoff_all = "takeoff_all";
            const std::string land = "land";
            const std::string land_all = "land_all";
            const std::string go_to = "goto";
            const std::string hold = "hold";
            const std::string external = "external";
            const std::string go_to_velocity = "goto_velocity";

            const std::string concurrent = "conc";
            const std::string wait = "wait";
            
            const std::string all = "all";
    };

    const std::string relocalize = "relocalization";
    const std::string eliminate = "eliminate";

    std::set<std::string> extract_names(
        const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides,
        const std::string &pattern);

    /** 
     * @brief similar to the ROS euler rpy which does not require the tf library, 
     * hence independent of ROS but yield the same rotation 
    **/
    Eigen::Vector3d euler_rpy(Eigen::Matrix3d R);

    Eigen::Vector4f quat_to_vec4(Eigen::Quaterniond q);
    
    Eigen::Quaterniond vec4_to_quat(Eigen::Vector4f v);

    // Method to find the average of a set of rotation quaternions using Singular Value Decomposition
    /*
    * The algorithm used is described here:
    * https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872.pdf
    */
    // Taken from https://gist.github.com/PeteBlackerThe3rd/f73e9d569e29f23e8bd828d7886636a0 
    Eigen::Vector4f quaternion_average(
        std::vector<Eigen::Vector4f> quaternions);

    std::vector<std::string> split_space_delimiter(std::string str);

    std::vector<visibility_graph::obstacle> generate_disjointed_wall(
        std::vector<Eigen::Vector2d> vertices, std::pair<double, double> height_pair,
        double thickness);
    
    void load_april_tags(
        std::map<std::string, rclcpp::ParameterValue> parameter_overrides,
        std::map<std::string, tag> &april_tag_map, double tag_edge_size, bool is_center_origin);
    
    void load_obstacle_map(
        std::map<std::string, rclcpp::ParameterValue> parameter_overrides, double factor,
        std::vector<visibility_graph::obstacle> &obstacle_map, bool concave);

    double wrap_pi(double x);
}

#endif