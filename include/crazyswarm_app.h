/*
* crazyswarm_app.h
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

#include <memory>
#include <vector>
#include <regex>
#include <mutex>
#include <queue>
#include <string>
#include <ctime>

#include <Eigen/Dense>

#include "std_srvs/srv/empty.hpp"

#include "crazyflie_interfaces/srv/takeoff.hpp"
#include "crazyflie_interfaces/srv/land.hpp"
#include "crazyflie_interfaces/srv/go_to.hpp"
#include "crazyflie_interfaces/srv/set_group_mask.hpp"
#include "crazyflie_interfaces/msg/velocity_world.hpp"

#include "crazyswarm_application/msg/user_command.hpp"
#include "crazyswarm_application/msg/agents_state_feedback.hpp"
#include "crazyswarm_application/msg/agent_state.hpp"

#include "crazyswarm_application/msg/named_pose_array.hpp"
#include "crazyswarm_application/msg/named_pose.hpp"

#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "apriltag_msgs/msg/april_tag_detection.hpp" 

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include "common.h"
#include "agent.h"
#include "kdtree.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>

#include <CSVWriter.h>

using std_srvs::srv::Empty;

using crazyflie_interfaces::srv::Takeoff;
using crazyflie_interfaces::srv::Land;
using crazyflie_interfaces::srv::GoTo;
using crazyflie_interfaces::srv::SetGroupMask;
using crazyflie_interfaces::msg::VelocityWorld;

using crazyswarm_application::msg::UserCommand;
using crazyswarm_application::msg::AgentsStateFeedback;
using crazyswarm_application::msg::AgentState;

using apriltag_msgs::msg::AprilTagDetection;
using apriltag_msgs::msg::AprilTagDetectionArray;

using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TransformStamped;

using crazyswarm_application::msg::NamedPoseArray;
using crazyswarm_application::msg::NamedPose;

using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

using namespace common;

using namespace RVO;

using gtsam::symbol_shorthand::X; // state estimate
using gtsam::symbol_shorthand::Z; // measurement (marker)

namespace cs2
{
    class cs2_application : public rclcpp::Node
    {
        public:

            cs2_application()
                : Node("cs2_application"), clock(RCL_ROS_TIME), tf2_bc(this)
            {
                start_node_time = clock.now();

                // declare global commands
                this->declare_parameter("queue_size", 1);
                this->declare_parameter("visibility_expansion_factor", 1.0);
                this->declare_parameter("orca_static_expansion_factor", 1.0);
                this->declare_parameter("radio_connection_timeout", 1.0);
                this->declare_parameter("concave_obstacles", false);

                this->declare_parameter("trajectory_parameters.max_velocity", -1.0);
                this->declare_parameter("trajectory_parameters.maximum_yaw_change", -1.0);
                this->declare_parameter("trajectory_parameters.takeoff_land_velocity", -1.0);
                this->declare_parameter("trajectory_parameters.takeoff_height", -1.0);
                this->declare_parameter("trajectory_parameters.reached_threshold", -1.0);
                this->declare_parameter("trajectory_parameters.planning_rate", -1.0);
                this->declare_parameter("trajectory_parameters.communication_radius", -1.0);
                this->declare_parameter("trajectory_parameters.protected_zone", -1.0);
                this->declare_parameter("trajectory_parameters.planning_horizon_scale", -1.0);
                this->declare_parameter("trajectory_parameters.height_range", std::vector<double>{});

                this->declare_parameter("april_tag_parameters.camera_rotation", std::vector<double>{});
                this->declare_parameter("april_tag_parameters.time_threshold", -1.0);
                this->declare_parameter("april_tag_parameters.observation_threshold", -1.0);
                this->declare_parameter("april_tag_parameters.observation_limit", 1);
                this->declare_parameter("april_tag_parameters.center_origin", false);
                this->declare_parameter("tag_edge_size", 0.162);
                this->declare_parameter("log_path", "");

                max_queue_size = 
                    this->get_parameter("queue_size").get_parameter_value().get<int>();

                concave_obstacles = 
                    this->get_parameter("concave_obstacles").get_parameter_value().get<bool>();
                max_velocity = 
                    this->get_parameter("trajectory_parameters.max_velocity").get_parameter_value().get<double>();
                maximum_yaw_change = 
                    this->get_parameter("trajectory_parameters.maximum_yaw_change").get_parameter_value().get<double>();
                takeoff_land_velocity = 
                    this->get_parameter("trajectory_parameters.takeoff_land_velocity").get_parameter_value().get<double>();
                takeoff_height = 
                    this->get_parameter("trajectory_parameters.takeoff_height").get_parameter_value().get<double>();
                reached_threshold = 
                    this->get_parameter("trajectory_parameters.reached_threshold").get_parameter_value().get<double>();
                planning_rate = 
                    this->get_parameter("trajectory_parameters.planning_rate").get_parameter_value().get<double>();
                communication_radius = 
                    this->get_parameter("trajectory_parameters.communication_radius").get_parameter_value().get<double>();
                protected_zone = 
                    this->get_parameter("trajectory_parameters.protected_zone").get_parameter_value().get<double>();
                planning_horizon_scale = 
                    this->get_parameter("trajectory_parameters.planning_horizon_scale").get_parameter_value().get<double>();
                std::vector<double> height_range_vector = 
                    this->get_parameter("trajectory_parameters.height_range").get_parameter_value().get<std::vector<double>>();
                assert(height_range_vector.size() == 2);

                radio_connection_timeout = 
                    this->get_parameter("radio_connection_timeout").get_parameter_value().get<double>();
                visibility_expansion_factor = 
                    this->get_parameter("visibility_expansion_factor").get_parameter_value().get<double>();
                orca_static_expansion_factor = 
                    this->get_parameter("orca_static_expansion_factor").get_parameter_value().get<double>();
                std::vector<double> camera_rotation = 
                    this->get_parameter("april_tag_parameters.camera_rotation").get_parameter_value().get<std::vector<double>>();
                time_threshold = 
                    this->get_parameter("april_tag_parameters.time_threshold").get_parameter_value().get<double>();
                observation_threshold = 
                    this->get_parameter("april_tag_parameters.observation_threshold").get_parameter_value().get<double>();
                observation_limit =
                    this->get_parameter("april_tag_parameters.observation_limit").get_parameter_value().get<int>();
                center_origin = 
                    this->get_parameter("april_tag_parameters.center_origin").get_parameter_value().get<bool>();
                tag_edge_size = 
                    this->get_parameter("tag_edge_size").get_parameter_value().get<double>();
                log_path = 
                    this->get_parameter("log_path").get_parameter_value().get<std::string>();

                static_camera_transform = Eigen::Affine3d::Identity();
                // Quaterniond is w,x,y,z
                static_camera_transform.linear() =
                    Eigen::Quaterniond(camera_rotation[3], camera_rotation[0], 
                    camera_rotation[1], camera_rotation[2]).toRotationMatrix();
                
                std::cout << "camera_rotation:" << std::endl;
                std::cout << static_camera_transform.linear() << std::endl;

                height_range = 
                    std::make_pair(height_range_vector[0], height_range_vector[1]);

                time_threshold = 
                    this->get_parameter("april_tag_parameters.time_threshold").get_parameter_value().get<double>();

                // load crazyflies from params
                auto node_parameters_iface = this->get_node_parameters_interface();
                const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides =
                    node_parameters_iface->get_parameter_overrides();

                auto cf_names = extract_names(parameter_overrides, "robots");
                for (const auto &name : cf_names) 
                {
                    RCLCPP_INFO(this->get_logger(), "creating agent map for '%s'", name.c_str());

                    std::string str_copy = name;
                    // Remove cf from cfXX
                    str_copy.erase(0,2);
                    int id = std::stoi(str_copy);

                    std::vector<double> pos = parameter_overrides.at("robots." + name + ".initial_position").get<std::vector<double>>();
                    bool mission_capable = parameter_overrides.at("robots." + name + ".mission_capable").get<bool>();

                    agent_state a_s;
                    Eigen::Affine3d aff = Eigen::Affine3d::Identity();
                    aff.translation() = Eigen::Vector3d(pos[0], pos[1], pos[2]);
                    a_s.t = clock.now();
                    a_s.transform = aff;
                    a_s.flight_state = IDLE;
                    a_s.radio_connection = false;
                    a_s.completed = false;
                    a_s.mission_capable = mission_capable;

                    // RCLCPP_INFO(this->get_logger(), "(%s) %lf %lf %lf", name.c_str(), 
                    //     pos[0], pos[1], pos[2]);

                    // to get the index of string std::stoi(str_copy)
                    agents_states.insert(
                        std::pair<std::string, agent_state>(name, a_s));

                    std::function<void(const PoseStamped::SharedPtr)> pcallback = 
                        std::bind(&cs2_application::pose_callback,
                        this, std::placeholders::_1, --agents_states.end());
                    std::function<void(const Twist::SharedPtr)> vcallback = 
                        std::bind(&cs2_application::twist_callback,
                        this, std::placeholders::_1, --agents_states.end());
                    std::function<void(const AprilTagDetectionArray::SharedPtr)> tcallback = 
                        std::bind(&cs2_application::tag_callback, this, std::placeholders::_1, name);
                    
                    agent_struct tmp;
                    tmp.go_to = this->create_client<GoTo>(name + "/go_to");
                    tmp.land = this->create_client<Land>(name + "/land");
                    tmp.set_group = this->create_client<SetGroupMask>(name + "/set_group_mask");
                    tmp.emergency = this->create_client<Empty>(name + "/emergency");
                    
                    pose_sub.insert({name, this->create_subscription<PoseStamped>(
                        name + "/pose", 14, pcallback)});
                    vel_sub.insert({name, this->create_subscription<Twist>(
                        name + "/vel", 14, vcallback)});
                    tag_sub.insert({name, this->create_subscription<AprilTagDetectionArray>(
                        name + "/tag", 14, tcallback)});

                    tmp.vel_world_publisher = 
                        this->create_publisher<VelocityWorld>(name + "/cmd_velocity_world", 25);

                    tag_queue empty = tag_queue();

                    agents_tag_queue.insert({name, empty});
                    agents_comm.insert({name, tmp});
                
                    Agent new_rvo2_agent = Agent(
                        id, (float)(1/planning_rate), 10, (float)max_velocity, 
                        (float)communication_radius, (float)protected_zone, 
                        (float)(planning_horizon_scale * 1/planning_rate),
                        (float)height_range.first, (float)height_range.second);
                    rvo_agents.insert({name, new_rvo2_agent});

                    agents_loop_closure.insert({name, factor_graph()});

                    RCLCPP_INFO(this->get_logger(), "agent %s created", name.c_str());
                }

                std::vector<double> _pair_location_list = 
                    parameter_overrides.at("april_tags.pair_position").get<std::vector<double>>();
                std::vector<double> _pair_paper_list = 
                    parameter_overrides.at("april_tags.pair_paper_size").get<std::vector<double>>();
                assert(_pair_location_list.size() == 4);
                assert(_pair_paper_list.size() == 2);

                std::vector<Eigen::Vector2d> _pair_location;
                Eigen::Vector2d _paper_size = 
                    Eigen::Vector2d(_pair_paper_list[0], _pair_paper_list[1]);
                _pair_location.emplace_back(
                    Eigen::Vector2d(_pair_location_list[0], _pair_location_list[1]));
                _pair_location.emplace_back(
                    Eigen::Vector2d(_pair_location_list[2], _pair_location_list[3]));

                auto april_names = extract_names(parameter_overrides, "april_tags.tags");
                for (const auto &name : april_names) 
                {
                    bool _is_pair = false;
                    std::vector<std::string> _april_tags;
                    // name size is more than 5 (idXXX), then it would be in format idXXX idXXX
                    if (name.size() > 5)
                    {
                        _april_tags = split_space_delimiter(name);
                        _is_pair = true;
                    }
                    else 
                        _april_tags.emplace_back(name);
                    
                    for (size_t i = 0; i < _april_tags.size(); i++)
                    {
                        // Remove id from idXXX
                        _april_tags[i].erase(0,2);
                        // apparently stoi will remove leading 0s
                        int id = std::stoi(_april_tags[i]);

                        std::string purpose = parameter_overrides.at("april_tags.tags." + name + ".purpose").get<std::string>();
                        
                        if (strcmp(purpose.c_str(), relocalize.c_str()) == 0)
                        {
                            std::vector<double> location = 
                                parameter_overrides.at("april_tags.tags." + name + ".location").get<std::vector<double>>();
                            Eigen::Vector2d tag_pos = Eigen::Vector2d(location[0], location[1]);
                            
                            if (_is_pair)
                            {
                                std::string alignment = 
                                    parameter_overrides.at("april_tags.tags." + name + ".alignment").get<std::string>();
                                if (strcmp(alignment.c_str(), "top-left") == 0)
                                    tag_pos += _pair_location[i];
                                else if (strcmp(alignment.c_str(), "top-right") == 0)
                                    tag_pos += _pair_location[i] + Eigen::Vector2d(0.0, _paper_size.y());
                                else if (strcmp(alignment.c_str(), "bottom-right") == 0)
                                    tag_pos += _pair_location[i] + Eigen::Vector2d(_paper_size.x(), _paper_size.y());
                                else if (strcmp(alignment.c_str(), "bottom-left") == 0)
                                    tag_pos += _pair_location[i] + Eigen::Vector2d(_paper_size.x(), 0.0);
                            }

                            if (!center_origin)
                                tag_pos += Eigen::Vector2d(tag_edge_size/2.0, tag_edge_size/2.0);

                            april_relocalize.insert({id, tag_pos});
                        }
                        else if (strcmp(purpose.c_str(), eliminate.c_str()) == 0)
                            april_eliminate.insert({id, Eigen::Vector2d::Zero()});
                        
                        RCLCPP_INFO(this->get_logger(), "tag %s created (%s)", 
                            _april_tags[i].c_str(), purpose.c_str());
                    }
                }

                std::vector<visibility_graph::obstacle> visibility_obstacle_list;
                load_obstacle_map(
                    parameter_overrides, visibility_expansion_factor, 
                    visibility_obstacle_list, concave_obstacles);
                visibility_obstacle_map.obs = visibility_obstacle_list;
                visibility_obstacle_map.inflation = protected_zone;
                std::vector<visibility_graph::obstacle> orca_obstacle_list;
                load_obstacle_map(
                    parameter_overrides, orca_static_expansion_factor, 
                    orca_obstacle_list, concave_obstacles);
                orca_obstacle_map.obs = orca_obstacle_list;
                orca_obstacle_map.inflation = protected_zone;

                pose_publisher = 
                    this->create_publisher<NamedPoseArray>("poses", 7);
                
                target_publisher = 
                    this->create_publisher<MarkerArray>("targets", 7);
                
                agent_state_publisher = 
                    this->create_publisher<AgentsStateFeedback>("agents", 7);

                subscription_user = 
                    this->create_subscription<UserCommand>("user", 30, std::bind(&cs2_application::user_callback, this, _1));

                takeoff_all_client = this->create_client<Takeoff>("/all/takeoff");
                land_all_client = this->create_client<Land>("/all/land");

                tag_timer = this->create_wall_timer(
                    100ms, std::bind(&cs2_application::tag_timer_callback, this));

                auto t_planning = (1/planning_rate) * 1000ms;
                handler_timer = this->create_wall_timer(
                    t_planning, std::bind(&cs2_application::handler_timer_callback, this));

                RCLCPP_INFO(this->get_logger(), "end_constructor");

                // rotate z -90 then x -90 for it to be FRD
                nwu_to_rdf = enu_to_rdf = Eigen::Affine3d::Identity();
                nwu_to_rdf.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d(0,0,1)));
                
                nwu_to_enu = nwu_to_rdf;
                enu_to_rdf.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d(1,0,0)));
                nwu_to_rdf.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d(1,0,0)));
            };

        private:

            // parameters
            int max_queue_size;

            bool center_origin;
            bool concave_obstacles;

            double max_velocity;
            double maximum_yaw_change;
            double takeoff_land_velocity;
            double takeoff_height;
            double reached_threshold;
            double planning_rate;
            double communication_radius;
            double protected_zone;
            double planning_horizon_scale;
            // threshold parameters
            double time_threshold;
            double observation_threshold;
            double tag_edge_size;
            double visibility_expansion_factor;
            double orca_static_expansion_factor;

            double radio_connection_timeout;

            int observation_limit;

            std::string log_path;

            Eigen::Affine3d nwu_to_rdf;
            Eigen::Affine3d enu_to_rdf;
            Eigen::Affine3d nwu_to_enu;

            Eigen::Affine3d static_camera_transform;

            visibility_graph::global_map visibility_obstacle_map;
            visibility_graph::global_map orca_obstacle_map;

            rclcpp::Client<Takeoff>::SharedPtr takeoff_all_client;
            rclcpp::Client<Land>::SharedPtr land_all_client;

            std::map<std::string, agent_struct> agents_comm;
            std::map<std::string, agent_state> agents_states;

            std::map<std::string, rclcpp::Subscription<PoseStamped>::SharedPtr> pose_sub;
            std::map<std::string, rclcpp::Subscription<Twist>::SharedPtr> vel_sub;
            std::map<std::string, rclcpp::Subscription<AprilTagDetectionArray>::SharedPtr> tag_sub;

            std::map<std::string, tag_queue> agents_tag_queue;

            std::map<int, Eigen::Vector2d> april_eliminate;
            std::map<int, Eigen::Vector2d> april_found;
            std::map<int, Eigen::Vector2d> april_relocalize;
            std::map<std::string, Agent> rvo_agents;

            std::map<std::string, factor_graph> agents_loop_closure;

            std::pair<double, double> height_range;

            rclcpp::TimerBase::SharedPtr planning_timer;
            rclcpp::TimerBase::SharedPtr tag_timer;
            rclcpp::TimerBase::SharedPtr handler_timer;
        
            tf2_ros::TransformBroadcaster tf2_bc;

            std::mutex agent_update_mutex;
            std::mutex tag_queue_mutex;

            rclcpp::Clock clock;

            rclcpp::Time start_node_time;

            kdtree *kd_tree;

            rclcpp::Subscription<UserCommand>::SharedPtr subscription_user;

            rclcpp::Publisher<NamedPoseArray>::SharedPtr pose_publisher;
            rclcpp::Publisher<AgentsStateFeedback>::SharedPtr agent_state_publisher;
            rclcpp::Publisher<MarkerArray>::SharedPtr target_publisher;
            
            void conduct_planning(
                Eigen::Vector3d &desired, std::string mykey, agent_state state);

            void user_callback(const UserCommand::SharedPtr msg);

            void pose_callback(
                const PoseStamped::SharedPtr msg, 
                std::map<std::string, agent_state>::iterator state);
            
            void twist_callback(
                const Twist::SharedPtr msg, 
                std::map<std::string, agent_state>::iterator state);
            
            void tag_callback(const AprilTagDetectionArray::SharedPtr& msg, std::string name);

            // timers
            void tag_timer_callback();
            void handler_timer_callback(); 

            void send_land_and_update(
                std::map<std::string, agent_state>::iterator s,
                std::map<std::string, agent_struct>::iterator c);

            void handle_eliminate(
                std::map<std::string, agent_state>::iterator s, tag t);

            bool handle_relocalize(
                std::queue<agent_state> &q, tag t, 
                std::map<int, Eigen::Vector2d>::iterator tag_pose, 
                std::string id);
    
            void gtsam_pose_optimization(
                Eigen::Affine3d &pose, 
                std::map<std::string, factor_graph>::iterator fact_it,
                std::map<std::string, agent_state>::iterator agent_it);
    };
}
