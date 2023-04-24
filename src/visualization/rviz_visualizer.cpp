/*
* rviz_visualizer.cpp
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
#include "math.h"

#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rviz_2d_overlay_msgs/msg/overlay_text.hpp"
#include "crazyswarm_application/msg/agents_state_feedback.hpp"
#include "crazyswarm_application/msg/agent_state.hpp"

#include "std_msgs/msg/color_rgba.hpp"

#include <tf2_ros/transform_broadcaster.h>

#include "rclcpp/rclcpp.hpp"

#include "common.h"

#include <Eigen/Dense>

using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;
using geometry_msgs::msg::TransformStamped;
using rviz_2d_overlay_msgs::msg::OverlayText;
using crazyswarm_application::msg::AgentsStateFeedback;
using crazyswarm_application::msg::AgentState;
using std_msgs::msg::ColorRGBA;

using namespace std::chrono_literals;

using namespace common;

using std::placeholders::_1;

class RvizVisualizer : public rclcpp::Node
{
    private:

        rclcpp::Clock clock;

        rclcpp::Publisher<MarkerArray>::SharedPtr tag_publisher;
        rclcpp::Publisher<OverlayText>::SharedPtr text_publisher;
        rclcpp::Publisher<Marker>::SharedPtr obstacle_publisher;
        rclcpp::Publisher<Marker>::SharedPtr orca_obstacle_publisher;
        rclcpp::Publisher<Marker>::SharedPtr visibility_obstacle_publisher;

        rclcpp::Subscription<AgentsStateFeedback>::SharedPtr agent_state_subscriber;

        rclcpp::TimerBase::SharedPtr visualizing_timer;

        std::map<std::string, tag> april_tags;

        std::string mesh_path;

        tf2_ros::TransformBroadcaster tf2_bc;

        Eigen::Affine3d nwu_to_rdf;
        Eigen::Affine3d enu_to_rdf;

        bool concave_obstacles;

        std::vector<visibility_graph::obstacle> global_obstacle_list;
        std::vector<visibility_graph::obstacle> visibility_obstacle_list;
        std::vector<visibility_graph::obstacle> orca_obstacle_list;

        visualization_msgs::msg::Marker global_obs_visualize;
        visualization_msgs::msg::Marker visibility_obs_visualize;
        visualization_msgs::msg::Marker orca_obs_visualize;

        double visibility_expansion_factor;
        double orca_static_expansion_factor;

        void visualizing_timer_callback()
        {
            obstacle_publisher->publish(global_obs_visualize);
            orca_obstacle_publisher->publish(orca_obs_visualize);
            visibility_obstacle_publisher->publish(visibility_obs_visualize);
            show_global_tag();
        }

        void agents_state_callback(
            const AgentsStateFeedback::SharedPtr msg)
        {
            rclcpp::Time now = clock.now();

            AgentsStateFeedback copy = *msg;
            OverlayText text_msg;
            text_msg.action = OverlayText::ADD;
            text_msg.horizontal_alignment = OverlayText::LEFT;
            text_msg.vertical_alignment = OverlayText::TOP;
            text_msg.width = 256;
            text_msg.height = 512;
            text_msg.text_size = 8.0;
            text_msg.line_width = 256;
            ColorRGBA fg;
            fg.r = 25; fg.g = 255; fg.b = 240; fg.a = 0.8; 
            // text_msg.fg_color = fg; 
            text_msg.font = "DejaVu Sans Mono";
            
            std::string text;
            std::map<int, agent_state> agents_map;

            // copy agent messages into local states
            for (auto &agent : copy.agents)
            {
                std::string str_copy = agent.id;
                // Remove cf from cfXX
                str_copy.erase(0,2);
                int id = std::stoi(str_copy);

                agent_state state;
                state.flight_state = agent.flight_state;
                state.radio_connection = agent.connected;
                state.completed = agent.completed;

                agents_map.insert({id, state});
            }

            for (auto it = agents_map.begin(); 
                it != agents_map.end(); it++)
                call_state_text(it, text);

            text_msg.text = text;
            text_publisher->publish(text_msg);
        }

        void call_state_text(
            std::map<int, agent_state>::iterator agent, 
            std::string &text)
        {  
            text += "cf" + std::to_string(agent->first) + " ";

            // cf1, cf10, cf100
            if (agent->first / 10 == 0)
                text += "-";
            if (agent->first / 100 == 0)
                text += "-";

            text += " connected:";
            text = text + (agent->second.radio_connection ? "y" : "n") + " task:";
            text = text + (agent->second.completed ? "y" : "n") + " state:";
            switch (agent->second.flight_state)
            {
                case IDLE:
                    text += "IDL";
                    break;
                case TAKEOFF:
                    text += "TKF";
                    break;
                case MOVE:
                    text += "MOV";
                    break;
                case MOVE_VELOCITY:
                    text += "MVV";
                    break;
                case INTERNAL_TRACKING:
                    text += "INT";
                    break;
                case HOVER:
                    text += "HVR";
                    break;
                case LAND:
                    text += "LND";
                    break;
                case EMERGENCY:
                    text += "EMG";
                    break;
                default:
                    text += "ERR";
                    break;
            }

            text += " ";
        }

        void show_global_tag()
        {
            std::string frame = "/world";
            MarkerArray tag_array;
            for (auto &[name, tag_struct] : april_tags)
            {
                Marker mk;
                mk.header.frame_id = frame;
                mk.header.stamp = clock.now();
                mk.ns = name;
                mk.id = tag_struct.id;
                mk.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
                mk.action = visualization_msgs::msg::Marker::ADD;
                mk.mesh_resource = "file://" + mesh_path;
                Eigen::Vector3d pos = tag_struct.transform.translation();
                mk.pose.position.x = pos.x();
                mk.pose.position.y = pos.y();
                mk.pose.position.z = pos.z();
                mk.pose.orientation.x = 0.0;
                mk.pose.orientation.y = 0.0;
                mk.pose.orientation.z = 0.0;
                mk.pose.orientation.w = 1.0;
                mk.scale.x = 1;
                mk.scale.y = 1;
                mk.scale.z = 1;
                if (std::strcmp(tag_struct.type.c_str(), relocalize.c_str()) == 0)
                    mk.color.b = mk.color.g = mk.color.r = 1.0f;
                else
                    mk.color.g = 1.0f;

                mk.mesh_use_embedded_materials = true;
                mk.color.a = 1.0;

                tag_array.markers.push_back(mk);

                // send a transform for this pose
                // Eigen::Quaterniond q(nwu_to_rdf.linear());
                
                // geometry_msgs::msg::TransformStamped msg2;
                // msg2.header.frame_id = frame;
                // msg2.header.stamp = clock.now();
                // msg2.child_frame_id = name;
                // msg2.transform.translation.x = pos.x();
                // msg2.transform.translation.y = pos.y();
                // msg2.transform.translation.z = pos.z();
                // msg2.transform.rotation.x = q.x();
                // msg2.transform.rotation.y = q.y();
                // msg2.transform.rotation.z = q.z();
                // msg2.transform.rotation.w = q.w();
                // tf2_bc.sendTransform(msg2);
            }

            tag_publisher->publish(tag_array);
        }

        void obstacles_to_vertices_vector(
            std::vector<visibility_graph::obstacle> &obs_list,
            visualization_msgs::msg::Marker &msg,
            Eigen::Vector3d rgb, int id)
        {
            std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> vect_vert;

            // vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> format
            for (visibility_graph::obstacle &obs : obs_list)
            {
                // Get the centroid 
                Eigen::Vector2d centroid;
                std::vector<Eigen::Vector2d> vert;             

                int obs_vert_pair_size, obs_hori_pair_size;
                obs_vert_pair_size = obs_hori_pair_size = obs.v.size();

                // Add lines for verticals
                for (int i = 0; i < obs_vert_pair_size; i++)
                {
                    std::pair<Eigen::Vector3d, Eigen::Vector3d> vert_pair;
                    vert_pair.first = Eigen::Vector3d(obs.v[i].x(), obs.v[i].y(), obs.h.first);
                    vert_pair.second = Eigen::Vector3d(obs.v[i].x(), obs.v[i].y(), obs.h.second);
                    vect_vert.push_back(vert_pair);
                }
                // Add lines for horizontals
                for (int i = 0; i < obs_hori_pair_size; i++)
                {
                    std::pair<Eigen::Vector3d, Eigen::Vector3d> vert_pair;
                    vert_pair.first = Eigen::Vector3d(
                        obs.v[i % obs_hori_pair_size].x(), 
                        obs.v[i % obs_hori_pair_size].y(), obs.h.first);
                    vert_pair.second = Eigen::Vector3d(
                        obs.v[(i+1) % obs_hori_pair_size].x(), 
                        obs.v[(i+1) % obs_hori_pair_size].y(), obs.h.first);
                    vect_vert.push_back(vert_pair);

                    vert_pair.first = Eigen::Vector3d(
                        obs.v[i % obs_hori_pair_size].x(), 
                        obs.v[i % obs_hori_pair_size].y(), obs.h.second);
                    vert_pair.second = Eigen::Vector3d(
                        obs.v[(i+1) % obs_hori_pair_size].x(), 
                        obs.v[(i+1) % obs_hori_pair_size].y(), obs.h.second);
                    vect_vert.push_back(vert_pair);
                }

            }

            msg.header.frame_id = "/world";
            msg.header.stamp = clock.now();
            msg.type = visualization_msgs::msg::Marker::LINE_LIST;
            msg.action = visualization_msgs::msg::Marker::ADD;

            msg.id = id;

            msg.color.r = rgb.x();
            msg.color.g = rgb.y();
            msg.color.b = rgb.z();

            msg.color.a = 0.75;

            msg.scale.x = 0.05;

            // Create the vertices line list
            for (auto &vert_pair : vect_vert)
            {
                geometry_msgs::msg::Point p1, p2;
                p1.x = vert_pair.first.x();
                p2.x = vert_pair.second.x();

                p1.y = vert_pair.first.y();
                p2.y = vert_pair.second.y();

                p1.z = vert_pair.first.z();
                p2.z = vert_pair.second.z();

                msg.points.push_back(p1);
                msg.points.push_back(p2);
            }
        }

    public:

        RvizVisualizer()
        : Node("rviz_visualizer"), clock(RCL_ROS_TIME), tf2_bc(this)
        {
            tag_publisher = this->create_publisher<MarkerArray>("rviz/tag", 10);

            text_publisher = this->create_publisher<OverlayText>("rviz/text", 10);
            
            obstacle_publisher = this->create_publisher<Marker>("rviz/obstacles", 10);
            orca_obstacle_publisher = this->create_publisher<Marker>("rviz/orca", 10);
            visibility_obstacle_publisher = this->create_publisher<Marker>("rviz/visibility", 10);

            visualizing_timer = this->create_wall_timer(
                1000ms, std::bind(&RvizVisualizer::visualizing_timer_callback, this));
        
            this->declare_parameter("mesh_path", "");
            this->declare_parameter("concave_obstacles", false);
            this->declare_parameter("visibility_expansion_factor", 1.0);
            this->declare_parameter("orca_static_expansion_factor", 1.0);

            visibility_expansion_factor = 
                this->get_parameter("visibility_expansion_factor").get_parameter_value().get<double>();
            orca_static_expansion_factor = 
                this->get_parameter("orca_static_expansion_factor").get_parameter_value().get<double>();
            
            mesh_path = 
                this->get_parameter("mesh_path").get_parameter_value().get<std::string>();
            concave_obstacles = 
                this->get_parameter("concave_obstacles").get_parameter_value().get<bool>();

            // load tags and obstacles from params
            auto node_parameters_iface = this->get_node_parameters_interface();
            const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides =
                node_parameters_iface->get_parameter_overrides();

            load_april_tags(parameter_overrides, april_tags, 0.0, true);
            // actual map
            load_obstacle_map(
                parameter_overrides, 0.0, global_obstacle_list,
                concave_obstacles);
            obstacles_to_vertices_vector(
                global_obstacle_list, global_obs_visualize, 
                Eigen::Vector3d(1.0, 1.0, 1.0), 1);
            // visibility map
            load_obstacle_map(
                parameter_overrides, visibility_expansion_factor, 
                visibility_obstacle_list, concave_obstacles);
            obstacles_to_vertices_vector(
                visibility_obstacle_list, visibility_obs_visualize, 
                Eigen::Vector3d(0.0, 1.0, 1.0), 2);
            // visibility map
            load_obstacle_map(
                parameter_overrides, orca_static_expansion_factor, 
                orca_obstacle_list, concave_obstacles);
            obstacles_to_vertices_vector(
                orca_obstacle_list, orca_obs_visualize, 
                Eigen::Vector3d(1.0, 1.0, 0.0), 3);

            agent_state_subscriber = 
                this->create_subscription<AgentsStateFeedback>("agents",
                2, std::bind(&RvizVisualizer::agents_state_callback, this, _1));
        
            // rotate z -90 then x -90 for it to be RDF
            nwu_to_rdf = enu_to_rdf = Eigen::Affine3d::Identity();
            nwu_to_rdf.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d(0,0,1)));
            
            enu_to_rdf.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d(1,0,0)));
            nwu_to_rdf.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d(1,0,0)));
                
        }        
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RvizVisualizer>());
    rclcpp::shutdown();
    return 0;
}