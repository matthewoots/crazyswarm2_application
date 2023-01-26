#include <memory>
#include <vector>
#include <regex>
#include <mutex>
#include <queue>
#include <string>

#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "rclcpp/rclcpp.hpp"

#include "common.h"

#include <Eigen/Dense>

using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;

using namespace std::chrono_literals;

using namespace common;

class RvizVisualizer : public rclcpp::Node
{
    public:

        RvizVisualizer()
        : Node("rviz_visualizer"), clock(RCL_ROS_TIME)
        {

            tag_publisher = this->create_publisher<MarkerArray>("rviz/tag", 10);
            
            visualizing_timer = this->create_wall_timer(
                1000ms, std::bind(&RvizVisualizer::visualizing_timer_callback, this));
        
            this->declare_parameter("mesh_path", "");

            mesh_path = 
                this->get_parameter("mesh_path").get_parameter_value().get<std::string>();

            // load tags and obstacles from params
            auto node_parameters_iface = this->get_node_parameters_interface();
            const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides =
                node_parameters_iface->get_parameter_overrides();
        
            auto tags = extract_names(parameter_overrides, "april_tags");
            for (const auto &tag_name : tags) 
            {
                tag tmp;
                std::string name = tag_name;
                tmp.id = parameter_overrides.at("april_tags." + tag_name + ".id").get<int>();
                tmp.type = parameter_overrides.at("april_tags." + tag_name + ".purpose").get<std::string>();
                std::vector<double> pos = parameter_overrides.at("april_tags." + tag_name + ".location").get<std::vector<double>>();
                tmp.transform.translation() = 
                    Eigen::Vector3d(pos[0], pos[1], pos[2]);
                
                april_tags.insert({name, tmp});
                
                RCLCPP_INFO(this->get_logger(), "tag %s: [%.3lf, %.3lf, %.3lf]", 
                    tag_name.c_str(), pos[0], pos[1], pos[2]);
            }

            auto obstacles = extract_names(parameter_overrides, "environment.obstacles");
            for (const auto &obs : obstacles) 
            {
                
            }
        }

    private:

        rclcpp::Clock clock;

        rclcpp::Publisher<MarkerArray>::SharedPtr tag_publisher;

        rclcpp::TimerBase::SharedPtr visualizing_timer;

        std::map<std::string, tag> april_tags;

        std::string mesh_path;

        void visualizing_timer_callback()
        {
            show_global_tag();
            show_obstacles_tag();
        }

        void show_global_tag()
        {
            MarkerArray tag_array;
            for (auto &[name, tag_struct] : april_tags)
            {
                Marker mk;
                mk.header.frame_id = "/world";
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
                mk.color.b = mk.color.g = mk.color.r =1.0f;
                mk.mesh_use_embedded_materials = true;
                mk.color.a = 1.0;

                tag_array.markers.push_back(mk);
            }

            tag_publisher->publish(tag_array);
        }

        void show_obstacles_tag()
        {
            
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RvizVisualizer>());
    rclcpp::shutdown();
    return 0;
}