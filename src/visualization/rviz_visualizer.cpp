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

        rclcpp::Subscription<AgentsStateFeedback>::SharedPtr agent_state_subscriber;

        rclcpp::TimerBase::SharedPtr visualizing_timer;

        std::map<std::string, tag> april_tags;

        std::string mesh_path;

        double scale_factor;

        tf2_ros::TransformBroadcaster tf2_bc;

        Eigen::Affine3d nwu_to_rdf;
        Eigen::Affine3d enu_to_rdf;

        void visualizing_timer_callback()
        {
            show_global_tag();
            show_obstacles_tag();
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
            text_msg.text_size = (double)(text_msg.width) / scale_factor;
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
                    mk.color.r = 1.0f;

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

        void show_obstacles_tag()
        {
            
        }

    public:

        RvizVisualizer()
        : Node("rviz_visualizer"), clock(RCL_ROS_TIME), tf2_bc(this)
        {

            tag_publisher = this->create_publisher<MarkerArray>("rviz/tag", 10);

            text_publisher = this->create_publisher<OverlayText>("rviz/text", 10);
            
            visualizing_timer = this->create_wall_timer(
                1000ms, std::bind(&RvizVisualizer::visualizing_timer_callback, this));
        
            this->declare_parameter("mesh_path", "");
            
            this->declare_parameter("rviz.text.scale_factor", 1.0);

            mesh_path = 
                this->get_parameter("mesh_path").get_parameter_value().get<std::string>();

            scale_factor = 
                this->get_parameter("rviz.text.scale_factor").get_parameter_value().get<double>();

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
                if (pos.empty())
                    continue;
                
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