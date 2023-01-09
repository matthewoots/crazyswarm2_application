#include <memory>
#include <vector>
#include <regex>
#include <mutex>
#include <queue>
#include <string>

#include <Eigen/Dense>

#include "crazyflie_interfaces/srv/takeoff.hpp"
#include "crazyflie_interfaces/srv/land.hpp"
#include "crazyflie_interfaces/srv/go_to.hpp"

#include "crazyswarm_application/msg/user_command.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include <rclcpp/rclcpp.hpp>

using crazyflie_interfaces::srv::Takeoff;
using crazyflie_interfaces::srv::Land;
using crazyflie_interfaces::srv::GoTo;

using std::placeholders::_1;

using namespace std::chrono_literals;

namespace cs2
{
    class cs2_application : public rclcpp::Node
    {
        public:
            cs2_application()
                : Node("cs2_application")
            {
                // declare global commands
                this->declare_parameter("trajectory_parameters.max_velocity", -1.0);
                this->declare_parameter("trajectory_parameters.takeoff_land_velocity", -1.0);
                this->declare_parameter("trajectory_parameters.takeoff_height", -1.0);

                max_velocity = this->get_parameter("trajectory_parameters.max_velocity").get_parameter_value().get<double>();

                takeoff_land_velocity = this->get_parameter("trajectory_parameters.takeoff_land_velocity").get_parameter_value().get<double>();

                takeoff_height = this->get_parameter("trajectory_parameters.takeoff_height").get_parameter_value().get<double>();

                // load crazyflies from params
                auto node_parameters_iface = this->get_node_parameters_interface();
                const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides =
                    node_parameters_iface->get_parameter_overrides();

                auto cf_names = extract_names(parameter_overrides, "robots");
                for (const auto &name : cf_names) 
                {
                    RCLCPP_INFO(this->get_logger(), "creating map for '%s'", name.c_str());

                    go_to_map.insert({name, this->create_client<GoTo>(name + "/go_to")});
                    land_map.insert({name, this->create_client<Land>(name + "/land")});

                    // std::string str_copy = name;
                    // // Remove cf from cfXX
                    // str_copy.erase(0,2);

                    std::vector<double> pos = parameter_overrides.at("robots." + name + ".initial_position").get<std::vector<double>>();

                    Eigen::Affine3d aff;
                    aff.translation() = Eigen::Vector3d(pos[0], pos[1], pos[2]);

                    // to get the index of string std::stoi(str_copy)
                    agents_pose.insert(
                        std::pair<std::string, Eigen::Affine3d>(name, aff));

                    std::function<void(const geometry_msgs::msg::PoseStamped::SharedPtr)> callback = 
                        std::bind(&cs2_application::pose_callback,
                        this, std::placeholders::_1, agents_pose.end());

                    pose_map.insert({name, 
                        this->create_subscription<geometry_msgs::msg::PoseStamped>(name + "/pose", 5, callback)});
                }

                subscription_user = 
                    this->create_subscription<crazyswarm_application::msg::UserCommand>("user", 10, std::bind(&cs2_application::user_callback, this, _1));

                takeoff_all_client = this->create_client<Takeoff>("/all/takeoff");
                land_all_client = this->create_client<Land>("/all/land");

                RCLCPP_INFO(this->get_logger(), "end_constructor");
            };

            std::string takeoff_all = "takeoff_all";
            std::string land_all = "land_all";
            std::string land = "land";
            std::string go_to = "goto";

        private:

            std::string name_;

            double max_velocity;
            double takeoff_land_velocity;
            double takeoff_height;

            rclcpp::Client<Takeoff>::SharedPtr takeoff_all_client;
            rclcpp::Client<Land>::SharedPtr land_all_client;
            std::map<std::string, rclcpp::Client<GoTo>::SharedPtr> go_to_map;
            std::map<std::string, rclcpp::Client<Land>::SharedPtr> land_map;
            std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_map;
            std::map<std::string, Eigen::Affine3d> agents_pose;

            rclcpp::Subscription<crazyswarm_application::msg::UserCommand>::SharedPtr subscription_user;

            std::set<std::string> extract_names(
                const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides,
                const std::string &pattern);

            void user_callback(const crazyswarm_application::msg::UserCommand::SharedPtr msg);

            void pose_callback(
                const geometry_msgs::msg::PoseStamped::SharedPtr msg, 
                std::map<std::string, Eigen::Affine3d>::iterator pose);
    };
}
