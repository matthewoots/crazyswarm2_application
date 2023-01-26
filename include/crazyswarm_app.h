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
#include "crazyflie_interfaces/srv/set_group_mask.hpp"
#include "crazyflie_interfaces/msg/velocity_world.hpp"

#include "crazyswarm_application/msg/user_command.hpp"
#include "crazyswarm_application/msg/agents_state_feedback.hpp"
#include "crazyswarm_application/msg/agent_state.hpp"

#include "motion_capture_tracking_interfaces/msg/named_pose_array.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "apriltag_msgs/msg/april_tag_detection.hpp" 

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <rclcpp/rclcpp.hpp>

#include "common.h"
#include "agent.h"
#include "kdtree.h"

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
using geometry_msgs::msg::Twist;

using motion_capture_tracking_interfaces::msg::NamedPoseArray;

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

using namespace common;

using namespace RVO;

namespace cs2
{
    class cs2_application : public rclcpp::Node
    {
        public:

            cs2_application()
                : Node("cs2_application"), clock(RCL_ROS_TIME)
            {
                // declare global commands
                this->declare_parameter("queue_size", 1);

                this->declare_parameter("trajectory_parameters.max_velocity", -1.0);
                this->declare_parameter("trajectory_parameters.takeoff_land_velocity", -1.0);
                this->declare_parameter("trajectory_parameters.takeoff_height", -1.0);
                this->declare_parameter("trajectory_parameters.reached_threshold", -1.0);
                this->declare_parameter("trajectory_parameters.planning_rate", -1.0);
                this->declare_parameter("trajectory_parameters.communication_radius", -1.0);
                this->declare_parameter("trajectory_parameters.protected_zone", -1.0);
                this->declare_parameter("trajectory_parameters.planning_horizon_scale", -1.0);

                this->declare_parameter("april_tag_parameters.camera_rotation");
                this->declare_parameter("april_tag_parameters.time_threshold", -1.0);
                this->declare_parameter("april_tag_parameters.eliminate_threshold", -1.0);

                max_queue_size = 
                    this->get_parameter("queue_size").get_parameter_value().get<int>();

                max_velocity = 
                    this->get_parameter("trajectory_parameters.max_velocity").get_parameter_value().get<double>();
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

                std::vector<double> camera_rotation = 
                    this->get_parameter("april_tag_parameters.camera_rotation").get_parameter_value().get<std::vector<double>>();
                time_threshold = 
                    this->get_parameter("april_tag_parameters.time_threshold").get_parameter_value().get<double>();
                eliminate_threshold = 
                    this->get_parameter("april_tag_parameters.eliminate_threshold").get_parameter_value().get<double>();

                static_camera_transform = Eigen::Affine3d::Identity();
                // Quaterniond is w,x,y,z
                static_camera_transform.linear() =
                    Eigen::Quaterniond(camera_rotation[3], camera_rotation[0], 
                    camera_rotation[1], camera_rotation[2]).toRotationMatrix();
                
                std::cout << "camera_rotation:" << std::endl;
                std::cout << static_camera_transform.linear() << std::endl;

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

                    agent_state a_s;
                    Eigen::Affine3d aff = Eigen::Affine3d::Identity();
                    aff.translation() = Eigen::Vector3d(pos[0], pos[1], pos[2]);
                    a_s.t = clock.now();
                    a_s.transform = aff;
                    a_s.flight_state = IDLE;
                    a_s.radio_connection = false;
                    a_s.completed = false;

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
                    
                    pose_sub.insert({name, this->create_subscription<PoseStamped>(
                        name + "/pose", 7, pcallback)});
                    vel_sub.insert({name, this->create_subscription<Twist>(
                        name + "/vel", 7, vcallback)});
                    tag_sub.insert({name, this->create_subscription<AprilTagDetectionArray>(
                        name + "/tag", 7, tcallback)});

                    tmp.vel_world_publisher = 
                        this->create_publisher<VelocityWorld>(name + "/cmd_velocity_world", 10);

                    // RCLCPP_INFO(this->get_logger(), "%lf", (--agents_states.end())->second.transform(0,0));
                    tag_queue empty = tag_queue();

                    agents_tag_queue.insert({name, empty});
                    agents_comm.insert({name, tmp});
                
                    Agent new_rvo2_agent = Agent(
                        id, (float)(1/planning_rate), 10, (float)max_velocity, 
                        (float)communication_radius, (float)protected_zone, 
                        (float)(planning_horizon_scale * 1/planning_rate));
                    rvo_agents.insert({name, new_rvo2_agent});
                }

                pose_publisher = 
                    this->create_publisher<NamedPoseArray>("poses", 7);
                
                agent_state_publisher = 
                    this->create_publisher<AgentsStateFeedback>("agents", 7);

                subscription_user = 
                    this->create_subscription<UserCommand>("user", 7, std::bind(&cs2_application::user_callback, this, _1));

                takeoff_all_client = this->create_client<Takeoff>("/all/takeoff");
                land_all_client = this->create_client<Land>("/all/land");

                tag_timer = this->create_wall_timer(
                    200ms, std::bind(&cs2_application::tag_timer_callback, this));

                auto t_planning = (1/planning_rate) * 1000ms;
                handler_timer = this->create_wall_timer(
                    t_planning, std::bind(&cs2_application::handler_timer_callback, this));

                RCLCPP_INFO(this->get_logger(), "end_constructor");

            };

        private:

            // parameters
            int max_queue_size;

            double max_velocity;
            double takeoff_land_velocity;
            double takeoff_height;
            double reached_threshold;
            double planning_rate;
            double communication_radius;
            double protected_zone;
            double planning_horizon_scale;
            // threshold parameters
            double time_threshold;
            double eliminate_threshold;

            Eigen::Affine3d static_camera_transform;

            rclcpp::Client<Takeoff>::SharedPtr takeoff_all_client;
            rclcpp::Client<Land>::SharedPtr land_all_client;

            std::map<std::string, agent_struct> agents_comm;
            std::map<std::string, agent_state> agents_states;

            std::map<std::string, rclcpp::Subscription<PoseStamped>::SharedPtr> pose_sub;
            std::map<std::string, rclcpp::Subscription<Twist>::SharedPtr> vel_sub;
            std::map<std::string, rclcpp::Subscription<AprilTagDetectionArray>::SharedPtr> tag_sub;

            std::map<std::string, tag_queue> agents_tag_queue;

            std::map<int, std::string> april_eliminate;
            std::map<int, Eigen::Vector2d> april_relocalize;
            std::map<std::string, Agent> rvo_agents;

            rclcpp::TimerBase::SharedPtr planning_timer;
            rclcpp::TimerBase::SharedPtr tag_timer;
            rclcpp::TimerBase::SharedPtr handler_timer;

            std::mutex agent_update_mutex;
            std::mutex tag_queue_mutex;

            rclcpp::Clock clock;

            kdtree *kd_tree;

            rclcpp::Subscription<UserCommand>::SharedPtr subscription_user;

            rclcpp::Publisher<NamedPoseArray>::SharedPtr pose_publisher;
            rclcpp::Publisher<AgentsStateFeedback>::SharedPtr agent_state_publisher;

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

            void handle_eliminate(
                std::map<std::string, agent_state>::iterator s, tag t);

            bool handle_relocalize(
                std::queue<agent_state> &q, tag t, Eigen::Affine3d fused);
    };
}
