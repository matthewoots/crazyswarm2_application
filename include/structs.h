#include <memory>
#include <vector>
#include <regex>
#include <mutex>
#include <queue>
#include <string>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "crazyflie_interfaces/srv/land.hpp"
#include "crazyflie_interfaces/srv/go_to.hpp"
#include "crazyflie_interfaces/srv/set_group_mask.hpp"
#include "crazyflie_interfaces/msg/velocity_world.hpp"

using crazyflie_interfaces::srv::Land;
using crazyflie_interfaces::srv::GoTo;
using crazyflie_interfaces::srv::SetGroupMask;
using crazyflie_interfaces::msg::VelocityWorld;

struct agent_struct
{
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
    size_t flight_state;
    bool radio_connection;
    bool completed;
};

struct tag
{
    rclcpp::Time t;
    uint8_t id;
    Eigen::Affine3d transform;
    Eigen::Vector2i pixel_center;
};

struct tag_queue
{
    std::queue<tag> t_queue;
    std::queue<agent_state> s_queue;
};

enum fsm
{
    IDLE, // Have not taken off
    TAKEOFF, // Taking off sequence
    MOVE, // Move according to external command (change target)
    MOVE_VELOCITY, // Move according to external command (change target)
    INTERNAL_TRACKING, // Internal command logic takes precedence over external
    HOVER, // Stop and hover
    LAND // Landing sequence
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
        const std::string go_to_velocity = "goto_velocity";

        const std::string concurrent = "conc";
        const std::string wait = "wait";
        
        const std::string all = "all";
        const std::string external = "ext";
};

const std::string relocalize = "relocalization";
const std::string eliminate = "eliminate";