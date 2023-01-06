#include "crazyswarm_app.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<cs2::cs2_application>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    
    return 0;
}

std::set<std::string> cs2::cs2_application::extract_names(
    const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides,
    const std::string &pattern)
{
    std::set<std::string> result;
    for (const auto &i : parameter_overrides)
    {
        if (i.first.find(pattern) == 0)
        {
            size_t start = pattern.size() + 1;
            size_t end = i.first.find(".", start);
            result.insert(i.first.substr(start, end - start));
        }
    }
    return result;
}

void cs2::cs2_application::user_callback(
    const crazyswarm_application::msg::UserCommand::SharedPtr msg)
{
    using namespace cs2;
    crazyswarm_application::msg::UserCommand copy = *msg;

    if (strcmp(copy.cmd.c_str(), takeoff_all.c_str()) == 0 || 
        strcmp(copy.cmd.c_str(), land_all.c_str()) == 0)
    {
        bool is_takeoff_all = strcmp(copy.cmd.c_str(), takeoff_all.c_str()) == 0;

        RCLCPP_INFO(this->get_logger(), "%s", is_takeoff_all ? "takeoff request all" : "land request all");

        if (is_takeoff_all)
        {
            auto request = std::make_shared<Takeoff::Request>();
            request->group_mask = 0;
            request->height = takeoff_height;
            request->duration.sec = takeoff_height / takeoff_land_velocity;
            // while (!takeoff_all_client->wait_for_service())
            //     RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            
            auto result = takeoff_all_client->async_send_request(request); 
            // auto response = result.get();
        }
        else
        {
            auto request = std::make_shared<Land::Request>();
            request->group_mask = 0;
            request->height = 0.0;
            request->duration.sec = takeoff_height / takeoff_land_velocity;
            // while (!land_all_client->wait_for_service()) 
            //     RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            
            auto result = land_all_client->async_send_request(request);
            // auto response = result.get();
        }

        RCLCPP_INFO(this->get_logger(), "%s", is_takeoff_all ? "takeoff_all_sent" : "land_all_sent");
    }
    else if (strcmp(copy.cmd.c_str(), land.c_str()) == 0 || 
        strcmp(copy.cmd.c_str(), go_to.c_str()) == 0)
    {
        bool is_go_to = strcmp(copy.cmd.c_str(), go_to.c_str()) == 0;

        std::queue<int> check_queue;
        for (size_t i = 0; i < copy.uav_id.size(); i++)
        {
            if (check_queue.size() == copy.uav_id.size())
                break;
            
            if (is_go_to)
                for (std::pair<std::string, rclcpp::Client<Land>::SharedPtr> element : land_map) 
                {
                    if (strcmp(element.first.c_str(), copy.uav_id[i].c_str()) == 0)
                    {
                        RCLCPP_INFO(this->get_logger(), "land request for %s", element.first.c_str());
                        auto request = std::make_shared<Land::Request>();
                        request->group_mask = 0;
                        request->height = 0.0;
                        request->duration.sec = takeoff_height / takeoff_land_velocity;
                        auto result = element.second->async_send_request(request);
                        check_queue.push(i);
                        break;
                    }
                }
            else
                for (std::pair<std::string, rclcpp::Client<GoTo>::SharedPtr> element : go_to_map) 
                {
                    if (strcmp(element.first.c_str(), copy.uav_id[i].c_str()) == 0)
                    {
                        RCLCPP_INFO(this->get_logger(), "go_to request for %s", element.first.c_str());
                        auto request = std::make_shared<GoTo::Request>();
                        double distance = 5.0;
                        request->group_mask = 0;
                        request->relative = false;
                        request->goal = copy.goal;
                        request->duration.sec = distance / max_velocity;
                        auto result = element.second->async_send_request(request);
                        check_queue.push(i);
                        break;
                    }
                }
        }

        if (check_queue.size() != copy.uav_id.size())
        {
            while (!check_queue.empty())
            {
                copy.uav_id.erase(copy.uav_id.begin() + (check_queue.front()-1));
                check_queue.pop();
            }
            
            std::cout << "uav_id not found ";
            for (const auto& i: copy.uav_id)
                std::cout << i << " ";
            std::cout << std::endl;

            RCLCPP_INFO(this->get_logger(), "%s", is_go_to ? 
                "go_to_sent unfinished" : "land_sent unfinished");
            return;
        }
        else
            RCLCPP_INFO(this->get_logger(), "%s", is_go_to ? 
                "go_to_sent" : "land_sent");
    }
    else
        RCLCPP_ERROR(this->get_logger(), "wrong command type, resend");

}