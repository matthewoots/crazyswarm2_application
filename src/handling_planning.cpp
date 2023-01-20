#include "crazyswarm_app.h"

void cs2::cs2_application::conduct_planning() 
{
    rclcpp::Time now = this->get_clock()->now();

}

void cs2::cs2_application::handler_timer_callback() 
{
    AgentsStateFeedback agents_feedback;

    // Iterate through the agents
    for (auto &[key, agent] : agents_states)
    {
        rclcpp::Time now = this->get_clock()->now();
        switch (agent.flight_state)
        {
            case IDLE:
            {
                break;
            }

            case HOVER: 
            {
                Eigen::Vector3d vel_target = 
                    agent.previous_target - agent.transform.translation();
                VelocityWorld vel_msg;
                vel_msg.header.stamp = clock.now();
                vel_msg.vel.x = vel_target.x();
                vel_msg.vel.y = vel_target.y();
                vel_msg.vel.z = vel_target.z();
                vel_msg.height = agent.previous_target.z();
                vel_msg.yaw = 0.0;
                auto it = agents_comm.find(key);
                if (it != agents_comm.end())
                    it->second.vel_world_publisher->publish(vel_msg);
                
                // agent.completed = false;
                break;
            }
            case TAKEOFF: case LAND: case MOVE:
            {
                bool is_land = 
                    (agent.flight_state == LAND);

                if (agent.target_queue.empty())
                {
                    // we do not need to handle the velocity here since:
                    // cffirmware land service handles it for us
                    // after popping the takeoff/goto queue till it is empty, change state to hover
                    agent.flight_state = is_land ? IDLE : HOVER;
                    agent.completed = true;
                    break;
                }
                double pose_difference = 
                    (agent.target_queue.front() - agent.transform.translation()).norm();
                if (pose_difference < reached_threshold)
                {
                    agent.previous_target = agent.target_queue.front();
                    agent.target_queue.pop();
                }
                break;
            }
            case MOVE_VELOCITY:
            {
                if (agent.target_queue.empty())
                {
                    agent.flight_state = HOVER;
                    agent.completed = true;
                    break;
                }
                double pose_difference = 
                    (agent.target_queue.front() - agent.transform.translation()).norm();

                VelocityWorld vel_msg;
                Eigen::Vector3d vel_target;
                vel_msg.height = agent.target_queue.front().z();

                if (pose_difference < reached_threshold)
                {
                    vel_target = Eigen::Vector3d::Zero();
                    agent.previous_target = agent.target_queue.front();
                    agent.target_queue.pop();
                    break;
                }
                else if (pose_difference < max_velocity)
                    vel_target = 
                        agent.target_queue.front() - agent.transform.translation(); 
                else
                    vel_target = 
                        (agent.target_queue.front() - agent.transform.translation()).normalized() * max_velocity;
                
                
                RCLCPP_INFO(this->get_logger(), "go_to_velocity %s (%.3lf %.3lf %.3lf)", 
                    key.c_str(), vel_target.x(), vel_target.y(), vel_target.z());

                vel_msg.header.stamp = clock.now();
                vel_msg.vel.x = vel_target.x();
                vel_msg.vel.y = vel_target.y();
                vel_msg.vel.z = vel_target.z();

                // check the difference in heading
                // Eigen::Vector3d rpy = 
                //     agent.transform.eulerAngles(2,1,0).reverse();
                vel_msg.yaw = 0.0;
                
                auto it = agents_comm.find(key);
                if (it != agents_comm.end())
                    it->second.vel_world_publisher->publish(vel_msg);
            }
            case INTERNAL_TRACKING:
            {
                break;
            }
        }

        AgentState agentstate;
        agentstate.id = key;
        agentstate.flight_state = agent.flight_state;
        agentstate.connected = agent.radio_connection;
        agentstate.completed = agent.completed;

        agents_feedback.agents.push_back(agentstate);
    }

    // publish the flight state message
    agents_feedback.header.stamp = clock.now();
    agent_state_publisher->publish(agents_feedback);
}