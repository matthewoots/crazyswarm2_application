#include "crazyswarm_app.h"

void cs2::cs2_application::conduct_planning(
    Eigen::Vector3d &desired, std::string mykey, agent_state state) 
{
    auto it = rvo_agents.find(mykey);
    if (it == rvo_agents.end())
        return;

    // Construct KD-tree
    kd_tree = kd_create(3);
    for (auto &[key, agent] : agents_states)
    { 
        if (strcmp(mykey.c_str(), key.c_str()) == 0)
            continue;
        Eval_agent *node = new Eval_agent;
        node->position_ = agent.transform.translation().cast<float>();
        node->velocity_ = agent.velocity.cast<float>();
        node->radius_ = (float)protected_zone;
        int v = kd_insert3(
            kd_tree, node->position_.x(), 
            node->position_.y(), node->position_.z(),
            node);
    }

    struct kdres *neighbours;
    neighbours = kd_nearest_range3(
        kd_tree, state.transform.translation().x(), 
        state.transform.translation().y(), 
        state.transform.translation().z(),
        communication_radius);

    float communication_radius_float = (float)communication_radius;

    it->second.clearAgentNeighbor();

    while (!kd_res_end(neighbours))
    {
        double pos[3];
        Eval_agent *agent = (Eval_agent*)kd_res_item(neighbours, pos);
        
        it->second.insertAgentNeighbor(*agent, communication_radius_float);
        // store range query result so that we dont need to query again for rewire;
        kd_res_next(neighbours); // go to next in kd tree range query result
    }

    kd_free(kd_tree);

    if (!it->second.noNeighbours())
    {
        agent_update_mutex.lock();
        it->second.updateState(
            state.transform.translation().cast<float>(), 
            state.velocity.cast<float>(), 
            desired.cast<float>());
        agent_update_mutex.unlock();

        it->second.computeNewVelocity();
        Eigen::Vector3f new_desired = it->second.getVelocity();
        desired = new_desired.cast<double>();
    }
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
                double pose_difference = 
                    (agent.previous_target - agent.transform.translation()).norm();

                Eigen::Vector3d vel_target;
                if (pose_difference < max_velocity)
                    vel_target = 
                        agent.previous_target - agent.transform.translation(); 
                else
                    vel_target = 
                        (agent.previous_target - agent.transform.translation()).normalized() * max_velocity;
                
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
                rclcpp::Time start = clock.now();

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
                }
                else if (pose_difference < max_velocity)
                    vel_target = 
                        agent.target_queue.front() - agent.transform.translation(); 
                else
                {
                    vel_target = 
                        (agent.target_queue.front() - agent.transform.translation()).normalized() * max_velocity;
                    conduct_planning(vel_target, key, agent);
                }

                double duration_seconds = (clock.now() - start).seconds();
                RCLCPP_INFO(this->get_logger(), "go_to_velocity %s (%.3lf %.3lf %.3lf) time (%.3lfms)", 
                    key.c_str(), vel_target.x(), vel_target.y(), vel_target.z(), duration_seconds * 1000.0);

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
                break;
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