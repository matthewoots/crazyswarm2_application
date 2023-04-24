/*
* planning.cpp
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

#include "crazyswarm_app.h"

void cs2::cs2_application::conduct_planning(
    Eigen::Vector3d &desired, std::string mykey, agent_state state) 
{
    auto it = rvo_agents.find(mykey);
    if (it == rvo_agents.end())
        return;

    if (agents_states.size() > 1)
    {
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

        // clear agent neighbour before adding in new neighbours and obstacles
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
    }

    // add in the static obstacles as static neighbours
    // find the obstacles that lie on the plane first
    if (!orca_obstacle_map.obs.empty())
    {
        float communication_radius_float = (float)communication_radius;
        visibility_graph::global_map copy = orca_obstacle_map;
        copy.start_end.first = state.transform.translation();
        copy.t = visibility_graph::get_affine_transform(
            copy.start_end.first, Eigen::Vector3d(0.0, 0.0, 0.0), "nwu");

        // check all obstacles and add in edges that may have collision
        std::vector<visibility_graph::obstacle> rot_polygons;
        std::vector<Eigen::Vector3d> debug_point_vertices;
        visibility_graph::get_polygons_on_plane(
            copy, Eigen::Vector3d(0.0, 0.0, 1.0), 
            rot_polygons, debug_point_vertices, true);
        
        // threshold is communication_radius
        for (auto &poly : rot_polygons)
        {
            for (size_t i = 0; i < poly.v.size(); i++)
            {
                size_t j = (i + 1) % (poly.v.size());
                double distance;
                Eigen::Vector2d closest_point;

                visibility_graph::get_point_to_line(
                    Eigen::Vector2d::Zero(), poly.v[i], poly.v[j],
                    distance, closest_point);
                if (distance < communication_radius)
                {
                    // distribute from poly.v[i] poly.v[j]
                    Eigen::Vector3d vi = 
                        copy.t.inverse() * Eigen::Vector3d(poly.v[i].x(), poly.v[i].y(), 0.0);
                    Eigen::Vector3d vj = 
                        copy.t.inverse() * Eigen::Vector3d(poly.v[j].x(), poly.v[j].y(), 0.0);
                    size_t div = (size_t)std::ceil((vj - vi).norm() / (protected_zone * 1.5));
                    double separation = (vj - vi).norm() / div;
                    Eigen::Vector3d dir = (vj - vi).normalized();

                    for (size_t j = 1; j < div-1; j++)
                    {
                        Eigen::Vector3d pos = vi + dir * j * separation;
                        
                        if ((pos - state.transform.translation()).norm() < communication_radius)
                        {
                            Eval_agent static_point;
                            static_point.position_ = pos.cast<float>();
                            static_point.velocity_ = Eigen::Vector3f::Zero();
                            static_point.radius_ = (float)protected_zone/3;
                            it->second.insertAgentNeighbor(static_point, communication_radius_float);
                        }
                        // std::cout << mykey << " obstacle_static " << static_point.position_.transpose() << std::endl;
                    }
                }
            }
        }
    }

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
    MarkerArray target_array;

    double rad_to_deg = 180.0 / M_PI;

    // Iterate through the agents
    for (auto &[key, agent] : agents_states)
    {
        rclcpp::Time now = this->get_clock()->now();
        switch (agent.flight_state)
        {
            case IDLE: case EMERGENCY:
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
                        (agent.previous_target - agent.transform.translation()); 
                else
                    vel_target = 
                        (agent.previous_target - agent.transform.translation()).normalized() * max_velocity;
                
                conduct_planning(vel_target, key, agent);
                
                VelocityWorld vel_msg;
                vel_msg.header.stamp = clock.now();
                vel_msg.vel.x = vel_target.x();
                vel_msg.vel.y = vel_target.y();
                vel_msg.vel.z = vel_target.z();
                vel_msg.height = agent.previous_target.z();
                vel_msg.yaw = agent.previous_yaw * rad_to_deg;
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
                    Eigen::Vector3d rpy = 
                        euler_rpy(agent.transform.linear());
                    agent.previous_yaw = rpy.z();
                    
                    agent_update_mutex.lock();
                    agent.target_queue.pop();
                    agent_update_mutex.unlock();
                }
                
                if (is_land)
                    agent.completed = true;

                break;
            }
            case MOVE_VELOCITY: case INTERNAL_TRACKING:
            {
                rclcpp::Time start = clock.now();

                if (agent.target_queue.empty())
                {
                    // move velocity
                    if (agent.flight_state == MOVE_VELOCITY)
                    {
                        agent.flight_state = HOVER;
                        agent.completed = true;
                    }
                    // internal tracking
                    else
                    {
                        auto it = agents_comm.find(key);
                        if (it == agents_comm.end())
                            continue;

                        send_land_and_update(agents_states.find(key), it);                        
                        agent.completed = true;
                    }
                    
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
                    Eigen::Vector3d rpy = 
                        euler_rpy(agent.transform.linear());
                    agent.previous_yaw = rpy.z();

                    agent_update_mutex.lock();
                    agent.target_queue.pop();
                    agent_update_mutex.unlock();

                    break;
                }
                else if (pose_difference < max_velocity)
                    vel_target = 
                        (agent.target_queue.front() - agent.transform.translation()); 
                else
                {
                    vel_target = 
                        (agent.target_queue.front() - agent.transform.translation()).normalized() * max_velocity;
                }

                conduct_planning(vel_target, key, agent);

                double duration_seconds = (clock.now() - start).seconds();
                RCLCPP_INFO(this->get_logger(), "go_to_velocity %s (%.3lf %.3lf %.3lf) time (%.3lfms)", 
                    key.c_str(), vel_target.x(), vel_target.y(), vel_target.z(), duration_seconds * 1000.0);

                vel_msg.header.stamp = clock.now();
                vel_msg.vel.x = vel_target.x();
                vel_msg.vel.y = vel_target.y();
                vel_msg.vel.z = vel_target.z();

                // check the difference in heading
                Eigen::Vector3d rpy = 
                    euler_rpy(agent.transform.linear());
                
                double yaw_target;
                if (agent.target_yaw - rpy.z() * rad_to_deg < -180.0)
                    yaw_target = agent.target_yaw - (rpy.z() * rad_to_deg - 360.0);
                else if (agent.target_yaw - rpy.z() * rad_to_deg > 180.0)
                    yaw_target = agent.target_yaw - (rpy.z() * rad_to_deg + 360.0);
                else
                    yaw_target = agent.target_yaw - rpy.z() * rad_to_deg;

                // std::cout << yaw_target << std::endl;
                double dir = yaw_target / std::abs(yaw_target);
                if (!std::isnan(dir))
                {
                    yaw_target = std::min(std::abs(yaw_target), maximum_yaw_change);                
                    yaw_target *= dir;
                    if (std::abs(yaw_target - agent.target_yaw) > maximum_yaw_change * 1.5)
                        yaw_target += rpy.z() * rad_to_deg;
                    else
                        yaw_target = agent.target_yaw;
                }
                else
                    yaw_target += rpy.z() * rad_to_deg;

                // std::cout << rpy.z() << "/" << wrap_pi(yaw_target) / rad_to_deg << std::endl;
                vel_msg.yaw = wrap_pi(yaw_target);
                // vel_msg.yaw = yaw_target;
                // vel_msg.yaw = agent.target_yaw;
                // vel_msg.yaw = 0.0;
                
                auto it = agents_comm.find(key);
                if (it != agents_comm.end())
                    it->second.vel_world_publisher->publish(vel_msg);
                break;
            }

            default:
                break;
            
        }

        std::string str_copy = key;
        // Remove cf from cfXX
        str_copy.erase(0,2);
        int id = std::stoi(str_copy);

        Marker target;
        target.header.frame_id = "/world";
        target.header.stamp = clock.now();
        target.type = visualization_msgs::msg::Marker::LINE_STRIP;
        target.id = id;
        target.action = visualization_msgs::msg::Marker::ADD;
        target.pose.orientation.x = 0.0;
        target.pose.orientation.y = 0.0;
        target.pose.orientation.z = 0.0;
        target.pose.orientation.w = 1.0;
        target.scale.x = 0.005;
        target.color.r = 0.5;
        target.color.g = 0.5;
        target.color.b = 1.0;
        target.color.a = 1.0;

        if(!agent.target_queue.empty())
        {
            // copy to prevent deleting the main target queue
            std::queue<Eigen::Vector3d> target_copy = agent.target_queue;

            geometry_msgs::msg::Point p;
            p.x = agent.transform.translation().x();
            p.y = agent.transform.translation().y();
            p.z = agent.transform.translation().z();
            target.points.push_back(p);

            while (!target_copy.empty())
            {
                geometry_msgs::msg::Point p;
                p.x = target_copy.front().x();
                p.y = target_copy.front().y();
                p.z = target_copy.front().z();
                target.points.push_back(p);

                target_copy.pop();
            }
        }

        AgentState agentstate;
        agentstate.id = key;
        agentstate.flight_state = agent.flight_state;
        agent.radio_connection = 
            ((clock.now() - agent.t).seconds() < radio_connection_timeout);
        agentstate.connected = agent.radio_connection;
        agentstate.completed = agent.completed;
        agentstate.mission_capable = agent.mission_capable;

        agents_feedback.agents.push_back(agentstate);
        target_array.markers.push_back(target);
    }

    // publish the flight state message
    agents_feedback.header.stamp = clock.now();
    agent_state_publisher->publish(agents_feedback);

    // publish the target data
    target_publisher->publish(target_array);
}