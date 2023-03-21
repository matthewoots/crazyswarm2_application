/*
* april_tag.cpp
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

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>

void cs2::cs2_application::tag_callback(
    const AprilTagDetectionArray::SharedPtr& msg, std::string name)
{
    AprilTagDetectionArray copy = *msg;

    auto iterator = agents_tag_queue.find(name);
    if (iterator == agents_tag_queue.end())
        return;

    auto agent_it = agents_states.find(name);
    if (agent_it == agents_states.end())
        return;
    
    if (copy.detections.empty())
        return;

    std::vector<tag> tag_vect;

    // Push the detection in to the queue
    for (AprilTagDetection &d : copy.detections)
    {
        if (Eigen::Vector3d(d.pose.pose.position.x,
            d.pose.pose.position.y, d.pose.pose.position.z).norm() > observation_threshold)
            continue;

        tag tmp;
        tmp.id = d.id;
        tmp.t = copy.header.stamp;
        tmp.transform.translation() = Eigen::Vector3d(
            d.pose.pose.position.x,
            d.pose.pose.position.y,
            d.pose.pose.position.z
        );
        // tmp.transform.translation() = Eigen::Vector3d(
        //     d.pose.pose.position.z,
        //     -d.pose.pose.position.x,
        //     -d.pose.pose.position.y
        // );

        Eigen::Quaterniond q;
        q.w() = d.pose.pose.orientation.w;
        q.x() = d.pose.pose.orientation.x;
        q.y() = d.pose.pose.orientation.y;
        q.z() = d.pose.pose.orientation.z;

        tmp.transform.linear() = q.toRotationMatrix();

        Eigen::Affine3d tag_to_body = 
            static_camera_transform * nwu_to_rdf * tmp.transform;
            
        tmp.transform = tag_to_body;

        tmp.pixel_center.x() = d.centre.x;
        tmp.pixel_center.y() = d.centre.y;

        tag_queue_mutex.lock();
        iterator->second.t_queue.push(tmp);
        tag_queue_mutex.unlock();

        tag_vect.emplace_back(tmp);
    }

    // RCLCPP_INFO(this->get_logger(), 
    //     "agent %s detected tag (size %ld)", name.c_str(), iterator->second.t_queue.size());

    for (auto &single : tag_vect)
    {
        TransformStamped msg2;
        msg2.header.frame_id = name;
        msg2.header.stamp = clock.now();
        msg2.child_frame_id = "april" + std::to_string(single.id);
        
        Eigen::Quaterniond q_tb(single.transform.linear());
        
        // tag_to_body
        msg2.transform.translation.x = single.transform.translation().x();
        msg2.transform.translation.y = single.transform.translation().y();
        msg2.transform.translation.z = single.transform.translation().z();
        msg2.transform.rotation.x = q_tb.x();
        msg2.transform.rotation.y = q_tb.y();
        msg2.transform.rotation.z = q_tb.z();
        msg2.transform.rotation.w = q_tb.w();
        tf2_bc.sendTransform(msg2);

        // Eigen::Affine3d tag_to_world = 
        //     agent_it->second.transform * single.transform;
        // Eigen::Quaterniond q_tw(tag_to_world.linear());

        // msg2.header.frame_id = "/world";
        // msg2.child_frame_id = "april_world" + std::to_string(single.id);
        // msg2.transform.translation.x = tag_to_world.translation().x();
        // msg2.transform.translation.y = tag_to_world.translation().y();
        // msg2.transform.translation.z = tag_to_world.translation().z();
        // msg2.transform.rotation.x = q_tw.x();
        // msg2.transform.rotation.y = q_tw.y();
        // msg2.transform.rotation.z = q_tw.z();
        // msg2.transform.rotation.w = q_tw.w();
        // tf2_bc.sendTransform(msg2);
    }
}

void cs2::cs2_application::tag_timer_callback()
{
    std::map<std::string, tag_queue>::iterator it;
    // We need to handle both task elements and relocalization
    // Iterate through the agents

    for (it = agents_tag_queue.begin(); 
        it != agents_tag_queue.end(); it++)
    {
        rclcpp::Time tag_start = clock.now();

        // Continue if there are no tags
        if (it->second.t_queue.empty())
            continue;

        // Continue if there is no agent to represent
        auto agent_it = agents_states.find(it->first);
        if (agent_it == agents_states.end())
            continue;

        bool tag_saved = false;
        bool trigger_localize = false;
        tag save_tag;

        // Continue if there is no agent to represent
        auto fact_it = agents_loop_closure.find(it->first);
        if (fact_it == agents_loop_closure.end())
            continue;

        while(1)
        {
            std::queue<agent_state> copy = it->second.s_queue;
            // Processed all the tags in the queue
            if (it->second.t_queue.empty())
                break;
            
            auto it_eliminate = april_eliminate.find(it->second.t_queue.front().id);
            if (it_eliminate != april_eliminate.end() && 
                !tag_saved && agent_it->second.flight_state != INTERNAL_TRACKING)
            {
                save_tag = it->second.t_queue.front();
                tag_saved = true;
            }

            auto it_relocate = april_relocalize.find(it->second.t_queue.front().id);
            
            if (it_relocate != april_relocalize.end())
            {
                if (handle_relocalize(copy, it->second.t_queue.front(), 
                    it_relocate, it->first))
                {
                    trigger_localize = true;
                    // RCLCPP_INFO(this->get_logger(), 
                    //     "agent %s handling tag %d", 
                    //     it->first.c_str(), it->second.t_queue.front().id);
                }
                else
                {
                    // RCLCPP_ERROR(this->get_logger(), 
                    //     "agent %s cannot relocalize tag %d", 
                    //     it->first.c_str(), it->second.t_queue.front().id);
                }
            }
            
            it->second.t_queue.pop();
        }   

        // handle eliminate
        if (tag_saved)
        {
            handle_eliminate(agent_it, save_tag);
        }
        
        NamedPoseArray pose_correction;
        
        // handle relocalization
        if (trigger_localize && 
            fact_it->second.observations.size() > observation_limit)
        {
            Eigen::Affine3d pose_opt;
            gtsam_pose_optimization(pose_opt, fact_it, agent_it);

            // get current time
            auto time = clock.now();
            pose_correction.header.stamp = time;

            Eigen::Vector3d trans = pose_opt.translation();
            Eigen::Quaterniond quat(pose_opt.linear());

            TransformStamped msg2;
            msg2.header.frame_id = "/world";
            msg2.header.stamp = time;
            msg2.child_frame_id = "slam" + it->first;
            msg2.transform.translation.x = trans.x();
            msg2.transform.translation.y = trans.y();
            msg2.transform.translation.z = trans.z();
            msg2.transform.rotation.w = quat.w();
            msg2.transform.rotation.x = quat.x();
            msg2.transform.rotation.y = quat.y();
            msg2.transform.rotation.z = quat.z();
            tf2_bc.sendTransform(msg2);

            // clear previous observations if over limit
            fact_it->second.observations.clear();

            NamedPose pose;
            pose.name = it->first;
            pose.pose.position.x = trans.x();
            pose.pose.position.y = trans.y();
            pose.pose.position.z = trans.z();

            pose.pose.orientation.x = quat.x();
            pose.pose.orientation.y = quat.y();
            pose.pose.orientation.z = quat.z();
            pose.pose.orientation.w = quat.w();

            double magnitude_1 = std::pow(10,-1);
            double magnitude_2 = std::pow(10,-2);
            double distance = trans.norm();
            if (distance > observation_limit)
                continue;
            
            pose.sd_position = 
                1/(observation_limit - distance) * magnitude_1;
            pose.sd_quaternion = 
                1/(observation_limit - distance) * magnitude_2;

            pose_correction.poses.emplace_back(pose);
        }

        // publish external pose correction
        if (!pose_correction.poses.empty())
            pose_publisher->publish(pose_correction);

        // Clear the state queue so that state subscriber can update
        while(!it->second.s_queue.empty()) 
            it->second.s_queue.pop();
        
        RCLCPP_INFO(this->get_logger(), 
            "agent %s tag_handle_time %.3lfms", it->first.c_str(), 
            (clock.now() - tag_start).seconds() * 1000);   
    }
    
}

void cs2::cs2_application::handle_eliminate(
    std::map<std::string, agent_state>::iterator s, tag t)
{
    RCLCPP_INFO(this->get_logger(), 
        "agent %s handle eliminate for tag %d", s->first.c_str(), t.id);   

    // if agent state is IDLE TAKEOFF or LAND, return
    if (s->second.flight_state == TAKEOFF || 
        s->second.flight_state == IDLE ||
        s->second.flight_state == LAND)
        return;
    
    // distance rejection
    if (t.transform.translation().norm() > observation_threshold)
        return;

    // tag not found
    auto tag_it = april_eliminate.find(t.id);
    if (tag_it == april_eliminate.end())
        return;

    s->second.flight_state = INTERNAL_TRACKING;

    // find the position of the tag and move towards it
    while (!s->second.target_queue.empty()) 
        s->second.target_queue.pop();
    // Eigen::Affine3d tag_to_world = s->second.transform * 
    //     static_camera_transform * t.transform;
    
    // world -> body body -> tag 
    Eigen::Affine3d tag_to_world = 
        s->second.transform * t.transform;
    
    s->second.target_queue.push(Eigen::Vector3d(
        tag_to_world.translation().x(),
        tag_to_world.translation().y(),
        s->second.transform.translation().z()/1.5));

    april_found.insert({t.id, 
        Eigen::Vector2d(tag_to_world.translation().x(),
        tag_to_world.translation().y())});

    std::time_t ttime = time(0);
    tm *local_time = localtime(&ttime);

    CSVWriter csv;
    csv.newRow() << 
        1900 + local_time->tm_year <<
        1 + local_time->tm_mon <<
        local_time->tm_mday <<
        1 + local_time->tm_hour <<
        1 + local_time->tm_min <<
        1 + local_time->tm_sec <<
        (int)(t.id) << 
        tag_to_world.translation().x() << 
        tag_to_world.translation().y();
    csv.writeToFile(log_path, true);

    // erase the tag since we handled it
    april_eliminate.erase(tag_it);
}

bool cs2::cs2_application::handle_relocalize(
    std::queue<agent_state> &q, tag t, 
    std::map<int, Eigen::Vector2d>::iterator tag_pose,
    std::string agent_id)
{
    double eps = pow(10, -5);
    double time_threshold_copy = time_threshold;
    agent_state selected_agent_state; 

    while (!q.empty())
    {
        double time_difference = abs((q.front().t - t.t).seconds());
        // RCLCPP_INFO(this->get_logger(), 
        //     "time_difference tag %d (%.3lfs/%.3lfs)", t.id, time_difference, time_threshold_copy);
        
        // Accept if within a threshold
        if (time_difference < time_threshold_copy)
        {
            selected_agent_state = q.front();
            time_threshold_copy = time_difference;
        }
        q.pop();
    }

    // If there is no acceptable relocalization topic found
    if (abs(time_threshold_copy - time_threshold) < eps)
        return false;

    auto loop_it = agents_loop_closure.find(agent_id);
    if (loop_it == agents_loop_closure.end())
        return false;

    bool found = false;
    // milliseconds
    long milli_time = 
        static_cast<long>(std::round((t.t - start_node_time).seconds() * 1000));

    // iterate through to find any observations that have around the same time stamp
    for (auto it = loop_it->second.observations.begin(); 
        it != loop_it->second.observations.end(); it++)
    {
        // if time stamp is found within threshold, add in this marker
        if (abs((it->first - milli_time)) < time_threshold/2)
        {
            it->second.marker.push(t);
            found = true;
            break;
        } 
    }

    // if a marker is not found, add in a new observation
    if (!found)
    {
        observation obs;
        obs.marker.push(t);
        obs.pose = selected_agent_state.transformEigen2Gtsam();

        loop_it->second.observations.insert({milli_time, obs});
    }

    // selected_agent_state.transform * static_camera_transform * t.transform 
    // Eigen::Affine3d tag_to_world = 
    //     selected_agent_state.transform * static_camera_transform * 
    //     t.transform;

    // Eigen::Vector3d trans = tag_to_world.translation();
            
    // RCLCPP_INFO(this->get_logger(), 
    //     "(tag_to_world) tag %d (%.3lf, %.3lf, %.3lf) (%.3lf, %.3lf)", 
    //     tag_pose->first, trans.x(), trans.y(), trans.z(),
    //     tag_pose->second.x(), tag_pose->second.y());

    return true;
}

void cs2::cs2_application::gtsam_pose_optimization(
    Eigen::Affine3d &pose, 
    std::map<std::string, factor_graph>::iterator fact_it,
    std::map<std::string, agent_state>::iterator agent_it)
{
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    
    // factor graph: X state, Z are measurements
    // X1 -> X2 -> X3
    // |     |     |
    // v     v     v
    // Z1    Z2    Z3

    double rSigma = 0.03;
    double tSigma = 0.02;

    auto noise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector6::Constant(rSigma));

    gtsam::Pose3 previous_pose;

    uint32_t id = 0;

    for (auto it = fact_it->second.observations.begin(); 
        it != fact_it->second.observations.end(); it++)
    {
        uint8_t idx = std::distance(
            fact_it->second.observations.begin(), it);

        bool update_x = false;

        gtsam::Pose3 wtb;

        // Eigen::Affine3d agent_to_world;
        // agent_to_world.translation() = 
        //     Eigen::Vector3d(it->second.pose.x(),
        //     it->second.pose.y(), it->second.pose.z());

        // agent_to_world.linear() = it->second.pose.rotation().matrix();

        std::queue<tag> copy = it->second.marker;

        if (copy.empty())
            continue;
        
        while (!copy.empty())
        {
            // uint8_t id = copy.front().id;

            // RCLCPP_INFO(this->get_logger(), 
            //     "tag_id %d state_id %d", id, idx);

            auto it_reloc = 
                april_relocalize.find(copy.front().id);

            Eigen::Affine3d world_to_tag = nwu_to_enu;
            world_to_tag.translation() = Eigen::Vector3d(
                it_reloc->second.x(), it_reloc->second.y(), 0.0);

            // estimated pose - back track from camera absolute pos
            if (!update_x)
            {
                // world -> tag tag -> camera camera -> body
                Eigen::Affine3d world_to_body = world_to_tag * 
                    copy.front().transform.inverse() * static_camera_transform.inverse();
                
                tag world_to_body_tag;
                world_to_body_tag.transform = world_to_body;
                wtb = world_to_body_tag.transformEigen2Gtsam();

                initial.insert(X(idx), wtb);
                update_x = true;
            }            

            gtsam::Pose3 prior(
                gtsam::Rot3(nwu_to_enu.linear()), 
                gtsam::Point3{it_reloc->second.x(), it_reloc->second.y(), 0.0});

            graph.addPrior(Z(id), prior, noise);

            // tag to body
            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                X(idx), Z(id), copy.front().transformEigen2Gtsam(), noise));

            initial.insert(Z(id), prior);

            copy.pop();

            id++;
        }

        // since it is the beginning of the vector, do not add the between factor
        if (it == fact_it->second.observations.begin())
        {
            previous_pose = wtb;
            continue;
        }
        
        // z(i-1) -> z(i)
        gtsam::Pose3 relative_pose = 
            previous_pose.inverse() * wtb;

        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
            X(idx-1), X(idx), relative_pose, noise));
        
        previous_pose = wtb;
    }

    gtsam::LevenbergMarquardtParams params;

    auto optimizer =
        gtsam::LevenbergMarquardtOptimizer(graph, initial, params);

    // optimizeSafely is inherited
    auto result = optimizer.optimizeSafely();

    std::vector<Eigen::Vector4f> quaternions_error_vector;
    size_t div = fact_it->second.observations.size();
    Eigen::Vector3d translation_error_average = 
        Eigen::Vector3d::Zero();
    for (auto it = fact_it->second.observations.begin(); 
        it != fact_it->second.observations.end(); it++)
    {
        uint8_t idx = std::distance(
            fact_it->second.observations.begin(), it);
        
        gtsam::Pose3 opt_pose = result.at<gtsam::Pose3>(X(idx));
        translation_error_average += Eigen::Vector3d(
            opt_pose.x() - it->second.pose.x(), 
            opt_pose.y() - it->second.pose.y(),
            opt_pose.z() - it->second.pose.z());

        Eigen::Quaterniond q_curr(it->second.pose.rotation().matrix());
        Eigen::Quaterniond q_opt(opt_pose.rotation().matrix());

        // get the difference/error in quaternions
        Eigen::Quaterniond q_diff = q_opt * q_curr.inverse();

        quaternions_error_vector.emplace_back(quat_to_vec4(q_diff));
    }

    Eigen::Vector4f quat = 
        quaternion_average(quaternions_error_vector);
    translation_error_average /= (double)div;

    Eigen::Quaterniond q(agent_it->second.transform.linear());
    
    pose.translation() = translation_error_average + agent_it->second.transform.translation();
    pose.linear() = (vec4_to_quat(quat) * q).toRotationMatrix();
}