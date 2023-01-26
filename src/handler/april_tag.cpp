#include "crazyswarm_app.h"

void cs2::cs2_application::tag_callback(
    const AprilTagDetectionArray::SharedPtr& msg, std::string name)
{
    AprilTagDetectionArray copy = *msg;

    auto iterator = agents_tag_queue.find(name);
    if (iterator == agents_tag_queue.end())
        return;
    // Push the detection in to the queue
    for (AprilTagDetection &d : copy.detections)
    {
        tag tmp;
        tmp.id = d.id;
        tmp.t = copy.header.stamp;
        tmp.transform.translation() = Eigen::Vector3d(
            d.pose.pose.position.x,
            d.pose.pose.position.y,
            d.pose.pose.position.z
        );

        Eigen::Quaterniond q;
        q.w() = d.pose.pose.orientation.w;
        q.x() = d.pose.pose.orientation.x;
        q.y() = d.pose.pose.orientation.y;
        q.z() = d.pose.pose.orientation.z;

        tmp.transform.linear() = q.toRotationMatrix();

        tmp.pixel_center.x() = d.centre.x;
        tmp.pixel_center.y() = d.centre.y;

        tag_queue_mutex.lock();
        iterator->second.t_queue.push(tmp);
        tag_queue_mutex.unlock();
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
        // Continue if there are no tags
        if (it->second.t_queue.empty())
            continue;

        // Continue if there is no agent to represent
        auto agent_it = agents_states.find(it->first);
        if (agent_it == agents_states.end())
            continue;

        bool tag_saved = false;
        tag save_tag;
        std::queue<Eigen::Affine3d> fused_pose_check;
        while(1)
        {
            std::queue<agent_state> copy = it->second.s_queue;
            // Processed all the tags in the queue
            if (it->second.t_queue.empty())
                break;
            
            auto it_eliminate = april_eliminate.find(it->second.t_queue.front().id);
            if (it_eliminate != april_eliminate.end())
            {
                save_tag = it->second.t_queue.front();
                tag_saved = true;
            }

            auto it_relocate = april_relocalize.find(it->second.t_queue.front().id);
            
            if (it_relocate != april_relocalize.end())
            {
                Eigen::Affine3d pose;
                if (handle_relocalize(copy, it->second.t_queue.front(), pose))
                    fused_pose_check.push(pose);
                else
                {
                    RCLCPP_ERROR(this->get_logger(), 
                        "agent %s cannot relocalize tag %d", 
                        it->first.c_str(), it->second.t_queue.front().id);
                    return;
                }
                
                RCLCPP_INFO(this->get_logger(), 
                    "agent %s handling tag %d", 
                    it->first.c_str(), it->second.t_queue.front().id);
            }
            
            it->second.t_queue.pop();
        }   

        // handle eliminate
        if (tag_saved)
            handle_eliminate(agent_it, save_tag);
        
        // handle relocalization
        if (!fused_pose_check.empty())
        {
            NamedPoseArray pose_correction;
            // get current time
            auto time = clock.now();
            pose_correction.header.stamp = time;
            // publish external pose correction
            pose_publisher->publish(pose_correction);
        }

        while(!it->second.s_queue.empty()) 
            it->second.s_queue.pop();
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
    if (t.transform.translation().norm() > eliminate_threshold)
        return;

    // tag not found
    auto tag_it = april_eliminate.find(t.id);
    if (tag_it == april_eliminate.end())
        return;

    s->second.flight_state = INTERNAL_TRACKING;

    // find the position of the tag and move towards it

    // erase the tag since we handled it
    april_eliminate.erase(tag_it);
}

bool cs2::cs2_application::handle_relocalize(
    std::queue<agent_state> &q, tag t, Eigen::Affine3d fused)
{
    double eps = pow(10, -5);
    double time_threshold_copy = time_threshold;
    Eigen::Affine3d selected_agent_transform; 

    while (!q.empty())
    {
        double time_difference = abs((q.front().t - t.t).seconds());
        // Accept if within a threshold
        if (time_difference < time_threshold_copy)
        {
            selected_agent_transform = q.front().transform;
            time_threshold_copy = time_difference;
        }
        q.pop();
    }

    // If there is no acceptable relocalization topic found
    if ((time_threshold_copy - time_threshold) < eps)
        return false;

    // selected_agent_transform * static_camera_transform * t.transform 
    Eigen::Affine3d tag_to_world = 
        selected_agent_transform * static_camera_transform * 
        t.transform;
    
    

    return true;
}