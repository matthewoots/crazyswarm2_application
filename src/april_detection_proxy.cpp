/*
* april_detection_proxy.cpp
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

#include <memory>
#include <vector>
#include <regex>
#include <mutex>
#include <queue>
#include <string>

#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "apriltag_msgs/msg/april_tag_detection.hpp" 

#include "rosgraph_msgs/msg/clock.hpp"

#include "rclcpp/rclcpp.hpp"

#include "common.h"

#include <Eigen/Dense>

using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Point;
using apriltag_msgs::msg::AprilTagDetection;
using apriltag_msgs::msg::AprilTagDetectionArray;
using rosgraph_msgs::msg::Clock;

using namespace std::chrono_literals;

using namespace common;

using std::placeholders::_1;

#define deg_to_rad 0.0174533

class AprilDectectionProxy : public rclcpp::Node
{
    private:
        struct fov
        {
            double v; // vertical radians
            double h; // horizontal radians
            double clamp_distance;
        };

        struct camera_frame
        {
            Eigen::Affine3d pose;
            std::vector<Eigen::Vector3d> poly;
        };

        rclcpp::Clock clock;

        rclcpp::Time sim_time;

        std::map<std::string, rclcpp::Subscription<PoseStamped>::SharedPtr> pose_sub;
        
        std::map<std::string, rclcpp::Publisher<AprilTagDetectionArray>::SharedPtr> tag_pub;

        std::map<std::string, camera_frame> agent_camera_frame;

        Eigen::Affine3d camera_rotation_mat;

        rclcpp::Publisher<MarkerArray>::SharedPtr camera_fov_publisher;

        rclcpp::Subscription<Clock>::SharedPtr clock_sub;

        std::map<std::string, tag> april_tags;

        rclcpp::TimerBase::SharedPtr camera_frame_timer;

        fov field_of_view;

        Eigen::Vector3d extruded_lines[4];

        Eigen::Affine3d nwu_to_rdf;
        Eigen::Affine3d enu_to_rdf;

        Eigen::Vector3d rotate_yaw(double y, Eigen::Vector3d v)
        {
            Eigen::Matrix3d yaw;
            yaw << std::cos(y), -std::sin(y), 0,
                std::sin(y), std::cos(y), 0,
                0, 0, 1;

            return yaw * v;
        }

        bool line_plane_intersection(
            Eigen::Vector3d& contact, Eigen::Vector3d ray, Eigen::Vector3d rayOrigin, 
            Eigen::Vector3d normal, Eigen::Vector3d coord) 
        {
            // https://stackoverflow.com/questions/7168484/3d-line-segment-and-plane-intersection
            // get d value
            float d = normal.dot(coord);
            if (normal.dot(ray) == 0) {
                return false; // No intersection, the line is parallel to the plane
            }
            if (normal.dot(ray) > 0) {
                return false; // No intersection, since line is moving away from plane
            }
            // Compute the X value for the directed line ray intersecting the plane
            float x = (d - normal.dot(rayOrigin)) / normal.dot(ray);
            // output contact point
            contact = rayOrigin + ray.normalized()*x; //Make sure your ray vector is normalized
            return true;
        }

        Eigen::Vector3d rotate_vect_rpy(double r, double p, double y)
        {
            Eigen::Quaterniond q;
            q = Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())
                * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ());
            
            Eigen::Quaterniond point;
            point.w() = 0;
            point.vec() = Eigen::Vector3d(1, 0, 0);
            Eigen::Quaterniond rotatedP = q * point * q.inverse(); 

            return rotatedP.vec();
        }

        bool point_in_polygon(Eigen::Vector2d p, 
            std::vector<Eigen::Vector3d> &verts)
        {
            // https://wrf.ecse.rpi.edu/Research/Short_Notes/pnpoly.html
            bool inside = false;
            for (int i = 0, j = verts.size() - 1; i < verts.size(); j = i++)
            {
                if ((verts[i].y() > p.y()) != (verts[j].y() > p.y()) &&
                    p.x() < ((verts[j].x() - verts[i].x()) * (p.y() - verts[i].y()) / (verts[j].y() - verts[i].y()) + 
                    verts[i].x()))
                    inside = !inside;
            }
            return inside;
        }

    public:

        AprilDectectionProxy()
        : Node("april_dectection_proxy"), clock(RCL_ROS_TIME)
        {
            this->declare_parameter("april_tag_parameters.camera_rotation", std::vector<double>{});
            this->declare_parameter("sim.hfov", -1.0);
            this->declare_parameter("sim.vfov", -1.0);
            this->declare_parameter("sim.observation.clamp_distance", -1.0);

            std::vector<double> camera_rotation = 
                this->get_parameter("april_tag_parameters.camera_rotation").get_parameter_value().get<std::vector<double>>();
            field_of_view.h = 
                this->get_parameter("sim.hfov").get_parameter_value().get<double>() 
                * deg_to_rad;
            field_of_view.v = 
                this->get_parameter("sim.vfov").get_parameter_value().get<double>() 
                * deg_to_rad;
            field_of_view.clamp_distance = 
                this->get_parameter("sim.observation.clamp_distance").get_parameter_value().get<double>();

            // load crazyflies from params
            auto node_parameters_iface = this->get_node_parameters_interface();
            const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides =
                node_parameters_iface->get_parameter_overrides();

            auto cf_names = extract_names(parameter_overrides, "robots");
            for (const auto &name : cf_names) 
            {
                agent_camera_frame.insert({name, camera_frame()});

                std::function<void(const PoseStamped::SharedPtr)> pcallback = 
                    std::bind(&AprilDectectionProxy::pose_callback,
                    this, std::placeholders::_1, --agent_camera_frame.end());
                pose_sub.insert({name, this->create_subscription<PoseStamped>(
                    name + "/pose", 7, pcallback)});
                
                tag_pub.insert({name, this->create_publisher<AprilTagDetectionArray>(
                    name + "/tag", 7)});
            }

            // Eigen::Vector3d ypr = Eigen::Quaterniond(camera_rotation[3], camera_rotation[0], 
            //     camera_rotation[1], camera_rotation[2]).
            //     toRotationMatrix().eulerAngles(2, 1, 0);

            Eigen::Quaterniond camera_rotation_q = 
                Eigen::Quaterniond(camera_rotation[3], camera_rotation[0], 
                camera_rotation[1], camera_rotation[2]);

            camera_rotation_mat = Eigen::Affine3d::Identity();

            camera_rotation_mat.linear() = 
                camera_rotation_q.toRotationMatrix();
            
            Eigen::Vector3d rpy = euler_rpy(
                camera_rotation_mat.linear());

            std::cout << "rpy: " << rpy.transpose() << std::endl;

            // 0 --> 1
            //       |
            //       v
            // 3 <-- 2
            // Using NWU
            // top left (pitch up yaw left) 
            extruded_lines[0] = rotate_vect_rpy(
                0.0, rpy[1] - field_of_view.v/2.0, rpy[2] + field_of_view.h/2.0);

            // top right (pitch up yaw right)
            extruded_lines[1] = rotate_vect_rpy(
                0.0, rpy[1] - field_of_view.v/2.0, rpy[2] - field_of_view.h/2.0);

            // bottom right (pitch down yaw right)
            extruded_lines[2] = rotate_vect_rpy(
                0.0, rpy[1] + field_of_view.v/2.0, rpy[2] - field_of_view.h/2.0);
            
            // bottom left (pitch down yaw left)
            extruded_lines[3] = rotate_vect_rpy(
                0.0, rpy[1] + field_of_view.v/2.0, rpy[2] + field_of_view.h/2.0);

            std::cout << "extruded_lines0: " << 
                extruded_lines[0].transpose() << " - " << rpy[1] - field_of_view.v/2.0 << " " <<
                rpy[2] + field_of_view.h/2.0 << std::endl;
            std::cout << "extruded_lines1: " << 
                extruded_lines[1].transpose() << " - " << rpy[1] - field_of_view.v/2.0 << " " <<
                rpy[2] - field_of_view.h/2.0 << std::endl;
            std::cout << "extruded_lines2: " << 
                extruded_lines[2].transpose() << " - " << rpy[1] + field_of_view.v/2.0 << " " <<
                rpy[2] - field_of_view.h/2.0 << std::endl;
            std::cout << "extruded_lines3: " << 
                extruded_lines[3].transpose() << " - " << rpy[1] + field_of_view.v/2.0 << " " <<
                rpy[2] + field_of_view.h/2.0 << std::endl;

            camera_fov_publisher = 
                this->create_publisher<MarkerArray>("rviz/fov", 10);

            load_april_tags(parameter_overrides, april_tags, 0.0, true);

            camera_frame_timer = this->create_wall_timer(
                50ms, std::bind(&AprilDectectionProxy::camera_timer_callback, this));
        
            clock_sub = 
                this->create_subscription<Clock>("/clock", 1, std::bind(
                &AprilDectectionProxy::clock_callback, this, _1));

            // rotate z -90 then x -90 for it to be RDF
            nwu_to_rdf = enu_to_rdf = Eigen::Affine3d::Identity();
            nwu_to_rdf.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d(0,0,1)));
            
            enu_to_rdf.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d(1,0,0)));
            nwu_to_rdf.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d(1,0,0)));
        }

        void clock_callback(const Clock::SharedPtr msg)
        {
            // get sim_clock value
            sim_time = 
                rclcpp::Time(msg->clock.sec, msg->clock.nanosec);
        }

        void pose_callback(const PoseStamped::SharedPtr msg, 
            std::map<std::string, camera_frame>::iterator it)
        {
            Eigen::Affine3d transform;
            PoseStamped copy = *msg;
            transform.translation() = 
                Eigen::Vector3d(copy.pose.position.x, 
                copy.pose.position.y, 
                copy.pose.position.z);
            Eigen::Quaterniond q = Eigen::Quaterniond(
                copy.pose.orientation.w, copy.pose.orientation.x,
                copy.pose.orientation.y, copy.pose.orientation.z);
            
            transform.linear() = q.toRotationMatrix();

            Eigen::Vector3d euler = euler_rpy(transform.linear());
            // RCLCPP_INFO(this->get_logger(), "rpy(%.3lf, %.3lf, %.3lf)", 
            //     euler.x(), euler.y(), euler.z());

            it->second.pose = transform;
            it->second.poly.clear();

            // iterate through the 4 extruded lines
            for (size_t i = 0; i < 4; i++)
            {
                Eigen::Vector3d contact = Eigen::Vector3d::Zero();
                Eigen::Vector3d rot_ext_lines = 
                    rotate_yaw(euler.z(), extruded_lines[i]);

                if (line_plane_intersection(
                    contact, rot_ext_lines, transform.translation() + Eigen::Vector3d(0.0, 0.0, 0.05), 
                    Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(0.0, 0.0, 0.0)))
                {
                    Eigen::Vector2d vect = Eigen::Vector2d(contact.x(), contact.y()) - 
                        Eigen::Vector2d(transform.translation().x(), transform.translation().y());
                    if (vect.norm() > field_of_view.clamp_distance)
                    {
                        Eigen::Vector2d new_contact_plane = vect.normalized() * field_of_view.clamp_distance + 
                            Eigen::Vector2d(transform.translation().x(), transform.translation().y());
                        contact.x() = new_contact_plane.x();
                        contact.y() = new_contact_plane.y();
                    }
                }
                else
                {
                    Eigen::Vector2d vect = Eigen::Vector2d(rot_ext_lines.x(), rot_ext_lines.y()); 
                    // point is definitely out of the boundary hence we clamp it
                    Eigen::Vector2d new_contact_plane = vect.normalized() * field_of_view.clamp_distance + 
                        Eigen::Vector2d(transform.translation().x(), transform.translation().y());
                    contact.x() = new_contact_plane.x();
                    contact.y() = new_contact_plane.y();
                }
                
                // RCLCPP_INFO(this->get_logger(), "%ld contact(%.3lf, %.3lf, %.3lf)", 
                //     i, contact.x(), contact.y(), contact.z());
                it->second.poly.emplace_back(contact);
            }
        }

        void camera_timer_callback()
        {
            MarkerArray camera_fov;

            for (auto &[name, camera] : agent_camera_frame)
            {
                std::string str_copy = name;
                // Remove cf from cfXX
                str_copy.erase(0,2);
                int id = std::stoi(str_copy);

                Marker camera_frame_marker;
                camera_frame_marker.header.frame_id = "/world";
                camera_frame_marker.header.stamp = clock.now();
                camera_frame_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
                camera_frame_marker.id = id;
                camera_frame_marker.action = visualization_msgs::msg::Marker::ADD;
                camera_frame_marker.pose.orientation.x = 0.0;
                camera_frame_marker.pose.orientation.y = 0.0;
                camera_frame_marker.pose.orientation.z = 0.0;
                camera_frame_marker.pose.orientation.w = 1.0;
                camera_frame_marker.scale.x = 0.005;
                camera_frame_marker.color.r = 1.0;
                camera_frame_marker.color.g = 1.0;
                camera_frame_marker.color.b = 1.0;
                camera_frame_marker.color.a = 1.0;

                if (camera.poly.empty())
                    continue;
                
                for (int i = 0; i < (int)camera.poly.size(); i++)
                {
                    int next, current = i;
                    if (i+1 >= (int)camera.poly.size())
                        next = 0;
                    else
                        next = i+1;

                    geometry_msgs::msg::Point p;
                    p.x = camera.poly[current].x();
                    p.y = camera.poly[current].y();
                    p.z = camera.poly[current].z();
                    camera_frame_marker.points.push_back(p);
                    p.x = camera.poly[next].x();
                    p.y = camera.poly[next].y();
                    p.z = camera.poly[next].z();
                    camera_frame_marker.points.push_back(p);
                }

                for (int i = 0; i < (int)camera.poly.size(); i++)
                {
                    geometry_msgs::msg::Point p;
                    p.x = camera.poly[i].x();
                    p.y = camera.poly[i].y();
                    p.z = camera.poly[i].z();
                    camera_frame_marker.points.push_back(p);
                    p.x = camera.pose.translation().x();
                    p.y = camera.pose.translation().y();
                    p.z = camera.pose.translation().z();
                    camera_frame_marker.points.push_back(p);
                }

                camera_fov.markers.push_back(camera_frame_marker);
            
                std::vector<Eigen::Affine3d> transforms;
                AprilTagDetectionArray tag_detection;

                // since this is a simulation, there is a need to convert to sim_time hence we can take 
                // tag_detection.header.stamp = sim_time;
                tag_detection.header.stamp = clock.now();

                // check for tags detected
                for (auto &[str_id, tag] : april_tags)
                    if (point_in_polygon(Eigen::Vector2d(
                        tag.transform.translation().x(),
                        tag.transform.translation().y()), camera.poly))
                    {
                        AprilTagDetection detection;

                        // create the transformation of the april tag
                        Eigen::Affine3d tag_transform = Eigen::Affine3d::Identity();
                        tag_transform.translation() = tag.transform.translation();

                        // cam -> body * body -> world * world -> tag
                        Eigen::Affine3d rel_transform = 
                            nwu_to_rdf.inverse() * camera_rotation_mat.inverse() * camera.pose.inverse() * tag_transform;

                        Eigen::Vector3d saved_translation = rel_transform.translation();

                        rel_transform.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d(0,0,1)));

                        rel_transform.translation() = saved_translation;

                        // Eigen::Affine3d rel_transform = camera.pose.inverse() * tag_transform;
                        // pose should be relative
                        transforms.emplace_back(rel_transform);

                        detection.family = "36h11";
                        detection.id = tag.id;

                        Eigen::Quaterniond q(rel_transform.linear());
                        detection.pose.pose.orientation.x = q.x();
                        detection.pose.pose.orientation.y = q.y();
                        detection.pose.pose.orientation.z = q.z();
                        detection.pose.pose.orientation.w = q.w();
                        detection.pose.pose.position.x = rel_transform.translation().x();
                        detection.pose.pose.position.y = rel_transform.translation().y();
                        detection.pose.pose.position.z = rel_transform.translation().z();
                        tag_detection.detections.push_back(detection);
                    }

                auto it = tag_pub.find(name);
                if (it != tag_pub.end() && !tag_detection.detections.empty())
                    it->second->publish(tag_detection);                
            }

            camera_fov_publisher->publish(camera_fov);
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AprilDectectionProxy>());
    rclcpp::shutdown();
    return 0;
}