/*
* common.cpp
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

#include "common.h"

std::set<std::string> common::extract_names(
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

Eigen::Vector4f common::quat_to_vec4(Eigen::Quaterniond q)
{
    Eigen::Vector4f v;
    v << q.w(), q.x(), q.y(), q.z();
    return v;
}

Eigen::Quaterniond common::vec4_to_quat(Eigen::Vector4f v)
{
    return Eigen::Quaterniond(v[0], v[1], v[2], v[3]);
}

Eigen::Vector4f common::quaternion_average(std::vector<Eigen::Vector4f> quaternions)
{
    if (quaternions.size() == 0)
    {
        std::cerr << "Error trying to calculate the average quaternion of an empty set!\n";
        return Eigen::Vector4f::Zero();
    }

    // first build a 4x4 matrix which is the elementwise sum of the product of each quaternion with itself
    Eigen::Matrix4f A = Eigen::Matrix4f::Zero();

    for (int q=0; q<quaternions.size(); ++q)
        A += quaternions[q] * quaternions[q].transpose();

    // normalise with the number of quaternions
    A /= quaternions.size();

    // Compute the SVD of this 4x4 matrix
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::VectorXf singularValues = svd.singularValues();
    Eigen::MatrixXf U = svd.matrixU();

    // find the eigen vector corresponding to the largest eigen value
    int largestEigenValueIndex;
    float largestEigenValue;
    bool first = true;

    for (int i=0; i<singularValues.rows(); ++i)
    {
        if (first)
        {
            largestEigenValue = singularValues(i);
            largestEigenValueIndex = i;
            first = false;
        }
        else if (singularValues(i) > largestEigenValue)
        {
            largestEigenValue = singularValues(i);
            largestEigenValueIndex = i;
        }
    }

    Eigen::Vector4f average;
    average(0) = U(0, largestEigenValueIndex);
    average(1) = U(1, largestEigenValueIndex);
    average(2) = U(2, largestEigenValueIndex);
    average(3) = U(3, largestEigenValueIndex);

    return average;
}

Eigen::Vector3d common::euler_rpy(Eigen::Matrix3d R)
{
    Eigen::Vector3d euler_out;
    // Each vector is a row of the matrix
    Eigen::Vector3d m_el[3];
    m_el[0] = Eigen::Vector3d(R(0,0), R(0,1), R(0,2));
    m_el[1] = Eigen::Vector3d(R(1,0), R(1,1), R(1,2));
    m_el[2] = Eigen::Vector3d(R(2,0), R(2,1), R(2,2));

    // Check that pitch is not at a singularity
    if (std::abs(m_el[2].x()) >= 1)
    {
        euler_out.z() = 0;

        // From difference of angles formula
        double delta = std::atan2(m_el[2].y(),m_el[2].z());
        if (m_el[2].x() < 0)  //gimbal locked down
        {
            euler_out.y() = M_PI / 2.0;
            euler_out.x() = delta;
        }
        else // gimbal locked up
        {
            euler_out.y() = -M_PI / 2.0;
            euler_out.x() = delta;
        }
    }
    else
    {
        euler_out.y() = - std::asin(m_el[2].x());

        euler_out.x() = std::atan2(m_el[2].y()/std::cos(euler_out.y()), 
            m_el[2].z()/std::cos(euler_out.y()));

        euler_out.z() = std::atan2(m_el[1].x()/std::cos(euler_out.y()), 
            m_el[0].x()/std::cos(euler_out.y()));
    }

    return euler_out;
}

std::vector<std::string> common::split_space_delimiter(
        std::string str)
{
    std::stringstream ss(str);
    std::istream_iterator<std::string> begin(ss);
    std::istream_iterator<std::string> end;
    std::vector<std::string> vstrings(begin, end);
    return vstrings;
}

std::vector<visibility_graph::obstacle> common::generate_disjointed_wall(
    std::vector<Eigen::Vector2d> vertices, 
    std::pair<double, double> height_pair,
    double thickness)
{
    std::vector<visibility_graph::obstacle> obs_vector;

    double half_thickness = thickness / 2.0;

    if (vertices.size() < 2)
        return obs_vector;

    std::vector<Eigen::Vector2d> base_vertices;

    Eigen::Vector2d direction = 
        (vertices[1] - vertices[0]).normalized();
    Eigen::Vector2d perpendicular_direction = 
        Eigen::Vector2d(-direction.y(), direction.x());
    
    base_vertices.emplace_back(
        vertices[0] + perpendicular_direction * half_thickness);
    base_vertices.emplace_back(
        vertices[0] - perpendicular_direction * half_thickness);
    
    // condition that vertices.size() > 2
    if (vertices.size() > 2)
        for (size_t i = 1; i < vertices.size()-1; i++)
        {
            Eigen::Vector2d direction1 = 
                (vertices[i-1] - vertices[i]).normalized();
            Eigen::Vector2d direction2 = 
                (vertices[i+1] - vertices[i]).normalized();
            
            double dot = direction1.dot(direction2);
            double angle = std::acos(dot);
            
            double x2 = direction1.x() * std::cos(angle/2) - direction1.y() * std::sin(angle/2);
            double y2 = direction1.x() * std::sin(angle/2) + direction1.y() * std::cos(angle/2);
            
            Eigen::Vector2d direction3 = 
                (Eigen::Vector2d(x2, y2) - vertices[i]).normalized();
            
            double mag = half_thickness / std::sin(angle/2);

            base_vertices.emplace_back(vertices[i] + direction3 * mag);
            base_vertices.emplace_back(vertices[i] - direction3 * mag);
        }

    direction = 
        (vertices[vertices.size()-1] - vertices[vertices.size()-2]).normalized();
    perpendicular_direction = 
        Eigen::Vector2d(-direction.y(), direction.x());
    
    base_vertices.emplace_back(
        vertices[vertices.size()-1] + perpendicular_direction * half_thickness);
    base_vertices.emplace_back(
        vertices[vertices.size()-1] - perpendicular_direction * half_thickness);

    for (size_t i = 0; i < base_vertices.size()/2 - 1; i++)
    {
        visibility_graph::obstacle obs;
        std::vector<Eigen::Vector2d> unorganized_verts;
        std::vector<Eigen::Vector2d> organized_verts;
        Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
        unorganized_verts.emplace_back(base_vertices[i*2 + 0]);
        centroid += base_vertices[i*2 + 0];
        unorganized_verts.emplace_back(base_vertices[i*2 + 1]);
        centroid += base_vertices[i*2 + 1];
        unorganized_verts.emplace_back(base_vertices[i*2 + 2]);
        centroid += base_vertices[i*2 + 2];
        unorganized_verts.emplace_back(base_vertices[i*2 + 3]);
        centroid += base_vertices[i*2 + 3];

        centroid /= 4.0;

        visibility_graph::graham_scan(unorganized_verts, centroid, "ccw", organized_verts);

        obs.v = organized_verts;
        obs.h = height_pair;
        obs.c = centroid;
        obs_vector.emplace_back(obs);
    }

    return obs_vector;
}