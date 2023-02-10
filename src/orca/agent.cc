/*
 * agent.cc (modified)
 * RVO2-3D Library
 *
 * SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <https://gamma.cs.unc.edu/RVO2/>
 */

#include "agent.h"

namespace RVO 
{

  /**
   * @brief     Solves a one-dimensional linear program on a specified line
   *            subject to linear constraints defined by planes and a spherical
   *            constraint.
   * @param[in] planes       Planes defining the linear constraints.
   * @param[in] planeNo      The plane on which the line lies.
   * @param[in] line         The line on which the one-dimensional linear program
   *                         is solved.
   * @param[in] radius       The radius of the spherical constraint.
   * @param[in] optVelocity  The optimization velocity.
   * @param[in] directionOpt True if the direction should be optimized.
   * @param[in] result       A reference to the result of the linear program.
   * @return True if successful.
   */
  bool Agent::linearProgram1(const std::vector<Plane> &planes, std::size_t planeNo,
                      const Line &line, float radius, const Eigen::Vector3f &optVelocity,
                      bool directionOpt,
                      Eigen::Vector3f &result) { /* NOLINT(runtime/references) */
    const float dotProduct = line.point.dot(line.direction);
    const float discriminant =
        dotProduct * dotProduct + radius * radius - line.point.dot(line.point);

    if (discriminant < 0.0F) {
      /* Max speed sphere fully invalidates line. */
      return false;
    }

    const float sqrtDiscriminant = std::sqrt(discriminant);
    float tLeft = -dotProduct - sqrtDiscriminant;
    float tRight = -dotProduct + sqrtDiscriminant;

    for (std::size_t i = 0U; i < planeNo; ++i) {
      const float numerator = (planes[i].point - line.point).dot(planes[i].normal);
      const float denominator = line.direction.dot(planes[i].normal);

      if (denominator * denominator <= RVO3D_EPSILON) {
        /* Lines line is (almost) parallel to plane i. */
        if (numerator > 0.0F) {
          return false;
        }

        continue;
      }

      const float t = numerator / denominator;

      if (denominator >= 0.0F) {
        /* Plane i bounds line on the left. */
        tLeft = std::max(tLeft, t);
      } else {
        /* Plane i bounds line on the right. */
        tRight = std::min(tRight, t);
      }

      if (tLeft > tRight) {
        return false;
      }
    }

    if (directionOpt) {
      /* Optimize direction. */
      if (optVelocity.dot(line.direction) > 0.0F) {
        /* Take right extreme. */
        result = line.point + tRight * line.direction;
      } else {
        /* Take left extreme. */
        result = line.point + tLeft * line.direction;
      }
    } else {
      /* Optimize closest point. */
      const float t = line.direction.dot(optVelocity - line.point);

      if (t < tLeft) {
        result = line.point + tLeft * line.direction;
      } else if (t > tRight) {
        result = line.point + tRight * line.direction;
      } else {
        result = line.point + t * line.direction;
      }
    }

    return true;
  }

  /**
   * @brief      Solves a two-dimensional linear program on a specified plane
   *             subject to linear constraints defined by planes and a spherical
   *             constraint.
   * @param[in]  planes       Planes defining the linear constraints.
   * @param[in]  planeNo      The plane on which the two-dimensional linear
   *                          program is solved.
   * @param[in]  radius       The radius of the spherical constraint.
   * @param[in]  optVelocity  The optimization velocity.
   * @param[in]  directionOpt True if the direction should be optimized.
   * @param[out] result       A reference to the result of the linear program.
   * @return     True if successful.
   */
  bool Agent::linearProgram2(const std::vector<Plane> &planes, std::size_t planeNo,
                      float radius, const Eigen::Vector3f &optVelocity, bool directionOpt,
                      Eigen::Vector3f &result) { /* NOLINT(runtime/references) */
    const float planeDist = planes[planeNo].point.dot(planes[planeNo].normal);
    const float planeDistSq = planeDist * planeDist;
    const float radiusSq = radius * radius;

    if (planeDistSq > radiusSq) {
      /* Max speed sphere fully invalidates plane planeNo. */
      return false;
    }

    const float planeRadiusSq = radiusSq - planeDistSq;

    const Eigen::Vector3f planeCenter = planeDist * planes[planeNo].normal;

    if (directionOpt) {
      /* Project direction optVelocity on plane planeNo. */
      const Eigen::Vector3f planeOptVelocity =
          optVelocity -
          optVelocity.dot(planes[planeNo].normal) * planes[planeNo].normal;
      const float planeOptVelocityLengthSq = planeOptVelocity.dot(planeOptVelocity);

      if (planeOptVelocityLengthSq <= RVO3D_EPSILON) {
        result = planeCenter;
      } else {
        result =
            planeCenter + std::sqrt(planeRadiusSq / planeOptVelocityLengthSq) *
                              planeOptVelocity;
      }
    } else {
      /* Project point optVelocity on plane planeNo. */
      result = optVelocity +
              (planes[planeNo].point - optVelocity).dot(planes[planeNo].normal) * planes[planeNo].normal;

      /* If outside planeCircle, project on planeCircle. */
      if (result.dot(result) > radiusSq) {
        const Eigen::Vector3f planeResult = result - planeCenter;
        const float planeResultLengthSq = planeResult.dot(planeResult);
        result = planeCenter +
                std::sqrt(planeRadiusSq / planeResultLengthSq) * planeResult;
      }
    }

    for (std::size_t i = 0U; i < planeNo; ++i) {
      if (planes[i].normal.dot(planes[i].point - result) > 0.0F) {
        /* Result does not satisfy constraint i. Compute new optimal result.
        * Compute intersection line of plane i and plane planeNo.
        */
        Eigen::Vector3f crossProduct = planes[i].normal.cross(planes[planeNo].normal);

        if (crossProduct.dot(crossProduct) <= RVO3D_EPSILON) {
          /* Planes planeNo and i are (almost) parallel, and plane i fully
          * invalidates plane planeNo.
          */
          return false;
        }

        Line line;
        line.direction = crossProduct.normalized();
        const Eigen::Vector3f lineNormal = line.direction.cross(planes[planeNo].normal);
        line.point =
            planes[planeNo].point +
            ((planes[i].point - planes[planeNo].point).dot(planes[i].normal) /
            lineNormal.dot(planes[i].normal)) * lineNormal;

        if (!linearProgram1(planes, i, line, radius, optVelocity, directionOpt,
                            result)) {
          return false;
        }
      }
    }

    return true;
  }

  /**
   * @brief      Solves a three-dimensional linear program subject to linear
   *             constraints defined by planes and a spherical constraint.
   * @param[in]  planes       Planes defining the linear constraints.
   * @param[in]  radius       The radius of the spherical constraint.
   * @param[in]  optVelocity  The optimization velocity.
   * @param[in]  directionOpt True if the direction should be optimized.
   * @param[out] result       A reference to the result of the linear program.
   * @return     The number of the plane it fails on, and the number of planes if
   *             successful.
   */
  std::size_t Agent::linearProgram3(const std::vector<Plane> &planes, float radius,
                            const Eigen::Vector3f &optVelocity, bool directionOpt,
                            Eigen::Vector3f &result) { /* NOLINT(runtime/references) */
    if (directionOpt) {
      /* Optimize direction. Note that the optimization velocity is of unit length
      * in this case.
      */
      result = optVelocity * radius;
    } else if (optVelocity.dot(optVelocity) > radius * radius) {
      /* Optimize closest point and outside circle. */
      result = optVelocity.normalized() * radius;
    } else {
      /* Optimize closest point and inside circle. */
      result = optVelocity;
    }

    for (std::size_t i = 0U; i < planes.size(); ++i) {
      if (planes[i].normal.dot(planes[i].point - result) > 0.0F) {
        /* Result does not satisfy constraint i. Compute new optimal result. */
        const Eigen::Vector3f tempResult = result;

        if (!linearProgram2(planes, i, radius, optVelocity, directionOpt,
                            result)) {
          result = tempResult;
          return i;
        }
      }
    }

    return planes.size();
  }

  /**
   * @brief      Solves a four-dimensional linear program subject to linear
   *             constraints defined by planes and a spherical constraint.
   * @param[in]  planes     Planes defining the linear constraints.
   * @param[in]  beginPlane The plane on which the three-dimensional linear
   *                        program failed.
   * @param[in]  radius     The radius of the spherical constraint.
   * @param[out] result     A reference to the result of the linear program.
   */
  void Agent::linearProgram4(const std::vector<Plane> &planes, std::size_t beginPlane,
                      float radius,
                      Eigen::Vector3f &result) { /* NOLINT(runtime/references) */
    float distance = 0.0F;

    for (std::size_t i = beginPlane; i < planes.size(); ++i) {
      if (planes[i].normal.dot(planes[i].point - result) > distance) {
        /* Result does not satisfy constraint of plane i. */
        std::vector<Plane> projPlanes;

        for (std::size_t j = 0U; j < i; ++j) {
          Plane plane;

          const Eigen::Vector3f crossProduct = planes[j].normal.cross(planes[i].normal);

          if (crossProduct.norm() * crossProduct.norm() <= RVO3D_EPSILON) {
            /* Plane i and plane j are (almost) parallel. */
            if (planes[i].normal.dot(planes[j].normal) > 0.0F) {
              /* Plane i and plane j point in the same direction. */
              continue;
            }

            /* Plane i and plane j point in opposite direction. */
            plane.point = 0.5F * (planes[i].point + planes[j].point);
          } else {
            /* Plane.point is point on line of intersection between plane i and
            * plane j.
            */
            const Eigen::Vector3f lineNormal = crossProduct.cross(planes[i].normal);
            plane.point =
                planes[i].point +
                ((planes[j].point - planes[i].point).dot(planes[j].normal) /
                lineNormal.dot(planes[j].normal)) * lineNormal;
          }

          plane.normal = (planes[j].normal - planes[i].normal).normalized();
          projPlanes.push_back(plane);
        }

        const Eigen::Vector3f tempResult = result;

        if (linearProgram3(projPlanes, radius, planes[i].normal, true, result) <
            projPlanes.size()) {
          /* This should in principle not happen. The result is by definition
          * already in the feasible region of this linear program. If it fails,
          * it is due to small floating point error, and the current result is
          * kept.
          */
          result = tempResult;
        }

        distance = planes[i].normal.dot(planes[i].point - result);
      }
    }
  }

  void Agent::computeNewVelocity() {
    orcaPlanes_.clear();
    const float invTimeHorizon = 1.0F / timeHorizon_;

    /* Create agent ORCA planes. */
    for (std::size_t i = 0U; i < agentNeighbors_.size(); ++i) {
      const Eval_agent other = agentNeighbors_[i].second;
      const Eigen::Vector3f relativePosition = other.position_ - position_;
      const Eigen::Vector3f relativeVelocity = velocity_ - other.velocity_;
      const float distSq = relativePosition.dot(relativePosition);
      const float combinedRadius = radius_ + other.radius_;
      const float combinedRadiusSq = combinedRadius * combinedRadius;

      Plane plane;
      Eigen::Vector3f u;

      if (distSq > combinedRadiusSq) {
        /* No collision. */
        const Eigen::Vector3f w = relativeVelocity - invTimeHorizon * relativePosition;
        /* Vector from cutoff center to relative velocity. */
        const float wLengthSq = w.dot(w);

        const float dotProduct = w.dot(relativePosition);

        if (dotProduct < 0.0F &&
            dotProduct * dotProduct > combinedRadiusSq * wLengthSq) {
          /* Project on cut-off circle. */
          const float wLength = std::sqrt(wLengthSq);
          const Eigen::Vector3f unitW = w / wLength;

          plane.normal = unitW;
          u = (combinedRadius * invTimeHorizon - wLength) * unitW;
        } else {
          /* Project on cone. */
          const float a = distSq;
          const float b = relativePosition.dot(relativeVelocity);
          const float c = std::pow((relativeVelocity).norm(),2) -
                          std::pow((relativePosition.cross(relativeVelocity)).norm(),2) /
                              (distSq - combinedRadiusSq);
          const float t = (b + std::sqrt(b * b - a * c)) / a;
          const Eigen::Vector3f ww = relativeVelocity - t * relativePosition;
          const float wwLength = ww.norm();
          const Eigen::Vector3f unitWW = ww / wwLength;

          plane.normal = unitWW;
          u = (combinedRadius * t - wwLength) * unitWW;
        }
      } else {
        /* Collision. */
        const float invTimeStep = 1.0F / timeStep_;
        const Eigen::Vector3f w = relativeVelocity - invTimeStep * relativePosition;
        const float wLength = w.norm();
        const Eigen::Vector3f unitW = w / wLength;

        plane.normal = unitW;
        u = (combinedRadius * invTimeStep - wLength) * unitW;
      }

      plane.point = velocity_ + 0.5F * u;
      orcaPlanes_.push_back(plane);
    }

    const std::size_t planeFail = linearProgram3(
        orcaPlanes_, maxSpeed_, prefVelocity_, false, newVelocity_);

    if (planeFail < orcaPlanes_.size()) {
      linearProgram4(orcaPlanes_, planeFail, maxSpeed_, newVelocity_);
    }
  }

  void Agent::insertAgentNeighbor(const Eval_agent agent, float &rangeSq) {

    const float distSq = std::pow((position_ - agent.position_).norm(),2);

    // communication radius is being handled by kdtree
    agentNeighbors_.emplace_back(std::make_pair(distSq, agent));

  }

  void Agent::clearAgentNeighbor() {

    agentNeighbors_.clear();
  }

  void Agent::update() 
  {
    velocity_ = newVelocity_;
    position_ += velocity_ * timeStep_;
  }

  void Agent::updateState(
    Eigen::Vector3f pos, Eigen::Vector3f vel,
    Eigen::Vector3f pref_vel) 
  {
    position_ = pos;
    velocity_ = vel;
    prefVelocity_ = pref_vel;
  };
}
