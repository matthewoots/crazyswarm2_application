/*
 * Agent.h (modified)
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

#ifndef RVO3D_AGENT_H_
#define RVO3D_AGENT_H_

/**
 * @file  Agent.h
 * @brief Contains the Agent class.
 */

#include <cstddef>
#include <utility>
#include <vector>
#include <algorithm>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace RVO 
{
  /**
   * @brief A sufficiently small positive number.
   */
  const float RVO3D_EPSILON = 0.00001F;

  struct Plane
  {
    /**
     * @brief A point on the plane.
     */
    Eigen::Vector3f point;

    /**
     * @brief The normal to the plane.
     */
    Eigen::Vector3f normal;
  };

  struct Eval_agent
  {
    Eigen::Vector3f position_;
    Eigen::Vector3f velocity_;
    float radius_;
  };

  /**
   * @brief Defines a directed line.
   */
  struct Line 
  {
    /**
     * @brief The direction of the directed line.
     */
    Eigen::Vector3f direction;

    /**
     * @brief A point on the directed line.
     */
    Eigen::Vector3f point;
  };

  /**
   * @brief Defines an agent in the simulation.
   */
  class Agent 
  {
    public:

      void linearProgram4(const std::vector<Plane> &planes, std::size_t beginPlane,
        float radius, Eigen::Vector3f &result);

      std::size_t linearProgram3(const std::vector<Plane> &planes, float radius,
        const Eigen::Vector3f &optVelocity, bool directionOpt, Eigen::Vector3f &result);
      
      bool linearProgram2(const std::vector<Plane> &planes, std::size_t planeNo,
        float radius, const Eigen::Vector3f &optVelocity, bool directionOpt,
        Eigen::Vector3f &result);

      bool linearProgram1(const std::vector<Plane> &planes, std::size_t planeNo,
        const Line &line, float radius, const Eigen::Vector3f &optVelocity,
        bool directionOpt, Eigen::Vector3f &result);

      /**
       * @brief     Constructs an agent instance.
       * @param[in] sim The simulator instance.
       */
      Agent(size_t id, float timeStep, 
        size_t maxNeighbors, float maxSpeed, float neighborDist,
        float radius, float timeHorizon, float minHeight, 
        float maxHeight)
        : id_(id), timeStep_(timeStep), maxNeighbors_(maxNeighbors),
        maxSpeed_(maxSpeed), neighborDist_(neighborDist),
        radius_(radius), timeHorizon_(timeHorizon), 
        minHeight_(minHeight), maxHeight_(maxHeight_){};

      /**
       * @brief Destroys this agent instance.
       */
      ~Agent() {};

      /**
       * @brief Computes the new velocity of this agent.
       */
      void computeNewVelocity();

      void clearAgentNeighbor();

      /**
       * @brief     Inserts an agent neighbor into the set of neighbors of this
       *            agent.
       * @param[in] agent   A pointer to the agent to be inserted.
       * @param[in] rangeSq The squared range around this agent.
       */
      void insertAgentNeighbor(const Eval_agent agent,
                              float &rangeSq); /* NOLINT(runtime/references) */

      /**
       * @brief Updates the three-dimensional position and three-dimensional
       *        velocity of this agent.
       */
      void update();

      Eigen::Vector3f getVelocity() {return newVelocity_;};

      bool noNeighbours() {return agentNeighbors_.empty();};

      void updateState(Eigen::Vector3f pos, 
        Eigen::Vector3f vel, Eigen::Vector3f pref_vel);

    private:

      /* Not implemented. */
      // Agent(const Agent &other);

      /* Not implemented. */
      // Agent &operator=(const Agent &other);

      Eigen::Vector3f newVelocity_;
      Eigen::Vector3f position_;
      Eigen::Vector3f prefVelocity_;
      Eigen::Vector3f velocity_;
      std::size_t id_;
      std::size_t maxNeighbors_;
      float maxSpeed_;
      float neighborDist_;
      float radius_;
      float timeHorizon_;
      float timeStep_;
      float minHeight_;
      float maxHeight_;
      std::vector<std::pair<float, const Eval_agent>> agentNeighbors_;
      std::vector<Plane> orcaPlanes_;

  };
} /* namespace RVO */

#endif /* RVO3D_AGENT_H_ */
