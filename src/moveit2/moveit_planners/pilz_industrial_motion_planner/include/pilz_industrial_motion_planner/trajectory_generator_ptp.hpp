/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <moveit/planning_scene/planning_scene.hpp>

#include <eigen3/Eigen/Eigen>
#include <pilz_industrial_motion_planner/trajectory_generation_exceptions.hpp>
#include <pilz_industrial_motion_planner/trajectory_generator.hpp>
#include <pilz_industrial_motion_planner/velocity_profile_atrap.hpp>

namespace pilz_industrial_motion_planner
{
// TODO date type of units

CREATE_MOVEIT_ERROR_CODE_EXCEPTION(PtpVelocityProfileSyncFailed, moveit_msgs::msg::MoveItErrorCodes::FAILURE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(PtpNoIkSolutionForGoalPose, moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION);

/**
 * @brief This class implements a point-to-point trajectory generator based on
 * VelocityProfileATrap.
 */
class TrajectoryGeneratorPTP : public TrajectoryGenerator
{
public:
  /**
   * @brief Constructor of PTP Trajectory Generator
   * @throw TrajectoryGeneratorInvalidLimitsException
   * @param model: a map of joint limits information
   */
  TrajectoryGeneratorPTP(const moveit::core::RobotModelConstPtr& robot_model,
                         const pilz_industrial_motion_planner::LimitsContainer& planner_limits,
                         const std::string& group_name);

private:
  void extractMotionPlanInfo(const planning_scene::PlanningSceneConstPtr& scene,
                             const planning_interface::MotionPlanRequest& req, MotionPlanInfo& info) const override;

  /**
   * @brief plan ptp joint trajectory with zero start velocity
   * @param start_pos
   * @param goal_pos
   * @param joint_trajectory
   * @param group_name
   * @param velocity_scaling_factor
   * @param acceleration_scaling_factor
   * @param sampling_time
   */
  void planPTP(const std::map<std::string, double>& start_pos, const std::map<std::string, double>& goal_pos,
               trajectory_msgs::msg::JointTrajectory& joint_trajectory, double velocity_scaling_factor,
               double acceleration_scaling_factor, double sampling_time);

  void plan(const planning_scene::PlanningSceneConstPtr& scene, const planning_interface::MotionPlanRequest& req,
            const MotionPlanInfo& plan_info, double sampling_time,
            trajectory_msgs::msg::JointTrajectory& joint_trajectory) override;

private:
  static constexpr double MIN_MOVEMENT = 0.001;
  pilz_industrial_motion_planner::JointLimitsContainer joint_limits_;
  // most strict joint limits
  JointLimit most_strict_limit_;
};

}  // namespace pilz_industrial_motion_planner
