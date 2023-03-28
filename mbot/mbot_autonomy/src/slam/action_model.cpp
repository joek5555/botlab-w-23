#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <common_utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>


ActionModel::ActionModel(void)
: k1_(0.01f)
, k2_(0.01f)
, min_dist_(0.0025)
, min_theta_(0.02)
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
}


void ActionModel::resetPrevious(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    previousPose_ = odometry;
    previous_theta_ = previousPose_.theta;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    bool moved = 0;

    if(odometry != previousPose_){
        moved = 1;
        dx_ = odometry.x - previousPose_.x;
        dy_ = odometry.y - previousPose_.y;
        dtheta_ = odometry.theta - previousPose_.theta;
        ds_ = sqrt(dx_ * dx_ + dy_ * dy_);
        alpha = atan2(dy_, dx_) - previousPose_.theta;

        turn1_std_ = k1_ * abs(alpha);
        travel_std_ = k2 * abs(ds_);
        turn2_std_ = k1 * abs(dtheta_ - alpha);
    }

    previousPose_ = odometry;

    return moved;
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    mbot_lcm_msgs::particle_t newSample = sample;

    std::random_device rd;
    std::mt19937 gen(rd());

    std::normal_distribution<float> turn1_distribution(0, turn1_std_);
    std::normal_distribution<float> travel_distribution(0, travel_std_);
    std::normal_distribution<float> turn2_distribution(0, turn2_std_);

    newSample.x += (ds_ + travel_distribution(gen)) * cos(previous_theta_ + alpha + turn1_distribution(gen));
    newSample.y += (ds_ + travel_distribution(gen)) * sin(previous_theta_ + alpha + turn1_distribution(gen));
    newSample.theta += dtheta_ + turn1_distribution(gen) + turn2_distribution(gen);

    previous_theta_ = previousPose_.theta;

    return newSample;
}
