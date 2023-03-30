#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <common_utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>


ActionModel::ActionModel(void)
: k1_(0.01f)
, k2_(0.4f)
, min_dist_(0.0025)
, min_theta_(0.02)
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
}


void ActionModel::resetPrevious(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    previousPose_ = odometry;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    bool moved = 0;

    if(odometry.x != previousPose_.x || odometry.y != previousPose_.y || odometry.theta != previousPose_.theta){
        moved = 1;
        dx_ = odometry.x - previousPose_.x;
        dy_ = odometry.y - previousPose_.y;
        dtheta_ = angle_diff(odometry.theta, previousPose_.theta);
        ds_ = sqrt(dx_ * dx_ + dy_ * dy_);
        alpha = angle_diff(atan2(dy_, dx_), previousPose_.theta);

        // standard deviation is scaled based on distance traveled or angle turned

        turn1_var_ = k1_ * abs(alpha);
        travel_var_ = k2_ * abs(ds_);
        turn2_var_ = k1_ * abs(angle_diff(dtheta_, alpha));
    }

    // set previous pose to current pose
    previousPose_ = odometry;

    return moved;
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    mbot_lcm_msgs::particle_t newSample = sample;

    std::random_device rd;
    // create three different random numbers
    std::mt19937 gen1(rd());
    std::mt19937 gen2(rd());
    std::mt19937 gen3(rd());

    // create three normal distributions with zero mean and the standard deviations defined before
    std::normal_distribution<float> turn1_distribution(0, turn1_var_);
    std::normal_distribution<float> travel_distribution(0, travel_var_);
    std::normal_distribution<float> turn2_distribution(0, turn2_var_);

    newSample.parent_pose = newSample.pose; 

    newSample.pose.x += (ds_ + travel_distribution(gen2)) * cos(newSample.parent_pose.theta + alpha + turn1_distribution(gen1));
    newSample.pose.y += (ds_ + travel_distribution(gen2)) * sin(newSample.parent_pose.theta + alpha + turn1_distribution(gen1));
    newSample.pose.theta += dtheta_ + turn1_distribution(gen1) + turn2_distribution(gen3);
    newSample.pose.theta = wrap_to_pi(newSample.pose.theta);

    

    return newSample;
}
