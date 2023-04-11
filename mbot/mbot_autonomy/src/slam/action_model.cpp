#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <common_utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>


ActionModel::ActionModel(void)
: k1_(0.015f) // 0.01
, k2_(0.7f) // 0.4
//, min_dist_(0.0025)
//, min_theta_(0.02)
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    std::random_device rd;
    numberGenerator_ = std::mt19937(rd());
}


void ActionModel::resetPrevious(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    previousOdometry_ = odometry;
    //previousPose_ = odometry;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////

    if(!initialized_){
        previousOdometry_ = odometry;
        initialized_ = true;
    }

    float deltaX = odometry.x - previousOdometry_.x;
    float deltaY = odometry.y - previousOdometry_.y;
    float deltaTheta = odometry.theta - previousOdometry_.theta;
    trans_ = std::sqrt(deltaX*deltaX + deltaY*deltaY);
    rot1_ = angle_diff(std::atan2(deltaY, deltaX), previousOdometry_.theta);
    
    float direction = 1.0;
    if(std::abs(rot1_) > M_PI/2.0){
        rot1_ = angle_diff(M_PI, rot1_);
        direction = -1.0;
    }

    rot2_ = angle_diff(deltaTheta, rot1_);

    moved_ = (deltaX != 0.0) || (deltaY != 0.0) || (deltaTheta != 0.0);

    if(moved_){
        rot1Std_ = std::sqrt(k1_ * std::abs(rot1_));
        transStd_ = std::sqrt(k2_ * std::abs(trans_));
        rot2Std_ = std::sqrt(k1_ * std::abs(rot2_));
    }

    trans_ *= direction;
    previousOdometry_ = odometry;
    utime_ = odometry.utime;

    return moved_;
    
    /*
    //our code
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
    */
    
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    
    mbot_lcm_msgs::particle_t newSample = sample;

    float sampledRot1 = std::normal_distribution<>(rot1_, rot1Std_)(numberGenerator_);
    float sampledTrans = std::normal_distribution<>(trans_, transStd_)(numberGenerator_);
    float sampledRot2 = std::normal_distribution<>(rot2_, rot2Std_)(numberGenerator_);

    newSample.pose.x += sampledTrans * cos(sample.pose.theta + sampledRot1);
    newSample.pose.y += sampledTrans * sin(sample.pose.theta + sampledRot1);
    newSample.pose.theta = wrap_to_pi(sample.pose.theta + sampledRot1 + sampledRot2);
    newSample.pose.utime = utime_;
    newSample.parent_pose = sample.pose;

    return newSample;
    

    /*
    // our code
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
    */
    
}
