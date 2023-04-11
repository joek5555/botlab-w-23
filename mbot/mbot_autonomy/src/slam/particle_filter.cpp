#include <utils/grid_utils.hpp>
#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/pose_xyt_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <cassert>
#include <common_utils/geometric/angle_functions.hpp>

bool sortBtWeight (mbot_lcm_msgs::particle_t i, mbot_lcm_msgs::particle_t j) { return (i.weight>j.weight); }

ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles),
  samplingAugmentation(0.5, 0.9, numParticles),
  distribution_quality(1),
  quality_reinvigoration_percentage(0.1)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const mbot_lcm_msgs::pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    posteriorPose_ = pose;

    for (auto& particle : posterior_) {
        particle.pose.x = posteriorPose_.x;
        particle.pose.y = posteriorPose_.y;
        particle.pose.theta = wrap_to_pi(posteriorPose_.theta);
        particle.pose.utime = pose.utime;
        particle.parent_pose = particle.pose;
        particle.weight = 1.0 / kNumParticles_;
        
    }
}

void ParticleFilter::initializeFilterRandomly(const OccupancyGrid& map)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
}

void ParticleFilter::resetOdometry(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    actionModel_.resetPrevious(odometry);
}


mbot_lcm_msgs::pose_xyt_t ParticleFilter::updateFilter(const mbot_lcm_msgs::pose_xyt_t& odometry,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if(hasRobotMoved){
        auto prior = resamplePosteriorDistribution(); // originally passed &map to function
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        // OPTIONAL TODO: Add reinvigoration step
        posteriorPose_ = estimatePosteriorPose(posterior_);
        posteriorPose_.utime = odometry.utime;
    }
    

    //std::cout << "Number of particles: " << posterior_.size() << std::endl;
    /*
    std::sort (posterior_.begin(), posterior_.end(), sortBtWeight);

    int num_top_particles = int(kNumParticles_ * percent_of_top_particles);

    ParticleList posterior_top_particles = {posterior_.begin(), posterior_.begin() + num_top_particles};

    
    mbot_lcm_msgs::pose_xyt_t average_pose_top_particles = computeParticlesAverage(posterior_top_particles);
    posteriorPose_ = average_pose_top_particles;
    */
    //std::cout << average_pose_top_particles.x << ", " << average_pose_top_particles.y << ", " << average_pose_top_particles.theta << 
    //    ", num_particles: " << posterior_top_particles.size() << std::endl;

    return posteriorPose_;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::updateFilterActionOnly(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if(hasRobotMoved)
    {    
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = proposal;
    }

    posteriorPose_ = odometry;

    return posteriorPose_;
}



mbot_lcm_msgs::pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


mbot_lcm_msgs::particles_t ParticleFilter::particles(void) const
{
    mbot_lcm_msgs::particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}





ParticleList ParticleFilter::resamplePosteriorDistribution() // originally input to function was (const OccupancyGrid* map)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    /*
    ParticleList prior = posterior_;
    double sample_weight = 1/ kNumParticles_;
    std::random_device rd;
    std::mt19937 generator(rd());
    std::normal_distribution<> dist(0.0, 0.07);

    for(auto& p : prior){
        p.pose.x = posteriorPose_.x + dist(generator);
        p.pose.y = posteriorPose_.y + dist(generator);
        p.pose.theta = posteriorPose_.theta + dist(generator);
        p.pose.utime = posteriorPose_.utime;
        p.parent_pose = posteriorPose_;
        p.weight = sample_weight;
    }
    return prior;
    */
    
    

   
    ParticleList prior;

    double inverse_num_particles = 1.0/(double)kNumParticles_;
    std::default_random_engine gen;
    std::uniform_real_distribution<double> distribution(0.0, inverse_num_particles);
    double rand_number = distribution(gen); // random number between 0 and 1/ num_particles

    int posterior_particle_index = 0;
    double sum_particle_weights = posterior_[0].weight; // set the sum of particle weights to the first particle weight
    double step_size = 0;
    for(int prior_particle_index = 0; prior_particle_index < kNumParticles_; prior_particle_index ++){
        step_size = rand_number + prior_particle_index * inverse_num_particles; 
        while(step_size > sum_particle_weights){ // if the step size is greater than the sum of particle weights
                                                 // then we do not add the posterior particle index to the prior
                                                 // and we move on to the next posterior particle
            posterior_particle_index ++;
            sum_particle_weights += posterior_[posterior_particle_index].weight;
        }
        prior.push_back(posterior_[posterior_particle_index]);
        //prior.back().weight = inverse_num_particles;
    }
    
    
    std::sort (prior.begin(), prior.end(), sortBtWeight);

    int num_top_particles = int(kNumParticles_ * percent_of_top_particles);
    
    ParticleList prior_top_particles = {prior.begin(), prior.begin() + num_top_particles};
    
    mbot_lcm_msgs::pose_xyt_t average_pose_top_particles = computeParticlesAverage(prior_top_particles);

    std::random_device rd;
    std::mt19937 generator = std::mt19937(rd());
    std::normal_distribution<> dist_xy(0.0, covariance_of_sampled_particles_xy);
    std::normal_distribution<> dist_theta(0.0, covariance_of_sampled_particles_theta);
    //dist(generator)

    
    for(int i = num_top_particles; i < prior.size(); i++){
        prior[i].pose.x = average_pose_top_particles.x + dist_xy(generator);
        prior[i].pose.y = average_pose_top_particles.y + dist_xy(generator);
        prior[i].pose.theta = wrap_to_pi(average_pose_top_particles.theta + dist_theta(generator));
    }
    
    for(int i = 0 ; i < prior.size(); i++){
        prior[i].weight = inverse_num_particles;
    }
    
    //std::cout << average_pose_top_particles.x << ", " << average_pose_top_particles.y << ", " << average_pose_top_particles.theta << 
        //", num_particles: " << prior_top_particles.size() << std::endl;
    return prior;
    
    
}


ParticleList ParticleFilter::computeProposalDistribution(const ParticleList& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    ParticleList proposal;

    for(int i = 0; i < prior.size(); i++){
        proposal.push_back(actionModel_.applyAction(prior[i]));
    }

    return proposal;
}


ParticleList ParticleFilter::computeNormalizedPosterior(const ParticleList& proposal,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the
    ///////////       particles in the proposal distribution
    ParticleList posterior;
    double sumWeights = 0.0;
    //std::cout << "//////////////   list of weights   /////////////////////" << std::endl;
    for(auto& p : proposal){
        mbot_lcm_msgs::particle_t weighted = p;
        weighted.weight = sensorModel_.likelihood(weighted, laser, map);
        sumWeights += weighted.weight;
        posterior.push_back(weighted);
    }
    /*
    std::sort (posterior_.begin(), posterior_.end(), sortBtWeight);

    std::cout << "top Weights" << std::endl;
    for(int i = 0; i < posterior.size(); i++){
        std::cout << posterior[i].weight << std::endl;
    }
    */
   
    /*
    std::cout << "lowest Weights" << std::endl;
    for(int i = posterior.size() - 1; i > posterior.size() - 26; i--){
        std::cout << posterior[i].weight << std::endl;
    }
    */

    for(auto& p : posterior){
        p.weight /= sumWeights;
    }
    return posterior;
}


mbot_lcm_msgs::pose_xyt_t ParticleFilter::estimatePosteriorPose(const ParticleList& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    mbot_lcm_msgs::pose_xyt_t pose;
    double xAvg = 0.0;
    double yAvg = 0.0;
    double cosAvg = 0.0;
    double sinAvg = 0.0;
    for(auto& p: posterior){
        xAvg += p.weight * p.pose.x;
        yAvg += p.weight * p.pose.y;
        cosAvg += p.weight * std::cos(p.pose.theta);
        sinAvg += p.weight * std::sin(p.pose.theta);
    }
    pose.x = xAvg;
    pose.y = yAvg;
    pose.theta = std::atan2(sinAvg, cosAvg);
    return pose;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::computeParticlesAverage(const ParticleList& particles_to_average)
{
    //////// TODO: Implement your method for computing the average of a pose distribution
    mbot_lcm_msgs::pose_xyt_t avg_pose;

    double xAvg = 0.0;
    double yAvg = 0.0;
    double cosAvg = 0.0;
    double sinAvg = 0.0;
    for(auto& p: particles_to_average){
        xAvg += p.pose.x;
        yAvg += p.pose.y;
        cosAvg += std::cos(p.pose.theta);
        sinAvg += std::sin(p.pose.theta);
    }
    xAvg /= particles_to_average.size();
    yAvg /= particles_to_average.size();
    cosAvg /= particles_to_average.size();
    sinAvg /= particles_to_average.size();

    avg_pose.x = xAvg;
    avg_pose.y = yAvg;
    avg_pose.theta = std::atan2(sinAvg, cosAvg);

    return avg_pose;
}
