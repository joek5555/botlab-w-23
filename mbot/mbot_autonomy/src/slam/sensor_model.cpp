#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/grid_utils.hpp>
#include <common_utils/geometric/point.hpp>
SensorModel::SensorModel(void)
:   ray_stride_(1)
{
}

double SensorModel::likelihood(const mbot_lcm_msgs::particle_t& sample, 
                               const mbot_lcm_msgs::lidar_t& scan, 
                               const OccupancyGrid& map)
{
    double likelihood = 1.0;
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose, ray_stride_);
    int i = 0;

    for (auto& ray : movingScan){
        i++;
        if(ray.range < max_lidar_range){
            Point<double> endpt(ray.origin.x + ray.range * std::cos(ray.theta),
                                ray.origin.y + ray.range * std::sin(ray.theta));
            auto rayEndPt = global_position_to_grid_cell(endpt, map);
            if (map.logOdds(rayEndPt.x, rayEndPt.y) > 0.0){
                likelihood += 100.0;
                std::cout << i << " ray entered first if statement" << std::endl;
            }
            else if (map.logOdds(rayEndPt.x+1, rayEndPt.y) > 0.0 ||
                     map.logOdds(rayEndPt.x+1, rayEndPt.y+1) > 0.0 || 
                     map.logOdds(rayEndPt.x, rayEndPt.y+1) > 0.0 ||
                     map.logOdds(rayEndPt.x-1, rayEndPt.y+1) > 0.0 ||
                     map.logOdds(rayEndPt.x-1, rayEndPt.y) > 0.0 ||
                     map.logOdds(rayEndPt.x-1, rayEndPt.y-1) > 0.0 ||
                     map.logOdds(rayEndPt.x, rayEndPt.y-1) > 0.0 ||
                     map.logOdds(rayEndPt.x+1, rayEndPt.y-1) > 0.0){
                likelihood += 75.0;
                std::cout << i << " ray entered second if statement" << std::endl;
            }
            else if (map.logOdds(rayEndPt.x+2, rayEndPt.y) > 0.0 ||
                     map.logOdds(rayEndPt.x+2, rayEndPt.y+1) > 0.0 || 
                     map.logOdds(rayEndPt.x+2, rayEndPt.y+2) > 0.0 ||
                     map.logOdds(rayEndPt.x+1, rayEndPt.y+2) > 0.0 ||
                     map.logOdds(rayEndPt.x, rayEndPt.y+2) > 0.0 ||
                     map.logOdds(rayEndPt.x-1, rayEndPt.y+2) > 0.0 ||
                     map.logOdds(rayEndPt.x-2, rayEndPt.y+2) > 0.0 ||
                     map.logOdds(rayEndPt.x-2, rayEndPt.y+1) > 0.0 || 
                     map.logOdds(rayEndPt.x-2, rayEndPt.y) > 0.0 ||
                     map.logOdds(rayEndPt.x-2, rayEndPt.y-1) > 0.0 ||
                     map.logOdds(rayEndPt.x-2, rayEndPt.y-2) > 0.0 ||
                     map.logOdds(rayEndPt.x-1, rayEndPt.y-2) > 0.0 ||
                     map.logOdds(rayEndPt.x, rayEndPt.y-2) > 0.0 ||
                     map.logOdds(rayEndPt.x+1, rayEndPt.y-2) > 0.0 ||
                     map.logOdds(rayEndPt.x+2, rayEndPt.y-2) > 0.0 ||
                     map.logOdds(rayEndPt.x+2, rayEndPt.y-1) > 0.0){
                likelihood += 50.0;
                std::cout << i << " ray entered third if statement" << std::endl;
            }
        }
    }
    // TODO
    return likelihood;
} 