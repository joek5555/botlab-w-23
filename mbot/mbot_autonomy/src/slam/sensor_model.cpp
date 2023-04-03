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
    double likelihood = 0.0;
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose, ray_stride_);

    for (auto& ray : movingScan){
        Point<double> endpt(ray.origin.x + ray.range * std::cos(ray.theta),
                            ray.origin.y + ray.range * std::sin(ray.theta));
        auto rayEndPt = global_position_to_grid_cell(endpoint, map);
        if (map.logOdds(rayEndPt.x, rayEndPt.y) > 0.0){
            likelihood += 1.0;
        }
    }
    // TODO
    return likelihood;
}