#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/grid_utils.hpp>
#include <common_utils/geometric/point.hpp>

#include <iostream>
#include <fstream>

SensorModel::SensorModel(void)
:   ray_stride_(1)
{
}

double SensorModel::likelihood(const mbot_lcm_msgs::particle_t& sample, 
                               const mbot_lcm_msgs::lidar_t& scan, 
                               const OccupancyGrid& map)
{

    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose);
    double scanScore;

    for(auto& ray : movingScan){
        Point<double> endpoint(ray.origin.x + ray.range * std::cos(ray.theta),
                                ray.origin.y + ray.range * std::sin(ray.theta));
        auto rayEnd = global_position_to_grid_cell(endpoint, map);
        if(map.logOdds(rayEnd.x, rayEnd.y) > 0.0){
            scanScore += 1.0;
        }

    }

    return scanScore;
    /*
    //our code
    //int isThereAGoodScan = 0;
    double likelihood = 1.0;
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose, ray_stride_);
    int i = 0;

    for (auto& ray : movingScan){
        i++;
        if(ray.range < max_lidar_range){
            Point<double> endpt(ray.origin.x + ray.range * std::cos(ray.theta),
                                ray.origin.y + ray.range * std::sin(ray.theta));
            auto rayEndPt = global_position_to_grid_cell(endpt, map);
            likelihood += map.logOdds(rayEndPt.x, rayEndPt.y);

            
            if (map.logOdds(rayEndPt.x, rayEndPt.y) > 0.0){
                //likelihood += 0.1; //100
                
                isThereAGoodScan += 1;
                //std::ofstream myfile;
                //myfile.open ("laser_scan_log.txt");
                //myfile << i << " ray entered first if statement" << scan.utime << std::endl;
                //myfile.close();
                //std::cout << i << " ray entered first if statement" << std::endl;
            }
            else if (map.logOdds(rayEndPt.x+1, rayEndPt.y) > 0.0 ||
                     map.logOdds(rayEndPt.x+1, rayEndPt.y+1) > 0.0 || 
                     map.logOdds(rayEndPt.x, rayEndPt.y+1) > 0.0 ||
                     map.logOdds(rayEndPt.x-1, rayEndPt.y+1) > 0.0 ||
                     map.logOdds(rayEndPt.x-1, rayEndPt.y) > 0.0 ||
                     map.logOdds(rayEndPt.x-1, rayEndPt.y-1) > 0.0 ||
                     map.logOdds(rayEndPt.x, rayEndPt.y-1) > 0.0 ||
                     map.logOdds(rayEndPt.x+1, rayEndPt.y-1) > 0.0){
                //likelihood += 10.0; // 50
                //likelihood += (map.logOdds(rayEndPt.x, rayEndPt.y)/4);
                //isThereAGoodScan = 1;
                //std::ofstream myfile;
                //myfile.open ("laser_scan_log.txt");
                //myfile << i << " ray entered second if statement at time" << scan.utime << std::endl;
                //myfile.close();
                //std::cout << i << " ray entered second if statement" << std::endl;
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
                //likelihood += 5.0; // 20
                //likelihood += (map.logOdds(rayEndPt.x, rayEndPt.y)/10);
                //isThereAGoodScan = 1;
                //std::ofstream myfile;
                //myfile.open ("laser_scan_log.txt");
                //myfile << i << " ray entered third if statement" << scan.utime << std::endl;
                //myfile.close();
                //std::cout << i << " ray entered third if statement" << std::endl;
            }
            
        }
    }
    //if (isThereAGoodScan){
        //isThereAGoodScan = 0;
        //std::cout << "good laser scan at time " << scan.utime << std::endl;

    //}
    // TODO

    //std::cout << isThereAGoodScan << std::endl;
    return likelihood;

    */
} 