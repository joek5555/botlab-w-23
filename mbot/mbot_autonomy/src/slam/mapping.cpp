#include <slam/mapping.hpp>
#include <utils/grid_utils.hpp>
#include <numeric>
#include <chrono> 
using namespace std::chrono; 

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}


void Mapping::updateMap(const mbot_lcm_msgs::lidar_t& scan,
                        const mbot_lcm_msgs::pose_xyt_t& pose,
                        OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////

    MovingLaserScan moving_laser_scan(scan, previousPose_, pose, 1);
    for(std::size_t i = 0; i < moving_laser_scan.size(); i++){  // iterate through all the rays
        const adjusted_ray_t& current_ray = moving_laser_scan[i]

        std::vector<Point<int>> free_cells = bresenham(current_ray, map);
        for(std::size_t j = 0; j < free_cells.size(); j++){
            cell_x = free_cells[j].x;
            cell_y = free_cells[j].y;
            map.setLogOdds(cell_x, cell_y, map.logOdds(cell_x, cell_y) + missOdds); // no need to add prior because prior is 0
        }
        if(current_ray.range < maxLaserDistance){ // if the range is less than the max range, then the endpoint is occupied
            Point<int> endpoint_cell = global_position_to_grid_cell(Point<double>(
                ray.origin.x + ray.range * std::cos(ray.theta),
                ray.origin.y + ray.range * std::sin(ray.theta)
                ), map);
            endcell_x = endpoint_cell.x;
            endcell_y = endpoint_cell.y;
            map.setLogOdds(endcell_x, endcell_y, map.logOdds(endcell_x, endcell_y) + missOdds); // no need to add prior because prior is 0
        }
    }

}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
//////////////// TODO: Implement your endpoint score ///////////////////////
}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
//////////////// TODO: Implement your ray score ///////////////////////
}

/*
Takes the ray and map, and returns a vector of map cells to check
*/
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    // Get global positions 
    Point<float> f_end = global_position_to_grid_position(
        Point<float>(
            ray.origin.x + ray.range * std::cos(ray.theta),
            ray.origin.y + ray.range * std::sin(ray.theta)
            ), 
        map
        );

    // Cells
    Point<int> start_cell = global_position_to_grid_cell(ray.origin, map);
    Point<int> end_cell;
    end_cell.x = static_cast<int>(f_end.x);
    end_cell.y = static_cast<int>(f_end.y);
    std::vector<Point<int>> cells_touched;
    //////////////// TODO: Implement Bresenham's Algorithm ////////////////
    float x0 = ray.origin.x;
    float y0 = ray.origin.y;
    float x1 = ray.range*cos(ray.theta);
    float y1 = ray.range*sin(ray.theta);
    float dx = abs(x1-x0);
    float dy = abs(y1-y0);
    float sx = x0<x1 ? 1 : -1;
    float sy = y0<y1 ? 1 : -1;
    float err = dx-dy;
    float x = x0;
    float y = y0;

    while (x != x1 || y != y1){
        //map.setLogOdds(x,y,map.logOdds(x,y)-1);
        float err2 = 2*err;
        if (err2 >= -dy){
            err -=dy;
            x += sx;
        }
        if (err2 <= dx){
            err += dx;
            y += sy;
        }
        cells_touched.push_back(x,y)
    }
    return cells_touched;
}

std::vector<Point<int>> Mapping::divideAndStepAlongRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    auto end_cell = global_position_to_grid_cell(Point<double>(
        ray.origin.x + ray.range * std::cos(ray.theta),
        ray.origin.y + ray.range * std::sin(ray.theta)
        ), map);
    //////////////// TODO: Implement divide and step ////////////////
    std::vector<Point<int>> cells_touched;
    return cells_touched;
}
