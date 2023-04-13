#include <planning/astar.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

using namespace std::chrono;

mbot_lcm_msgs::robot_path_t search_for_path(mbot_lcm_msgs::pose_xyt_t start,
                                             mbot_lcm_msgs::pose_xyt_t goal,
                                             const ObstacleDistanceGrid& distances,
                                             const SearchParams& params)
{
    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
     ////////////////// TODO: Implement your A* search here //////////////////////////
    Node goalNode(goalCell.x, goalCell.y);
    Node startNode(startCell.x, startCell.y);
    startNode.parent = NULL;
    startNode.g_cost = 0;
    startNode.h_cost = h_cost(&startNode, &goalNode, distances);
    PriorityQueue queue;
    PriorityQueue visited;
    mbot_lcm_msgs::robot_path_t path;
    queue.push(&startNode);

    while (!queue.empty()) {
        Node currNode = *queue.pop();
        if(currNode == goalNode) {
            std::vector<Node*>  node_path = extract_node_path(&currNode, &startNode);
            std::vector<mbot_lcm_msgs::pose_xyt_t> pose_path = extract_pose_path(node_path, distances);
            path.path = pose_path;
            path.path_length = path.path.size();
            path.utime = start.utime;
            break;
        }
        auto children = expand_node(&currNode, distances, params);
        for (int i = 0; i < children.size(); i++) {
            if (!visited.is_member(children[i])) {
                children[i]->h_cost = h_cost(children[i], &goalNode, distances);
                children[i]->g_cost = g_cost(children[i], &goalNode, distances, params);
                queue.push(children[i]);
            }
        }
    }
    return path;
}



double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
{
    // TODO: Return calculated h cost
    double h_cost = 0;
    //Euclidean Distance
    h_cost = sqrt((double)(pow(from->cell.x - goal->cell.x, 2) + pow(from->cell.y - goal->cell.y, 2)));
    return h_cost;
}

double g_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // TODO: Return calculated g cost
    double g_cost = from->parent->g_cost;

    if(abs(from->cell.x - from->parent->cell.x) == 0 || abs(from->cell.y - from->parent->cell.y) == 0){
        g_cost += 1.0;
    }
    else{
        g_cost += 1.4;
    }
    if(distances(from->cell.x, from->cell.y) < params.maxDistanceWithCost){
        g_cost += pow(params.maxDistanceWithCost - distances(from->cell.x, from->cell.y), params.distanceCostExponent);
    }
       
    return g_cost;
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // TODO: Return children of a given node that are not obstacles
    std::vector<Node*> children;
    const int xDeltas[8] = {1, 1,  1,   0, 0, -1, -1, -1};
    const int yDeltas[8] = {0, 1, -1, -1,  1,  1, -1,  0};
    for (int i = 0; i < 8; i++) {
        if (distances.isCellInGrid(node->cell.x + xDeltas[i], node->cell.y + yDeltas[i])){
            if(distances(node->cell.x + xDeltas[i], node->cell.y + yDeltas[i]) > params.minDistanceToObstacle){

                Node child(node->cell.x + xDeltas[i], node->cell.y + yDeltas[i]);
                child.parent = node;
                children.push_back(&child);
            }
        }
    }
    return children;
}

std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node)
{
    // TODO: Generate path by following parent nodes
    std::vector<Node*> path;
    Node* currNode = goal_node;
    while(currNode != start_node && currNode != NULL) {
        path.insert(path.begin(), currNode);
        currNode = currNode->parent;
    }
    return path;
}

// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose_xyt_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances)
{
    // TODO: prune the path to generate sparse waypoints
    std::vector<mbot_lcm_msgs::pose_xyt_t> path;
    for (int i = 0; i < nodes.size(); i++) {
        mbot_lcm_msgs::pose_xyt_t pose;
        Point<double> point = grid_position_to_global_position(Point<double>(nodes[i]->cell.x, nodes[i]->cell.y), distances);
        pose.x = point.x;
        pose.y = point.y;
        path.push_back(pose);
    }
    for (int i = 0; i < path.size(); i++) {
        if (i == path.size() - 1) {
            path[i].theta = 0;
        } else {
            path[i].theta = calculate_theta(path[i], path[i+1]);
        }
    }
    return path;
}


float calculate_theta(mbot_lcm_msgs::pose_xyt_t pose1, mbot_lcm_msgs::pose_xyt_t pose2) {
    float delta_x = pose2.x - pose1.x;
    float delta_y = pose2.y - pose1.y;
    return atan2(delta_y, delta_x);
}

bool is_in_list(Node* node, std::vector<Node*> list)
{
    for (auto &&item : list)
    {
        if (*node == *item) return true;
    }
    return false;
}

Node* get_from_list(Node* node, std::vector<Node*> list)
{
    for (auto &&n : list)
    {
        if (*node == *n) return n;
    }
    return NULL;
    
}
