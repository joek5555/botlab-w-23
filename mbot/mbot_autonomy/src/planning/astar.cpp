#include <planning/astar.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>

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
    queue.push(&startNode);

    while (!queue.empty()) {
        Node currNode = *queue.pop();
        if(currNode == goalNode) {
            return;  //find path
        }
        auto children = expand_node(&currNode, distances, params);
        for (int i = 0; i < children.size(); i++) {
            if (visited.is_member(children[i]))
            children[i]->h_cost = h_cost(children[i], &goalNode, distances);
            queue.push(children[i]);
        }
    }

    mbot_lcm_msgs::robot_path_t path;
    return path;
}



double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
{
    // TODO: Return calculated h cost
    double h_cost = 0;
    //Euclidean Distance
    h_cost = sqrt(pow(from->cell.x - goal->cell.x, 2) + pow(from->cell.y - goal->cell.y, 2));
    return h_cost;
}
double g_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // TODO: Return calculated g cost
    double g_cost = 0;   
    return g_cost;
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // TODO: Return children of a given node that are not obstacles
    std::vector<Node*> children;
    const int xDeltas[8] = {1, 1,  1,   0, 0, -1, -1, -1};
    const int yDeltas[8] = {0, 1, -1, -1,  1,  1, -1,  0};
    for (int i = 0; i < 8; i++) {
        if (distances.isCellInGrid(node->cell.x + xDeltas[i], node->cell.y + yDeltas[i]) 
            && distances(node->cell.x + xDeltas[i], node->cell.y + yDeltas[i]) > params.minDistanceToObstacle) {

            Node child(node->cell.x + xDeltas[i], node->cell.y + yDeltas[i]);
            child.parent = node;
            if (xDeltas[i] != 0 && yDeltas[i] != 0) {
                child.g_cost = node->g_cost + 1.41;        //diagonals
            } else {
                child.g_cost = node->g_cost + 1;
            }
        }
    }
    return children;
}

std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node)
{
    // TODO: Generate path by following parent nodes
    std::vector<Node*> path;
    return path;
}

// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose_xyt_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances)
{
    // TODO: prune the path to generate sparse waypoints
    std::vector<mbot_lcm_msgs::pose_xyt_t> path;
    return path;
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
