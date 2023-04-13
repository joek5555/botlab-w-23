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
    std::cout << "Start of search_for_path, grid_x: " << distances.widthInCells() << ", grid_y: " << distances.heightInCells() << std::endl;
    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
     ////////////////// TODO: Implement your A* search here //////////////////////////
    Node goalNode(goalCell.x, goalCell.y);
    Node startNode(startCell.x, startCell.y);
    Node* startNode_pointer = &startNode;
    startNode_pointer->parent = NULL;
    startNode_pointer->g_cost = 0;
    startNode_pointer->h_cost = h_cost(&startNode, &goalNode, distances);
    PriorityQueue queue;
    PriorityQueue visited;
    mbot_lcm_msgs::robot_path_t path;
    queue.push(startNode_pointer);
    bool not_first_time = 0;
    int count = 0;

    while (!queue.empty()) {
        
        Node* currNode = queue.pop();
        if(!visited.is_member(currNode)){
            count ++;
            std::cout << "size of count: " << count << std::endl;
            visited.push(currNode);
        
            if(not_first_time)
                //std::cout<< "current node x: "<< currNode->cell.x << ", y:" << currNode->cell.y << "parent is x: " << currNode->parent->cell.x << ", y: " << currNode->parent->cell.y << std::endl;
            not_first_time = 1;
            if(*currNode == goalNode) {
                //std::cout << "made it to goal" << std::endl;
                std::vector<Node*>  node_path = extract_node_path(currNode, &startNode);
                //std::cout << "1" << std::endl;
                std::vector<mbot_lcm_msgs::pose_xyt_t> pose_path = extract_pose_path(node_path, distances);
                //std::cout << "2" << std::endl;
                path.path = pose_path;
                //std::cout << "3" << std::endl;
                path.path_length = path.path.size();
                //std::cout << "4" << std::endl;
                path.utime = start.utime;
                //std::cout << "5" << std::endl;
                break;
            }
            std::vector<Node*> children = expand_node(currNode, distances, params);
            //std::cout << "number of children: " << children.size() << std::endl;
            for (int i = 0; i < children.size(); i++) {
                if (visited.is_member(children[i]) == 0){
                    //std::cout << "child cell x: " << children[i]->cell.x << ", y: " << children[i]->cell.y << ", parent x: " <<children[i]->parent->cell.x << ", y: " << children[i]->parent->cell.y   << std::endl;
                    children[i]->h_cost = h_cost(children[i], &goalNode, distances);
                    //std::cout << "calculate h" << std::endl;
                    children[i]->g_cost = g_cost(children[i], &goalNode, distances, params);
                    //std::cout << "calculate g" << std::endl;
                    queue.push(children[i]);
                    //std::cout << "queue push child parent x: " << children[i]->parent->cell.x << ", y: "<< children[i]->parent->cell.y << std::endl;
                }
            }   
        }
    }
    std::cout << "queue is empty" << std::endl;

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
    //std::cout << "4child cell x: " << from->cell.x << ", y: " << from->cell.y << ", parent x: " <<from->parent->cell.x << ", y: " << from->parent->cell.y   << std::endl;
    if(abs(from->cell.x - from->parent->cell.x) == 0 || abs(from->cell.y - from->parent->cell.y) == 0){
        g_cost += 1.0;
        //std::cout << "2" << std::endl;
    }
    else{
        g_cost += 1.4;
        //std::cout << "3" << std::endl;
    }
    if(distances(from->cell.x, from->cell.y) < params.maxDistanceWithCost){
        g_cost += pow(params.maxDistanceWithCost - distances(from->cell.x, from->cell.y), params.distanceCostExponent);
     //   std::cout << "2.3" << std::endl;
    }
    //std::cout << "4" << std::endl;
    return g_cost;
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // TODO: Return children of a given node that are not obstacles
    //std::cout << "expanding_node, x: " << node->cell.x << ", y: " << node->cell.y << std::endl;
    std::vector<Node*> children;
    const int xDeltas[8] = {1, 1,  1,   0, 0, -1, -1, -1};
    const int yDeltas[8] = {0, 1, -1, -1,  1,  1, -1,  0};
    for (int i = 0; i < 8; i++) {
        if (distances.isCellInGrid(node->cell.x + xDeltas[i], node->cell.y + yDeltas[i])){
            if(distances(node->cell.x + xDeltas[i], node->cell.y + yDeltas[i]) > params.minDistanceToObstacle + 0.01){
                //Node child(node->cell.x + xDeltas[i], node->cell.y + yDeltas[i]);
                if(distances(node->cell.x + xDeltas[i], node->cell.y + yDeltas[i]) < 0.115)
                    std::cout << "cell distance: " << distances(node->cell.x + xDeltas[i], node->cell.y + yDeltas[i]) << ", param distance: " << params.minDistanceToObstacle << std::endl;
                Node* child = new Node(node->cell.x + xDeltas[i], node->cell.y + yDeltas[i]);
                child->parent = node;
                //std::cout << "child cell x: " << child->cell.x << ", y: " << child->cell.y << ", parent x: " <<child->parent->cell.x << ", y: " << child->parent->cell.y   << std::endl;
                children.push_back(child);
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
    while((currNode == start_node) == 0) {
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
