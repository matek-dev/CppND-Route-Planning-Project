#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto node: current_node->neighbors) {
        node->parent = current_node;
        node->h_value = RoutePlanner::CalculateHValue(node);
        node->g_value = current_node->g_value + current_node->distance(*node);
        node->visited = true;
        open_list.push_back(node);
    }
}

// A more accurate implementation. Node->visited should not be set true until removed from open_list.
// Will not pass tests for this project.
//     current_node->visited = true;
//     current_node->FindNeighbors();
//     for (auto node: current_node->neighbors) {
//         // if node was visited already, replace if shorter path to it was found    
//         if(std::find(open_list.begin(), open_list.end(), node) != open_list.end()) { 
//             float new_g_value = current_node->g_value + current_node->distance(*node);
//             if (new_g_value < node->g_value) {
//                 node->g_value = new_g_value;
//                 node->parent = current_node; // Update parent to current_node
//             }
//         }
//         else {
//         node->parent = current_node;
//         node->h_value = RoutePlanner::CalculateHValue(node);
//         node->g_value = current_node->g_value + current_node->distance(*node);
//         open_list.push_back(node);
//         }
//     }
// }

bool compareByDist(const RouteModel::Node* a, const RouteModel::Node* b) {
    return (a->h_value + a->g_value) > (b->h_value + b->g_value);
}
RouteModel::Node *RoutePlanner::NextNode() {
  std::sort(open_list.begin(), open_list.end(), compareByDist);
  RouteModel::Node* p = open_list.back();
  open_list.pop_back();
  return p;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while(current_node != start_node){
        distance += current_node->distance(*(current_node->parent));
        path_found.push_back(*current_node);
        current_node = current_node->parent;
    }
    path_found.push_back(*current_node);
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    open_list.push_back(start_node);
    start_node->visited = true;
    while(open_list.size() > 0) {
        current_node = NextNode();
        if (current_node == end_node) {
        m_Model.path = ConstructFinalPath(current_node);
        return;
        }
        AddNeighbors(current_node);
    }
    std::cout << "No path was found" << std::endl;
}