#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float startx, float starty, float endx, float endy): m_Model(model) {
    // Convert inputs to percentage:
    startx *= 0.01;
    starty *= 0.01;
    endx *= 0.01;
    endy *= 0.01;

  start_node=&m_Model.FindClosestNode(startx,starty);
  end_node=& m_Model.FindClosestNode(endx,endy);

}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node){
  //std::cout<<node->distance(*end_node);
  return node->distance(*end_node);
}



void RoutePlanner::AddNeighbors(RouteModel::Node *cnode) {
  cnode->FindNeighbors();
   for(auto V : cnode->neighbors){
     V->parent = cnode;
     V->h_value = CalculateHValue(V);
     V->g_value = cnode->g_value + cnode->distance(*V); 
     open_list.push_back(V);
     V->visited = true;
   }
}



bool Compare(const RouteModel::Node *a, const RouteModel::Node *b) {

float f1 = a->g_value + a->h_value;

float f2 = b->g_value + b->h_value;

return f1 > f2;

}


RouteModel::Node *RoutePlanner::NextNode() {
  std::sort(open_list.begin(), open_list.end(), Compare);

RouteModel::Node *lowest_node = open_list.back();

open_list.pop_back();

return lowest_node;
}



std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;

    // TODO: Implement your solution here.
    std::vector<RouteModel::Node> path_found;
    while (current_node != start_node) {
        distance += current_node->distance(*current_node->parent);
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

    // TODO: Implement your solution here.
  start_node->visited = true;
    open_list.push_back(start_node);
    
    current_node = start_node;
    while (current_node != end_node) {
        AddNeighbors(current_node);
        current_node = NextNode();
    }
    m_Model.path = ConstructFinalPath(current_node);

}