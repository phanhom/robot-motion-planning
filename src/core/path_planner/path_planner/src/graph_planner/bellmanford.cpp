#include <limits>
#include <unordered_map>
#include <vector>
#include "path_planner/graph_planner/bellman_ford.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Construct a new BellmanFord object
 * @param costmap_ros The costmap representing the planning environment
 */
BellmanFordPathPlanner::BellmanFordPathPlanner(costmap_2d::Costmap2DROS* costmap_ros)
  : PathPlanner(costmap_ros)
{
}

/**
 * @brief Bellman-Ford implementation
 * @param start          Start node
 * @param goal           Goal node
 * @param path           Optimal path consists of Nodes
 * @param expand         Nodes explored during the process
 * @return true if a path is found, false otherwise
 */
bool BellmanFordPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  Node start_node(start.x(), start.y());
  Node goal_node(goal.x(), goal.y());
  start_node.set_id(grid2Index(start_node.x(), start_node.y()));
  goal_node.set_id(grid2Index(goal_node.x(), goal_node.y()));

  path.clear();
  expand.clear();

  // Initialize cost and predecessor
  std::unordered_map<int, double> cost_map;
  std::unordered_map<int, int> predecessor_map;

  for (int i = 0; i < map_size_; ++i)
  {
    cost_map[i] = std::numeric_limits<double>::infinity();
    predecessor_map[i] = -1;
  }

  cost_map[start_node.id()] = 0.0;

  // Bellman-Ford algorithm
  for (int i = 0; i < map_size_ - 1; ++i)
  {
    bool updated = false;

    for (int y = 0; y < costmap_->getSizeInCellsY(); ++y)
    {
      for (int x = 0; x < costmap_->getSizeInCellsX(); ++x)
      {
        int node_id = grid2Index(x, y);

        if (cost_map[node_id] == std::numeric_limits<double>::infinity())
          continue;

        for (const auto& motion : motions)
        {
          int nx = x + motion.x();
          int ny = y + motion.y();
          int neighbor_id = grid2Index(nx, ny);

          // Skip out-of-bound or obstacle nodes
          if (neighbor_id < 0 || neighbor_id >= map_size_ || costmap_->getCharMap()[neighbor_id] >= costmap_2d::LETHAL_OBSTACLE)
            continue;

          double new_cost = cost_map[node_id] + motion.g();

          if (new_cost < cost_map[neighbor_id])
          {
            cost_map[neighbor_id] = new_cost;
            predecessor_map[neighbor_id] = node_id;
            updated = true;
          }
        }
      }
    }

    if (!updated)
      break;
  }

  // Trace back the path if the goal is reached
  if (cost_map[goal_node.id()] == std::numeric_limits<double>::infinity())
    return false;

  for (int current_id = goal_node.id(); current_id != start_node.id(); current_id = predecessor_map[current_id])
  {
    int x, y;
    index2Grid(current_id, x, y);
    path.emplace_back(x, y, 0);
  }
  path.emplace_back(start.x(), start.y(), 0);

  std::reverse(path.begin(), path.end());
  return true;
}

int BellmanFordPathPlanner::grid2Index(int x, int y) const
{
  return y * costmap_->getSizeInCellsX() + x;
}

}  // namespace path_planner
}  // namespace rmp
