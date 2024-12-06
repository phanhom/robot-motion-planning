#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_BELLMAN_FORD_H
#define RMP_PATH_PLANNER_GRAPH_PLANNER_BELLMAN_FORD_H

#include "path_planner/path_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Class for objects that plan using the Bellman-Ford algorithm
 */
class BellmanFordPathPlanner : public PathPlanner
{
public:
  /**
   * @brief Construct a new BellmanFord object
   * @param costmap_ros The costmap representing the planning environment
   */
  BellmanFordPathPlanner(costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Bellman-Ford implementation
   * @param start          Start node
   * @param goal           Goal node
   * @param path           Optimal path consists of Nodes
   * @param expand         Nodes explored during the process
   * @return true if a path is found, false otherwise
   */
  bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand);

private:
  using Node = rmp::common::structure::Node<int>;
  const std::vector<Node> motions = {
    { 0, 1, 1.0 }, { 1, 0, 1.0 }, { 0, -1, 1.0 }, { -1, 0, 1.0 },
    { 1, 1, std::sqrt(2) }, { 1, -1, std::sqrt(2) }, { -1, 1, std::sqrt(2) }, { -1, -1, std::sqrt(2) },
  };

  int grid2Index(int x, int y) const;  // Converts grid coordinates to a 1D index
};
}  // namespace path_planner
}  // namespace rmp
#endif
