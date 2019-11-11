/**
 * DistanceChecker: utility class for quickly evaluating feasibility of
 * trajectories
 */

#pragma once

#include <deque>
#include <tuple>
#include <valarray>

#include "planner_types.hpp"

namespace rr {

class DistanceChecker {
  public:
    /**
     * Constructor
     * @param hitbox Hitbox of the robot
     * @param map_size Map size limits for cached distances. Set this so that it
     * is not possible for paths to leave this box
     */
    DistanceChecker(const CenteredBox& hitbox, const CenteredBox& map_size);

    /**
     * Calculate the shortest distance from any part of the robot to any obstacle
     * point in the map
     * @param pose Future pose relative to current/origin (0,0,0)
     * @return bool - collision detected
     *         double - clearing distance to nearest obstacle
     */
    [[nodiscard]] std::optional<double> GetCollisionDistance(const Pose& pose) const;

    /**
     * Given a map, cache the nearest neighbors. Fill the cache outwards from
     * locations containing obstacle points.
     * @param map Point cloud map representation
     */
    void SetMap(const PCLMap& map);

    /**
     * Determine if a point is in collision with the robot's current position. If
     * true, something is fishy.
     * @param relative_point Point in robot's coordinate frame
     * @return true if point is inside robot's hitbox, false otherwise
     */
    [[nodiscard]] bool GetCollision(const PCLPoint& relative_point) const;

    /**
     * Convenience method for the distance between two PointCloud points
     * @param p1 Point 1
     * @param p2 Point 2
     * @return Euclidean distance between the points
     */
    [[nodiscard]] double Dist(const PCLPoint& p1, const PCLPoint& p2) const;

  private:
    /*
     * Caching system: map from discretized x, y location to its nearest neighbor
     * in the map/obstacle point cloud
     */
    struct CacheEntry {
        PCLPoint location;                              // x, y location represented by this entry
        std::vector<const PCLPoint*> might_hit_points;  // map points which are close enough to check
                                                        // collisions
        const PCLPoint* nearest_point;                  // nearest neighbor in obstacle map
        const CacheEntry* parent;                       // BFS parent
    };

    /*
     * Get the index of a cache element from the x, y location and vice versa
     */
    [[nodiscard]] int GetCacheIndex(double x, double y) const;
    [[nodiscard]] PCLPoint GetPointFromIndex(int i) const;

    std::vector<CacheEntry> cache_;      // cache storage
    std::deque<int> cache_updates_;      // queue for updating the cache in breadth-first order
    std::valarray<bool> cache_visited_;  // boolean mask for tracking which locations have been updated
    int cache_rows_front_;
    int cache_rows_back_;
    int cache_rows_;
    int cache_cols_left_;
    int cache_cols_right_;
    int cache_cols_;
    const CenteredBox map_size_;
    const double cache_resolution_;

    const CenteredBox hitbox_;
    double hitbox_corner_dist_;
};

}  // namespace rr
