#ifndef LINEFOLLOW_PLANNER_H_
#define LINEFOLLOW_PLANNER_H_
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>
#include <ros/ros.h>

#include <angles/angles.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <base_local_planner/costmap_model.h>
#include <base_local_planner/world_model.h>

namespace linefollow_planner {
class LinefollowPlanner : public nav_core::BaseGlobalPlanner {
public:
  LinefollowPlanner();
  LinefollowPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

  void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

  bool makePlan(const geometry_msgs::PoseStamped &start,
                const geometry_msgs::PoseStamped &goal,
                std::vector<geometry_msgs::PoseStamped> &plan);

  void pathCallback(const nav_msgs::Path path);

  private:
    costmap_2d::Costmap2DROS *costmap_ros_;
    double step_size_, min_dist_from_robot_;
    costmap_2d::Costmap2D *costmap_;
    base_local_planner::WorldModel
        *world_model_; ///< @brief The world model that the controller will use

    double footprintCost(double x_i, double y_i, double theta_i);

    bool initialized_;

    nav_msgs::Path linefollow_path_;
    ros::Subscriber path_sub_;
    ros::Publisher plan_pub_;

  };
}; // namespace linefollow_planner
#endif
