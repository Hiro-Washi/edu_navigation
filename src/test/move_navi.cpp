#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <navfn/navfn_ros.h>

enum class NavState{
  STADBY,
  WAIT_PLAN,
  MOVING
};


