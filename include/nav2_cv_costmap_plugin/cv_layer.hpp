#ifndef CV_LAYER_HPP_
#define CV_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"

#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>

namespace nav2_cv_costmap_plugin
{

class CvLayer : public nav2_costmap_2d::Layer
{
public:
  CvLayer();
  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x, double * max_y);
  virtual void updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual void reset(){return;}
  virtual void onFootprintChanged();
  virtual bool isClearable(){return false;}

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ObjectPoseSub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ObjectSub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr GoalSub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr GoalPub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void objectCallback(const std_msgs::msg::String::SharedPtr msg);
  void setCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_x_cell, int min_y_cell, int max_x_cell, int max_y_cell, unsigned int map_x, unsigned int map_y, unsigned char * master_array);
  void onlyLeft(nav2_costmap_2d::Costmap2D & master_grid, unsigned char * master_array, unsigned int size_x, unsigned int size_y, double resolution);
  void coverObject(nav2_costmap_2d::Costmap2D & master_grid, unsigned char * master_array, unsigned int size_x, unsigned int size_y, double resolution);
  void red(nav2_costmap_2d::Costmap2D & master_grid, unsigned char * master_array, unsigned int size_x, unsigned int size_y, double resolution);
  void green(nav2_costmap_2d::Costmap2D & master_grid, unsigned char * master_array, unsigned int size_x, unsigned int size_y, double resolution);
  void yellow(nav2_costmap_2d::Costmap2D & master_grid, unsigned char * master_array, unsigned int size_x, unsigned int size_y, double resolution);
  geometry_msgs::msg::PoseStamped makeTransform(geometry_msgs::msg::PoseStamped pose_in_);

  bool filter(std::string object);

  geometry_msgs::msg::PoseStamped goal;
  geometry_msgs::msg::PoseStamped traffic_stop;
  geometry_msgs::msg::PoseStamped traffic_stop_cam;
  geometry_msgs::msg::PoseStamped traffic_light_position;
  geometry_msgs::msg::PoseStamped pose_in_;
  geometry_msgs::msg::PoseStamped pose_out_;

  rclcpp::Duration duration;
  rclcpp::Time current_time;
  rclcpp::Time track_time;

  float closed_area_;
  float distance;
  float point_x;
  float point_y;
  float orientation_x;
  float orientation_y;

  double forbidden_zone_cost_;
  double camera_min_detection_range_;
  double camera_max_detection_range_;
  double last_min_x_;
  double last_min_y_;
  double last_max_x_;
  double last_max_y_;

  int time_to_close_;
  int filter_capacity_;

  unsigned int map_x;
  unsigned int map_y;

  bool need_recalculation_;
  bool filtered;
  bool map_coordinates_are_published;
  bool hold_onlyLeft;
  bool hold_object;
  bool red_light;
  bool goal_rviz;

  std::string object;
  std::queue<std::string> last_queue_objects;

};

}  // namespace nav2_cv_costmap_plugin

#endif  // CV_LAYER_HPP_
