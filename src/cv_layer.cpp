#include "nav2_cv_costmap_plugin/cv_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <geometry_msgs/msg/quaternion.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"

#include <cmath>
#include <tuple>
#include <string>
#include <queue>
#include <algorithm>

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_cv_costmap_plugin
{

CvLayer::CvLayer()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max()),
  duration(std::chrono::seconds(0))
{
}

void
CvLayer::onInitialize()
{
  auto node = node_.lock(); 
  ObjectPoseSub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>("/objectCoordinateInCameraFrame", 10, std::bind(&CvLayer::poseCallback, this, std::placeholders::_1));
  ObjectSub_ = node->create_subscription<std_msgs::msg::String>("/detectedObject", 10, std::bind(&CvLayer::objectCallback, this, std::placeholders::_1));
  GoalSub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&CvLayer::goalCallback, this, std::placeholders::_1));
  GoalPub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("closed_area", rclcpp::ParameterValue(1.0));
  declareParameter("time_to_close", rclcpp::ParameterValue(10));
  declareParameter("filter_capacity", rclcpp::ParameterValue(10));
  declareParameter("camera_min_detection_range", rclcpp::ParameterValue(0.3));
  declareParameter("camera_max_detection_range", rclcpp::ParameterValue(4.0));
  declareParameter("forbidden_zone_cost", rclcpp::ParameterValue(0.75));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "closed_area", closed_area_);
  node->get_parameter(name_ + "." + "time_to_close", time_to_close_);
  node->get_parameter(name_ + "." + "filter_capacity", filter_capacity_);
  node->get_parameter(name_ + "." + "camera_min_detection_range", camera_min_detection_range_);
  node->get_parameter(name_ + "." + "camera_max_detection_range", camera_max_detection_range_);
  node->get_parameter(name_ + "." + "forbidden_zone_cost", forbidden_zone_cost_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  need_recalculation_ = false;
  current_ = true;

  map_coordinates_are_published = false;
  filtered = false;
  hold_onlyLeft = false;
  hold_object = false;
  red_light = false;
  goal_rviz = false;

}

void CvLayer::updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (need_recalculation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_recalculation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
}

void CvLayer::onFootprintChanged()
{
  need_recalculation_ = true;
}

void CvLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {return;}

  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX();
  unsigned int size_y = master_grid.getSizeInCellsY();
  double resolution = master_grid.getResolution();

  if (object == "leftOnly"){onlyLeft(master_grid, master_array, size_x, size_y, resolution);} 
  else if (object == "red"){red(master_grid, master_array, size_x, size_y, resolution);}
  else if (object == "green"){green(master_grid, master_array, size_x, size_y, resolution);}
  else if (object == "yellow"){yellow(master_grid, master_array, size_x, size_y, resolution);}

  if (hold_object == true){coverObject(master_grid, master_array, size_x, size_y, resolution);}
}

void CvLayer::setCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_x_cell, int min_y_cell, int max_x_cell, int max_y_cell, unsigned int map_x, unsigned int map_y, unsigned char * master_array)
{
  for (int j = min_y_cell; j <= max_y_cell; j++) {
    for (int i = min_x_cell; i <= max_x_cell; i++) {
      // Calculate the rotated and shifted coordinates of the cell
      double rotated_x = i * orientation_x - j * orientation_y + map_x;
      double rotated_y = i * orientation_y + j * orientation_x + map_y;
      // Set the cell as a lethal obstacle
      int index = master_grid.getIndex(rotated_x, rotated_y);
      master_array[index] = LETHAL_OBSTACLE*forbidden_zone_cost_;
    }
  }
}

bool CvLayer::filter(std::string object)
{
    if (pose_in_.pose.position.x > camera_min_detection_range_ && pose_in_.pose.position.x <= camera_max_detection_range_) {distance = pose_in_.pose.position.x;} // specifies the range in which a camera provides stable detections
    else {return false;}

    if (last_queue_objects.size() == filter_capacity_) {last_queue_objects.pop();}

    if (object == "sync" && !last_queue_objects.empty()) {object = last_queue_objects.back();}

    last_queue_objects.push(object);

    // std::stringstream ss;
    // ss << "Queue Contents: ";
    // std::queue<std::string> temp = last_queue_objects;
    // while (!temp.empty())
    // {
    //     ss << temp.front() << " ";
    //     temp.pop();
    // }
    // RCLCPP_INFO(rclcpp::get_logger("nav2_cv_costmap_plugin"), "Queue Contents: %s", ss.str().c_str());

    if (last_queue_objects.size() < filter_capacity_) {return false;}

    std::string first = last_queue_objects.front();

    for (std::size_t i = 0; i < filter_capacity_; ++i) 
    {
        std::string current = last_queue_objects.front();
        last_queue_objects.pop();
        if (current != first) 
        {
            return false;
        }
        last_queue_objects.push(current);
    }
    return true;
}

void CvLayer::coverObject(nav2_costmap_2d::Costmap2D & master_grid, unsigned char * master_array, unsigned int size_x, unsigned int size_y, double resolution)
{
  if (filtered == true && map_coordinates_are_published == false)
  {
    pose_out_= makeTransform(pose_in_);
    point_x = pose_out_.pose.position.x;
    point_y = pose_out_.pose.position.y;
    tf2::Quaternion quat_tf;
    geometry_msgs::msg::Quaternion quat_msg = pose_out_.pose.orientation;
    tf2::fromMsg(quat_msg, quat_tf);
    double roll{}, pitch{}, yaw{};
    tf2::Matrix3x3 m(quat_tf);
    m.getRPY(roll, pitch, yaw);
    orientation_x = cos(yaw);
    orientation_y = sin(yaw);

    try 
    {
      master_grid.worldToMap(point_x, point_y, map_x, map_y);
    } 
      catch (const std::exception& e) 
      {
        // Handle the out-of-bounds conversion here
        RCLCPP_INFO(rclcpp::get_logger("nav2_cv_costmap_plugin"), "Error in worldToMap conversion: %s", e.what());
        return;
      }

    map_coordinates_are_published = true;
    auto node = node_.lock();
    current_time = node->now();
  }

  if (map_coordinates_are_published == true)
  {
      auto node = node_.lock();
      track_time = node->now(); 
      duration = track_time - current_time;
  }

  if (map_coordinates_are_published == true && duration.seconds() < time_to_close_)
  {
    int boundary_cells = static_cast<int>(closed_area_ / resolution);
    int min_x_cell_center = -boundary_cells / 2;
    int min_y_cell_center = -boundary_cells / 2;
    int max_x_cell_center = boundary_cells / 2;
    int max_y_cell_center = boundary_cells / 2;
    setCosts(master_grid, min_x_cell_center, min_y_cell_center, max_x_cell_center, max_y_cell_center, map_x, map_y, master_array);
    if (hold_onlyLeft == true)
    {
      int min_x_cell_right = -2*boundary_cells;
      int min_y_cell_right = -2*boundary_cells;
      int max_x_cell_right = -boundary_cells / 2;
      int max_y_cell_right = -boundary_cells / 2;
      setCosts(master_grid, min_x_cell_right, min_y_cell_right, max_x_cell_right, max_y_cell_right, map_x, map_y, master_array);
    }

  }
  else
  {
    map_coordinates_are_published = false;
    filtered = false;
    hold_object = false;
    hold_onlyLeft = false;
  }
}

void CvLayer::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  pose_in_ = *msg;
}

void CvLayer::objectCallback(const std_msgs::msg::String::SharedPtr msg)
{
  object = msg->data.c_str();
  if (filtered == false) 
  {
    filtered=filter(object);
    // RCLCPP_INFO(rclcpp::get_logger("nav2_cv_costmap_plugin"), "Filter returns: %s", filtered ? "true" : "false");
    }
}

void CvLayer::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (msg && msg->pose.position.z == 0.0)
  {
    goal = *msg;
    goal_rviz = true;
  }
}

void CvLayer::onlyLeft(nav2_costmap_2d::Costmap2D & master_grid, unsigned char * master_array, unsigned int size_x, unsigned int size_y, double resolution)
{
  if (filtered == true)
  {
    hold_object = true;
    hold_onlyLeft = true;
  }
}

void CvLayer::red(nav2_costmap_2d::Costmap2D & master_grid, unsigned char * master_array, unsigned int size_x, unsigned int size_y, double resolution)
{
  if (filtered == true){hold_object = true;}
  if (red_light == false && distance < camera_max_detection_range_ && distance > camera_min_detection_range_ && goal_rviz == true)
  {
    traffic_stop_cam =  pose_in_;
    traffic_stop_cam.pose.position.x = distance*0.2;
    traffic_stop = makeTransform(traffic_stop_cam);
    traffic_light_position = makeTransform(pose_in_);

    double stop_x = traffic_light_position.pose.position.x;
    double stop_y = traffic_light_position.pose.position.y;
    double object_x = traffic_stop.pose.position.x;
    double object_y = traffic_stop.pose.position.y;
    double dx = stop_x - object_x;
    double dy = stop_y - object_y;
    double angle = atan2(dy, dx);
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, angle); 
    geometry_msgs::msg::Quaternion ros_quaternion = tf2::toMsg(quaternion);
    traffic_stop.pose.orientation = ros_quaternion;
    traffic_stop.pose.position.z = 0.1;
    GoalPub_->publish(traffic_stop);
    red_light = true;
  }
}

void CvLayer::yellow(nav2_costmap_2d::Costmap2D & master_grid, unsigned char * master_array, unsigned int size_x, unsigned int size_y, double resolution)
{
  if (filtered == true){hold_object = true;}
}

void CvLayer::green(nav2_costmap_2d::Costmap2D & master_grid, unsigned char * master_array, unsigned int size_x, unsigned int size_y, double resolution)
{
  if (filtered == true){hold_object = true;}
  if (red_light == true)
  {
    red_light = false;
    GoalPub_->publish(goal);
  }
}

geometry_msgs::msg::PoseStamped CvLayer::makeTransform(geometry_msgs::msg::PoseStamped pose_in_){
  try 
  {     
    geometry_msgs::msg::PoseStamped pose_out_;
    tf_buffer_->transform<geometry_msgs::msg::PoseStamped>(pose_in_, pose_out_, "map",
        tf2::Duration(std::chrono::seconds(1)));
    pose_out_.pose.position.z = 0.0;
    return pose_out_;
  } 
    catch (const std::exception &ex) 
    {
      RCLCPP_INFO(rclcpp::get_logger("nav2_cv_costmap_plugin"), "An error occurred: %s", ex.what());
      throw std::runtime_error("Error transforming pose");
    }
}

}  // namespace nav2_gradient_costmap_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_cv_costmap_plugin::CvLayer, nav2_costmap_2d::Layer)
