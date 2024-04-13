# nav2_cv_costmap_plugin
A plugin that subscribes to object detection results, marks forbidden zones, and recalculates the endpoint as needed.

## Dependencies
- ROS2 Humble

## Install
- Clone the package:
```bashrc
git clone https://github.com/45kmh/nav2_cv_costmap_plugin.git
```
# Adjust  `parameters.yaml`:
    global_costmap:
      plugins: ["static_layer", "obstacle_layer", "inflation_layer", "cv_layer"]
      cv_layer:
        plugin: nav2_cv_costmap_plugin/CvLayer
        enabled: True
        # Side length (in meters) of a rectangular forbidden zone   
        closed_area: 1.0
        # The duration of the zone's existence since it was set
        time_to_close: 20
        # Sets how many times in a row the object must be detected to set the zone
        # More reliable object detection -> smaller filter_capacity value 
        filter_capacity: 5
        camera_min_detection_range: 0.3
        camera_max_detection_range: 4.0
        # Maximum value is 1 equal to the lethal cost
        forbidden_zone_cost: 0.75
  
