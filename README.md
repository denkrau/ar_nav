# ar_nav
A ROS package offering an autonomous flight including waypoint navigation with a quadcopter using ArUco markers

## Installation
Clone repository and compile it
```
git clone https://github.com/denkrau/ar_nav.git
catkin_make ar_nav
```
This builds two nodes. The correct node for hovering is `single` and for waypoint navigation `multi`.

## Dependencies (non-native)
The following packages are necessary in order to run `ar_nav` properly:
- ar_nav
- crazyflie_ros
- tud_img_prep

## Usage
If necessary, edit the marker board files in `/data/` before running the node with
```
rosrun ar_nav single [params]
```
or
```
rosrun ar_nav multi [params]
```
Due to the many parameters, the recommended way is using the launch files with
```
roslaunch ar_nav ar_nav_prep_single.launch
```
or
```
roslaunch ar_nav ar_nav_prep_multi.launch
```

## Interaction
### Subscriber
- marker_pose `<geometry_msgs::TransformStamped>` (in `single` `<geometry_msgs::PoseStamped>`)

### Publisher
- cf_pose `<geometry_msgs::PoseStamped>`
- debug_pose `<geometry_msgs::PoseStamped>`

### Services
- next_waypoint `<std_srvs::Empty::Request&, std_srvs::Empty::Response&>`
- prev_waypoint `<std_srvs::Empty::Request&, std_srvs::Empty::Response&>`

### Parameter
- marker_pose_topic `<std::string>`
- world_frame `<std::string>`
- cf_frame `<std::string>`
- waypoints `<std::string>` (seperated by `|`)
- method `<std::string>` (either `auto` or `manual`)
