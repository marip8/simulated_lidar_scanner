#include <ros/ros.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf2_ros/transform_broadcaster.h>

visualization_msgs::Marker makeVisualMarker(std::string& scanner_frame)
{
  static int idx = 0;
  visualization_msgs::Marker marker;
  marker.header.frame_id = scanner_frame;
  marker.header.stamp = ros::Time(0);
  marker.id = idx++;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 0.075;
  marker.scale.y = 0.075;
  marker.scale.z = 0.075;

  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;

  marker.pose.orientation.w = 1.0;

  return marker;
}

std::vector<visualization_msgs::InteractiveMarkerControl> make6DOFControls()
{
  std::vector<visualization_msgs::InteractiveMarkerControl> controls;

  // Move and rotate X-axis
  {
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1.0 / std::sqrt(2);
    control.orientation.x = 1.0 / std::sqrt(2);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    controls.push_back(control);

    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    controls.push_back(control);
  }

  // Move and rotate Y-axis
  {
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1.0 / std::sqrt(2);
    control.orientation.y = 1.0 / std::sqrt(2);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    controls.push_back(control);

    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    controls.push_back(control);
  }

  // Move and rotate Z-axis
  {
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1.0 / std::sqrt(2);
    control.orientation.z = 1.0 / std::sqrt(2);

    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    controls.push_back(control);

    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    controls.push_back(control);
  }

  return controls;
}

visualization_msgs::InteractiveMarker makeInteractiveMarker(std::string& scanner_frame,
                                                            std::string& world_frame)
{
  visualization_msgs::InteractiveMarker m;
  m.header.stamp = ros::Time(0);
  m.header.frame_id = world_frame;
  m.scale = 1.0;
  m.name = scanner_frame;

  m.pose.position.x = m.pose.position.y = m.pose.position.z = 0.0;
  m.pose.orientation.x = m.pose.orientation.y = m.pose.orientation.z = 0.0;
  m.pose.orientation.w = 1.0;

  // Visual marker
  visualization_msgs::Marker visual = makeVisualMarker(scanner_frame);

  // Controls
  m.controls = make6DOFControls();
  {
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1.0;
    control.always_visible = true;
    control.markers.push_back(visual);
    m.controls.push_back(control);
  }

  return m;
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "scanner_relocator");

  // Set up ROS node handle
  ros::NodeHandle pnh("~");

  // Get ROS parameters
  std::string world_frame;
  if(!pnh.getParam("world_frame", world_frame))
  {
    ROS_FATAL("'world_frame' parameter must be set");
    return 1;
  }

  std::vector<std::string> scanner_frames;
  if(!pnh.getParam("scanner_frames", scanner_frames))
  {
    ROS_FATAL("'scanner_frames' parameter must be set");
    return 1;
  }

  // Set up TF broadcaster for changing scanner frames
  tf2_ros::TransformBroadcaster broadcaster;

  // Set up interactive marker server
  interactive_markers::InteractiveMarkerServer server("scanner_relocator");

  // Create an interactive marker for each of the remaining scanners
  for(size_t i = 0; i < scanner_frames.size(); ++i)
  {
    visualization_msgs::InteractiveMarker int_marker = makeInteractiveMarker(scanner_frames[i], world_frame);
    server.insert(int_marker);
  }
  server.applyChanges();
  ros::spinOnce();

  // Loop
  ros::Rate loop(10.0f);
  std::size_t seq = 0;
  while(ros::ok())
  {
    for(size_t i = 0; i < scanner_frames.size(); ++i)
    {
      visualization_msgs::InteractiveMarker int_marker;
      server.get(scanner_frames[i], int_marker);

      geometry_msgs::TransformStamped transform;
      transform.header.frame_id = world_frame;
      transform.header.stamp = ros::Time::now();
      transform.header.seq = seq;
      transform.child_frame_id = scanner_frames[i];

      transform.transform.translation.x = int_marker.pose.position.x;
      transform.transform.translation.y = int_marker.pose.position.y;
      transform.transform.translation.z = int_marker.pose.position.z;

      transform.transform.rotation.w = int_marker.pose.orientation.w;
      transform.transform.rotation.x = int_marker.pose.orientation.x;
      transform.transform.rotation.y = int_marker.pose.orientation.y;
      transform.transform.rotation.z = int_marker.pose.orientation.z;

      broadcaster.sendTransform(transform);
    }

    ++seq;

    ros::spinOnce();
    loop.sleep();
  }

  return 0;
}
