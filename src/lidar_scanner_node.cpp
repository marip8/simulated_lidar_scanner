#include <simulated_lidar_scanner/lidar_scanner_simulator.h>
#include <simulated_lidar_scanner/scene_builder.h>

#include <memory>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace simulated_lidar_scanner;

template <typename T>
T get(const ros::NodeHandle& nh, const std::string& key)
{
  T val;
  if (!nh.getParam(key, val))
    throw std::runtime_error("Failed to get '" + key + "' parameter");
  return val;
}

std::vector<SceneObject> getParams(const XmlRpc::XmlRpcValue& resources)
{
  std::vector<SceneObject> scene_objects;

  for (int i = 0; i < resources.size(); ++i)
  {
    const XmlRpc::XmlRpcValue& mesh = resources[i];
    const XmlRpc::XmlRpcValue& file = mesh["file"];
    const XmlRpc::XmlRpcValue& pose = mesh["pose"];
    const std::string path = static_cast<std::string>(file);
    std::vector<double> vals;
    for (int j = 0; j < 6; ++j)
    {
      double val = static_cast<double>(pose[j]);
      vals.push_back(val);
    }

    SceneObject obj(path, vals);
    scene_objects.push_back(obj);
  }

  return scene_objects;
}

ScannerParams loadScannerParams(const ros::NodeHandle& nh)
{
  ScannerParams sim;
  sim.theta_span = get<double>(nh, "scanner/theta_span") * M_PI / 180.0;
  sim.phi_span = get<double>(nh, "scanner/phi_span") * M_PI / 180.0;
  sim.theta_points = get<int>(nh, "scanner/theta_points");
  sim.phi_points = get<int>(nh, "scanner/phi_points");
  sim.los_variance = get<double>(nh, "scanner/los_variance");
  sim.orthogonal_variance = get<double>(nh, "scanner/orthogonal_variance");
  sim.max_incidence_angle = get<double>(nh, "scanner/max_incidence_angle") * M_PI / 180.0;
  sim.max_distance = get<double>(nh, "scanner/max_distance");
  return sim;
}

int main(int argc, char** argv)
{
  try
  {
    // Set up ROS.
    ros::init(argc, argv, "lidar_sim_node");

    // Set up ROS node handle
    ros::NodeHandle nh, pnh("~");

    // Load the general parameters
    const std::string world_frame = get<std::string>(pnh, "world_frame");
    const std::string scanner_frame = get<std::string>(pnh, "scanner_frame");
    const ros::Rate rate(get<double>(pnh, "scan_frequency"));
    const bool z_out = get<bool>(pnh, "z_out");

    // Get scanner parameters
    const ScannerParams sim = loadScannerParams(pnh);

    // Create a ROS tf listener to get the scanner frame
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    // Create scanner class
    LidarScannerSim scanner(sim, z_out);

    // Create and set the scanner scene
    try
    {
      std::vector<SceneObject> scene_objects = getParams(get<XmlRpc::XmlRpcValue>(pnh, "resources"));
      scanner.setScannerScene(createVTKSceneFromSceneObjects(scene_objects));
    }
    catch (const std::exception& ex)
    {
      ROS_WARN("Failed to parse mesh resource parameters");
      ROS_WARN("Building scene from static geometry in URDF");
      scanner.setScannerScene(createVTKSceneFromURDF());
    }

    ROS_INFO_STREAM("Simulated LIDAR scanner initialization completed. Looking up data from scanner frame '"
                    << scanner_frame << "' in world frame '" << world_frame << "'. Beginning data acquisition...");

    // Create a ROS publisher to publish the scan data
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::PointCloud2>("sensor_data/" + scanner_frame, 1, true);

    ros::TimerCallback cb = [&](const ros::TimerEvent&) -> void {
      // Get TF frame from absolute static world origin to scanner focal point
      geometry_msgs::TransformStamped transform;
      try
      {
        // Set scanner transform and dynamically changing elements of the scene
        scanner.getNewScanData(tf2::transformToEigen(buffer.lookupTransform(world_frame, scanner_frame, ros::Time(0))));
        pcl::PointCloud<pcl::PointNormal>::Ptr data = scanner.getScanDataPointCloud();

        // Convert scan data from private class object to ROS msg
        sensor_msgs::PointCloud2 scan_data_msg;
        pcl::toROSMsg(*data, scan_data_msg);
        scan_data_msg.header.stamp = ros::Time::now();
        scan_data_msg.header.frame_id = scanner_frame;
        scan_pub.publish(scan_data_msg);
      }
      catch (tf2::TransformException ex)
      {
        ROS_ERROR_STREAM(ex.what());
      }
    };

    ros::Timer timer = nh.createTimer(rate.expectedCycleTime(), cb);
    ros::spin();
  }
  catch (const std::exception& ex)
  {
    std::cerr << ex.what() << std::endl;
    return -1;
  }

  return 0;
}
