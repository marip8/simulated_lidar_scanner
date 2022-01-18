#pragma once

#include <simulated_lidar_scanner/synthetic_lidar_scanner/vtkLidarScanner.h>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace simulated_lidar_scanner
{
struct ScannerParams
{
  /** @brief Horizontal span of the field of view of the sensor (rad) */
  double theta_span;
  /** @brief Vertical span of the field of view of the sensor (rad) */
  double phi_span;
  /** @brief Sensor noise along the line of sight (LOS) direction (m)*/
  double los_variance;
  /** @brief Sensor noise in the direction orthogonal to the line of sight (m) */
  double orthogonal_variance;
  /** @brief Maximum angle of incidence to a surface that will result in data (rad). */
  double max_incidence_angle;
  /** @brief Maxmium measurement distance of the sensor (m) */
  double max_distance;
  /** @brief Number of sensor data points in the horizontal (theta) field of view span of the sensor */
  int theta_points;
  /** @brief Number of sensor data points in the vertical (phi) field of view span of the sensor */
  int phi_points;
};

class LidarScannerSim
{
public:
  LidarScannerSim(const ScannerParams& sim);

  inline void setScannerScene(const vtkSmartPointer<vtkPolyData> &scene)
  {
    scanner_->SetScene(scene);
  }

  void setScannerTransform(const Eigen::Isometry3d& frame);

  void getNewScanData(const Eigen::Isometry3d& scanner_transform);

  inline vtkSmartPointer<vtkPolyData> getScanPolyData() const
  {
    return scan_data_vtk_;
  }

  inline pcl::PointCloud<pcl::PointNormal>::Ptr getScanDataPointCloud() const
  {
    return scan_data_cloud_;
  }

private:
  vtkSmartPointer<vtkLidarScanner> scanner_;
  vtkSmartPointer<vtkPolyData> scan_data_vtk_;
  pcl::PointCloud<pcl::PointNormal>::Ptr scan_data_cloud_;

  bool enable_incidence_filter_ = false;
  double max_incidence_angle_;
  bool enable_distance_filter_ = false;
  double max_distance_;
};

} // namespace simulated_lidar_scanner
