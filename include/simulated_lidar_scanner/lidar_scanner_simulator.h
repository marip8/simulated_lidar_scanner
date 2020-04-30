#ifndef LIDAR_SCANNER_SIMULATOR_H
#define LIDAR_SCANNER_SIMULATOR_H

#include <simulated_lidar_scanner/synthetic_lidar_scanner/vtkLidarScanner.h>
#include <simulated_lidar_scanner/scanner_params.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class LidarScannerSim
{
public:
  LidarScannerSim(const scanner_params& sim);

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

#endif // LIDAR_SCANNER_SIMULATOR_H
