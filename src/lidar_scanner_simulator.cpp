#include <simulated_lidar_scanner/lidar_scanner_simulator.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <vtkTransform.h>
#include <vtkPointData.h>

namespace
{
pcl::PointCloud<pcl::PointNormal> vtkToPCL(vtkPolyData *pdata)
{
  pcl::PointCloud<pcl::PointNormal> cloud;
  cloud.points.reserve(pdata->GetPoints()->GetNumberOfPoints());

  for (int i = 0; i < pdata->GetPoints()->GetNumberOfPoints(); ++i)
  {
    pcl::PointNormal pt;
    const double *ptr = pdata->GetPoints()->GetPoint(i);
    pt.x = ptr[0];
    pt.y = ptr[1];
    pt.z = ptr[2];

    const double *norm = pdata->GetPointData()->GetNormals()->GetTuple(i);
    pt.normal_x = norm[0];
    pt.normal_y = norm[1];
    pt.normal_z = norm[2];

    cloud.push_back(pt);
  }

  return cloud;
}

bool incidenceFilter(const pcl::PointNormal& pt,
                     const double max_angle)
{
  Eigen::Vector3d ray (pt.x, pt.y, pt.z);
  Eigen::Vector3d normal (pt.normal_x, pt.normal_y, pt.normal_z);
  const double c_theta = normal.dot(ray.normalized());
  return (c_theta > max_angle);
}

bool distanceFilter(const pcl::PointNormal& pt,
                    const double max_dist)
{
  const double dist2 = pt.x*pt.x + pt.y*pt.y + pt.z*pt.z;
  return (dist2 > max_dist*max_dist);
}

} // namespace anonymous

namespace simulated_lidar_scanner
{
LidarScannerSim::LidarScannerSim(const ScannerParams& sim)
{
  // Initialize scanner and scan data
  scanner_ = vtkSmartPointer<vtkLidarScanner>::New();
  scan_data_vtk_ = vtkSmartPointer<vtkPolyData>::New();
  scan_data_cloud_.reset(new pcl::PointCloud<pcl::PointNormal> ());

  // Set scanner parameters
  scanner_->SetPhiSpan(sim.phi_span);
  scanner_->SetThetaSpan(sim.theta_span);
  scanner_->SetNumberOfThetaPoints(sim.theta_points);
  scanner_->SetNumberOfPhiPoints(sim.phi_points);
  scanner_->SetLOSVariance(sim.los_variance);
  scanner_->SetOrthogonalVariance(sim.orthogonal_variance);
  scanner_->SetStoreRays(false);
  scanner_->SetCreateMesh(false);

  if(sim.max_incidence_angle > 0.0)
  {
    enable_incidence_filter_ = true;
    max_incidence_angle_ = -std::cos(sim.max_incidence_angle);
  }

  if(sim.max_distance > 0.0)
  {
    enable_distance_filter_ = true;
    max_distance_ = sim.max_distance;
  }
}

void LidarScannerSim::setScannerTransform(const Eigen::Isometry3d& frame)
{
  vtkSmartPointer<vtkMatrix4x4> vtk_matrix = vtkSmartPointer<vtkMatrix4x4>::New();

  for (unsigned row = 0; row < 3; ++row)
  {
    for(unsigned int col = 0; col < 4; ++col)
    {
      vtk_matrix->SetElement(row, col, frame.matrix()(row, col));
    }
  }

  // Create transform from matrix; rotate such that sensor axis is Z axis
  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  transform->SetMatrix(vtk_matrix);
  transform->RotateX(90.0f);

  // Set and update transform in scanner object
  scanner_->SetTransform(transform);
}

void LidarScannerSim::getNewScanData(const Eigen::Isometry3d& scanner_transform)
{
  // Set the scanner transform
  setScannerTransform(scanner_transform);

  // Perform the scan and get the points
  scanner_->PerformScan();
  scanner_->GetValidOutputPoints(scan_data_vtk_);

  pcl::PointCloud<pcl::PointNormal> cloud = vtkToPCL(scan_data_vtk_);

  // Transform point cloud into scanner frame
  pcl::transformPointCloudWithNormals(cloud, *scan_data_cloud_, scanner_transform.inverse().matrix());

  // Cull points whose angle of incidence is greater than the specified tolerance
  if(enable_incidence_filter_)
  {
    scan_data_cloud_->erase(std::remove_if(scan_data_cloud_->points.begin(),
                                           scan_data_cloud_->points.end(),
                                           boost::bind(incidenceFilter, _1, max_incidence_angle_)),
                            scan_data_cloud_->end());
  }

  if(enable_distance_filter_)
  {
    scan_data_cloud_->erase(std::remove_if(scan_data_cloud_->points.begin(),
                                           scan_data_cloud_->points.end(),
                                           boost::bind(distanceFilter, _1, max_distance_)),
                            scan_data_cloud_->end());
  }
}

} // namespace simulated_lidar_scanner
