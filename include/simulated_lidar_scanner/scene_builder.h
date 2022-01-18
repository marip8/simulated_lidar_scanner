#pragma once

#include <Eigen/Eigen>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <urdf/model.h>
#include <visualization_msgs/MarkerArray.h>

namespace simulated_lidar_scanner
{
struct SceneObject
{
  SceneObject() = default;
  SceneObject(const std::string& filename, const Eigen::Isometry3d& transform);
  SceneObject(const std::string& filename, const std::vector<double>& pose);

  std::string filename;
  Eigen::Isometry3d transform;
};

class SceneBuilder
{
public:
  SceneBuilder();

  void createVTKSceneFromURDF();

  void createVTKSceneFromMeshResources(const std::vector<SceneObject>& scene_objects);

  inline visualization_msgs::MarkerArray getMarkerArray() const
  {
    return mesh_resource_marker_array_;
  }

  vtkSmartPointer<vtkPolyData> scene;

private:
  void getSceneGeometry();

  void getLinkGeometry(const std::vector<urdf::VisualSharedPtr>& root_link_visuals, const urdf::Pose& joint_pose);

  void getLinkChildGeometry(const std::vector<urdf::JointSharedPtr>& joints);

  void changeFilenames();

  void vtkSceneFromMeshFiles();

  void meshResourcesToVTK();

  std::vector<SceneObject> scene_data_;
  urdf::Model urdf_model_;
  visualization_msgs::MarkerArray mesh_resource_marker_array_;
};

} // namespace simulated_lidar_scanner
