#pragma once

#include <Eigen/Eigen>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>

namespace simulated_lidar_scanner
{
/** @brief Representation of an observable object in a VTK scene */
struct SceneObject
{
  SceneObject() = default;
  SceneObject(const std::string& filename, const Eigen::Isometry3d& transform);
  SceneObject(const std::string& filename, const std::vector<double>& pose);

  /** @brief File name of the mesh resource */
  std::string filename;
  /** @brief Transform relative to the scene root frame */
  Eigen::Isometry3d transform;
};

/** @brief Creates a VTK scene from the static geometry in a URDF */
vtkSmartPointer<vtkPolyData> createVTKSceneFromURDF(const std::string& urdf_parameter = "/robot_description");

/** @brief Creates a VTK scene from a list of scene objects */
vtkSmartPointer<vtkPolyData> createVTKSceneFromSceneObjects(std::vector<SceneObject> scene_objects);

} // namespace simulated_lidar_scanner
