#include <simulated_lidar_scanner/scene_builder.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <urdf/model.h>
#include <boost/filesystem.hpp>

// VTK Utils
#include <vtkSTLReader.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkAppendPolyData.h>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <urdf/model.h>

namespace
{
  vtkSmartPointer<vtkPolyData> readSTLFile(const std::string& file)
  {
    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(file.c_str());
    reader->SetMerging(1);
    reader->Update();

    return reader->GetOutput();
  }

  vtkSmartPointer<vtkTransform> urdfPoseToVTKTransform(const urdf::Pose& pose)
  {
    urdf::Rotation q = pose.rotation;
    urdf::Vector3 p = pose.position;
    double w, x, y, z, denom;

    if(q.w > 1) {q.normalize();}
    w = 2*std::acos(q.w);
    denom = std::sqrt(1 - q.w*q.w);
    if(denom < 0.001)
    {
      // Choose an arbitrary axis since the rotation angle ~= 0
      x = 0.0; //q.x;
      y = 0.0; //q.y;
      z = 1.0; //q.z;
    }
    else
    {
      x = q.x / denom;
      y = q.y / denom;
      z = q.z / denom;
    }

    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->Translate(p.x, p.y, p.z);
    transform->RotateWXYZ(w*180/M_PI, x, y, z);

    return transform;
  }

  Eigen::Isometry3d urdfPoseToEigen(const urdf::Pose& pose)
  {
    Eigen::Isometry3d transform (Eigen::Isometry3d::Identity());
    transform.translate(Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z));
    transform.rotate(Eigen::Quaterniond(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z));

    return transform;
  }

  vtkSmartPointer<vtkTransform> eigenTransformToVTK(const Eigen::Isometry3d& transform)
  {
    vtkSmartPointer<vtkTransform> vtk_transform = vtkSmartPointer<vtkTransform>::New();

    Eigen::Translation3d translation (transform.translation());
    vtk_transform->Translate(translation.x(), translation.y(), translation.z());

    Eigen::AngleAxisd rotation (transform.rotation());
    vtk_transform->RotateWXYZ(rotation.angle()*180.0/M_PI, rotation.axis()[0], rotation.axis()[1], rotation.axis()[2]);

    return vtk_transform;
  }

//  visualization_msgs::Marker createMeshResourceMarker(const std::string& filename,
//                                                      const Eigen::Isometry3d& transform,
//                                                      const std::string& frame,
//                                                      const int id)
//  {
//    visualization_msgs::Marker marker;
//    marker.header.frame_id = frame;
//    marker.header.stamp = ros::Time::now();
//    marker.id = id;

//    marker.action = marker.ADD;
//    marker.type = marker.MESH_RESOURCE;
//    marker.lifetime = ros::Duration(0);
//    marker.frame_locked = false;
//    marker.mesh_resource = filename;
//    marker.mesh_use_embedded_materials = true;

//    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;

//    Eigen::Translation3d translation (transform.translation());
//    Eigen::AngleAxisd rotation (transform.rotation());

//    marker.pose.position.x = translation.x();
//    marker.pose.position.x = translation.y();
//    marker.pose.position.x = translation.z();
//    marker.pose.orientation.w = rotation.angle();
//    marker.pose.orientation.x = rotation.axis()[0];
//    marker.pose.orientation.y = rotation.axis()[1];
//    marker.pose.orientation.z = rotation.axis()[2];

//    return marker;
//  }
}

namespace simulated_lidar_scanner
{
SceneObject::SceneObject(const std::string& filename, const Eigen::Isometry3d& transform) : filename(filename), transform(transform)
{
}

SceneObject::SceneObject(const std::string& filename, const std::vector<double>& pose)
  : filename(filename), transform(Eigen::Isometry3d::Identity())
{
  if (pose.size() != 6)
  {
    throw std::runtime_error("Pose does not have 6 elements!");
  }
  transform.translate(Eigen::Vector3d(pose[0], pose[1], pose[2]));
  transform.rotate(Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX()));
  transform.rotate(Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()));
  transform.rotate(Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ()));
}

std::vector<SceneObject> getLinkGeometry(const std::vector<urdf::VisualSharedPtr>& visuals,
                                         const urdf::Pose& joint_pose)
{
  std::vector<SceneObject> objects;
  for (auto it = visuals.begin(); it != visuals.end(); ++it)
  {
    urdf::VisualConstSharedPtr vis = *it;
    if (vis->geometry->type == urdf::Geometry::MESH)
    {
      Eigen::Isometry3d link_transform = urdfPoseToEigen(vis->origin);
      Eigen::Isometry3d joint_transform = urdfPoseToEigen(joint_pose);
      Eigen::Isometry3d transform(joint_transform * link_transform);

      SceneObject obj(std::dynamic_pointer_cast<const urdf::Mesh>(vis->geometry)->filename, transform);
      objects.push_back(obj);
    }
  }
  return objects;
}

std::vector<SceneObject> getLinkChildGeometry(const std::vector<urdf::JointSharedPtr>& joints,
                                              const urdf::Model& urdf_model_)
{
  std::vector<SceneObject> objects;

  for (auto it = joints.begin(); it != joints.end(); ++it)
  {
    urdf::JointSharedPtr joint = *it;
    if (joint->type == urdf::Joint::FIXED)
    {
      urdf::LinkSharedPtr link;
      urdf_model_.getLink(joint->child_link_name, link);
      std::vector<SceneObject> link_objects =
          getLinkGeometry(link->visual_array, joint->parent_to_joint_origin_transform);

      objects.insert(objects.end(), link_objects.begin(), link_objects.end());

      // Check if this static link has static link children
      if (link->child_joints.size() > 0)
      {
        std::vector<SceneObject> child_objects = getLinkChildGeometry(link->child_joints, urdf_model_);
        objects.insert(objects.end(), child_objects.begin(), child_objects.end());
      }
    }
  }

  return objects;
}

std::vector<SceneObject> changeFilenames(std::vector<SceneObject> scene_data_)
{
  // Create a vector of iterators to filenames that aren't in the correct format
  std::vector<std::vector<SceneObject>::iterator> erase_its;

  // Define the word to search for
  const std::string prefix = "package://";

  // Iterate through all filenames to find the defined prefix
  for (auto it = scene_data_.begin(); it != scene_data_.end(); ++it)
  {
    // Create a reference to the current file name
    SceneObject& obj = *it;

    // Find the prefix to remove in the current filename
    size_t pos = obj.filename.find(prefix);
    if (pos != std::string::npos)
    {
      // Erase prefix from filename
      obj.filename.erase(pos, prefix.length());

      // Find the first "/" in the filename (indicating end of package name)
      size_t start_pos = obj.filename.find_first_of("/", 0);

      // Create package name string and get the full package name file path
      std::string pkg_name = obj.filename.substr(pos, start_pos);
      std::string pkg_path = ros::package::getPath(pkg_name);
      if (pkg_path.length() == 0)
      {
        // Save the iterator to the filename if the ROS package where it came from is not found
        ROS_INFO("Package not found: %s", pkg_name.c_str());
        auto erase_it = std::find_if(scene_data_.begin(), scene_data_.end(),
                                     [&obj](const SceneObject& x) { return x.filename == obj.filename; });
        erase_its.push_back(erase_it);
        continue;
      }

      // Erase the package name from the front of the filename and replace with full package path
      obj.filename.erase(0, start_pos);
      obj.filename.insert(obj.filename.begin(), pkg_path.begin(), pkg_path.end());
    }
    else
    {
      // Save the iterator to the filename if it doesn't start with the defined prefix
      ROS_INFO("Filename not in correct format: %s", obj.filename.c_str());
      auto erase_it = std::find_if(scene_data_.begin(), scene_data_.end(),
                                   [&obj](const SceneObject& x) { return x.filename == obj.filename; });
      erase_its.push_back(erase_it);
    }
  }

  // Erase all filenames that weren't in the correct format
  for (auto it = erase_its.begin(); it != erase_its.end(); ++it)
  {
    scene_data_.erase(*it);
  }

  return scene_data_;
}

std::vector<SceneObject> getSceneGeometry(const urdf::Model& urdf_model_)
{
  // Get the root link of the URDF
  urdf::LinkConstSharedPtr root_link = urdf_model_.getRoot();
  if (!root_link)
    throw std::runtime_error("No root link set in URDF");

  // Check if the root link has any mesh geometry
  std::vector<urdf::VisualSharedPtr> root_link_visuals = root_link->visual_array;
  urdf::Pose base_pose;
  base_pose.position.x = base_pose.position.y = base_pose.position.z = 0.0f;
  base_pose.rotation.w = base_pose.rotation.x = base_pose.rotation.z = base_pose.rotation.w = 0.0f;
  std::vector<SceneObject> link_objects = getLinkGeometry(root_link_visuals, base_pose);

  // Check joints of the root link for static links
  std::vector<urdf::JointSharedPtr> root_joints = root_link->child_joints;
  std::vector<SceneObject> child_objects = getLinkChildGeometry(root_joints, urdf_model_);

  // Resolve package:// URI in filenames to get full file paths
  std::vector<SceneObject> objects;
  objects.insert(objects.end(), link_objects.begin(), link_objects.end());
  objects.insert(objects.end(), child_objects.begin(), child_objects.end());
  return changeFilenames(objects);
}

vtkSmartPointer<vtkPolyData> vtkSceneFromMeshFiles(const std::vector<SceneObject>& scene_data_)
{
  if (scene_data_.empty())
    throw std::runtime_error("Scene data object is empty");

  vtkSmartPointer<vtkAppendPolyData> append_filter = vtkSmartPointer<vtkAppendPolyData>::New();
  for (auto it = scene_data_.begin(); it != scene_data_.end(); ++it)
  {
    boost::filesystem::path path(it->filename);
    if (!boost::filesystem::exists(path))
      throw std::runtime_error("File at '" + it->filename + "' does not exist");

    vtkSmartPointer<vtkPolyData> vtk_poly = readSTLFile(it->filename);
    if(!vtk_poly)
      throw std::runtime_error("Unable to read file '" + it->filename + "'");

    // Apply the input transformation to the VTK poly data
    vtkSmartPointer<vtkTransformPolyDataFilter> transform_filter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    vtkSmartPointer<vtkTransform> transform = eigenTransformToVTK(it->transform);
    transform_filter->SetInputData(vtk_poly);
    transform_filter->SetTransform(transform);
    transform_filter->Update();

    // Append this VTK object to the previous geometries
    append_filter->AddInputData(transform_filter->GetOutput());
  }
  append_filter->Update();

  return append_filter->GetOutput();
}

vtkSmartPointer<vtkPolyData> createVTKSceneFromURDF(const std::string& urdf_parameter)
{
  urdf::Model urdf_model_;
  if (!urdf_model_.initParam(urdf_parameter))
    throw std::runtime_error("URDF parameter '" + urdf_parameter + "' must be set");

  // Get filenames for the meshes of all static links
  std::vector<SceneObject> scene_data_ = getSceneGeometry(urdf_model_);
  if (scene_data_.empty())
    throw std::runtime_error("No static geometry present in URDF");

  return vtkSceneFromMeshFiles(scene_data_);
}

vtkSmartPointer<vtkPolyData> createVTKSceneFromSceneObjects(std::vector<SceneObject> scene_objects)
{
  // Replace package URI in filenames
  return vtkSceneFromMeshFiles(changeFilenames(scene_objects));
}

} // namespace simulated_lidar_scanner
