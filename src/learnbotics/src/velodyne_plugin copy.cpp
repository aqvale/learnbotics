#include <algorithm>
#include <assert.h>

#include <../include/velodyne_plugin.hpp>

#include <ros/ros.h>

namespace gazebo
{
  VelodynePlugin::VelodynePlugin(){}

  void VelodynePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    this->model = _model;

    ROS_WARN("Init Velodyne Plugin");
  }

  GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}