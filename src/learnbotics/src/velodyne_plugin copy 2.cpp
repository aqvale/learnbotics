#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <functional>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ignition/math/Vector3.hh>

#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

#include "std_msgs/Float32.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <thread>

#include <ros/console.h>


using namespace std;

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class ModelRobot : public ModelPlugin
  {
    
    /// \brief A node used for transport
    private: transport::NodePtr node;

      /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;

      /// \brief Pointer to the model.
    private: physics::ModelPtr model;

      /// \brief Pointer to the joint.
    private: physics::JointPtr joint;

      /// \brief A PID controller for the joint.
    private: common::PID pid;

    /// \brief Constructor
    public: ModelRobot() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      this->model = _parent;
      // Listen to the update event. This event is broadcast every
      // simulation iteration.

      // Just output a message for now
      ROS_WARN("Loaded ModelPush Plugin with parent...");
      ROS_WARN("The plugin is attach to model %s", model->GetName().c_str());
      ROS_WARN("Counts joints %d", model->GetJointCount());
      this->joint = model->GetJoint("capa_JOINT_wheel_right");

    }

    public: void SetVelocity(const double &_vel)
    {
      // Set the joint's target velocity.
      this->model->GetJointController()->SetVelocityTarget(
          this->joint->GetScopedName(), _vel);
    }

    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    private: void OnMsg(ConstVector3dPtr &_msg)
    {
      this->SetVelocity(_msg->x());
    }

  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(ModelRobot)
}
#endif