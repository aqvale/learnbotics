#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <map>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

  class Joint;
  class Entity;

  class VelodynePlugin : public ModelPlugin {

    enum OdomSource
    {
        ENCODER = 0,
        WORLD = 1,
    };
    public:
      VelodynePlugin();
      ~VelodynePlugin();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      void SetVelocity();

    private:
      void OnMsg(ConstVector3dPtr &_msg);

      /// \brief A node used for transport
      transport::NodePtr node;

      /// \brief A subscriber to a named topic.
      transport::SubscriberPtr sub;

      /// \brief Pointer to the model.
      physics::ModelPtr model;

      /// \brief Pointer to the joint.
      physics::JointPtr joint;

      /// \brief A PID controller for the joint.
      common::PID pid;
  };

}

#endif
