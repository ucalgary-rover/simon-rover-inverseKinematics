#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class VelodynePlugin : public ModelPlugin
  {
    /// Constructor
    public: VelodynePlugin() {}
    
    /// Handle an incoming message from ROS
    public: void OnRosMsg(const std_msgs::msg::Float32::SharedPtr _msg)
    {
      this->SetVelocity(_msg->data);
    }

    /// ROS helper function that processes messages
    private: void QueueThread()
    {
      // Spin the ROS executor
      rclcpp::spin(std::make_shared<rclcpp::Node>("gazebo_client"));
    }

    /// The load function is called by Gazebo when the plugin is inserted into simulation
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Safety check
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
        return;
      }

      // Store the model pointer for convenience.
      this->model = _model;

      // Get the first joint. We are making an assumption about the model
      // having one joint that is the rotational joint.
      this->joint = _model->GetJoints()[0];

      // Setup a P-controller, with a gain of 0.1.
      this->pid = common::PID(0.1, 0, 0);

      // Apply the P-controller to the joint.
      this->model->GetJointController()->SetVelocityPID(
          this->joint->GetScopedName(), this->pid);

      // Default to zero velocity
      double velocity = 0;

      // Check that the velocity element exists, then read the value
      if (_sdf->HasElement("velocity"))
        velocity = _sdf->Get<double>("velocity");

      this->SetVelocity(velocity);

      // Create the node
      this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
      this->node->Init(this->model->GetWorld()->Name());

      // Create a topic name
      std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";

      // Subscribe to the topic, and register a callback
      this->sub = this->node->Subscribe(topicName,
         &VelodynePlugin::OnMsg, this);
         
      // Spin up the queue helper thread.
      this->rosQueueThread = std::thread(std::bind(&VelodynePlugin::QueueThread, this));
    }

    /// Set the velocity of the Velodyne
    public: void SetVelocity(const double &_vel)
    {
      // Set the joint's target velocity.
      this->model->GetJointController()->SetVelocityTarget(
          this->joint->GetScopedName(), _vel);
    }

    /// Handle incoming message
    private: void OnMsg(ConstVector3dPtr &_msg)
    {
      this->SetVelocity(_msg->x());
    }

    /// Pointer to the model.
    private: physics::ModelPtr model;

    /// Pointer to the joint.
    private: physics::JointPtr joint;

    /// A PID controller for the joint.
    private: common::PID pid;
    
    /// Pointer to the transport node
    private: transport::NodePtr node;

    /// A subscriber to a named topic.
    private: transport::SubscriberPtr sub;

    /// A ROS callback queue that helps process messages
    private: rclcpp::CallbackGroup::SharedPtr rosQueue;

    /// A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
  };

  // Register the plugin with Gazebo
  GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
#endif

