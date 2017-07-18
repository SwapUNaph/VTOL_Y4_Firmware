/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Edited by: Swapneel Naphade, NITK, India (Tiltrotor model)
 */

#include <math.h>
#include "gazebo_motor_model.h"

namespace gazebo {

GazeboMotorModel::~GazeboMotorModel() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  use_pid_ = false;
}

void GazeboMotorModel::InitializeParams() {}

void GazeboMotorModel::Publish() {
  turning_velocity_msg_.set_data(joint_->GetVelocity(0));
  motor_velocity_pub_->Publish(turning_velocity_msg_);
}

void GazeboMotorModel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model_ = _model;

  namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  if (_sdf->HasElement("jointName"))
    joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a jointName, where the rotor is attached.\n";
  if (_sdf->HasElement("tiltJointName"))
    tilt_joint_name_ = _sdf->GetElement("tiltJointName")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a TiltJointName, where the rotor tilt axis is attached (Only for Tilt Rotors).\n";
    
  // Get the pointer to the joint.
  joint_ = model_->GetJoint(joint_name_);
  tilt_joint_= model_->GetJoint(tilt_joint_name_);
  if (joint_ == NULL)
    gzthrow("[gazebo_motor_model] Couldn't find specified joint \"" << joint_name_ << "\".");
  // setup joint control pid to control joint
  if (_sdf->HasElement("joint_control_pid"))
  {
    sdf::ElementPtr pid = _sdf->GetElement("joint_control_pid");
    double p = 0.1;
    if (pid->HasElement("p"))
      p = pid->Get<double>("p");
    double i = 0;
    if (pid->HasElement("i"))
      i = pid->Get<double>("i");
    double d = 0;
    if (pid->HasElement("d"))
      d = pid->Get<double>("d");
    double iMax = 0;
    if (pid->HasElement("iMax"))
      iMax = pid->Get<double>("iMax");
    double iMin = 0;
    if (pid->HasElement("iMin"))
      iMin = pid->Get<double>("iMin");
    double cmdMax = 3;
    if (pid->HasElement("cmdMax"))
      cmdMax = pid->Get<double>("cmdMax");
    double cmdMin = -3;
    if (pid->HasElement("cmdMin"))
      cmdMin = pid->Get<double>("cmdMin");
    pid_.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
    use_pid_ = true;
  }
  else
  {
    use_pid_ = false;
  }

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a linkName of the rotor.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_motor_model] Couldn't find specified link \"" << link_name_ << "\".");


  if (_sdf->HasElement("motorNumber"))
    motor_number_ = _sdf->GetElement("motorNumber")->Get<int>();
  else
    gzerr << "[gazebo_motor_model] Please specify a motorNumber.\n";

  if (_sdf->HasElement("turningDirection")) {
    std::string turning_direction = _sdf->GetElement("turningDirection")->Get<std::string>();
    if (turning_direction == "cw")
      turning_direction_ = turning_direction::CW;
    else if (turning_direction == "ccw")
      turning_direction_ = turning_direction::CCW;
    else
      gzerr << "[gazebo_motor_model] Please only use 'cw' or 'ccw' as turningDirection.\n";
  }
  else
    gzerr << "[gazebo_motor_model] Please specify a turning direction ('cw' or 'ccw').\n";

  getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic_, command_sub_topic_);
  getSdfParam<std::string>(_sdf, "motorSpeedPubTopic", motor_speed_pub_topic_,
                           motor_speed_pub_topic_);

  getSdfParam<double>(_sdf, "rotorDragCoefficient", rotor_drag_coefficient_, rotor_drag_coefficient_);
  getSdfParam<double>(_sdf, "rollingMomentCoefficient", rolling_moment_coefficient_,
                      rolling_moment_coefficient_);
  getSdfParam<double>(_sdf, "maxRotVelocity", max_rot_velocity_, max_rot_velocity_);
  getSdfParam<double>(_sdf, "motorConstant", motor_constant_, motor_constant_);
  getSdfParam<double>(_sdf, "momentConstant", moment_constant_, moment_constant_);

  getSdfParam<double>(_sdf, "timeConstantUp", time_constant_up_, time_constant_up_);
  getSdfParam<double>(_sdf, "timeConstantDown", time_constant_down_, time_constant_down_);
  getSdfParam<double>(_sdf, "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_, 10);

  /*
  std::cout << "Subscribing to: " << motor_test_sub_topic_ << std::endl;
  motor_sub_ = node_handle_->Subscribe<mav_msgs::msgs::MotorSpeed>("~/" + model_->GetName() + motor_test_sub_topic_, &GazeboMotorModel::testProto, this);
  */

  // Set the maximumForce on the joint. This is deprecated from V5 on, and the joint won't move.
#if GAZEBO_MAJOR_VERSION < 5
  //joint_->SetMaxForce(0, max_force_);
#endif
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboMotorModel::OnUpdate, this, _1));

  command_sub_ = node_handle_->Subscribe<mav_msgs::msgs::CommandMotorSpeed>("~/" + model_->GetName() + command_sub_topic_, &GazeboMotorModel::VelocityCallback, this);
  motor_velocity_pub_ = node_handle_->Advertise<std_msgs::msgs::Float>("~/" + model_->GetName() + motor_speed_pub_topic_, 1);

  // Create the first order filter.
  rotor_velocity_filter_.reset(new FirstOrderFilter<double>(time_constant_up_, time_constant_down_, ref_motor_rot_vel_));
}

// Protobuf test
/*
void GazeboMotorModel::testProto(MotorSpeedPtr &msg) {
  std::cout << "Received message" << std::endl;
  std::cout << msg->motor_speed_size()<< std::endl;
  for(int i; i < msg->motor_speed_size(); i++){
    std::cout << msg->motor_speed(i) <<" ";
  }
  std::cout << std::endl;
}
*/

// This gets called by the world update start event.
void GazeboMotorModel::OnUpdate(const common::UpdateInfo& _info) {
  sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = _info.simTime.Double();
  UpdateForcesAndMoments();
  Publish();
}

void GazeboMotorModel::VelocityCallback(CommandMotorSpeedPtr &rot_velocities) {
  if(rot_velocities->motor_speed_size() < motor_number_) {
    std::cout  << "You tried to access index " << motor_number_
      << " of the MotorSpeed message array which is of size " << rot_velocities->motor_speed_size() << "." << std::endl;
  } else ref_motor_rot_vel_ = std::min(static_cast<double>(rot_velocities->motor_speed(motor_number_)), static_cast<double>(max_rot_velocity_));
}

void GazeboMotorModel::UpdateForcesAndMoments() {
	physics::Link_V parent_links = link_->GetParentJointsLinks();
	
  motor_rot_vel_ = joint_->GetVelocity(0);
  
  if(tilt_joint_ != NULL)
	motor_tilt_pos_ = tilt_joint_->GetAngle(0).Radian();
  else
	motor_tilt_pos_ = 0.0;
	
	 //std::cout << joint_->GetName() << " tilt : " << motor_tilt_pos_ << std::endl;

  
  if (motor_rot_vel_ / (2 * M_PI) > 1 / (2 * sampling_time_)) {
    gzerr << "Aliasing on motor [" << motor_number_ << "] might occur. Consider making smaller simulation time steps or raising the rotor_velocity_slowdown_sim_ param.\n";
  }
  double real_motor_velocity = motor_rot_vel_ * rotor_velocity_slowdown_sim_;
  double force = real_motor_velocity * real_motor_velocity * motor_constant_;

  // scale down force linearly with forward speed
  // XXX this has to be modelled better
  math::Vector3 body_velocity = link_->GetWorldLinearVel();
  double vel = body_velocity.GetLength();
  double scalar = 1 - vel / 50.0; // at 50 m/s the rotor will not produce any force anymore
  scalar = math::clamp(scalar, 0.0, 1.0);
  force *= scalar;
  // Apply a force to the link.
  math::Vector3 relPose =  math::Vector3(link_->GetRelativePose().pos.x, link_->GetRelativePose().pos.y, link_->GetRelativePose().pos.z);
  math::Vector3 relForce = math::Vector3(force * sin(motor_tilt_pos_), 0, force * cos(motor_tilt_pos_));
  link_->AddRelativeForce(math::Vector3(0, 0, force * cos(motor_tilt_pos_)));
  parent_links.at(0)->AddRelativeForce(math::Vector3(force * sin(motor_tilt_pos_), 0, 0));
  
  //parent_links.at(0)->AddRelativeForce(relForce);
  parent_links.at(0)->AddRelativeTorque(relPose.Cross(math::Vector3(force * sin(motor_tilt_pos_), 0, 0)));
  
  //std::cout << "Base Link Force : "<<parent_links.at(0)->GetRelativeForce()<< std::endl;
  //parent_links.at(0)->AddForceAtRelativePosition(math::Vector3(force * sin(motor_tilt_pos_), 0, 0), math::Vector3(relPose.pos.x, relPose.pos.y, relPose.pos.z) );
  
   //std::cout << joint_->GetName() << " Force : " << math::Vector3(force * sin(motor_tilt_pos_), 0, force * cos(motor_tilt_pos_)) << std::endl;
  // Forces from Philppe Martin's and Erwan Salaün's
  // 2010 IEEE Conference on Robotics and Automation paper
  // The True Role of Accelerometer Feedback in Quadrotor Control
  // - \omega * \lambda_1 * V_A^{\perp}
  math::Vector3 joint_axis = joint_->GetGlobalAxis(0);
  //math::Vector3 body_velocity = link_->GetWorldLinearVel();
  math::Vector3 body_velocity_perpendicular = body_velocity - (body_velocity * joint_axis) * joint_axis;
  math::Vector3 air_drag = -std::abs(real_motor_velocity) * rotor_drag_coefficient_ * body_velocity_perpendicular;
  // Apply air_drag to link.
  link_->AddForce(air_drag);
  // Moments
  // Getting the parent link, such that the resulting torques can be applied to it.
  
  // The tansformation from the parent_link to the link_.
  math::Pose pose_difference = link_->GetWorldCoGPose() - parent_links.at(0)->GetWorldCoGPose();
  math::Vector3 drag_torque( 0, 0, -turning_direction_ * force * moment_constant_);
  // Transforming the drag torque into the parent frame to handle arbitrary rotor orientations.
  math::Vector3 drag_torque_parent_frame = pose_difference.rot.RotateVector(drag_torque);
  parent_links.at(0)->AddRelativeTorque(drag_torque_parent_frame);

  math::Vector3 rolling_moment;
  // - \omega * \mu_1 * V_A^{\perp}
  rolling_moment = -std::abs(real_motor_velocity) * rolling_moment_coefficient_ * body_velocity_perpendicular;
  parent_links.at(0)->AddTorque(rolling_moment);
  // Apply the filter on the motor's velocity.
  double ref_motor_rot_vel;
  ref_motor_rot_vel = rotor_velocity_filter_->updateFilter(ref_motor_rot_vel_, sampling_time_);

#if 0 //FIXME: disable PID for now, it does not play nice with the PX4 CI system.
  if (use_pid_)
  {
    double err = joint_->GetVelocity(0) - turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_;
    double rotorForce = pid_.Update(err, sampling_time_);
    joint_->SetForce(0, rotorForce);
    // gzerr << "rotor " << joint_->GetName() << " : " << rotorForce << "\n";
  }
  else
  {
#if GAZEBO_MAJOR_VERSION >= 7
    // Not desirable to use SetVelocity for parts of a moving model
    // impact on rest of the dynamic system is non-physical.
    joint_->SetVelocity(0, turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_);
#elif GAZEBO_MAJOR_VERSION >= 6
    // Not ideal as the approach could result in unrealistic impulses, and
    // is only available in ODE
    joint_->SetParam("fmax", 0, 2.0);
    joint_->SetParam("vel", 0, turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_);
#endif
  }
#else
  joint_->SetVelocity(0, turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_);
#endif /* if 0 */
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMotorModel);
}
