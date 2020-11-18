#include "primary_rostest_base_class.h"

PrimaryRostestBaseClass::PrimaryRostestBaseClass(string target_node_name)
{
  mavrosStatePublisher_ = n.advertise<mavros_msgs::State>(am::am_topics::MAVROS_STATE, 100);
  nodeLifeCycleStateSubscription_ = n.subscribe(am_super_topics::LIFECYCLE_STATE, 1000, &PrimaryRostestBaseClass::nodeLifeCycleStateCallback,this);
  missionStateSubscription_ = n.subscribe(am_super_topics::SUPER_STATE, 1000, &PrimaryRostestBaseClass::missionStateCallback,this);
  operatorCommandPublisher_ = n.advertise<brain_box_msgs::OperatorCommand>(am_super_topics::OPERATOR_COMMAND,100);
  controllerStatePublisher_ = n.advertise<brain_box_msgs::ControllerState>(am_super_topics::CONTROLLER_STATE, 100);

  target_node_name_= target_node_name;
}

void PrimaryRostestBaseClass::publishOperatorCommand(uint8_t command)
{
  brain_box_msgs::OperatorCommand msg;
  msg.node_name = ros::this_node::getName();
  msg.command = command;
  operatorCommandPublisher_.publish(msg);
}

void PrimaryRostestBaseClass::publishControllerState(uint8_t state)
{
  brain_box_msgs::ControllerState msg;
  msg.node_name = ros::this_node::getName();
  msg.state = state;
  controllerStatePublisher_.publish(msg);
}

/**Publish happy state on demand to act as the acs_flight_api */
void PrimaryRostestBaseClass::publishState()
{
  mavros_msgs::State state;
  state.connected = true;
  mavrosStatePublisher_.publish(state);
}

/** when configured and Inactive, arm is sent to prepare all nodes for takeoff*/
void PrimaryRostestBaseClass::arm()
{
  ROS_INFO("operator sending arm command");
  publishOperatorCommand(brain_box_msgs::OperatorCommand::ARM);
}

/** when armed, signals for the props to spin and takeoff */
void PrimaryRostestBaseClass::launch()
{
  ROS_INFO("operator sending launch command");
  publishOperatorCommand(brain_box_msgs::OperatorCommand::LAUNCH);
}

void PrimaryRostestBaseClass::landed()
{
  ROS_INFO("controller sending landed state");
  publishControllerState(brain_box_msgs::ControllerState::COMPLETED);
}

bool PrimaryRostestBaseClass::nodeStateReceived(string node_name,LifeCycleState state)
{
    if(node_states_.count(node_name)){
      int key = 2;     
      auto lower_it = node_states_.lower_bound(node_name); 
      auto upper_it = node_states_.upper_bound(node_name); 
        
      while (lower_it != upper_it)     
      { 
          if (lower_it -> first == node_name) {
            brain_box_msgs::LifeCycleState state_msg = lower_it -> second;
            if((LifeCycleState)state_msg.state == state){
              return true;
            }
          }
          lower_it++; 
      }     
      return false; 
    }
}

bool PrimaryRostestBaseClass::missionStateReceived(uint8_t mission_state)
{
  ROS_INFO("Mission Station: Received: %i  Total: %i",mission_state, (int) mission_states_.size());
  return mission_states_.back() == mission_state;
}

void PrimaryRostestBaseClass::nodeLifeCycleStateCallback(const brain_box_msgs::LifeCycleState& msg)
{ 
  node_states_.emplace(msg.node_name,msg);
}

void PrimaryRostestBaseClass::missionStateCallback(const brain_box_msgs::VxState& msg)
{
  mission_states_.insert(mission_states_.end(),msg.state);
}

void PrimaryRostestBaseClass::waitUntilMissionState(const uint8_t mssion_state, float sleep)
{
  ros::Rate loop_rate(sleep);
  while (!missionStateReceived(mssion_state) && ros::ok() )
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void PrimaryRostestBaseClass::waitUntil(const LifeCycleState state, float sleep){
  ros::Rate loop_rate(sleep);
  while (!nodeStateReceived(target_node_name_,state) && ros::ok() )
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}