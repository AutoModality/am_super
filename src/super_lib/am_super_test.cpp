#include <am_utils/am_ros2_utility.h>
#include <super_lib/am_super_test.h>

#include <sstream>

std::shared_ptr<rclcpp::Node> am::Node::node;

AMSuperTest::AMSuperTest(string target_node_name) : nh_(std::make_shared<rclcpp::Node>(target_node_name))
{
  createPubsSubs();

  target_node_name_= target_node_name;
  if(target_node_name_[0] != '/')
  {
    target_node_name_= '/' + target_node_name_;
  }
  TEST_LOG("Target node name:" << target_node_name_,"AXSO");
}

AMSuperTest::AMSuperTest() : nh_(std::make_shared<rclcpp::Node>("am_super_test"))
{
  createPubsSubs();

  am::getParam<std::string>("~target_node_name", target_node_name_, nh_->get_name());
  if(target_node_name_[0] != '/')
  {
    target_node_name_= '/' + target_node_name_;
  }
    
  TEST_LOG("Target node name:" << target_node_name_, "ANNQ");
}

void AMSuperTest::createPubsSubs()
{
  nodeLifeCycleStateSubscription_ = nh_->create_subscription<brain_box_msgs::msg::LifeCycleState>
    (am_super_topics::LIFECYCLE_STATE, 1000, 
    std::bind(&AMSuperTest::nodeLifeCycleStateCallback, this, std::placeholders::_1));
  missionStateSubscription_ = nh_->create_subscription<brain_box_msgs::msg::VxState>
    (am_super_topics::SUPER_STATE, 1000, 
    std::bind(&AMSuperTest::missionStateCallback, this, std::placeholders::_1));
  operatorCommandPublisher_ = nh_->create_publisher<brain_box_msgs::msg::OperatorCommand>
    (am_super_topics::OPERATOR_COMMAND,100);
  controllerStatePublisher_ = nh_->create_publisher<brain_box_msgs::msg::ControllerState>
    (am_super_topics::CONTROLLER_STATE, 100);
}

void AMSuperTest::publishOperatorCommand(uint8_t command)
{
  brain_box_msgs::msg::OperatorCommand msg;
  msg.node_name = nh_->get_name();
  msg.command = command;
  operatorCommandPublisher_->publish(msg);
}

void AMSuperTest::publishControllerState(uint8_t state)
{
  brain_box_msgs::msg::ControllerState msg;
  msg.node_name = nh_->get_name();
  msg.state = state;
  controllerStatePublisher_->publish(msg);
}

/** when configured and Inactive, arm is sent to prepare all nodes for takeoff*/
void AMSuperTest::arm()
{
  TEST_LOG("Operator sending ARM command","NQNA");
  publishOperatorCommand(brain_box_msgs::msg::OperatorCommand::ARM);
}

/** when armed, signals for the props to spin and takeoff */
void AMSuperTest::launch()
{
  TEST_LOG("Operator sending LAUNCH command","ANQP");
  publishOperatorCommand(brain_box_msgs::msg::OperatorCommand::LAUNCH);
}

void AMSuperTest::landed()
{
  TEST_LOG("Controller sending LANDED state","BSJO");
  publishControllerState(brain_box_msgs::msg::ControllerState::COMPLETED);
}

void AMSuperTest::cancel()
{
  TEST_LOG("Operator sending CANCEL command","LPOQ");
  publishOperatorCommand(brain_box_msgs::msg::OperatorCommand::CANCEL);
}
  
void AMSuperTest::pause()
{
  TEST_LOG("Operator sending PAUSE command","WOEB");
  publishOperatorCommand(brain_box_msgs::msg::OperatorCommand::PAUSE);
}

void AMSuperTest::resume()
{
  TEST_LOG("Operator sending RESUME command","ANQE");
  publishOperatorCommand(brain_box_msgs::msg::OperatorCommand::RESUME);
}

void AMSuperTest::abort()
{
  TEST_LOG("Operator sending ABORT command","EEEN");
  publishOperatorCommand(brain_box_msgs::msg::OperatorCommand::ABORT);
}

void AMSuperTest::manual()
{
  TEST_LOG("Operator sending MANUAL command","NQAY");
  publishOperatorCommand(brain_box_msgs::msg::OperatorCommand::MANUAL);
}

void AMSuperTest::shutdown()
{
  TEST_LOG("Operator sending SHUTDOWN command","VKSP");
  publishOperatorCommand(brain_box_msgs::msg::OperatorCommand::SHUTDOWN);
}

bool AMSuperTest::nodeStateReceived(string node_name,LifeCycleState state)
{
  if(node_states_.count(node_name)){
    int key = 2;     
    auto lower_it = node_states_.lower_bound(node_name); 
    auto upper_it = node_states_.upper_bound(node_name); 
      
    while (lower_it != upper_it)     
    { 
        if (lower_it -> first == node_name) {
          brain_box_msgs::msg::LifeCycleState state_msg = lower_it -> second;
          if((LifeCycleState)state_msg.state == state){
            return true;
          }
        }
        lower_it++; 
    }     
  }
  return false; 
}

bool AMSuperTest::missionStateReceived(uint8_t mission_state)
{
  for (auto state : mission_states_)  
  {
    if(state == mission_state) 
    {
      //Erase-remove idiom for removing all occurences from list
      mission_states_.erase(remove(mission_states_.begin(), mission_states_.end(), state), mission_states_.end());
      return true;
    }
  }
  return false;
}

void AMSuperTest::nodeLifeCycleStateCallback(const brain_box_msgs::msg::LifeCycleState& msg)
{ 
  node_states_.emplace(msg.node_name,msg);
}

void AMSuperTest::missionStateCallback(const brain_box_msgs::msg::VxState& msg)
{
  mission_states_.insert(mission_states_.end(),msg.state);
}

void AMSuperTest::waitUntilMissionState(const uint8_t mission_state, float sleep)
{
  waitUntilMissionState(mission_state,"NAWU",sleep);
}

void AMSuperTest::waitUntilMissionState(const uint8_t mission_state, std::string error_code, float sleep)
{
  rclcpp::Rate loop_rate(sleep);
  while (!missionStateReceived(mission_state) && rclcpp::ok() )
  {
    rclcpp::spin_some(nh_);
    loop_rate.sleep();
    TEST_LOG("[" << error_code << "] waiting to receive mission state: " << mission_state, "08JU");
  }
}

void AMSuperTest::waitUntil(const LifeCycleState state, float sleep){
  waitUntil(state,"XS32",sleep);
}

void AMSuperTest::waitUntil(const LifeCycleState state, const std::string log_code, float sleep){
  rclcpp::Rate loop_rate(sleep);
  while (!nodeStateReceived(nh_->get_name(),state) && rclcpp::ok() )
  {
    rclcpp::spin_some(nh_);
    loop_rate.sleep();
    TEST_LOG("[" << log_code << "]" << " waiting to receive node state: " << (int)state,"NBDC");
  }
}

void AMSuperTest::waitUntilStatus(const LifeCycleStatus& status, float sleep)
{
  waitUntil(status,"XWSQ",sleep);

}
void AMSuperTest::waitUntil(const LifeCycleStatus& status,  const std::string log_code, float sleep)
{
  rclcpp::Rate loop_rate(sleep);
  while (!nodeStatusReceived(nh_->get_name(),status) && rclcpp::ok() )
  {
    rclcpp::spin_some(nh_);
    loop_rate.sleep();
    TEST_LOG("[" << log_code << "]" << " waiting to receive node status: " << (int)status, "YTNJ" );
  }
}

bool AMSuperTest::nodeStatusReceived(string node_name, LifeCycleStatus status)
{
  if(node_states_.count(node_name)){
    int key = 2;     
    auto lower_it = node_states_.lower_bound(node_name); 
    auto upper_it = node_states_.upper_bound(node_name); 
      
    while (lower_it != upper_it)     
    { 
        if (lower_it -> first == node_name) {
          brain_box_msgs::msg::LifeCycleState state_msg = lower_it -> second;
          if((LifeCycleStatus)state_msg.status == status){
            return true;
          }
        }
        lower_it++; 
    }     
  }
  return false; 
}