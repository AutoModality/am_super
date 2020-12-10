/**
 * ROS Test node to ensure a node can successfully communicate with am_super
 * and retrieve it's status. Specifically, this test checks if am_super is in
 * the ARMED state 3 times.
 * 
 * The test node inherits from LifeCycle and must properly go through the lifecycle
 * states which are necessary to be "ARMED".  Assertions will be made to ensure this node
 * received a command to visit each state. 
 *
 * Times out after rostest configured in launch file.
 *
 * Test fails if number of times we receive 'ARMED' from super is less than 3.
 */
#include "ros/ros.h"                 // ros header file
#include <gtest/gtest.h>             // googletest header file
#include <brain_box_msgs/VxState.h>  // msg for status
#include <brain_box_msgs/OperatorCommand.h>  // to be armed, launch for state transitions
#include <brain_box_msgs/ControllerState.h>
#include <super_lib/am_life_cycle.h>
#include <super_lib/am_life_cycle_mediator.h>
#include <super_lib/am_super_topics.h>
#include <am_super/super_node_mediator.h>

using namespace std;
using namespace am;

/* SuperState - indicates if we received the command from super yet*/
bool booting = false;
bool ready = false; 
bool arming = false; 
bool armed = false;
bool in_auto = false;
bool disarming = false;
bool ready_after_disarming = false;

/* LifeCycle - indicates if we received the command yet for a nodde*/
bool super_unconfigured = false;
bool super_configuring = false;
bool super_inactive = false; 
bool super_active = false;
bool super_activating = false;

bool rostest_unconfigured = false;
bool rostest_configuring = false;
bool rostest_inactive = false;
bool rostest_active = false;
bool rostest_activating = false;
bool rostest_deactivating = false;
bool rostest_inactive_after_disarming = false;

constexpr string_view THIS_NODE_NAME = "/life_cycle_rostest";

constexpr string_view CORRECT = "CORRECT";  // represents the correct result in test
string_view order_status = CORRECT;         // used in test to verify order_status is correct

AMLifeCycleMediator life_cycle_mediator_;

const std::string SUPER_NODE_MEDIATOR = "am_super";
SuperNodeMediator super_node_mediator_(SUPER_NODE_MEDIATOR);

class LifeCycleNodeTest : public ::testing::Test, am::AMLifeCycle
{
  public:
    LifeCycleNodeTest()
    {
      ROS_INFO_STREAM("Constructing Lifecycle Node Test");
      //see launch file for "init_state" = UNCONFIGURED
    }

    void onConfigure()
    {
      AMLifeCycle::onConfigure();
      configured=true;
    }
    void onActivate()
    {
      if(!configured)
      {
        order_status = "ERROR: Attempting to activate before being configured";
      }
      AMLifeCycle::onActivate();
      activated=true;
    }

    void onCleanup()
    {
      AMLifeCycle::onCleanup();
      cleanedUp=true;
    }

    void onDeactivate()
    {
      AMLifeCycle::onDeactivate();
      deactivated=true;
    }

    void onDestroy()
    {
      AMLifeCycle::onDestroy();
      destroyed=true;
    }

    void onError()
    {
      AMLifeCycle::onError();
      errored=true;
    }

    void onShutdown()
    {
      AMLifeCycle::onShutdown();
      shutdown=true;
    }
    

  protected:
    bool configured = false;
    bool activated = false;
    bool cleanedUp = false;
    bool deactivated = false;
    bool destroyed = false;
    bool errored = false;
    bool shutdown = false;
};


/**
 * callback function for ROS test node whenever data is published
 *
 * @param msg custom message containing state information about am_super
 */
void missionStateCallback(const brain_box_msgs::VxState& msg)
{ 
  switch(msg.state)
  {
    case brain_box_msgs::VxState::BOOTING:
      ROS_INFO_STREAM("BOOTING received");
      booting = true;
      break;
    case brain_box_msgs::VxState::READY:
      ROS_INFO_STREAM("READY received");
      if(disarming)
        ready_after_disarming = true;
      else
        ready = true;
      break;
    case brain_box_msgs::VxState::ARMING:
      ROS_INFO_STREAM("ARMING received");
      arming = true;
      break;
    case brain_box_msgs::VxState::ARMED:
      ROS_INFO_STREAM("ARMED received");
      armed = true;
      break;
    case brain_box_msgs::VxState::AUTO:
      ROS_INFO_STREAM("AUTO received");
      in_auto = true;
      break;
    case brain_box_msgs::VxState::DISARMING:
      ROS_INFO_STREAM("DISARMING received");
      disarming = true;
  }
}

void nodeLifeCycleStateCallback(const brain_box_msgs::LifeCycleState& msg)
{ 
  LifeCycleState state = (LifeCycleState)msg.state;
  string_view state_string = life_cycle_mediator_.stateToString(state);
  ROS_INFO_STREAM("Node lifecycle state " << state_string << " received from " << msg.node_name);
  //FIXME: super's node name should come from a constant since it is also used in super
  if(super_node_mediator_.nodeNameIsSuper(msg.node_name))
  {
    switch(state)
    {
      case LifeCycleState::UNCONFIGURED:
        super_unconfigured = true;
        break;
      case LifeCycleState::CONFIGURING:
        super_configuring = true;
        break;
      case LifeCycleState::INACTIVE:
        super_inactive = true;
        break;
      case LifeCycleState::ACTIVE:
        super_active = true;
        break;
      case LifeCycleState::ACTIVATING:
        super_activating = true;
        break;
      default:
        ROS_WARN_STREAM(state_string << " unhandled for " << msg.node_name);
    }
  }
  else if(msg.node_name.find(THIS_NODE_NAME) != std::string::npos)
  {
     switch(state)
    {
      case LifeCycleState::UNCONFIGURED:
        rostest_unconfigured = true;
        break;
      case LifeCycleState::CONFIGURING:
        rostest_configuring = true;
        break;
      case LifeCycleState::INACTIVE:
        //going inactive after being active indicates disarming, but can't use disarming because of race condition
        if (rostest_active)
          rostest_inactive_after_disarming = true;
        else
          rostest_inactive = true;
        break;
      case LifeCycleState::ACTIVE:
        rostest_active = true;
        break;
      case LifeCycleState::ACTIVATING:
        rostest_activating = true;
        break;
      case LifeCycleState::DEACTIVATING:
        rostest_deactivating = true;
        break;
      default:
        ROS_WARN_STREAM(state_string << " unhandled for " << msg.node_name);
    }
  }
  else
  {
    ROS_ERROR_STREAM(msg.node_name << " not recognized.");
  }
  
}

TEST_F(LifeCycleNodeTest, testState_SuccessfulFlight)
{
  ros::NodeHandle n;

  //FIXME: All these topic names should be constants somewhere to be referenced
  ros::Subscriber missionStateSubscription = n.subscribe(am_super_topics::SUPER_STATE, 1000, missionStateCallback);
  ros::Subscriber nodeLifeCycleStateSubscription = n.subscribe(am_super_topics::LIFECYCLE_STATE, 1000, nodeLifeCycleStateCallback);
  ros::Publisher operatorCommandPublisher = n.advertise<brain_box_msgs::OperatorCommand>(am_super_topics::OPERATOR_COMMAND,100);
  ros::Publisher controllerStatePublisher = n.advertise<brain_box_msgs::ControllerState>(am_super_topics::CONTROLLER_STATE, 100);
  ros::Rate loop_rate(1);  // 1 Hz

  // check that we are booting
  {
    ROS_INFO_STREAM("Waiting to receive BOOTING from AMSuper (Ctrl-C to cancel)..\n");
    while (!booting && ros::ok() )
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
    ASSERT_TRUE(booting);
  }

  ROS_INFO_STREAM("BOOTING received ");

  //super must become active first
  {
    //super inactive is not guaranteed and we don't have any repeat messaging
    while ((!super_active) && ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
    ASSERT_TRUE(super_active) << "Super should activate itself after configuring.";
    //super_inactive is not a guaranteed message, so we can't assert it arrived
  }

  
  // check that we are ready
  {
    ROS_INFO_STREAM("Waiting to receive READY from AMSuper (Ctrl-C to cancel)..\n");

    ASSERT_FALSE(arming) << "Super should not be arming yet";
    while ((!ready  || !rostest_inactive) && ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }

    ASSERT_TRUE(ready) << "/am_super SuperState did not report a READY state";
    ASSERT_TRUE(rostest_inactive) << "/life_cycle_rostest LifeCycle did not report an INACTIVE state";
    ASSERT_FALSE(rostest_active) << "/life_cycle_rostest should not be active until arming";
  }

  // check that we are not arming yet, until operator command is sent
  {
    ASSERT_FALSE(arming) << "/am_super SuperState must wait for a trigger to transition to ARMING";
    ASSERT_FALSE(armed) << "/am_super SuperState should not be armed yet";
    ASSERT_EQ(order_status, CORRECT) << order_status;
    ASSERT_TRUE(configured) << "This node should be configured";
    ASSERT_FALSE(activated) << "This node should not be activated yet";
    ASSERT_FALSE(cleanedUp) << "This node should not have cleaned up yet";
    ASSERT_FALSE(deactivated) << "This node should not have deactivated yet";
    ASSERT_FALSE(destroyed) << "This node should not be destroyed yet";
    ASSERT_FALSE(errored) << "This node should not have an error right now";
    ASSERT_FALSE(shutdown) << "This node should not have shut down";
  }

  // now, let's arm it with the operator command
  {
    brain_box_msgs::OperatorCommand armCommand;
    armCommand.node_name = THIS_NODE_NAME;
    armCommand.command = brain_box_msgs::OperatorCommand::ARM;
    while(!arming && ros::ok())
    {
      operatorCommandPublisher.publish(armCommand);
      ros::spinOnce();
      loop_rate.sleep();
    }    
    ASSERT_TRUE(arming) << "Super should now be arming";
  }

  //now it must go armed once all the nodes go active
  {
    while(!armed && ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }    
    ASSERT_TRUE(armed) << "/am_super SuperState should now be armed";
    ASSERT_TRUE(super_active) << "/am_super LifeCycle should still be active";
    ASSERT_TRUE(rostest_active) << "/life_cycle_rostest LifeCycle should now be active";
    ASSERT_TRUE(rostest_activating) << " Rostest is active, but never was activating";
    ASSERT_TRUE(activated) << "This node should now be activated";
    ASSERT_FALSE(cleanedUp) << "This node should not have cleaned up yet";
    ASSERT_FALSE(deactivated) << "This node should not have deactivated yet";
    ASSERT_FALSE(destroyed) << "This node should not be destroyed yet";
    ASSERT_FALSE(errored) << "This node should not have an error right now";
    ASSERT_FALSE(shutdown) << "This node should not have shut down";
  }

  // now let's launch it with the operator command
  {
    brain_box_msgs::OperatorCommand launchCommand;
    launchCommand.node_name = THIS_NODE_NAME;
    launchCommand.command = brain_box_msgs::OperatorCommand::LAUNCH;
    
    while(!in_auto && ros::ok())
    {
      operatorCommandPublisher.publish(launchCommand);
      ros::spinOnce();
      loop_rate.sleep();
    }    
    ASSERT_TRUE(in_auto) << "/am_super SuperState should now be in AUTO";
  }

  /* assume flight is done once in AUTO, controller state COMPLETED will
  be published and super transitons AUTO --> DISARMING */
  {
    brain_box_msgs::ControllerState controllerState;
    controllerState.node_name = THIS_NODE_NAME;
    controllerState.state = brain_box_msgs::ControllerState::COMPLETED;

    while(!disarming && ros::ok())
    {
      controllerStatePublisher.publish(controllerState);
      ros::spinOnce();
      loop_rate.sleep();
    }
    ASSERT_TRUE(disarming);

    while(!rostest_deactivating && ros::ok())
    {
      //super should continually notify nodes to disarm
      ros::spinOnce();
      loop_rate.sleep();
    }
    ASSERT_TRUE(rostest_deactivating) << "rostest should have been notified of deactivation";

    while(!ready_after_disarming && ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
    ASSERT_TRUE(ready_after_disarming) << "Super should go into READY after DISARMING";
    ASSERT_TRUE(rostest_inactive_after_disarming) << "rostest must be inactive before system is ready again.";
  }

  /** TODO: instead of AUTO->DISARMING->READY do AUTO->MANUAL->DISARMING->READY and ensure the DEACTIVATE
   *  command is sent after we are in MANUAL*/
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "life_cycle_rostest");

  return RUN_ALL_TESTS();
}