#include <cstdint>

#include <ros/ros.h>

#include <brain_box_msgs/LifeCycleState.h>

namespace am
{

enum class LifeCycleState : std::uint8_t
{
	UNCONFIGURED		= brain_box_msgs::LifeCycleState::UNCONFIGURED,
	INACTIVE			= brain_box_msgs::LifeCycleState::INACTIVE,
	ACTIVE				= brain_box_msgs::LifeCycleState::ACTIVE,
	FINALIZED			= brain_box_msgs::LifeCycleState::FINALIZED,
	CONFIGURING			= brain_box_msgs::LifeCycleState::CONFIGURING,
	CLEANING_UP			= brain_box_msgs::LifeCycleState::CLEANING_UP,
	SHUTTING_DOWN		= brain_box_msgs::LifeCycleState::SHUTTING_DOWN,
	ACTIVATING			= brain_box_msgs::LifeCycleState::ACTIVATING,
	DEACTIVATING		= brain_box_msgs::LifeCycleState::DEACTIVATING,
	ERROR_PROCESSING	= brain_box_msgs::LifeCycleState::ERROR_PROCESSING,
  Count = 10
};

class AMLifeCycle
{
	LifeCycleState state_;

protected:
	ros::NodeHandle nh_;
	ros::Timer state_update_timer_;
	ros::Publisher state_pub_;

	/**
	* @brief Default constructor
	*/
	AMLifeCycle();

	/**
	* @brief Virtual destructor
	*/
	virtual ~AMLifeCycle();

	/**
	* @brief Returns the current state
	* @return State that the node is in
	*/
	LifeCycleState getLifeCycleState();

	virtual void configure();
	virtual void cleanup();
	virtual void activate();
	virtual void deactivate();
	virtual void shutdown();
	virtual void destroy();


	/**
	* @brief Function to be defined by the user.
	*        Called at the end of transition from UNCONFIGURED to INACTIVE.
	*/
	virtual void onConfigure();

	/**
	* @brief Function to be defined by the user.
	*        Called at the end of transition from INACTIVE to UNCONFIGURED.
	*/
	virtual void onCleanUp();

	/**
	* @brief Function to be defined by the user.
	*        Called at the end of transition from INACTIVE to ACTIVE.
	*/
	virtual void onActivate();

	/**
	* @brief Function to be defined by the user.
	*        Called at the end of transition from ACTIVE to INACTIVE.
	*/
	virtual void onDeactivate();

	/**
	* @brief Function to be defined by the user.
	*        Called at the end of transition from INACTIVE to FINALIZED.
	*/
	virtual void onShutdown();

	/**
	* @brief Function to be defined by the user.
	*        Called after an error and may transition to UNCONFIGURED or FINALIZED.
	*/
	virtual void onError();

	virtual void stateUpdateCB(const ros::TimerEvent& event);

}; // class AMLifeCycle

}; // namespace am

