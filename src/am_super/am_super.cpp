#include <functional>
#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>

#include <am_super/baby_sitter.h>
#include <am_super/super_state.h>

#include <brain_box_msgs/BlinkMCommand.h>
#include <brain_box_msgs/LifeCycleState.h>
#include <brain_box_msgs/LogControl.h>
#include <brain_box_msgs/StampedAltimeter.h>
#include <brain_box_msgs/Super2Status.h>
#include <brain_box_msgs/VxState.h>

#include <super_lib/am_life_cycle_types.h>
#include <super_lib/am_life_cycle.h>

#include <vb_util_lib/bag_logger.h>
#include <vb_util_lib/topics.h>
#include <vb_util_lib/trace.h>
#include <vb_util_lib/vb_main.h>


#if CUDA_FLAG
#include <cuda/cuda_utility_class.h>
#endif

#define NODE_NAME "am_super"

using namespace std;

namespace am
{

/**
 * node info struct
 */
struct SuperNodeInfo
{
    std::string name;           // node name in ROS
    int pid;                    // process id of node
    float cpu_usage;            // amount of cpu node is consuming
    float gpu_usage;            // amount of gpu node is consuming
    float mem_usage;            // amount of memory node is consuming
    LifeCycleState state;       // node lifecycle state
    LifeCycleStatus status;     // node lifecycle status
    bool manifested;            // nodes was in manfiest
    bool online;                // node is online
    ros::Time last_contact;     // last time a message was received from the node
};

/**
 * flight control state
 */
enum SuperFltCtrlState
{
    INIT,
    AUTO,
    HOLD
};

/**
 *  AM supervisor class. aggregates system state and system health and manages node lifecycle.
 *
 *  uses BabySitter instances to generate state and health for nodes that don't publish brain_box_msgs::LifeCycleState
 */
class AMSuper
{
private:
    static constexpr std::string_view STATE_BOOTING_STRING = "BOOTING";
    static constexpr std::string_view STATE_READY_STRING = "READY";
    static constexpr std::string_view STATE_ARMED_STRING = "ARMED";
    static constexpr std::string_view STATE_AUTO_FEATURE_STRING = "AUTO_FEATURE";
    static constexpr std::string_view STATE_AUTO_NO_FEATURE_STRING = "AUTO_NO_FEATURE";
    static constexpr std::string_view STATE_HOLD_STRING = "HOLD";
    static constexpr std::string_view STATE_UNKNOWN_STRING = "UNKNOWN";
    static constexpr std::string_view STATE_ERROR_STRING = "ERROR";

    /**
     * heartbeat log output period
     */
    const int LOG_THROTTLE_S = 10;

    ros::NodeHandle nh_;
    ros::Timer heartbeat_timer_;
    ros::Publisher super_status_pub_;
    ros::Publisher vstate_summary_pub_;
    ros::Publisher led_pub_;
    ros::Subscriber node_state_sub_;
    ros::Subscriber node_status_sub_;

    /**
     * system state
     */
    SuperState system_state_ = SuperState::BOOTING;

    /*
     * flight controller state
     */
    SuperFltCtrlState flt_ctrl_state_;

    /**
     * manifest node (generated from manifest param)
     */
    std::vector<string> manifest_;

    /**
     * map of all nodes in the system
     */
    map<string, SuperNodeInfo> nodes_;

    /**
     * number of nodes online
     */
    int num_nodes_online_;

    /**
     * number of manifested nodes online
     */
    int num_manifest_nodes_online_;

    /**
     * amount of time in seconds without hearing from a node that will cause it to go offline
     */
    double node_timeout_s_;

    /**
     * baglogger level
     */
    const int SU_LOG_LEVEL = 1;

    /**
     * ground station testing flag
     *
     * if true causes the system state to cycle thru all values
     */
    bool gcs_test_mode_;


    //
    // babysitters
    //
    const std::string NODE_BS_ALTIMETER = "can_node"; // TODO: replace with system global const
    typedef brain_box_msgs::StampedAltimeter altimeter_bs_msg_type ;
    am::BabySitter<altimeter_bs_msg_type> *altimeter_bs_;
    const std::string ALTIMETER_BS_TOPIC = "/sensor/distance/agl_lw"; // TODO: replace with system global const
    const int ALTIMETER_HZ = 20;

    const std::string NODE_BS_DJI = "dji_sdk"; // TODO: replace with system global const
    typedef sensor_msgs::Joy dji_bs_msg_type ;
    am::BabySitter<dji_bs_msg_type> *dji_bs_;
    const std::string DJI_BS_TOPIC = "/dji_sdk/rc"; // TODO: replace with system global const
    const int DJI_HZ = 50;

#if CUDA_FLAG
	std::shared_ptr<am::CudaUtility> gpu_info_;
#endif

public:
    AMSuper() :
            nh_("~")
    {
        ROS_INFO_STREAM(NODE_FUNC);

        ros::param::param<double>("~node_timeout_s", node_timeout_s_, 2.0);
        ROS_INFO_STREAM(NODE_FUNC << "node_timeout_s = " << node_timeout_s_);

        ros::param::param<bool>("~test_mode", gcs_test_mode_, false);
        ROS_INFO_STREAM(NODE_FUNC << "test_mode = " << gcs_test_mode_);

        /*
         * create initial node list from manifest and create babysitters as needed
         */
        num_nodes_online_ = 0;
        num_manifest_nodes_online_ = 0;
        // strip spaces from manifest param
        string tmp_manifest;
        ros::param::param<string>("~manifest", tmp_manifest, "");
        boost::erase_all(tmp_manifest, " ");
        // if a manifest has been specified
        if (tmp_manifest.size())
        {
            // split it based upon commas
            boost::split(manifest_, tmp_manifest, boost::is_any_of(","));
            ROS_INFO_STREAM(NODE_FUNC << "configuring nodes from manifest:");
            for (string &name : manifest_)
            {
                // create a new node in the list for each name in manifest
                SuperNodeInfo nr;
                nr.name = name;
                nr.pid = -1;
                nr.online = false;
                nr.last_contact = ros::Time(0);
                nr.manifested = true;
                nr.state = LifeCycleState::UNCONFIGURED;
                nr.status = LifeCycleStatus::OK;
                nodes_.insert(pair<string, SuperNodeInfo>(name, nr));
                ROS_INFO_STREAM(NODE_FUNC << "  " << name);

                // create babysitters based on hard coded node names
                if (!name.compare(NODE_BS_ALTIMETER))
                {
                    int altimeter_warn_ms, altimeter_error_ms;
                    calcBSTiming(ALTIMETER_HZ, altimeter_warn_ms,
                            altimeter_error_ms);
                    altimeter_bs_ = new am::BabySitter<
                            altimeter_bs_msg_type>(nh_,
                            BagLogger::instance(), name,
                            ALTIMETER_BS_TOPIC, altimeter_warn_ms,
                            altimeter_error_ms);
                }
                else if (!name.compare(NODE_BS_DJI))
                {
                    int dji_warn_ms, dji_error_ms;
                    calcBSTiming(DJI_HZ, dji_warn_ms, dji_error_ms);
                    dji_bs_ = new am::BabySitter<dji_bs_msg_type>(nh_,
                            BagLogger::instance(), name, DJI_BS_TOPIC,
                            dji_warn_ms, dji_error_ms);
                }
            }
        }
        reportSystemState();

#if CUDA_FLAG
        ROS_INFO("##########GPU Monitoring is ON##########");
        gpu_info_ = std::make_shared<am::CudaUtility>(nh_);
#endif

        super_status_pub_ = nh_.advertise<brain_box_msgs::Super2Status>(
                "/super/status", 1000);
        vstate_summary_pub_ = nh_.advertise<brain_box_msgs::VxState>(
                "/vstate/summary", 1000);
        led_pub_ = nh_.advertise<brain_box_msgs::BlinkMCommand>(
                am::am_topics::LED_BLINK, 1000);

        system_state_ = SuperState::BOOTING;
        flt_ctrl_state_ = SuperFltCtrlState::INIT;

        BagLogger::instance()->startLogging("SU", SU_LOG_LEVEL);

        // subs should always come at the end
        node_state_sub_ = nh_.subscribe("/node_state", 100, &AMSuper::nodeStateCB,
                this);
        node_status_sub_ = nh_.subscribe("/process/status", 100,
                &AMSuper::statusCB, this);

        heartbeat_timer_ = nh_.createTimer(ros::Duration(1.0),
                &AMSuper::heartbeatCB, this);
    }

    ~AMSuper()
    {
        // TODO: need a clean way of shutting down. i don't think ctrl-c calls this and
        // certainly power off doesn't. the best design is to have a power-off sensor
        // that sends a signal of some sort (i.e. publishes a message).
        BagLogger::instance()->stopLogging();
    }

    static const std::string_view &stateToString(SuperState state)
    {
        switch (state)
        {
        case SuperState::BOOTING:
            return STATE_BOOTING_STRING;
            break;
        case SuperState::READY:
            return STATE_READY_STRING;
            break;
        case SuperState::ARMED:
            return STATE_ARMED_STRING;
            break;
        case SuperState::AUTO_FEATURE:
            return STATE_AUTO_FEATURE_STRING;
            break;
        case SuperState::AUTO_NO_FEATURE:
            return STATE_AUTO_NO_FEATURE_STRING;
            break;
        case SuperState::HOLD:
            return STATE_HOLD_STRING;
            break;
        case SuperState::UNKNOWN:
            return STATE_UNKNOWN_STRING;
            break;
        case SuperState::ERROR:
        default:
            return STATE_ERROR_STRING;
            break;
        }
    }

private:
   /**
     * process LifeCycleState messages from nodes
     *
     * LifeCycle messages are sent once a second by the LifeCycle heartbeat, but may
     * come more frequently if a node chooses.
     */
    void nodeStateCB(
            const ros::MessageEvent<brain_box_msgs::LifeCycleState const> &event)
    {
        if (gcs_test_mode_)
        {
            return;
        }

        const brain_box_msgs::LifeCycleState::ConstPtr &rmsg =
                event.getMessage();

        /*
         * process the message if the node state is ACTIVE and status is OK. this allows
         * the normal timeout mechanism to transition the whole system to ERROR if the node
         * isn't reporting ACTIVE/OK.
         *
         * TODO: this should probably be more reactive. i.e. if a node sends a non-ACTIVE state
         * or a non-OK status, we should probably react to it immediately rather than wait
         * for the timeout.
         */
        if (rmsg->state != brain_box_msgs::LifeCycleState::STATE_ACTIVE &&
                rmsg->status != brain_box_msgs::LifeCycleState::STATUS_ERROR)
        {
            processState(rmsg->node_name, (LifeCycleState) (rmsg->state),
                    (LifeCycleStatus) (rmsg->status), rmsg->subsystem, rmsg->value, rmsg->process_id,
                    event.getReceiptTime());
        }

        // TODO: topic name should come from vb_util_lib::topics.h
        LOG_MSG("/node_state", rmsg, SU_LOG_LEVEL);
    }

    /**
     * process legacy messages from nodes
     */
    void statusCB(
            const ros::MessageEvent<brain_box_msgs::NodeStatus const> &event)
    {
        if (gcs_test_mode_)
        {
            return;
        }

        const brain_box_msgs::NodeStatus::ConstPtr &rmsg = event.getMessage();

        /*
         * legacy messages don't carry any state or status info so just process as ACTIVE/OK
         */
        processState(rmsg->node_name, LifeCycleState::ACTIVE, LifeCycleStatus::OK, rmsg->status,
                rmsg->value, rmsg->process_id, event.getReceiptTime());

        //TODO: topic name should come from vb_util_lib::topics.
        LOG_MSG("/process/status", rmsg, SU_LOG_LEVEL);
    }

    /**
     * process state
     * @param node_name_in
     * @param state
     * @param status
     * @param subsystem legacy mechanism for flight controller to report it's state
     * @param value legacy mechanism for flight controller to report it's state
     * @param pid
     * @param last_contact
     */
    void processState(const std::string &node_name_in,
            const am::LifeCycleState state, const am::LifeCycleStatus status,
            const std::string &subsystem, const std::string &value, const int pid,
            const ros::Time &last_contact)
    {

        // strip leading '/' from the node name if needed
        string node_name;
        if (node_name_in.at(0) == '/')
        {
            node_name = node_name_in.substr(1);
        }
        else
        {
            node_name = node_name_in;
        }

        // search for the node in the list
        bool nodes_changed = false;
        map<string, SuperNodeInfo>::iterator it;
        it = nodes_.find(node_name);
        if (it != nodes_.end())
        {
            // if we get here, the node is already in our list
            SuperNodeInfo &nr = it->second;
            if(nr.state != state)
            {
                ROS_INFO_STREAM(NODE_FUNC << node_name << " changed state to = " <<
                        AMLifeCycle::stateToString(state));
                nr.state = state;
                nodes_changed = true;
            }
            if(nr.status != status)
            {
                ROS_INFO_STREAM(NODE_FUNC << node_name << " changed status to = " <<
                        AMLifeCycle::statusToString(status));
                nr.state = state;
                nodes_changed = true;
            }
            if(nr.pid != pid)
            {
                ROS_WARN_STREAM(NODE_FUNC << node_name << " changed process id to = " << pid);
                nr.pid = pid;
                nodes_changed = true;
           }
            if (!nr.online)
            {
                ROS_INFO_STREAM(NODE_FUNC << "manifested node " << node_name << " came online");
                nr.online = true;
                num_nodes_online_++;
                if (nr.manifested)
                {
                    num_manifest_nodes_online_++;
                }
                nodes_changed = true;
            }
            nr.last_contact = last_contact;
        }
        else
        {
            // if we get here, we've never heard from this node before
            ROS_WARN_STREAM(NODE_FUNC << "unknown node " << node_name << " came online. state: " <<
                    AMLifeCycle::stateToString(state) << ", status: " << AMLifeCycle::statusToString(status));
            SuperNodeInfo nr;
            nr.name = node_name;
            nr.pid = pid;
            nr.online = true;
            nr.last_contact = last_contact;
            nr.manifested = false;
            nr.state = state;
            nr.status = status;
            nodes_.insert(pair<string, SuperNodeInfo>(node_name, nr));
            num_nodes_online_++;
            nodes_changed = true;
        }

        if(nodes_changed)
        {
            reportSystemState();
            checkForSystemStateTransition();
        }

        // cache flight controller state and check for state transition
        if (!node_name.compare("flight_controller")
                && !subsystem.compare("FLIGHT_CONTROL"))
        {
            bool flt_ctrl_state_changed = false;
            if (!value.compare("AUTO") && flt_ctrl_state_ != SuperFltCtrlState::AUTO)
            {
                flt_ctrl_state_ = SuperFltCtrlState::AUTO;
                flt_ctrl_state_changed = true;
            }
            else if (!value.compare("HOLD") && flt_ctrl_state_ != SuperFltCtrlState::HOLD)
            {
                flt_ctrl_state_ = SuperFltCtrlState::HOLD;
                flt_ctrl_state_changed = true;
            }
            if(flt_ctrl_state_changed)
            {
                ROS_INFO_STREAM_THROTTLE(1.0,
                        NODE_FUNC << "flight status: " << value);
                checkForSystemStateTransition();
            }
        }

    }

    /**
     * called once per second.
     *
     * times out nodes that haven't been heard from recently. reports on status to bag and trace logs.
     */
    void heartbeatCB(const ros::TimerEvent &event)
    {
#if CUDA_FLAG
        gpu_info_->display();
#endif
        if (system_state_ == SuperState::UNKNOWN)
        {
            setSystemState(SuperState::BOOTING);
        }
        brain_box_msgs::VxState state_msg;
        state_msg.state = (uint8_t)system_state_;
        vstate_summary_pub_.publish(state_msg);

        if (gcs_test_mode_)
        {
            // cycle thru states one per heartbeat
            system_state_ = static_cast<SuperState>(((uint8_t)system_state_ + 1) %
                    (uint8_t)SuperState::LAST_STATE);
            reportSystemState();
        }
        else
        {
            // cycle thru all the nodes in the list to look for a timeout
            ros::Time now = ros::Time().now();
            map<string, SuperNodeInfo>::iterator it;
            for (it = nodes_.begin(); it != nodes_.end(); it++)
            {
                SuperNodeInfo &nr = (*it).second;
                if (nr.online)
                {
                    ros::Duration time_since_contact = now
                            - nr.last_contact;
                    ros::Duration timeout_dur(node_timeout_s_);
                    if (time_since_contact > timeout_dur)
                    {
                        nr.online = false;
                        num_nodes_online_--;
                        if (nr.manifested)
                        {
                            num_manifest_nodes_online_--;
                        }
                        ROS_ERROR_STREAM(
                                NODE_FUNC << "node timed out:" << nr.name);
                        reportSystemState();
                    }
                }
            }
        }

        // check for state transition due to timeouts or anything else that changed since last heartbeat
        checkForSystemStateTransition();

        // publish and bag log super status message
        brain_box_msgs::Super2Status status_msg;
        status_msg.man = manifest_.size();
        status_msg.man_run = num_manifest_nodes_online_;
        status_msg.run = num_nodes_online_;
        map<string, SuperNodeInfo>::iterator it;
        for (it = nodes_.begin(); it != nodes_.end(); it++)
        {
            SuperNodeInfo &nr = (*it).second;
            status_msg.nodes.push_back(nr.name);
        }
        LOG_MSG("/status/super", status_msg, 1);
        if (super_status_pub_.getNumSubscribers() > 0)
        {
            super_status_pub_.publish(status_msg);
        }

        // report current status to trace log
        std::stringstream ss;
        genSystemState(ss);

        if (manifest_.size() != num_manifest_nodes_online_)
        {
            // if all manifested nodes aren't running, report as error
            ROS_ERROR_STREAM(NODE_FUNC << ss.str());
        }
        else
        {
            // if all manifested nodes are running, report as info
            ROS_INFO_STREAM_THROTTLE(LOG_THROTTLE_S, NODE_FUNC << ss.str());
        }

        // report nodes that aren't running to trace log as error
        for (it = nodes_.begin(); it != nodes_.end(); it++)
        {
            SuperNodeInfo &nr = (*it).second;
            if (!nr.online)
            {
                ROS_ERROR_STREAM(
                        NODE_FUNC << "node not running:" << nr.name);
            }
        }
    }

    /**
     * update stream with system state and status
     */
    void genSystemState(std::stringstream &ss)
    {
        ss << "state: " << stateToString(system_state_) << ", manifest: "
                << manifest_.size() << ", manifest online:" << num_manifest_nodes_online_
                << ", total online:" << num_nodes_online_;
    }

    /**
     * report system state and status to trace log
     */
    void reportSystemState()
    {
        std::stringstream ss;
        genSystemState(ss);
        ROS_INFO_STREAM(NODE_FUNC << ss.str());
    }

    /**
     * check for state transition based upon current state and values of member fields
     */
    void checkForSystemStateTransition()
    {
        switch (system_state_)
        {
        case SuperState::BOOTING:
            if (num_manifest_nodes_online_ == manifest_.size())
            {
                setSystemState(SuperState::READY);
            }
            break;
        case SuperState::READY:
            if (num_manifest_nodes_online_ < manifest_.size())
            {
                setSystemState(SuperState::ERROR);
            }
            else if (flt_ctrl_state_ == SuperFltCtrlState::AUTO)
            {
                setSystemState(SuperState::AUTO_FEATURE);
            }
            else if (flt_ctrl_state_ == SuperFltCtrlState::HOLD)
            {
                setSystemState(SuperState::HOLD);
            }
            break;
        case SuperState::AUTO_FEATURE:
            if (num_manifest_nodes_online_ < manifest_.size())
            {
                setSystemState(SuperState::ERROR);
            }
            else if (flt_ctrl_state_ == SuperFltCtrlState::HOLD)
            {
                setSystemState(SuperState::HOLD);
            }
            break;
        case SuperState::HOLD:
            if (num_manifest_nodes_online_ < manifest_.size())
            {
                setSystemState(SuperState::ERROR);
            }
            else if (flt_ctrl_state_ == SuperFltCtrlState::AUTO)
            {
                setSystemState(SuperState::AUTO_FEATURE);
            }
            break;
        case SuperState::ERROR:
            // once we are in error, we can't go anywhere
            break;
        }
    }

    /**
     * set the system state
     *
     * perform any actions associated with each state transition.
     *
     * publish the current led colors and system state
     */
    void setSystemState(SuperState state)
    {
        ROS_INFO_STREAM(
                NODE_FUNC << "request change system state from: " << stateToString(system_state_) << " to: " << stateToString(state));
        switch (system_state_)
        {
        case SuperState::BOOTING:
            system_state_ = state;
            break;
        case SuperState::READY:
            system_state_ = state;
            break;
        case SuperState::AUTO_FEATURE:
            system_state_ = state;
            break;
        case SuperState::HOLD:
            system_state_ = state;
            break;
        case SuperState::UNKNOWN:
            system_state_ = state;
            break;
        case SuperState::ERROR:
            system_state_ = state;
            break;
        }

        reportSystemState();

        sendLEDMessage();

        brain_box_msgs::VxState state_msg;
        state_msg.state = (uint8_t)system_state_;
        vstate_summary_pub_.publish(state_msg);
    }

    /**
     * send led color message based on raw values
     */
    void sendLEDMessage(int r, int g, int b, float period = 0.0)
    {
        brain_box_msgs::BlinkMCommand led_msg;
        led_msg.rgb.red = r;
        led_msg.rgb.green = g;
        led_msg.rgb.blue = b;
        led_msg.blink_rate = period;
        led_pub_.publish(led_msg);
    }

    /**
     * send led color message based on system state
     */
    void sendLEDMessage()
    {
        int r = 0, b = 0, g = 0;
        float rate = 0.0;

        switch (system_state_)
        {
        case SuperState::BOOTING:
            r = 0;
            g = 0;
            b = 255;
            rate = 2.0;
            break;
        case SuperState::READY:
            r = 0;
            g = 0;
            b = 255;
            rate = 0.0;
            break;
        case SuperState::AUTO_FEATURE:
            r = 0;
            g = 255;
            b = 0;
            rate = 0.0;
            break;
        case SuperState::HOLD:
            r = 255;
            g = 0;
            b = 0;
            rate = 2.0;
            break;
        case SuperState::ERROR:
            r = 255;
            g = 0;
            b = 0;
            rate = 0.0;
            break;
        }

        sendLEDMessage(r, g, b, rate);
    }

    /**
     * calculate babysitter timing params
     * @param hz expected frequency in hz
     * @param warn_ms calculated warning period in milliseconds
     * @param error_ms calculated error period in milliseconds
     */
    void calcBSTiming(int hz, int &warn_ms, int &error_ms)
    {
        warn_ms = (int) (1000.0 / hz * 2.0 + 0.5);
        error_ms = (int) (1000.0 / hz * 3.0 + 0.5);
    }

};

};

#ifdef TESTING
#else
int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);

    am::AMSuper node;

    ROS_INFO_STREAM(NODE_NAME << ": running...");

    ros::spin();
}
#endif
