#include <super_lib/am_life_cycle.h>
#include <am_super/baby_sitter.h>
#include <functional>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>

#include <brain_box_msgs/BlinkMCommand.h>
#include <brain_box_msgs/LifeCycleState.h>
#include <brain_box_msgs/LogControl.h>
#include <brain_box_msgs/StampedAltimeter.h>
#include <brain_box_msgs/Super2Status.h>
#include <brain_box_msgs/VxState.h>

#include <vb_util_lib/bag_logger.h>
#include <vb_util_lib/topics.h>
#include <vb_util_lib/trace.h>
#include <vb_util_lib/vb_main.h>

#define NODE_NAME "Super"

using namespace std;

namespace am
{

struct NodeRec
{
    std::string name;
    int pid;
    float cpu_usage;
    float mem_usage;
    bool running;
    bool manifested;
    ros::Time last_contact;
};

enum FCState
{
    NONE, AUTO, HOLD
};

class Super
{
private:
    const int LOG_PERIOD_S = 10;
    const int ALTIMETER_HZ = 20;
    const int LIDAR_HZ = 20;
    const int RC_HZ = 50;

    ros::NodeHandle nh_;
    int state_ = brain_box_msgs::VxState::BOOTING;
    ros::Timer heartbeat_timer_;
    ros::Publisher super_status_pub_;
    ros::Publisher vstate_summary_pub_;
    ros::Publisher led_pub_;
    ros::Subscriber node_state_sub_;
    ros::Subscriber node_status_sub_;
    ros::Subscriber log_control_sub_;
    ros::Subscriber lidar_sub_;
    BagLogger::BagLoggerLevel level_;
    int status_log_level_;
    map<string, NodeRec> nodes_;
    int run_nodes_;
    int man_run_nodes_;
    std::vector<string> manifest_;
    bool node_manifest_;
    double node_timeout_s_;
    const int LOG_LEVEL = 1;
    bool test_mode_;
    FCState fc_state_;

    am::BabySitter<sensor_msgs::PointCloud2> *lidar_bs_;
    am::BabySitter<brain_box_msgs::StampedAltimeter> *altimeter_bs_;
    am::BabySitter<sensor_msgs::Joy> *dji_bs_;

public:
    Super() :
            nh_("~")
    {
        ROS_INFO_STREAM(NODE_FUNC);

        ros::param::param<int>("~status_log_level", status_log_level_, 1);
        ROS_INFO_STREAM(
                NODE_FUNC << "status_log_level = " << status_log_level_);

        ros::param::param<bool>("~node_manifest", node_manifest_, false);
        ROS_INFO_STREAM(NODE_FUNC << "node_manifest = " << node_manifest_);

        ros::param::param<double>("~node_timeout_s", node_timeout_s_, 2.0);
        ROS_INFO_STREAM(NODE_FUNC << "node_timeout_s = " << node_timeout_s_);

        ros::param::param<bool>("~test_mode", test_mode_, false);
        ROS_INFO_STREAM(NODE_FUNC << "test_mode = " << test_mode_);

        run_nodes_ = 0;
        man_run_nodes_ = 0;
        string tmp_manifest;
        ros::param::param<string>("~manifest", tmp_manifest, "");
        boost::erase_all(tmp_manifest, " ");
        if (tmp_manifest.size())
        {
            boost::split(manifest_, tmp_manifest, boost::is_any_of(","));
            ROS_INFO_STREAM(NODE_FUNC << "configuring nodes from manifest:");
            for (string &name : manifest_)
            {
                NodeRec nr;
                nr.name = name;
                nr.pid = -1;
                nr.last_contact = ros::Time(0);
                nr.manifested = true;
                nr.running = false;
                nodes_.insert(pair<string, NodeRec>(name, nr));
                ROS_INFO_STREAM(NODE_FUNC << "  " << name);
                if (!name.compare("os1_node"))
                {
                    int lidar_warn_ms, lidar_error_ms;
                    calcTiming(LIDAR_HZ, lidar_warn_ms, lidar_error_ms);
                    lidar_bs_ = new am::BabySitter<sensor_msgs::PointCloud2>(
                            nh_, BagLogger::instance(), name,
                            am::am_topics::SENSOR_LIDAR_POINTCLOUD2,
                            lidar_warn_ms, lidar_error_ms, 5, 2000,
                            10000 /*, std::bind(&Super::errorCB, this) */);
                } else if (!name.compare("can_node"))
                {
                    int altimeter_warn_ms, altimeter_error_ms;
                    calcTiming(ALTIMETER_HZ, altimeter_warn_ms,
                            altimeter_error_ms);
                    altimeter_bs_ = new am::BabySitter<
                            brain_box_msgs::StampedAltimeter>(nh_,
                            BagLogger::instance(), name,
                            "/sensor/distance/agl_lw", altimeter_warn_ms,
                            altimeter_error_ms);
                } else if (!name.compare("dji_sdk"))
                {
                    int dji_warn_ms, dji_error_ms;
                    calcTiming(RC_HZ, dji_warn_ms, dji_error_ms);
                    dji_bs_ = new am::BabySitter<sensor_msgs::Joy>(nh_,
                            BagLogger::instance(), name, "/dji_sdk/rc",
                            dji_warn_ms, dji_error_ms);
                }
            }
        }
        printStatus();

        super_status_pub_ = nh_.advertise<brain_box_msgs::Super2Status>(
                "/super/status", 1000);
        vstate_summary_pub_ = nh_.advertise<brain_box_msgs::VxState>(
                "/vstate/summary", 1000);
        led_pub_ = nh_.advertise<brain_box_msgs::BlinkMCommand>(
                am::am_topics::LED_BLINK, 1000);

        level_ = BagLogger::OFF;
        state_ = brain_box_msgs::VxState::UNKNOWN;
        fc_state_ = FCState::NONE;

        BagLogger::instance()->startLogging("SU", LOG_LEVEL);

        // subs should always come at the end
        node_state_sub_ = nh_.subscribe("/node_state", 100, &Super::nodeStateCB,
                this);
        node_status_sub_ = nh_.subscribe("/process/status", 100,
                &Super::statusCB, this);
        log_control_sub_ = nh_.subscribe<brain_box_msgs::LogControl>(
                "/ctrl/log_control", 10, &Super::logControlCB, this);
        // lidar_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(am::am_topics::SENSOR_LIDAR_POINTCLOUD2, 10, &Super::lidarPC2CB, this);

        heartbeat_timer_ = nh_.createTimer(ros::Duration(1.0),
                &Super::heartbeatCB, this);
    }

    ~Super()
    {
        delete lidar_bs_;
        BagLogger::instance()->stopLogging();
    }

    void nodeStateCB(
            const ros::MessageEvent<brain_box_msgs::LifeCycleState const> &event)
    {
        if (test_mode_)
        {
            return;
        }

        const brain_box_msgs::LifeCycleState::ConstPtr &rmsg =
                event.getMessage();

        if (rmsg->status != brain_box_msgs::LifeCycleState::STATUS_ERROR)
        {
            processState(rmsg->node_name, (LifeCycleState) (rmsg->state),
                    rmsg->subsystem, rmsg->value, rmsg->process_id,
                    event.getReceiptTime());
        }
        LOG_MSG("/node_state", rmsg, LOG_LEVEL);
    }

    void statusCB(
            const ros::MessageEvent<brain_box_msgs::NodeStatus const> &event)
    {
        if (test_mode_)
        {
            return;
        }

        const brain_box_msgs::NodeStatus::ConstPtr &rmsg = event.getMessage();

        processState(rmsg->node_name, LifeCycleState::ACTIVE, rmsg->status,
                rmsg->value, rmsg->process_id, event.getReceiptTime());
        LOG_MSG("/process/status", rmsg, LOG_LEVEL);
    }

    void processState(const std::string &node_name_in,
            const am::LifeCycleState state, const std::string &status,
            const std::string &value, const int pid,
            const ros::Time &last_contact)
    {

        // strip leading '/' if needed
        string node_name;
        if (node_name_in.at(0) == '/')
        {
            node_name = node_name_in.substr(1);
        } else
        {
            node_name = node_name_in;
        }

        map<string, NodeRec>::iterator it;
        it = nodes_.find(node_name);
        if (it != nodes_.end())
        {
            if (!it->second.running)
            {
                ROS_INFO_STREAM(NODE_FUNC << "node came online: " << node_name);
                it->second.running = true;
                run_nodes_++;
                if (it->second.manifested)
                {
                    man_run_nodes_++;
                    printStatus();
                    checkState();
                }
            }
            it->second.last_contact = ros::Time().now();
        } else
        {
            ROS_INFO_STREAM(NODE_FUNC << "adding a new node: " << node_name);
            NodeRec nr;
            nr.name = node_name;
            nr.pid = pid;
            nr.last_contact = last_contact;
            nr.manifested = false;
            nr.running = true;
            nodes_.insert(pair<string, NodeRec>(node_name, nr));
            run_nodes_++;

            ROS_INFO_STREAM(NODE_FUNC << "state = " << (int)state);
            ROS_INFO_STREAM(NODE_FUNC << "status = " << status);
            ROS_INFO_STREAM(NODE_FUNC << "value = " << value);
            ROS_INFO_STREAM(NODE_FUNC << "process_id = " << pid);
            printStatus();
            checkState();
        }

        if (!node_name.compare("flight_controller")
                && !status.compare("FLIGHT_CONTROL"))
        {
            ROS_INFO_STREAM_THROTTLE(1.0,
                    NODE_FUNC << "flight status: " << value);
            if (!value.compare("AUTO"))
            {
                fc_state_ = FCState::AUTO;
            } else if (!value.compare("HOLD"))
            {
                fc_state_ = FCState::HOLD;
            }
            checkState();
        }

    }

    void logControlCB(const brain_box_msgs::LogControl::ConstPtr &msg)
    {
        if (!msg->enable)
        {
            ROS_INFO_STREAM(NODE_FUNC << "stop logging");
            BagLogger::instance()->stopLogging();
            level_ = BagLogger::OFF;
        }
    }

    void heartbeatCB(const ros::TimerEvent &event)
    {
        if (state_ == brain_box_msgs::VxState::UNKNOWN)
        {
            setState(brain_box_msgs::VxState::BOOTING);
        }
        brain_box_msgs::VxState state_msg;
        state_msg.state = state_;
        vstate_summary_pub_.publish(state_msg);

        if (test_mode_)
        {
            state_++;
            if (state_ > brain_box_msgs::VxState::LAST_STATE)
            {
                state_ = 0;
            }
            printState();
        } else
        {
            ros::Time now = ros::Time().now();
            map<string, NodeRec>::iterator it;
            for (it = nodes_.begin(); it != nodes_.end(); it++)
            {
                if (it->second.running)
                {
                    ros::Duration time_since_contact = now
                            - it->second.last_contact;
                    ros::Duration timeout_dur(node_timeout_s_);
                    if (time_since_contact > timeout_dur)
                    {
                        it->second.running = false;
                        run_nodes_--;
                        if (it->second.manifested)
                        {
                            man_run_nodes_--;
                        }
                        ROS_ERROR_STREAM(
                                NODE_FUNC << "node timed out:" << it->second.name);
                        printStatus();
                    }
                }
            }
        }

        checkState();

        brain_box_msgs::Super2Status status_msg;
        status_msg.man = manifest_.size();
        status_msg.man_run = man_run_nodes_;
        status_msg.run = run_nodes_;
        map<string, NodeRec>::iterator it;
        for (it = nodes_.begin(); it != nodes_.end(); it++)
        {
            status_msg.nodes.push_back(it->second.name);
        }
        LOG_MSG("/status/super", status_msg, 1);
        if (super_status_pub_.getNumSubscribers() > 0)
        {
            super_status_pub_.publish(status_msg);
        }

        std::stringstream ss;
        ss << NODE_FUNC << "state: " << parseState(state_) << ", man: "
                << manifest_.size() << ", man run:" << man_run_nodes_
                << ", run: " << run_nodes_;

        if (manifest_.size() != man_run_nodes_)
        {
            // if all manifested nodes aren't running, display in red
            ROS_ERROR_STREAM(ss.str());
        } else
        {
            ROS_INFO_STREAM_THROTTLE(LOG_PERIOD_S, ss.str());
        }

        // display nodes that aren't running in red
        for (it = nodes_.begin(); it != nodes_.end(); it++)
        {
            if (!it->second.running)
            {
                ROS_ERROR_STREAM(
                        NODE_FUNC << "node not running:" << it->second.name);
            }
        }
    }

    void errorCB()
    {
        setState(brain_box_msgs::VxState::ERROR);
    }

    bool findNode(const int process_id, map<string, NodeRec>::iterator &it)
    {
        for (it = nodes_.begin(); it != nodes_.end(); it++)
        {
            if (it->second.pid == process_id)
            {
                return true;
            }
        }
        return false;
    }

    std::string parseState(int state)
    {
        switch (state)
        {
        case brain_box_msgs::VxState::BOOTING:
            return "BOOTING";
            break;
        case brain_box_msgs::VxState::READY:
            return "READY";
            break;
        case brain_box_msgs::VxState::ARMED:
            return "ARMED";
            break;
        case brain_box_msgs::VxState::AUTO_FEATURE:
            return "AUTO_FEATURE";
            break;
        case brain_box_msgs::VxState::AUTO_NO_FEATURE:
            return "AUTO_NO_FEATURE";
            break;
        case brain_box_msgs::VxState::HOLD:
            return "HOLD";
            break;
        case brain_box_msgs::VxState::UNKNOWN:
            return "UNKNOWN";
            break;
        case brain_box_msgs::VxState::ERROR:
        default:
            return "ERROR";
            break;
        }
    }

    void printStatus()
    {
        ROS_INFO_STREAM(
                NODE_FUNC << "man: " << manifest_.size() << ", man run:" << man_run_nodes_ << ", run: " << run_nodes_);
    }

    void printState()
    {
        ROS_INFO_STREAM(NODE_FUNC << "state: " << parseState(state_));
    }

    void checkState()
    {
        switch (state_)
        {
        case brain_box_msgs::VxState::BOOTING:
            if (man_run_nodes_ == manifest_.size())
            {
                setState(brain_box_msgs::VxState::READY);
            }
            break;
        case brain_box_msgs::VxState::READY:
            if (man_run_nodes_ < manifest_.size())
            {
                setState(brain_box_msgs::VxState::ERROR);
            } else if (fc_state_ == FCState::AUTO)
            {
                setState(brain_box_msgs::VxState::AUTO_FEATURE);
            } else if (fc_state_ == FCState::HOLD)
            {
                setState(brain_box_msgs::VxState::HOLD);
            }
            break;
        case brain_box_msgs::VxState::AUTO_FEATURE:
            if (man_run_nodes_ < manifest_.size())
            {
                setState(brain_box_msgs::VxState::ERROR);
            } else if (fc_state_ == FCState::HOLD)
            {
                setState(brain_box_msgs::VxState::HOLD);
            }
            break;
        case brain_box_msgs::VxState::HOLD:
            if (man_run_nodes_ < manifest_.size())
            {
                setState(brain_box_msgs::VxState::ERROR);
            } else if (fc_state_ == FCState::AUTO)
            {
                setState(brain_box_msgs::VxState::AUTO_FEATURE);
            }
            break;
        case brain_box_msgs::VxState::ERROR:
            // once we are in error, we can't go anywhere
            break;
        }
    }

    void setState(int state)
    {
        ROS_INFO_STREAM(
                NODE_FUNC << "changing state from: " << parseState(state_) << " to: " << parseState(state));
        switch (state_)
        {
        case brain_box_msgs::VxState::BOOTING:
            state_ = state;
            break;
        case brain_box_msgs::VxState::READY:
            state_ = state;
            break;
        case brain_box_msgs::VxState::AUTO_FEATURE:
            state_ = state;
            break;
        case brain_box_msgs::VxState::HOLD:
            state_ = state;
            break;
        case brain_box_msgs::VxState::UNKNOWN:
            state_ = state;
            break;
        case brain_box_msgs::VxState::ERROR:
            state_ = state;
            break;
        }

        setLED(state_);
        printState();

        brain_box_msgs::VxState state_msg;
        state_msg.state = state_;
        vstate_summary_pub_.publish(state_msg);
    }

    void setLED(int r, int g, int b, float period = 0.0)
    {
        brain_box_msgs::BlinkMCommand led_msg;
        led_msg.rgb.red = r;
        led_msg.rgb.green = g;
        led_msg.rgb.blue = b;
        led_msg.blink_rate = period;
        led_pub_.publish(led_msg);
    }

    void setLED(int state)
    {
        int r = 0, b = 0, g = 0;
        float rate = 0.0;

        switch (state)
        {
        case brain_box_msgs::VxState::BOOTING:
            r = 0;
            g = 0;
            b = 255;
            rate = 2.0;
            break;
        case brain_box_msgs::VxState::READY:
            r = 0;
            g = 0;
            b = 255;
            rate = 0.0;
            break;
        case brain_box_msgs::VxState::AUTO_FEATURE:
            r = 0;
            g = 255;
            b = 0;
            rate = 0.0;
            break;
        case brain_box_msgs::VxState::HOLD:
            r = 255;
            g = 0;
            b = 0;
            rate = 2.0;
            break;
        case brain_box_msgs::VxState::ERROR:
            r = 255;
            g = 0;
            b = 0;
            rate = 0.0;
            break;
        }

        brain_box_msgs::BlinkMCommand led_msg;
        led_msg.rgb.red = r;
        led_msg.rgb.green = g;
        led_msg.rgb.blue = b;
        led_msg.blink_rate = rate;
        led_pub_.publish(led_msg);
    }

    void calcTiming(int hz, int &warn_ms, int &error_ms)
    {
        warn_ms = (int) (1000.0 / hz * 2.0 + 0.5);
        error_ms = (int) (1000.0 / hz * 3.0 + 0.5);
    }

};
}
;

#ifdef TESTING
#else
int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);

    am::Super node;

    ROS_INFO_STREAM(NODE_NAME << ": running...");

    ros::spin();
}
#endif
