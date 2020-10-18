#ifndef DEPLOY_MODULE_DEPLOY_MANAGER_H
#define DEPLOY_MODULE_DEPLOY_MANAGER_H

#include <string.h>

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#define STOP_WINCH  0

enum DeployState
{
    WAITING,
    TAKEDOWN,
    PURGE,
    LIFTUP
};

namespace deploy_module
{

class DeployManager
{
public:
    DeployManager();
    ~DeployManager();

    bool deploy(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void commandCB(const geometry_msgs::WrenchStamped::ConstPtr& msg);

    void resetState();

private:
    ros::Subscriber load_sub_;
    ros::Publisher unlock_cmd_pub_;
    ros::Publisher winch_cmd_pub_;
    ros::ServiceServer deploy_service_;

    DeployState state_;

    int load_;
    int unlock_load_th_;
    int takedown_speed_;
    int lift_up_speed_;
    bool initialized_;
};

} //namespace deploy_module

#endif //DEPLOY_MODULE_DEPLOY_MANAGER_H
