#include "deploy_module/deploy_manager.h"

namespace deploy_module
{

DeployManager::DeployManager()
    :initialized_(false),
     state_(WAITING)
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param("unlock_load_th", unlock_load_th_, -50000);
    private_nh.param("takedown_speed", takedown_speed_, 255);
    private_nh.param("lift_up_speed", lift_up_speed_, -255);

    load_sub_ = nh.subscribe<geometry_msgs::WrenchStamped>("/joule/load", 10, &DeployManager::commandCB, this);
    unlock_cmd_pub_ = nh.advertise<std_msgs::Bool>("/watt/unlock", 0);
    winch_cmd_pub_ = nh.advertise<std_msgs::Int32>("/joule/winch_speed", 0);

    deploy_service_ = nh.advertiseService("deploy", &DeployManager::deploy, this);
}

DeployManager::~DeployManager(){ }

bool DeployManager::deploy(std_srvs::Trigger::Request  &req,
                           std_srvs::Trigger::Response &res)
{
    if(!initialized_){
        ROS_WARN_NAMED("deploy_module","Winch controller has not recieved load cell data! ");
        return false;
    }

    ROS_INFO_NAMED("deploy_module","Service recieved deployment request. ");

    state_ = TAKEDOWN;

    while(ros::ok()){

        switch(state_)
        {
            case WAITING:
                {
                    break;
                }

            case TAKEDOWN:
                {
                    if(unlock_load_th_ < load_){

                        ros::Duration(0.5).sleep();

                        std_msgs::Int32 td_cmd_spd;
                        td_cmd_spd.data = 0;
                        winch_cmd_pub_.publish(td_cmd_spd);

                        state_ = PURGE;

                        ROS_INFO_NAMED("deploy_module","Totch down detected. Purge graund robot. ");
                        break;
                    }

                    std_msgs::Int32 td_cmd_spd;
                    td_cmd_spd.data = takedown_speed_;
                    winch_cmd_pub_.publish(td_cmd_spd);

                    break;
                }

            case PURGE:
                {
                    std_msgs::Bool unlock_cmd;
                    unlock_cmd.data = true;
                    unlock_cmd_pub_.publish(unlock_cmd);

                    res.success = true;
                    return true;

                    break;
                }

            case LIFTUP:
                {
                    std_msgs::Int32 lu_cmd_spd;
                    lu_cmd_spd.data = STOP_WINCH;
                    winch_cmd_pub_.publish(lu_cmd_spd);

                    break;
                }

            default:
                ROS_ERROR_NAMED("deploy_module","This case should never be reached, something is wrong, aborting");
                resetState();
                return false;
        }

        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }

    return true;
}

void DeployManager::commandCB(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    ROS_INFO_ONCE_NAMED("deploy_module","Load cell data recieved. ");

    initialized_ = true;
    load_ = msg->wrench.force.z;
}

void DeployManager::resetState()
{
    state_ = WAITING;

    std_msgs::Int32 td_cmd_spd;
    td_cmd_spd.data = STOP_WINCH;
    winch_cmd_pub_.publish(td_cmd_spd);

    std_msgs::Bool unlock_cmd;
    unlock_cmd.data = false;
    unlock_cmd_pub_.publish(unlock_cmd);

    ROS_INFO_NAMED("deploy_module","Reset deployment service. ");
}

} //namespace deploy_module
