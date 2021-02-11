#include "deploy_module/deploy_manager.h"

namespace deploy_module
{

DeployManager::DeployManager()
    :initialized_(false),
     state_(WAITING)
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param("cmd_frequency", cmd_frequency_, 10);
    private_nh.param("purge_timing", purge_timing_, 0.5);
    private_nh.param("unlock_load_th", unlock_load_th_, -300000.0);
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

    ros::Duration duration;
    ros::Time timer_start = ros::Time::now(), timer_end;
    ros::Rate rate(cmd_frequency_);

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
                    if(load_ < unlock_load_th_){

                        ros::Duration(purge_timing_).sleep();

                        timer_end = ros::Time::now();
                        duration = timer_end - timer_start;

                        std_msgs::Int32 td_cmd_spd;
                        td_cmd_spd.data = STOP_WINCH;
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

                    ros::Duration(purge_timing_).sleep();

                    timer_end = ros::Time::now();

                    state_ = LIFTUP;

                    break;
                }

            case LIFTUP:
                {
                    if(timer_end + ros::Duration(duration.toSec() - 2.0) < ros::Time::now()){

                        resetState();

                        ROS_INFO_NAMED("deploy_module","Lift up purge top board. ");
                        res.success = true;
                        return true;
                    }

                    std_msgs::Int32 lu_cmd_spd;
                    lu_cmd_spd.data = lift_up_speed_;
                    winch_cmd_pub_.publish(lu_cmd_spd);

                    break;
                }

            default:
                ROS_ERROR_NAMED("deploy_module","This case should never be reached, something is wrong, aborting");
                resetState();
                return false;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return true;
}

void DeployManager::commandCB(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    ROS_INFO_ONCE_NAMED("deploy_module","Load cell data recieved. ");

    initialized_ = true;
    load_ = (double)msg->wrench.force.z;
}

void DeployManager::resetState()
{
    state_ = WAITING;

    std_msgs::Int32 td_cmd_spd;
    td_cmd_spd.data = STOP_WINCH;
    winch_cmd_pub_.publish(td_cmd_spd);

    // std_msgs::Bool unlock_cmd;
    // unlock_cmd.data = false;
    // unlock_cmd_pub_.publish(unlock_cmd);

    ROS_INFO_NAMED("deploy_module","Reset deployment service. ");
}

} //namespace deploy_module
