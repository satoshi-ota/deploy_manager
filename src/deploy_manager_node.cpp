#include "deploy_module/deploy_manager.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "deploy_manager_node");
    deploy_module::DeployManager deploy_manager_node;

    ros::spin();
    return 0;
}
