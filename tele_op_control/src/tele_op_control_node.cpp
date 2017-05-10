#include <tele_op_control/tele_op_control.hpp>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"tele_op_control_node");

    TeleOp control;
    ros::spin();
    return 0;
}
