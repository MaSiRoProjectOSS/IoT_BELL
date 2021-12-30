
#include "TWELITE_app_cue.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TWELITE_app_cue>());
    rclcpp::shutdown();
    return 0;
}
