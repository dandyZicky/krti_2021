#include <ros/ros.h>
#include <controller/controller.h>
const float RATE = 5.0f;
double startTime, endTime;

void startMission(Controller::DroneController& control)
{
    ros::Rate rate(RATE);
    // control.missionAlpha();
    control.missionBeta();
    // control.missionGamma();

    while(ros::ok())
    {
        if(control.getAlt() <= 0.35)
        {
            endTime = ros::Time::now().toSec();
            ROS_INFO_STREAM_ONCE("MISSION COMPLETED IN " + std::to_string((endTime-startTime)) + " seconds");
            break;
        }
        rate.sleep();
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh("~");
    Controller::DroneController control = Controller::DroneController(nh);
    startTime = ros::Time::now().toSec(); ROS_INFO("TIMER START");

    startMission(control);

    return 0;
}