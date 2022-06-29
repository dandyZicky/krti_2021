#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>

namespace Controller{
class DroneController
{
private:
    ros::NodeHandle nh_;
    ros::Rate m_rate, m_rateFaster;
    ros::Publisher p_setVel, p_dropCounter, p_nullwp, p_setAttitude, p_isDropping, p_isLanding;

    ros::Subscriber s_dropzoneDetectedBool, s_elpDetectedBool, 
                    s_targetX_dropzone, s_targetY_dropzone, 
                    s_targetX, s_targetY, 
                    s_dropCounter, s_localPose, s_alt, s_fcuState, 
                    s_wpReached, s_centeringLimitDrop, s_centeringLimit,
                    s_IMU;

    ros::ServiceClient srv_setArm, srv_setTakeoff, srv_setMode;

    //Member Variable
    float m_alt;
    int m_dropCounter, m_targetX_dropzone, m_targetY_dropzone, m_targetX, m_targetY, m_centeringLimitDrop, m_centeringLimit;
    bool m_dropzoneDetectedBool, m_elpDetectedBool;
    
    mavros_msgs::WaypointReached m_nullWP;
    uint16_t m_wpReached;
    mavros_msgs::State m_fcuState;
    geometry_msgs::Vector3 m_linAcc, m_angVel, m_orient;

    //Member Variable for ROS Service Client
    mavros_msgs::CommandBool m_setArm;
    mavros_msgs::SetMode m_setMode;
    mavros_msgs::CommandTOL m_setTakeoff;
    mavros_msgs::PositionTarget m_setVel;
    mavros_msgs::AttitudeTarget m_setAttitude;
    std_msgs::Bool m_isDropping, m_isLanding;

    //Member Methods
    void initSub();
    void initPub();
    void initSrv();

    void setArm();
    void setDisarm();
    void setGuided();
    void setGuidedNOGPS();
    void setAuto();
    void setLand();
    void setTakeoff();
    void setRTL();
    void setNullWP();
    void centeringAndDrop();
    void centeringAndLand();
    void attitudePublish(double& thrust, double& x, double& y, double& z);
    void beta_centeringAndDrop();
    void roll();

    //Experimental functions ---under development---

    void stop();
    void rawTakeoff();

    //Member Methods Callback
    void cb_targetX_dropzone(const std_msgs::Int32::ConstPtr& data);
    void cb_targetY_dropzone(const std_msgs::Int32::ConstPtr& data);
    void cb_targetX(const std_msgs::Int32::ConstPtr& data);
    void cb_targetY(const std_msgs::Int32::ConstPtr& data);
    void cb_localPose(const geometry_msgs::PoseStamped::ConstPtr& data);
    void cb_alt(const std_msgs::Float64::ConstPtr& data);
    void cb_fcuState(const mavros_msgs::State::ConstPtr& data);
    void cb_wpReached(const mavros_msgs::WaypointReached::ConstPtr& data);
    void cb_centeringLimitDrop(const std_msgs::Int32::ConstPtr& data);
    void cb_centeringLimit(const std_msgs::Int32::ConstPtr& data);
    void cb_elpDetectedBool(const std_msgs::Bool::ConstPtr& data);
    void cb_dropzoneDetectedBool(const std_msgs::Bool::ConstPtr& data);
    void cb_dropCounter(const std_msgs::Int32::ConstPtr& data);
    void cb_IMU(const sensor_msgs::Imu::ConstPtr& data);
    
public:
    DroneController(ros::NodeHandle& nodehandle);
    ~DroneController();
    double getAlt();

    //MISSIONS
    void missionAlpha();
    void missionBeta();
    void missionGamma();
};
}