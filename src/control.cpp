#include <ros/ros.h>
#include <controller/controller.h>
#include <tf/transform_datatypes.h>
#include <iostream>

const int CENTERX = 320, CENTERY = 240, MAX_CENTERED_LIMIT = 12;

/*--------FAST VELOCITY UPDATE RATE SYSTEM--------*/
const int RATE = 20, RATE_FASTER = 100;
const double KP = 0.00100, KPCENTERED = 0.000420, smKP = 0.000250, smKPCENTERED = 0.00100,
             KD = 0.001820, KDCENTERED = 0.000110, smKD = 0.002330, smKDCENTERED = 0.000060,
             KI = 0.000100;

const double Z_GAIN_DOWN = -0.75, Z_GAIN_UP = 0.75,
             ALT = 1.0, DELAY_AFTER_DROP = 10.0, CENTERING_DROP_TIME_LIMIT = 5.0,
             CENTERING_ELP_LIMIT = 200.0, CENTERING_ELP_TIME_LIMIT = 5.0,
             ALT_SAFETY_LIMIT = 20.0, ALT_FOR_LAND = 1.10;

const int WP1 = 1, WP2 = 2, WP3 = 3, WP4 = 4;

/*-------------------Initiating Subscriber Callback------------------------*/
void Controller::DroneController::cb_fcuState(const mavros_msgs::State::ConstPtr& data){m_fcuState = *data; }

void Controller::DroneController::cb_wpReached(const mavros_msgs::WaypointReached::ConstPtr& data){m_wpReached = data->wp_seq;}

void Controller::DroneController::cb_alt(const std_msgs::Float64::ConstPtr& data){m_alt = data->data;}

void Controller::DroneController::cb_elpDetectedBool(const std_msgs::Bool::ConstPtr& data){m_elpDetectedBool = data->data;}

void Controller::DroneController::cb_targetX(const std_msgs::Int32::ConstPtr& data){m_targetX = data->data;}

void Controller::DroneController::cb_targetY(const std_msgs::Int32::ConstPtr& data){m_targetY = data->data;}

void Controller::DroneController::cb_dropzoneDetectedBool(const std_msgs::Bool::ConstPtr& data){m_dropzoneDetectedBool = data->data;}

void Controller::DroneController::cb_targetX_dropzone(const std_msgs::Int32::ConstPtr& data){m_targetX_dropzone = data->data;}

void Controller::DroneController::cb_targetY_dropzone(const std_msgs::Int32::ConstPtr& data){m_targetY_dropzone = data->data;}

void Controller::DroneController::cb_centeringLimitDrop(const std_msgs::Int32::ConstPtr& data){m_centeringLimitDrop = data->data;}

void Controller::DroneController::cb_centeringLimit(const std_msgs::Int32::ConstPtr& data){m_centeringLimit = data->data;}

void Controller::DroneController::cb_dropCounter(const std_msgs::Int32::ConstPtr& data){m_dropCounter = data->data;}

void Controller::DroneController::cb_IMU(const sensor_msgs::Imu::ConstPtr& data)
{

    tf::Quaternion quat;
    tf::quaternionMsgToTF(data->orientation, quat);

    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;

    m_linAcc.x = data->linear_acceleration.y;
    m_linAcc.y = -(data->linear_acceleration.x);
    m_linAcc.z = data->linear_acceleration.z;

    m_angVel = data->angular_velocity;
    m_orient = rpy;
}
/*----------------------------------------------------------------------------*/


/*---------------Initiating Subscriber Getter---------------*/
double Controller::DroneController::getAlt(){return m_alt;}
/*----------------------------------------------------------*/

/*---------------Initializing Subscriber--------------------*/
void Controller::DroneController::initSub()
{
    ROS_INFO("Initializing Subscribers!");
    s_fcuState = nh_.subscribe("/mavros/state", 1, &Controller::DroneController::cb_fcuState, this);
    s_wpReached = nh_.subscribe("/mavros/mission/reached", 1, &Controller::DroneController::cb_wpReached, this);
    s_alt = nh_.subscribe("/mavros/global_position/rel_alt", 1, &Controller::DroneController::cb_alt, this);
    s_dropzoneDetectedBool = nh_.subscribe("/gmfc/vision/dropzone_is_detected", 1, &Controller::DroneController::cb_dropzoneDetectedBool, this);
    s_targetX = nh_.subscribe("/gmfc/vision/elp_target_x", 1, &Controller::DroneController::cb_targetX, this);
    s_targetY = nh_.subscribe("/gmfc/vision/elp_target_y", 1, &Controller::DroneController::cb_targetY, this);
    s_targetX_dropzone = nh_.subscribe("/gmfc/vision/dropzone_target_x", 1, &Controller::DroneController::cb_targetX_dropzone, this);
    s_targetY_dropzone = nh_.subscribe("/gmfc/vision/dropzone_target_y", 1, &Controller::DroneController::cb_targetY_dropzone, this);
    s_centeringLimitDrop = nh_.subscribe("/gmfc/centering_drop_limit", 1, &Controller::DroneController::cb_centeringLimitDrop, this);
    s_centeringLimit = nh_.subscribe("/gmfc/centering_land_limit", 1, &Controller::DroneController::cb_centeringLimit, this);
    s_dropCounter = nh_.subscribe("/gmfc/drop_counter", 1, &Controller::DroneController::cb_dropCounter, this);
    s_elpDetectedBool = nh_.subscribe("/gmfc/vision/elp_is_detected", 1, &Controller::DroneController::cb_elpDetectedBool, this);
    s_IMU = nh_.subscribe("/mavros/imu/data", 1, &Controller::DroneController::cb_IMU, this);
}
/*----------------------------------------------------------*/

/*---------------Initializing Publisher---------------------*/
void Controller::DroneController::initPub()
{
    ROS_INFO("Initializing Publisher");
    p_nullwp = nh_.advertise<mavros_msgs::WaypointReached>("/mavros/mission/reached", 1);
    p_setVel = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    p_setAttitude = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
    p_isDropping = nh_.advertise<std_msgs::Bool>("/gmfc/control/isDropping", 1);
    p_isLanding = nh_.advertise<std_msgs::Bool>("/gmfc/control/isLanding", 1);
}
/*----------------------------------------------------------*/

/*-------------- Initializing Services Methods--------------*/
void Controller::DroneController::initSrv()
{
    ROS_INFO("Initializing Subscriber");
    srv_setArm = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    srv_setMode = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    srv_setTakeoff = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
}

void Controller::DroneController::setArm()
{
    m_setArm.request.value = 1;
    while(ros::ok() && !m_fcuState.armed)
    {
        srv_setArm.call(m_setArm);
        m_rateFaster.sleep();
    }
    ROS_INFO("COPTER IS ARMED");
}

void Controller::DroneController::setDisarm()
{
    m_setArm.request.value = 0;
    while(ros::ok() && m_fcuState.armed)
    {
        srv_setArm.call(m_setArm);
        m_rateFaster.sleep();
    }
    ROS_INFO("COPTER IS DISARMED");

}

void Controller::DroneController::setGuided()
{
    m_setMode.request.custom_mode = "GUIDED";
    while(ros::ok() && m_fcuState.mode != "GUIDED")
    {
        srv_setMode.call(m_setMode);
        m_rateFaster.sleep();
    }
    ROS_INFO("MODE IS GUIDED");
}

void Controller::DroneController::setGuidedNOGPS()
{
    m_setMode.request.custom_mode = "GUIDED_NOGPS";
    while(ros::ok() && m_fcuState.mode != "GUIDED_NOGPS")
    {
        srv_setMode.call(m_setMode);
        m_rateFaster.sleep();
    }
    ROS_INFO("MODE IS GUIDEDNOGPS");
}

void Controller::DroneController::setTakeoff()
{
    ROS_INFO("Taking off started!");
    m_setTakeoff.request.altitude = ALT;
    srv_setTakeoff.call(m_setTakeoff);
    ROS_INFO("Taking off to %f meters...", ALT);
    while(m_alt <= 0.8*ALT){m_rateFaster.sleep();}
    ROS_INFO("ALTITUDE ACHIEVED!");
}

void Controller::DroneController::setLand()
{
    m_setMode.request.custom_mode = "LAND";
    while(ros::ok() && m_fcuState.mode != "LAND")
    {
        srv_setMode.call(m_setMode);
        m_rateFaster.sleep();
    }
    ROS_INFO("MODE IS LAND!");
}

void Controller::DroneController::setAuto()
{
    m_setMode.request.custom_mode = "AUTO";
    while(ros::ok() && m_fcuState.mode != "AUTO")
    {
        srv_setMode.call(m_setMode);
        m_rateFaster.sleep();
    }
    ROS_INFO("MODE IS AUTO");
}

void Controller::DroneController::setRTL()
{
    m_setMode.request.custom_mode = "RTL";
    while(ros::ok() && m_fcuState.mode != "RTL")
    {
        srv_setMode.call(m_setMode);
        m_rateFaster.sleep();
    }
    ROS_INFO("MODE IS RTL");
}

void Controller::DroneController::setNullWP()
{
    m_nullWP.wp_seq = 0;   
    while(ros::ok() && m_wpReached != 0)
    {
        p_nullwp.publish<mavros_msgs::WaypointReached>(m_nullWP);   
        m_rateFaster.sleep();
    }
    ROS_INFO("WP SET TO NULL");
}

void Controller::DroneController::centeringAndDrop()
{
    ROS_INFO("STARTED CENTERING FOR DROPPING!");
    double prev_time = ros::Time::now().toSec(),
           prev_time_afdrop = ros::Time::now().toSec(),
           deltaPrevX{}, deltaPrevY{}, deltaX{}, deltaY{}, deltaXCum{}, 
           deltaYCum{}, kp{}, kd{}, kpcentered{}, kdcentered{},
           lebarDropzone{};

    m_setVel.coordinate_frame = 8;
    m_setVel.type_mask = 0;
    
    while(ros::ok())
    {
        if (m_dropzoneDetectedBool)
        {
            // Change gains depending on the target size (copter alt is better if the readings are reliable)
            if(m_centeringLimitDrop > 30)
            {
                kp = smKP; // KP/1.5
                kd = smKD; // KD/3
            }
            else if(m_centeringLimitDrop > 40)
            {
                kp = smKP/2;
                kd = smKD/2;
            }
            else
            {
                kp = KP;
                kd = KD;
            }

            deltaY = CENTERX - m_targetX_dropzone;
            deltaX = CENTERY - m_targetY_dropzone;

            m_setVel.velocity.x = deltaX * kp + kd*(deltaX-deltaPrevX);
            m_setVel.velocity.y = deltaY * kp + kd*(deltaY-deltaPrevY);

            deltaPrevX = deltaX;
            deltaPrevY = deltaY;

            if(-m_centeringLimitDrop < deltaY && deltaY < m_centeringLimitDrop
            && -m_centeringLimitDrop < deltaX && deltaX < m_centeringLimitDrop)
            {
                prev_time_afdrop = ros::Time::now().toSec();
                double current_time_afdrop = prev_time_afdrop;
                double delta_time_afdrop = current_time_afdrop - prev_time_afdrop;
                ROS_WARN("DELAY AFTER DROP START!");
                while(ros::ok() && (delta_time_afdrop < DELAY_AFTER_DROP))
                {
                    current_time_afdrop = ros::Time::now().toSec();
                    delta_time_afdrop = current_time_afdrop - prev_time_afdrop;
                    deltaY = CENTERX - m_targetX_dropzone;
                    deltaX = CENTERY - m_targetY_dropzone;

                    if(m_centeringLimitDrop > 30)
                    {
                        kpcentered = smKPCENTERED; // KPCENTERED/4
                        kdcentered = smKDCENTERED; // KDCENTERED/2
                    }
                    else if(m_centeringLimitDrop > 40)
                    {
                        kpcentered = smKPCENTERED/2;
                        kdcentered = smKDCENTERED/2;
                    }
                    else
                    {
                        kpcentered = KPCENTERED;
                        kdcentered = KDCENTERED;
                    };

                    if(-MAX_CENTERED_LIMIT < deltaY && deltaY < MAX_CENTERED_LIMIT
                    && -MAX_CENTERED_LIMIT < deltaX && deltaX < MAX_CENTERED_LIMIT)
                    {   
                        deltaXCum = deltaXCum + KI*deltaX;
                        deltaYCum = deltaYCum + KI*deltaY;
                        m_setVel.velocity.x = deltaX * KPCENTERED*1.2 + deltaXCum;
                        m_setVel.velocity.y = deltaY * KPCENTERED*1.2 + deltaYCum;
                        m_setVel.velocity.z = 0.00;
                    }
                    else
                    {   
                        m_setVel.velocity.x = deltaX * kpcentered + kdcentered*(deltaX-deltaPrevX);
                        m_setVel.velocity.y = deltaY * kpcentered + kdcentered*(deltaY-deltaPrevY);
                        m_setVel.velocity.z = 0.00;
                    };

                    deltaPrevX = deltaX;
                    deltaPrevY = deltaY;
                    p_setVel.publish<mavros_msgs::PositionTarget>(m_setVel);
                    
                    m_rate.sleep();
                }
                // on exit
                m_setVel.velocity.x = 0.0;
                m_setVel.velocity.y = 0.0;
                m_setVel.velocity.z = 0.0;
                // m_isDropping.data = 0;
                // p_isDropping.publish<std_msgs::Bool>(m_isDropping);
                break;
            }
            else
            {
                m_setVel.velocity.z = 0.0;
            }
            prev_time = ros::Time::now().toSec();
        }
        else
        {
            double current_time = ros::Time::now().toSec();
            double delta_time = current_time - prev_time;
            m_setVel.velocity.x = m_setVel.velocity.x / 1.02;
            m_setVel.velocity.y = m_setVel.velocity.y / 1.02;
            (m_setVel.velocity.z <= 0.020) ? m_setVel.velocity.z = m_setVel.velocity.z + 0.0005 : m_setVel.velocity.z = 0.020;

            
            if (delta_time > CENTERING_DROP_TIME_LIMIT)
            {
                prev_time_afdrop = ros::Time::now().toSec();
                double current_time_afdrop = prev_time_afdrop;
                double delta_time_afdrop = current_time_afdrop - prev_time_afdrop;
                ROS_WARN("DELAY AFTER FORCED DROP, START....");
                while(ros::ok() && (delta_time_afdrop < DELAY_AFTER_DROP))
                {
                    double current_time_afdrop = ros::Time::now().toSec();
                    delta_time_afdrop = current_time_afdrop-prev_time_afdrop;
                    m_setVel.velocity.x = 0.0;
                    m_setVel.velocity.y = 0.0;
                    m_setVel.velocity.z = 0.0;
                    p_setVel.publish<mavros_msgs::PositionTarget>(m_setVel);
                    m_rate.sleep();
                }
                m_setVel.velocity.x = 0.0;
                m_setVel.velocity.y = 0.0;
                m_setVel.velocity.z = 0.0;
                break;
            }
        }

        p_setVel.publish<mavros_msgs::PositionTarget>(m_setVel);
        
        m_rate.sleep();
    }
}

void Controller::DroneController::centeringAndLand()
{
    ROS_INFO("STARTED CENTERING AND LANDING");
    double prevTime, currTime, deltaX, deltaY;
    prevTime = ros::Time::now().toSec();
    m_setVel.coordinate_frame = 8;
    m_setVel.type_mask = 0;
    m_setVel.velocity.x = 0.0;
    m_setVel.velocity.y = 0.0;
    m_setVel.velocity.z = 0.0;

    while(ros::ok())
    {
        if(m_elpDetectedBool)
        {
            deltaX = CENTERY - m_targetY;
            deltaY = CENTERX - m_targetX;

            m_setVel.velocity.x = KPCENTERED * deltaX;
            m_setVel.velocity.y = KPCENTERED * deltaY;

            if(-CENTERING_ELP_LIMIT < m_targetX - CENTERX && m_targetX - CENTERX < CENTERING_ELP_LIMIT &&
            -CENTERING_ELP_LIMIT < m_targetY - CENTERY && m_targetY - CENTERY < CENTERING_ELP_LIMIT)
            {
                m_setVel.velocity.z = Z_GAIN_DOWN;
            }
            else
            {
                m_setVel.velocity.z = 0.0;
            }
            prevTime = ros::Time::now().toSec();
        }
        else
        {
            currTime = ros::Time::now().toSec();
            if((currTime-prevTime) > CENTERING_ELP_TIME_LIMIT)
            {
                m_setVel.velocity.x = 0.0;
                m_setVel.velocity.y = 0.0;
                m_setVel.velocity.z = Z_GAIN_UP/2;
                if (m_alt > ALT_SAFETY_LIMIT)
                {
                    setLand();
                    m_isLanding.data = 0;
                    p_isLanding.publish<std_msgs::Bool>(m_isLanding);
                    break;
                }
            }
            else
            {
                m_setVel.velocity.x = m_setVel.velocity.x / 1.10;
                m_setVel.velocity.y = m_setVel.velocity.y / 1.10;
                m_setVel.velocity.z = 0.0;
            }
        }
        p_setVel.publish(m_setVel);

        if(m_alt < ALT_FOR_LAND)
        {
            setLand();
            break;
        }
        
        m_rate.sleep();
    }


}
/*-----------------------------------------------------*/

/*--------------------Experimental function--------------*/
void Controller::DroneController::attitudePublish(double& thrust, double& x, double& y, double& z)
{
    m_setAttitude.type_mask = 128;
    m_setAttitude.header.stamp = ros::Time::now();
    m_setAttitude.thrust = thrust;
    m_setAttitude.body_rate.x = x; 
    m_setAttitude.body_rate.y = y;
    m_setAttitude.body_rate.z = z;
    p_setAttitude.publish<mavros_msgs::AttitudeTarget>(m_setAttitude);
}

void Controller::DroneController::beta_centeringAndDrop()
{
    ROS_INFO("STARTED CENTERING FOR DROPPING!");
    double prev_time = ros::Time::now().toSec(),
           prev_time_afdrop = ros::Time::now().toSec(),
           deltaPrevX{}, deltaPrevY{}, deltaX{}, deltaY{}, deltaXCum{}, deltaYCum{},
           deltaRoll{}, deltaPitch{},
           targetRoll{}, targetPitch{}, targetVelX{}, targetVelY{},
           delta_deltaX{}, delta_deltaY{}, estVelX{}, estVelY{},
           rateX{}, rateY{}, rateZ{}, thrust{},
           kp{}, kd{}, kpcentered{}, kdcentered{},
           lebarDropzone{};
    
    const double trim = 0.15, gainDeg = 1.2;

    while(ros::ok())
    {
        if (m_dropzoneDetectedBool)
        {
            // Change gains depending on the target size (copter alt is better if the readings are reliable)
            if(m_centeringLimitDrop > 30)
            {
                kp = smKP; // KP/1.5
                kd = smKD; // KD/3
            }
            else if(m_centeringLimitDrop > 40)
            {
                kp = smKP/2;
                kd = smKD/2;
            }
            else
            {
                kp = KP;
                kd = KD;
            }

            deltaX = -(CENTERX - m_targetX_dropzone); //Find positional error feedback
            deltaY = (CENTERY - m_targetY_dropzone);
            delta_deltaX = deltaX-deltaPrevX;
            delta_deltaY = deltaY-deltaPrevY;
            targetVelX = deltaX * kp + kd*(delta_deltaX); //Find optimal target velocity to minimize the positional error
            targetVelY = deltaY * kp + kd*(delta_deltaY);
            estVelX = m_linAcc.x*trim;
            estVelY = m_linAcc.y*trim;
            targetRoll = (targetVelX - estVelX) * gainDeg;  //Find optimal attitude according to target velocity
            targetPitch = (targetVelY - estVelY) * gainDeg;

            // if (targetRoll > 0.15) {targetRoll = 0.15;} else if (targetRoll < -0.15){targetRoll = -0.15;};
            // if (targetPitch > 0.15) {targetPitch = 0.15;} else if (targetPitch < -0.15){targetPitch = -0.15;};

            rateX = (targetRoll - m_orient.x)*5;  //Find optimal body rate to achieve optimal attitude
            rateY = (targetPitch - m_orient.y)*5;

            deltaPrevX = deltaX;
            deltaPrevY = deltaY;

            if(-m_centeringLimitDrop < deltaY && deltaY < m_centeringLimitDrop
            && -m_centeringLimitDrop < deltaX && deltaX < m_centeringLimitDrop)
            {
                prev_time_afdrop = ros::Time::now().toSec();
                double current_time_afdrop = prev_time_afdrop;
                double delta_time_afdrop = current_time_afdrop - prev_time_afdrop;
                ROS_WARN("DELAY AFTER DROP START!");
                while(ros::ok() && (delta_time_afdrop < DELAY_AFTER_DROP))
                {
                    current_time_afdrop = ros::Time::now().toSec();
                    delta_time_afdrop = current_time_afdrop - prev_time_afdrop;
                    deltaX = -(CENTERX - m_targetX_dropzone);
                    deltaY = (CENTERY - m_targetY_dropzone);
                    delta_deltaX = deltaX-deltaPrevX;
                    delta_deltaY = deltaY-deltaPrevY;
                    estVelX = m_linAcc.x*trim;
                    estVelY = m_linAcc.y*trim;

                    if(m_centeringLimitDrop > 30)
                    {
                        kpcentered = smKPCENTERED; // KPCENTERED/4
                        kdcentered = smKDCENTERED; // KDCENTERED/2
                    }
                    else if(m_centeringLimitDrop > 40)
                    {
                        kpcentered = smKPCENTERED/2;
                        kdcentered = smKDCENTERED/2;
                    }
                    else
                    {
                        kpcentered = KPCENTERED;
                        kdcentered = KDCENTERED;
                    };

                    if(-MAX_CENTERED_LIMIT < deltaY && deltaY < MAX_CENTERED_LIMIT
                    && -MAX_CENTERED_LIMIT < deltaX && deltaX < MAX_CENTERED_LIMIT)
                    {   
                        targetVelX = deltaX * kp + kd*(delta_deltaX);
                        targetVelY = deltaY * kp + kd*(delta_deltaY);
                        targetRoll = (targetVelX - estVelX) * gainDeg;
                        targetPitch = (targetVelY - estVelY) * gainDeg;
                        // if (targetRoll > 0.15) {targetRoll = 0.15;} else if (targetRoll < -0.15){targetRoll = -0.15;};
                        // if (targetPitch > 0.15) {targetPitch = 0.15;} else if (targetPitch < -0.15){targetPitch = -0.15;};
                        rateX = (targetRoll - m_orient.x)*5;
                        rateY = (targetPitch - m_orient.y)*5;
                        thrust = 0.50;
                    }
                    else
                    {   
                        targetVelX = deltaX * kp + kd*(delta_deltaX);
                        targetVelY = deltaY * kp + kd*(delta_deltaY);
                        targetRoll = (targetVelX - estVelX) * gainDeg;
                        targetPitch = (targetVelY - estVelY) * gainDeg;
                        // if (targetRoll > 0.15) {targetRoll = 0.15;} else if (targetRoll < -0.15){targetRoll = -0.15;};
                        // if (targetPitch > 0.15) {targetPitch = 0.15;} else if (targetPitch < -0.15){targetPitch = -0.15;};
                        rateX = (targetRoll - m_orient.x)*5;
                        rateY = (targetPitch - m_orient.y)*5;
                        thrust = 0.50;
                    };

                    deltaPrevX = deltaX;
                    deltaPrevY = deltaY;
                    
                    ROS_INFO("\nEstimated X Vel: %f\nEstimated Y Vel: %f\n------------------", estVelX, estVelY);
                    ROS_INFO("\nTarget X Vel: %f\nTarget Y Vel: %f\n------------------", targetVelX, targetVelY);
                    attitudePublish(thrust, rateX, rateY, rateZ);
                    
                    m_rate.sleep();
                }
                break;
            }
            else
            {
                thrust = 0.5;
            }
            prev_time = ros::Time::now().toSec();
        }
        else
        {
            double current_time = ros::Time::now().toSec();
            double delta_time = current_time - prev_time;
            rateX = rateX / 4.02;
            rateY = rateY / 4.02;
            (thrust <= 0.720) ? thrust = thrust + 0.05 : thrust = 0.720;

            
            if (delta_time > CENTERING_DROP_TIME_LIMIT)
            {
                prev_time_afdrop = ros::Time::now().toSec();
                double current_time_afdrop = prev_time_afdrop;
                double delta_time_afdrop = current_time_afdrop - prev_time_afdrop;
                ROS_WARN("DELAY AFTER FORCED DROP, START....");
                while(ros::ok() && (delta_time_afdrop < DELAY_AFTER_DROP))
                {
                    double current_time_afdrop = ros::Time::now().toSec();
                    delta_time_afdrop = current_time_afdrop-prev_time_afdrop;
                    rateX = 0.0;
                    rateY = 0.0;
                    thrust = 0.5;
                    attitudePublish(thrust, rateX, rateY, rateZ);
                    m_rate.sleep();
                }
                break;
            }
        }
        ROS_INFO("\nEstimated X Vel: %f\nEstimated Y Vel: %f\n------------------", estVelX, estVelY);
        ROS_INFO("\nTarget X Vel: %f\nTarget Y Vel: %f\n------------------", targetVelX, targetVelY);
        attitudePublish(thrust, rateX, rateY, rateZ);
        
        m_rate.sleep();
    }
}

void Controller::DroneController::roll()
{
    ROS_INFO("Roll Test!");
    double thrust{}, rateX{}, rateY{}, rateZ{};

    thrust = 0.5;

    while(ros::ok() && m_orient.x < 0.1062)
    {
        ROS_INFO_ONCE("5 degree roll");
        rateX = (0.1072 - m_orient.x); //roll 10 degree to the right
        attitudePublish(thrust, rateX, rateY, rateZ);
        ROS_INFO("%f", m_orient.x);
        ROS_INFO("\nx accel: %f\n\ny accel: %f", m_linAcc.x, m_linAcc.y);
        m_rateFaster.sleep();
    }

    while(ros::ok() && m_orient.x >= 0.0)
    {
        ROS_INFO_ONCE("0 degree roll");
        rateX = (-m_orient.x);
        attitudePublish(thrust, rateX, rateY, rateZ);
        ROS_INFO("%f", m_orient.x);
        ROS_INFO("\nx accel: %f\n\ny accel: %f", m_linAcc.x, m_linAcc.y);
        m_rateFaster.sleep();
    }

    while(ros::ok() && m_orient.x > -0.1062)
    {
        ROS_INFO_ONCE("-5 degree roll");
        rateX = (-0.1072 - m_orient.x); //roll 10 degree to the right
        attitudePublish(thrust, rateX, rateY, rateZ);
        ROS_INFO("%f", m_orient.x);
        ROS_INFO("\nx accel: %f\n\ny accel: %f", m_linAcc.x, m_linAcc.y);
        m_rateFaster.sleep();
    }

    while(ros::ok() && m_orient.x >= 0.0)
    {
        ROS_INFO_ONCE("0 degree roll");
        rateX = (-m_orient.x);
        attitudePublish(thrust, rateX, rateY, rateZ);
        ROS_INFO("%f", m_orient.x);
        ROS_INFO("\nx accel: %f\n\ny accel: %f", m_linAcc.x, m_linAcc.y);
        m_rateFaster.sleep();
    }

    while(ros::ok() && m_orient.y < 0.0872)
    {
        ROS_INFO_ONCE("5 degree pitch");
        rateY = (0.0872 - m_orient.y); //roll 10 degree to the right
        attitudePublish(thrust, rateX, rateY, rateZ);
        ROS_INFO("%f", m_orient.y);
        ROS_INFO("\nx accel: %f\n\ny accel: %f", m_linAcc.x, m_linAcc.y);
        m_rateFaster.sleep();
    }

    while(ros::ok() && m_orient.y > 0.0)
    {
        ROS_INFO_ONCE("0 degree pitch");
        rateY = (-m_orient.y); //roll 10 degree to the right
        attitudePublish(thrust, rateX, rateY, rateZ);
        ROS_INFO("%f", m_orient.y);
        ROS_INFO("\nx accel: %f\n\ny accel: %f", m_linAcc.x, m_linAcc.y);
        m_rateFaster.sleep();
    }
}

/*--------------------Mission Lists----------------------*/
void Controller::DroneController::missionAlpha()
{
    ROS_INFO("STARTING MISSION ALPHA");
    setNullWP();
    setGuided();
    setArm();
    setTakeoff();
    m_isDropping.data = 1;
    p_isDropping.publish<std_msgs::Bool>(m_isDropping);
    m_isLanding.data = 0;
    p_isLanding.publish<std_msgs::Bool>(m_isLanding);

    setAuto();
    while(ros::ok())
    {
        if(m_wpReached == WP1)
        {
            ROS_INFO_ONCE("REACHED WP %d", m_wpReached);
            break;
        }
        else{ROS_INFO_ONCE("GOING TO WP %d", (m_wpReached+1));}
        
        m_rateFaster.sleep();
    }
    setGuided();
    centeringAndDrop();

    setAuto();
    while(ros::ok())
    {
        if(m_wpReached == WP2)
        {
            ROS_INFO_ONCE("REACHED WP %d", m_wpReached);
            break;
        }
        else{ROS_INFO_ONCE("GOING TO WP %d", (m_wpReached+1));}
        
        m_rateFaster.sleep();
    }
    setGuided();
    centeringAndDrop();

    setAuto();
    while(ros::ok())
    {
        if(m_wpReached == WP3)
        {
            ROS_INFO_ONCE("REACHED WP %d", m_wpReached);
            break;
        }
        else{ROS_INFO_ONCE("GOING TO WP %d", (m_wpReached+1));}
        
        m_rateFaster.sleep();
    }
    setGuided();
    centeringAndDrop();

    setAuto();
    m_isDropping.data = 0;
    p_isDropping.publish<std_msgs::Bool>(m_isDropping);
    m_isLanding.data = 1;
    p_isLanding.publish<std_msgs::Bool>(m_isLanding);
    while(ros::ok())
    {
        if(m_wpReached == WP4)
        {
            ROS_INFO_ONCE("REACHED WP %d", m_wpReached);
            break;
        }
        else{ROS_INFO_ONCE("GOING TO WP %d", (m_wpReached+1));}
        
        m_rateFaster.sleep();
    }
    setGuided();
    centeringAndLand();
}

void Controller::DroneController::missionBeta()
{
    ROS_INFO("STARTING MISSION BETA");
    setNullWP();
    setGuided();
    setArm();
    setTakeoff();
    m_isDropping.data = 1;
    p_isDropping.publish<std_msgs::Bool>(m_isDropping);
    m_isLanding.data = 0;
    p_isLanding.publish<std_msgs::Bool>(m_isLanding);

    setAuto();
    while(ros::ok())
    {
        if(m_wpReached == WP1)
        {
            ROS_INFO_ONCE("REACHED WP %d", m_wpReached);
            break;
        }
        else{ROS_INFO_ONCE("GOING TO WP %d", (m_wpReached+1));}
        
        m_rateFaster.sleep();
    }
    setGuidedNOGPS();
    beta_centeringAndDrop();

    setAuto();
    while(ros::ok())
    {
        if(m_wpReached == WP2)
        {
            ROS_INFO_ONCE("REACHED WP %d", m_wpReached);
            break;
        }
        else{ROS_INFO_ONCE("GOING TO WP %d", (m_wpReached+1));}
        
        m_rateFaster.sleep();
    }
    setGuidedNOGPS();
    beta_centeringAndDrop();

    setAuto();
    while(ros::ok())
    {
        if(m_wpReached == WP3)
        {
            ROS_INFO_ONCE("REACHED WP %d", m_wpReached);
            break;
        }
        else{ROS_INFO_ONCE("GOING TO WP %d", (m_wpReached+1));}
        
        m_rateFaster.sleep();
    }
    setGuidedNOGPS();
    beta_centeringAndDrop();
    
    setRTL();
}

void Controller::DroneController::missionGamma()
{
    ROS_INFO("STARTING MISSION BETA");
    setNullWP();
    setGuided();
    setArm();
    setTakeoff();
    m_isDropping.data = 1;
    p_isDropping.publish<std_msgs::Bool>(m_isDropping);
    m_isLanding.data = 0;
    p_isLanding.publish<std_msgs::Bool>(m_isLanding);

    setAuto();
    while(ros::ok())
    {
        if(m_wpReached == WP1)
        {
            ROS_INFO_ONCE("REACHED WP %d", m_wpReached);
            break;
        }
        else{ROS_INFO_ONCE("GOING TO WP %d", (m_wpReached+1));}
        
        m_rateFaster.sleep();
    }
    setGuidedNOGPS();
    roll();
    setRTL();
}
/*-------------------------------------------------------*/


/*---------------Constructor & Destructor---------------*/
Controller::DroneController::DroneController(ros::NodeHandle& nodehandle)
    :nh_(nodehandle), m_rate(RATE), m_rateFaster(RATE_FASTER)
{
    ROS_INFO("Instantiating Controller");
    static ros::AsyncSpinner spinner(0);
    spinner.start();

    initSub();
    
    while(ros::ok && !m_fcuState.connected){
        ROS_INFO("CONNECTING TO FCU.......");
        m_rateFaster.sleep();
    }

    initPub();
    initSrv();
    ROS_INFO("Controller instantiated!");
}

Controller::DroneController::~DroneController()
{
    ros::shutdown();
    ROS_INFO("Shutting down controller...");
}

