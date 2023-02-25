#include <ros/ros.h>
#include <ars548_msg/AccelerationLateralCog.h>
#include <ars548_msg/AccelerationLongitudinalCog.h>
#include <ars548_msg/CharacteristicSpeed.h>
#include <ars548_msg/DrivingDirection.h>
#include <ars548_msg/SteeringAngleFrontAxle.h>
#include <ars548_msg/VelocityVehicle.h>
#include <ars548_msg/YawRate.h>


ros::Publisher acc_lateral_cog_pub;
ros::Publisher acc_longitudinal_cog_pub;
ros::Publisher characteristic_speed_pub;
ros::Publisher driving_direction_pub;
ros::Publisher steering_angle_pub;
ros::Publisher velocity_vehicle_pub;
ros::Publisher yaw_rate_pub;

void publishAccLateralCog()
{
    ars548_msg::AccelerationLateralCog msg;

    msg.header.frame_id = "/world";
    msg.header.stamp = ros::Time::now();

    msg.AccelerationLateralErrAmp = 0;
    msg.AccelerationLateralErrAmp_InvalidFlag = 0;
    msg.QualifierAccelerationLateral = 0;
    msg.AccelerationLateral = 0;
    msg.AccelerationLateral_InvalidFlag = 0;
    msg.AccelerationLateralEventDataQualifier = 0;

    acc_lateral_cog_pub.publish(msg);
}

void publishAccLongitudinalCog()
{
    ars548_msg::AccelerationLongitudinalCog msg;

    msg.header.frame_id = "/world";
    msg.header.stamp = ros::Time::now();

    msg.AccelerationLongitudinalErrAmp = 0;
    msg.AccelerationLongitudinalErrAmp_InvalidFlag = 0;
    msg.QualifierAccelerationLongitudinal = 0;
    msg.AccelerationLongitudinal = 0;
    msg.AccelerationLongitudinal_InvalidFlag = 0;
    msg.AccelerationLongitudinalEventDataQualifier = 0;

    acc_longitudinal_cog_pub.publish(msg);
}

void publishCharacteristicSpeed()
{
    ars548_msg::CharacteristicSpeed msg;

    msg.header.frame_id = "/world";
    msg.header.stamp = ros::Time::now();

    msg.CharacteristicSpeedErrAmp = 0;
    msg.QualifierCharacteristicSpeed = 0;
    msg.CharacteristicSpeed = 60;

    characteristic_speed_pub.publish(msg);
}

void publishDrivingDirection()
{
    ars548_msg::DrivingDirection msg;

    msg.header.frame_id = "/world";
    msg.header.stamp = ros::Time::now();

    msg.DrivingDirectionUnconfirmed = 0;
    msg.DrivingDirectionConfirmed = 1;

    driving_direction_pub.publish(msg);
}

void publishSteeringAngle()
{
    ars548_msg::SteeringAngleFrontAxle msg;

    msg.header.frame_id = "/world";
    msg.header.stamp = ros::Time::now();

    msg.QualifierSteeringAngleFrontAxle = 0;
    msg.SteeringAngleFrontAxleErrAmp = 0;
    msg.SteeringAngleFrontAxleErrAmp_InvalidFlag = 0;
    msg.SteeringAngleFrontAxle = 0;
    msg.SteeringAngleFrontAxle_InvalidFlag = 0;
    msg.SteeringAngleFrontAxleEventDataQualifier = 0;

    steering_angle_pub.publish(msg);
}

void publishVelocityVehicle()
{
    ars548_msg::VelocityVehicle msg;

    msg.header.frame_id = "/world";
    msg.header.stamp = ros::Time::now();

    msg.StatusVelocityNearStandstill = 0;
    msg.QualifierVelocityVehicle = 0;
    msg.VelocityVehicleEventDataQualifier = 0;
    msg.VelocityVehicle = 0;
    msg.VelocityVehicle_InvalidFlag = 0;

    velocity_vehicle_pub.publish(msg);
}

void publishYawRate()
{
    ars548_msg::YawRate msg;

    msg.header.frame_id = "/world";
    msg.header.stamp = ros::Time::now();

    msg.YawRateErrAmp = 0;
    msg.YawRateErrAmp_InvalidFlag = 0;
    msg.QualifierYawRate = 0;
    msg.YawRate = 0;
    msg.YawRate_InvalidFlag = 0;
    msg.YawRateEventDataQualifier = 0;

    yaw_rate_pub.publish(msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_radar_input_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    acc_lateral_cog_pub = nh.advertise<ars548_msg::AccelerationLateralCog>("/ars548_process/acc_lateral_cog", 10);
    acc_longitudinal_cog_pub = nh.advertise<ars548_msg::AccelerationLongitudinalCog>("/ars548_process/acc_longitudinal_cog", 10);
    characteristic_speed_pub = nh.advertise<ars548_msg::CharacteristicSpeed>("/ars548_process/characteristic_speed", 10);
    driving_direction_pub = nh.advertise<ars548_msg::DrivingDirection>("/ars548_process/driving_direction", 10);
    steering_angle_pub = nh.advertise<ars548_msg::SteeringAngleFrontAxle>("/ars548_process/steering_angle", 10);
    velocity_vehicle_pub = nh.advertise<ars548_msg::VelocityVehicle>("/ars548_process/velocity_vehicle", 10);
    yaw_rate_pub = nh.advertise<ars548_msg::YawRate>("/ars548_process/yaw_rate", 10);

    ros::Rate r(10);


    while(ros::ok())
    {
        publishAccLateralCog();
        publishAccLongitudinalCog();
        publishCharacteristicSpeed();
        publishDrivingDirection();
        publishSteeringAngle();
        publishVelocityVehicle();
        publishYawRate();

        r.sleep();
    }

    return 0;
}






