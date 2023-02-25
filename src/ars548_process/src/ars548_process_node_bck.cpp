#include <ros/ros.h>
#include <ars548_msg/objects.h>
#include <ars548_msg/ObjectList.h>
#include <ars548_msg/detections.h>
#include <ars548_msg/DetectionList.h>
#include <ars548_msg/RadarBasicStatus.h>
#include <ars548_msg/AccelerationLateralCog.h>
#include <ars548_msg/AccelerationLongitudinalCog.h>
#include <ars548_msg/CharacteristicSpeed.h>
#include <ars548_msg/DrivingDirection.h>
#include <ars548_msg/SteeringAngleFrontAxle.h>
#include <ars548_msg/VelocityVehicle.h>
#include <ars548_msg/YawRate.h>

#include <thread>

#include "udp_interface.h"
#include "data_struct.h"
#include "data_process.h"
#include "converttype.h"

UdpInterface udp_io;
DataProcess data_process;
ConvertType cvt;

char rec_data[40000];
char send_data[1024];
int rec_length;

RadarObjectList object_list;
RadarDetectionList detection_list;
RadarBasicStatus basic_status;


ros::Publisher objects_pub;
ros::Publisher detections_pub;
ros::Publisher status_pub;

void publishObjectList(RadarObjectList list)
{
    ars548_msg::objects obj;
    ars548_msg::ObjectList obj_list;
    int i;

    obj_list.object_array.clear();

    obj_list.serviceID = list.serviceID;
    obj_list.MethodID = list.MethodID;
    obj_list.data_length = list.data_length;
    obj_list.clientID = list.clientID;
    obj_list.sessionID = list.sessionID;
    obj_list.protocol_version = list.protocol_version;
    obj_list.interface_version = list.interface_version;
    obj_list.message_type = list.message_type;
    obj_list.return_code = list.return_code;
    obj_list.CRC = list.CRC;
    obj_list.Length = list.Length;
    obj_list.SQC = list.SQC;
    obj_list.DataID = list.DataID;
    obj_list.Timestamp_Nanoseconds = list.Timestamp_Nanoseconds;
    obj_list.Timestamp_Seconds = list.Timestamp_Seconds;
    obj_list.Timestamp_SyncStatus = list.Timestamp_SyncStatus;
    obj_list.EventDataQualifier = list.EventDataQualifier;
    obj_list.ExtendedQualifier = list.ExtendedQualifier;
    obj_list.ObjectList_NumOfObjects = list.ObjectList_NumOfObjects;

    for(i=0;i<list.ObjectList_NumOfObjects;i++)
    {
        obj.header.frame_id = "/world";
        obj.header.stamp = ros::Time::now();

        obj.u_StatusSensor = list.object_array[i].u_StatusSensor;
        obj.u_ID = list.object_array[i].u_ID;
        obj.u_Age = list.object_array[i].u_Age;
        obj.u_StatusMeasurement = list.object_array[i].u_StatusMeasurement;
        obj.u_StatusMovement = list.object_array[i].u_StatusMovement;
        obj.u_Position_InvalidFlags = list.object_array[i].u_Position_InvalidFlags;
        obj.u_Position_Reference = list.object_array[i].u_Position_Reference;
        obj.u_Position_X = list.object_array[i].u_Position_X;
        obj.u_Position_X_STD = list.object_array[i].u_Position_X_STD;
        obj.u_Position_Y = list.object_array[i].u_Position_Y;
        obj.u_Position_Y_STD = list.object_array[i].u_Position_Y_STD;
        obj.u_Position_Z = list.object_array[i].u_Position_Z;
        obj.u_Position_Z_STD = list.object_array[i].u_Position_Z_STD;
        obj.u_Position_CovarianceXY = list.object_array[i].u_Position_CovarianceXY;
        obj.u_Position_Orientation = list.object_array[i].u_Position_Orientation;
        obj.u_Position_Orientation_STD = list.object_array[i].u_Position_Orientation_STD;
        obj.u_Existence_InvalidFlags = list.object_array[i].u_Existence_InvalidFlags;
        obj.u_Existence_Probability = list.object_array[i].u_Existence_Probability;
        obj.u_Existence_PPV = list.object_array[i].u_Existence_PPV;
        obj.u_Classification_Car = list.object_array[i].u_Classification_Car;
        obj.u_Classification_Truck = list.object_array[i].u_Classification_Truck;
        obj.u_Classification_Motorcycle = list.object_array[i].u_Classification_Motorcycle;
        obj.u_Classification_Bicycle = list.object_array[i].u_Classification_Bicycle;
        obj.u_Classification_Pedestrian = list.object_array[i].u_Classification_Pedestrian;
        obj.u_Classification_Animal = list.object_array[i].u_Classification_Animal;
        obj.u_Classification_Hazard = list.object_array[i].u_Classification_Hazard;
        obj.u_Classification_Unknown = list.object_array[i].u_Classification_Unknown;
        obj.u_Classification_Overdrivable = list.object_array[i].u_Classification_Overdrivable;
        obj.u_Classification_Underdrivable = list.object_array[i].u_Classification_Underdrivable;
        obj.u_Dynamics_AbsVel_InvalidFlags = list.object_array[i].u_Dynamics_AbsVel_InvalidFlags;
        obj.f_Dynamics_AbsVel_X = list.object_array[i].f_Dynamics_AbsVel_X;
        obj.f_Dynamics_AbsVel_X_STD = list.object_array[i].f_Dynamics_AbsVel_X_STD;
        obj.f_Dynamics_AbsVel_Y = list.object_array[i].f_Dynamics_AbsVel_Y;
        obj.f_Dynamics_AbsVel_Y_STD = list.object_array[i].f_Dynamics_AbsVel_Y_STD;
        obj.f_Dynamics_AbsVel_CovarianceXY = list.object_array[i].f_Dynamics_AbsVel_CovarianceXY;
        obj.u_Dynamics_RelVel_InvalidFlags = list.object_array[i].u_Dynamics_RelVel_InvalidFlags;
        obj.f_Dynamics_RelVel_X = list.object_array[i].f_Dynamics_RelVel_X;
        obj.f_Dynamics_RelVel_X_STD = list.object_array[i].f_Dynamics_RelVel_X_STD;
        obj.f_Dynamics_RelVel_Y = list.object_array[i].f_Dynamics_RelVel_Y;
        obj.f_Dynamics_RelVel_Y_STD = list.object_array[i].f_Dynamics_RelVel_Y_STD;
        obj.f_Dynamics_RelVel_CovarianceXY = list.object_array[i].f_Dynamics_RelVel_CovarianceXY;
        obj.u_Dynamics_AbsAccel_InvalidFlags = list.object_array[i].u_Dynamics_AbsAccel_InvalidFlags;
        obj.f_Dynamics_AbsAccel_X = list.object_array[i].f_Dynamics_AbsAccel_X;
        obj.f_Dynamics_AbsAccel_X_STD = list.object_array[i].f_Dynamics_AbsAccel_X_STD;
        obj.f_Dynamics_AbsAccel_Y = list.object_array[i].f_Dynamics_AbsAccel_Y;
        obj.f_Dynamics_AbsAccel_Y_STD = list.object_array[i].f_Dynamics_AbsAccel_Y_STD;
        obj.f_Dynamics_AbsAccel_CovarianceXY = list.object_array[i].f_Dynamics_AbsAccel_CovarianceXY;
        obj.u_Dynamics_RelAccel_InvalidFlags = list.object_array[i].u_Dynamics_RelAccel_InvalidFlags;
        obj.f_Dynamics_RelAccel_X = list.object_array[i].f_Dynamics_RelAccel_X;
        obj.f_Dynamics_RelAccel_X_STD = list.object_array[i].f_Dynamics_RelAccel_X_STD;
        obj.f_Dynamics_RelAccel_Y = list.object_array[i].f_Dynamics_RelAccel_Y;
        obj.f_Dynamics_RelAccel_Y_STD = list.object_array[i].f_Dynamics_RelAccel_Y_STD;
        obj.f_Dynamics_RelAccel_CovarianceXY = list.object_array[i].f_Dynamics_RelAccel_CovarianceXY;
        obj.u_Dynamics_Orientation_InvalidFlags = list.object_array[i].u_Dynamics_Orientation_InvalidFlags;
        obj.u_Dynamics_Orientation_Rate_Mean = list.object_array[i].u_Dynamics_Orientation_Rate_Mean;
        obj.u_Dynamics_Orientation_Rate_STD = list.object_array[i].u_Dynamics_Orientation_Rate_STD;
        obj.u_Shape_Length_Status = list.object_array[i].u_Shape_Length_Status;
        obj.u_Shape_Length_Edge_InvalidFlags = list.object_array[i].u_Shape_Length_Edge_InvalidFlags;
        obj.u_Shape_Length_Edge_Mean = list.object_array[i].u_Shape_Length_Edge_Mean;
        obj.u_Shape_Length_Edge_STD = list.object_array[i].u_Shape_Length_Edge_STD;
        obj.u_Shape_Width_Status = list.object_array[i].u_Shape_Width_Status;
        obj.u_Shape_Width_Edge_InvalidFlags = list.object_array[i].u_Shape_Width_Edge_InvalidFlags;
        obj.u_Shape_Width_Edge_Mean = list.object_array[i].u_Shape_Width_Edge_Mean;
        obj.u_Shape_Width_Edge_STD = list.object_array[i].u_Shape_Width_Edge_STD;
       
        obj_list.object_array.push_back(obj);
    }

    objects_pub.publish(obj_list);

}

void publishDetectionList(RadarDetectionList list)
{
    ars548_msg::detections det;
    ars548_msg::DetectionList det_list;
    int i;
    det_list.detection_array.clear();

    det_list.serviceID = list.serviceID;
    det_list.MethodID = list.MethodID;
    det_list.data_length = list.data_length;
    det_list.clientID = list.clientID;
    det_list.sessionID = list.sessionID;
    det_list.protocol_version = list.protocol_version;
    det_list.interface_version = list.interface_version;
    det_list.message_type = list.message_type;
    det_list.return_code = list.return_code;
    det_list.CRC = list.CRC;
    det_list.Length = list.Length;
    det_list.SQC = list.SQC;
    det_list.DataID = list.DataID;
    det_list.Timestamp_Nanoseconds = list.Timestamp_Nanoseconds;
    det_list.Timestamp_Seconds = list.Timestamp_Seconds;
    det_list.Timestamp_SyncStatus = list.Timestamp_SyncStatus;
    det_list.EventDataQualifier = list.EventDataQualifier;
    det_list.ExtendedQualifier = list.ExtendedQualifier;
    det_list.Origin_InvalidFlags = list.Origin_InvalidFlags;
    det_list.Origin_Xpos = list.Origin_Xpos;
    det_list.Origin_Xstd = list.Origin_Xstd;
    det_list.Origin_Ypos = list.Origin_Ypos;
    det_list.Origin_Ystd = list.Origin_Ystd;
    det_list.Origin_Zpos = list.Origin_Zpos;
    det_list.Origin_Zstd = list.Origin_Zstd;
    det_list.Origin_Roll = list.Origin_Roll;
    det_list.Origin_Rollstd = list.Origin_Rollstd;
    det_list.Origin_Pitch = list.Origin_Pitch;
    det_list.Origin_Pitchstd = list.Origin_Pitchstd;
    det_list.Origin_Yaw = list.Origin_Yaw;
    det_list.Origin_Yawstd = list.Origin_Yawstd;
    det_list.List_InvalidFlags = list.List_InvalidFlags;
    det_list.List_RadVelDomain_Min = list.List_RadVelDomain_Min;
    det_list.List_RadVelDomain_Max = list.List_RadVelDomain_Max;
    det_list.List_NumOfDetections = list.List_NumOfDetections;
    det_list.Aln_AzimuthCorrection = list.Aln_AzimuthCorrection;
    det_list.Aln_ElevationCorrection = list.Aln_ElevationCorrection;
    det_list.Aln_Status = list.Aln_Status;

    for(i=0;i<list.List_NumOfDetections;i++)
    {
        det.header.frame_id = "/world";
        det.header.stamp = ros::Time::now();

        det.f_x = list.detection_array[i].f_x;
        det.f_y = list.detection_array[i].f_y;
        det.f_z = list.detection_array[i].f_z;
        det.u_InvalidFlags = list.detection_array[i].u_InvalidFlags;
        det.f_RangeRate = list.detection_array[i].f_RangeRate;
        det.f_RangeRateSTD = list.detection_array[i].f_RangeRateSTD;
        det.s_RCS = list.detection_array[i].s_RCS;
        det.u_MeasurementID = list.detection_array[i].u_MeasurementID;
        det.u_PositivePredictiveValue = list.detection_array[i].u_PositivePredictiveValue;
        det.u_Classification = list.detection_array[i].u_Classification;
        det.u_MultiTargetProbability = list.detection_array[i].u_MultiTargetProbability;
        det.u_ObjectID = list.detection_array[i].u_ObjectID;
        det.u_AmbiguityFlag = list.detection_array[i].u_AmbiguityFlag;
        det.u_SortIndex = list.detection_array[i].u_SortIndex;

        det_list.detection_array.push_back(det);
    }

    detections_pub.publish(det_list);    
}

void publishBasicStatus(RadarBasicStatus status)
{
    ars548_msg::RadarBasicStatus sts;

    sts.header.frame_id = "/world";
    sts.header.stamp = ros::Time::now();

    sts.Longitudinal = status.Longitudinal;
    sts.Lateral = status.Lateral;
    sts.Vertical = status.Vertical;
    sts.Yaw = status.Yaw;
    sts.Pitch = status.Pitch;
    sts.PlugOrientation = status.PlugOrientation;
    sts.Length = status.Length;
    sts.Width = status.Width;
    sts.Height = status.Height;
    sts.Wheelbase = status.Wheelbase;
    sts.MaximumDistance = status.MaximumDistance;
    sts.FrequencySlot = status.FrequencySlot;
    sts.CycleTime = status.CycleTime;
    sts.TimeSlot = status.TimeSlot;
    sts.HCC = status.HCC;
    sts.Powersave_Standstill = status.Powersave_Standstill;
    sts.SensorIPAddress_0 = status.SensorIPAddress_0;
    sts.SensorIPAddress_1 = status.SensorIPAddress_1;
    sts.Configuration_counter = status.Configuration_counter;
    sts.Status_LongitudinalVelocity = status.Status_LongitudinalVelocity;
    sts.Status_LongitudinalAcceleration = status.Status_LongitudinalAcceleration;
    sts.Status_LateralAcceleration = status.Status_LateralAcceleration;
    sts.Status_YawRate = status.Status_YawRate;
    sts.Status_SteeringAngle = status.Status_SteeringAngle;
    sts.Status_DrivingDirection = status.Status_DrivingDirection;
    sts.Status_CharacteristicSpeed = status.Status_CharacteristicSpeed;
    sts.Status_Radar = status.Status_Radar;

    status_pub.publish(sts);   
}

void ProcessRadarData(char *data, int len)
{
    switch(len)
    {
        case 9401:
            if(data_process.processObjectListMessage(data,&object_list))
            {
                if(object_list.ObjectList_NumOfObjects>0)
                    publishObjectList(object_list);    
            }
        break;

        case 35336:
            if(data_process.processDetectionListMessage(data,&detection_list))
            {
                if(detection_list.List_NumOfDetections)
                    publishDetectionList(detection_list);
            }
        break;

        case 69:
            if(data_process.processBasicStatusMessage(data,&basic_status))
            {
                publishBasicStatus(basic_status);
            }
        break;

        default:
        break;
    }
}

void AccLateralCogCallBack(const ars548_msg::AccelerationLateralCog& msg)
{   
    char temp[4];
    //service ID
    send_data[0] = 0;
    send_data[1] = 0;
    //Method ID
    send_data[2] = 0x01;
    send_data[3] = 0x41;
    //Length
    send_data[4] = 0;
    send_data[5] = 0;
    send_data[6] = 0;
    send_data[7] = 32;

    cvt.floatToByte(msg.AccelerationLateralErrAmp,temp);
    send_data[8] = temp[0];
    send_data[9] = temp[1];
    send_data[10] = temp[2];
    send_data[11] = temp[3];    

    send_data[12] = (char)msg.AccelerationLateralErrAmp_InvalidFlag;    
    send_data[13] = (char)msg.QualifierAccelerationLateral;    
   
    cvt.floatToByte(msg.AccelerationLateral,temp);
    send_data[14] = temp[0];
    send_data[15] = temp[1];
    send_data[16] = temp[2];
    send_data[17] = temp[3]; 
    
    send_data[18] = (char)msg.AccelerationLateral_InvalidFlag;    
    send_data[19] = (char)msg.AccelerationLateralEventDataQualifier;    

    for(int i=0;i<20;i++)
        send_data[20+i] = 0;

    udp_io.sendToRadar(send_data,40);
}

void AccLongitudinalCogCallBack(const ars548_msg::AccelerationLongitudinalCog& msg)
{ 
    char temp[4];
    //service ID
    send_data[0] = 0;
    send_data[1] = 0;
    //Method ID
    send_data[2] = 0x01;
    send_data[3] = 0x42;
    //Length
    send_data[4] = 0;
    send_data[5] = 0;
    send_data[6] = 0;
    send_data[7] = 32;

    cvt.floatToByte(msg.AccelerationLongitudinalErrAmp,temp);
    send_data[8] = temp[0];
    send_data[9] = temp[1];
    send_data[10] = temp[2];
    send_data[11] = temp[3];    

    send_data[12] = (char)msg.AccelerationLongitudinalErrAmp_InvalidFlag;    
    send_data[13] = (char)msg.QualifierAccelerationLongitudinal;    
   
    cvt.floatToByte(msg.AccelerationLongitudinal,temp);
    send_data[14] = temp[0];
    send_data[15] = temp[1];
    send_data[16] = temp[2];
    send_data[17] = temp[3]; 
    
    send_data[18] = (char)msg.AccelerationLongitudinal_InvalidFlag;    
    send_data[19] = (char)msg.AccelerationLongitudinalEventDataQualifier;    

    for(int i=0;i<20;i++)
        send_data[20+i] = 0;
    

    udp_io.sendToRadar(send_data,40);
}

void CharacteristicSpeedCallBack(const ars548_msg::CharacteristicSpeed& msg)
{ 
    //service ID
    send_data[0] = 0;
    send_data[1] = 0;
    //Method ID
    send_data[2] = 0x01;
    send_data[3] = 0x48;
    //Length
    send_data[4] = 0;
    send_data[5] = 0;
    send_data[6] = 0;
    send_data[7] = 11;

    send_data[8] = (char)msg.CharacteristicSpeedErrAmp;    
    send_data[9] = (char)msg.QualifierCharacteristicSpeed;    
    send_data[10] = (char)msg.CharacteristicSpeed;    

    for(int i=0;i<8;i++)
        send_data[11+i] = 0;

    udp_io.sendToRadar(send_data,19);
}

void DrivingDirectionCallBack(const ars548_msg::DrivingDirection& msg)
{ 
    //service ID
    send_data[0] = 0;
    send_data[1] = 0;
    //Method ID
    send_data[2] = 0x01;
    send_data[3] = 0x45;
    //Length
    send_data[4] = 0;
    send_data[5] = 0;
    send_data[6] = 0;
    send_data[7] = 22;

    send_data[8] = (char)msg.DrivingDirectionUnconfirmed;    
    send_data[9] = (char)msg.DrivingDirectionConfirmed;      

    for(int i=0;i<20;i++)
        send_data[10+i] = 0;

    udp_io.sendToRadar(send_data,30);
}

void SteeringAngleCallBack(const ars548_msg::SteeringAngleFrontAxle& msg)
{ 
    char temp[4];
    //service ID
    send_data[0] = 0;
    send_data[1] = 0;
    //Method ID
    send_data[2] = 0x01;
    send_data[3] = 0x47;
    //Length
    send_data[4] = 0;
    send_data[5] = 0;
    send_data[6] = 0;
    send_data[7] = 32;

    send_data[8] = (char)msg.QualifierSteeringAngleFrontAxle;  

    cvt.floatToByte(msg.SteeringAngleFrontAxleErrAmp,temp);
    send_data[9] = temp[0];
    send_data[10] = temp[1];
    send_data[11] = temp[2];
    send_data[12] = temp[3];    

    send_data[13] = (char)msg.SteeringAngleFrontAxleErrAmp_InvalidFlag;    
   
    cvt.floatToByte(msg.SteeringAngleFrontAxle,temp);
    send_data[14] = temp[0];
    send_data[15] = temp[1];
    send_data[16] = temp[2];
    send_data[17] = temp[3]; 
    
    send_data[18] = (char)msg.SteeringAngleFrontAxle_InvalidFlag;    
    send_data[19] = (char)msg.SteeringAngleFrontAxleEventDataQualifier;    

    for(int i=0;i<20;i++)
        send_data[20+i] = 0;
    
    udp_io.sendToRadar(send_data,40);
}

void VelocityVehicleCallBack(const ars548_msg::VelocityVehicle& msg)
{ 
    char temp[4];
    //service ID
    send_data[0] = 0;
    send_data[1] = 0;
    //Method ID
    send_data[2] = 0x01;
    send_data[3] = 0x43;
    //Length
    send_data[4] = 0;
    send_data[5] = 0;
    send_data[6] = 0;
    send_data[7] = 28;

    send_data[8] = (char)msg.StatusVelocityNearStandstill;  
    send_data[9] = (char)msg.QualifierVelocityVehicle;  
    send_data[10] = (char)msg.VelocityVehicleEventDataQualifier;  

    cvt.floatToByte(msg.VelocityVehicle,temp);
    send_data[11] = temp[0];
    send_data[12] = temp[1];
    send_data[13] = temp[2];
    send_data[14] = temp[3];    

    send_data[15] = (char)msg.VelocityVehicle_InvalidFlag;     

    for(int i=0;i<20;i++)
        send_data[16+i] = 0;
    
    udp_io.sendToRadar(send_data,36);
}

void YawRateCallBack(const ars548_msg::YawRate& msg)
{ 
    char temp[4];
    //service ID
    send_data[0] = 0;
    send_data[1] = 0;
    //Method ID
    send_data[2] = 0x01;
    send_data[3] = 0x46;
    //Length
    send_data[4] = 0;
    send_data[5] = 0;
    send_data[6] = 0;
    send_data[7] = 32;

    cvt.floatToByte(msg.YawRateErrAmp,temp);
    send_data[8] = temp[0];
    send_data[9] = temp[1];
    send_data[10] = temp[2];
    send_data[11] = temp[3];    

    send_data[12] = (char)msg.YawRateErrAmp_InvalidFlag;    
    send_data[13] = (char)msg.QualifierYawRate;    
   
    cvt.floatToByte(msg.YawRate,temp);
    send_data[14] = temp[0];
    send_data[15] = temp[1];
    send_data[16] = temp[2];
    send_data[17] = temp[3]; 
    
    send_data[18] = (char)msg.YawRate_InvalidFlag;    
    send_data[19] = (char)msg.YawRateEventDataQualifier;   



    for(int i=0;i<20;i++)
        send_data[20+i] = 0;
    
    udp_io.sendToRadar(send_data,40);
}

//thread for receive radar data
void receiveThread()
{
    while(ros::ok())
    {
        udp_io.receiveFromRadar(rec_data,rec_length);
        if(rec_length>0)
        {
            ProcessRadarData(rec_data,rec_length);    
        }
    }

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ars548_process_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    objects_pub = nh.advertise<ars548_msg::ObjectList>("/ars548_process/object_list", 10);
    detections_pub = nh.advertise<ars548_msg::DetectionList>("/ars548_process/detection_list", 10);
    status_pub = nh.advertise<ars548_msg::RadarBasicStatus>("/ars548_process/radar_status", 10);

    ros::Subscriber acc_lateral_sub = nh.subscribe("/ars548_process/acc_lateral_cog", 10, &AccLateralCogCallBack);
    ros::Subscriber acc_longitudinal_sub = nh.subscribe("/ars548_process/acc_longitudinal_cog", 10, &AccLongitudinalCogCallBack);
    ros::Subscriber characteristic_speed_sub = nh.subscribe("/ars548_process/characteristic_speed", 10, &CharacteristicSpeedCallBack);
    ros::Subscriber driving_direction_sub = nh.subscribe("/ars548_process/driving_direction", 10, &DrivingDirectionCallBack);
    ros::Subscriber steering_angle_sub = nh.subscribe("/ars548_process/steering_angle", 10, &SteeringAngleCallBack);
    ros::Subscriber velocity_vehicle_sub = nh.subscribe("/ars548_process/velocity_vehicle", 10, &VelocityVehicleCallBack);
    ros::Subscriber yaw_rate_sub = nh.subscribe("/ars548_process/yaw_rate", 10, &YawRateCallBack);

    udp_io.initUdpMulticastServer("224.0.2.2",42102);

    udp_io.initUdpUnicastClient("10.13.1.113",42101,42401);

    std::thread rec_thread = std::thread(receiveThread);  

    ros::spin();


    return 0;
}


