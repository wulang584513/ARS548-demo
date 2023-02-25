#ifndef DATA_STRUCT_H
#define DATA_STRUCT_H

typedef struct _RadarDetection
{
    float f_x;
    float f_y;
    float f_z;
    unsigned char u_InvalidFlags;
    float f_RangeRate;
    float f_RangeRateSTD;
    signed char s_RCS;
    unsigned short u_MeasurementID;
    unsigned char u_PositivePredictiveValue;
    unsigned char u_Classification;
    unsigned char u_MultiTargetProbability;
    unsigned short u_ObjectID;
    unsigned char u_AmbiguityFlag;
    unsigned short u_SortIndex;
}RadarDetection;

typedef struct _RadarObject
{
    unsigned short u_StatusSensor;
    unsigned int u_ID;
    unsigned short u_Age;
    unsigned char u_StatusMeasurement;
    unsigned char u_StatusMovement;
    unsigned short u_Position_InvalidFlags;
    unsigned char u_Position_Reference;
    float u_Position_X;
    float u_Position_X_STD;
    float u_Position_Y;
    float u_Position_Y_STD;
    float u_Position_Z;
    float u_Position_Z_STD;
    float u_Position_CovarianceXY;
    float u_Position_Orientation;
    float u_Position_Orientation_STD;
    unsigned char u_Existence_InvalidFlags;
    float u_Existence_Probability;
    float u_Existence_PPV;
    unsigned char u_Classification_Car;
    unsigned char u_Classification_Truck;
    unsigned char u_Classification_Motorcycle;
    unsigned char u_Classification_Bicycle;
    unsigned char u_Classification_Pedestrian;
    unsigned char u_Classification_Animal;
    unsigned char u_Classification_Hazard;
    unsigned char u_Classification_Unknown;
    unsigned char u_Classification_Overdrivable;
    unsigned char u_Classification_Underdrivable;
    unsigned char u_Dynamics_AbsVel_InvalidFlags;
    float f_Dynamics_AbsVel_X;
    float f_Dynamics_AbsVel_X_STD;
    float f_Dynamics_AbsVel_Y;
    float f_Dynamics_AbsVel_Y_STD;
    float f_Dynamics_AbsVel_CovarianceXY;
    unsigned char u_Dynamics_RelVel_InvalidFlags;
    float f_Dynamics_RelVel_X;
    float f_Dynamics_RelVel_X_STD;
    float f_Dynamics_RelVel_Y;
    float f_Dynamics_RelVel_Y_STD;
    float f_Dynamics_RelVel_CovarianceXY;
    unsigned char u_Dynamics_AbsAccel_InvalidFlags;
    float f_Dynamics_AbsAccel_X;
    float f_Dynamics_AbsAccel_X_STD;
    float f_Dynamics_AbsAccel_Y;
    float f_Dynamics_AbsAccel_Y_STD;
    float f_Dynamics_AbsAccel_CovarianceXY;
    unsigned char u_Dynamics_RelAccel_InvalidFlags;
    float f_Dynamics_RelAccel_X;
    float f_Dynamics_RelAccel_X_STD;
    float f_Dynamics_RelAccel_Y;
    float f_Dynamics_RelAccel_Y_STD;
    float f_Dynamics_RelAccel_CovarianceXY;
    unsigned char u_Dynamics_Orientation_InvalidFlags;
    float u_Dynamics_Orientation_Rate_Mean;
    float u_Dynamics_Orientation_Rate_STD;
    unsigned int u_Shape_Length_Status;
    unsigned char u_Shape_Length_Edge_InvalidFlags;
    float u_Shape_Length_Edge_Mean;
    float u_Shape_Length_Edge_STD;
    unsigned int u_Shape_Width_Status;
    unsigned char u_Shape_Width_Edge_InvalidFlags;
    float u_Shape_Width_Edge_Mean;
    float u_Shape_Width_Edge_STD;
}RadarObject;

typedef struct _RadarDetectionList
{
    unsigned short serviceID;
    unsigned short MethodID;
    unsigned int data_length;
    unsigned short clientID;
    unsigned short sessionID;
    unsigned char protocol_version;
    unsigned char interface_version;
    unsigned char message_type;
    unsigned char return_code;

    unsigned long long CRC;
    unsigned int Length;
    unsigned int SQC;
    unsigned int DataID;
    unsigned int Timestamp_Nanoseconds;
    unsigned int Timestamp_Seconds;
    unsigned char Timestamp_SyncStatus;
    unsigned int EventDataQualifier;
    unsigned char ExtendedQualifier;
    unsigned short Origin_InvalidFlags;
    float Origin_Xpos;
    float Origin_Xstd;
    float Origin_Ypos;
    float Origin_Ystd;
    float Origin_Zpos;
    float Origin_Zstd;
    float Origin_Roll;
    float Origin_Rollstd;
    float Origin_Pitch;
    float Origin_Pitchstd;
    float Origin_Yaw;
    float Origin_Yawstd;
    unsigned char List_InvalidFlags;
    RadarDetection detection_array[800];
    float List_RadVelDomain_Min;
    float List_RadVelDomain_Max;
    unsigned int List_NumOfDetections;
    float Aln_AzimuthCorrection;
    float Aln_ElevationCorrection;
    unsigned char Aln_Status;
}RadarDetectionList;

typedef struct _RadarObjectList
{
    unsigned short serviceID;
    unsigned short MethodID;
    unsigned int data_length;
    unsigned short clientID;
    unsigned short sessionID;
    unsigned char protocol_version;
    unsigned char interface_version;
    unsigned char message_type;
    unsigned char return_code;

    unsigned long long CRC;
    unsigned int Length;
    unsigned int SQC;
    unsigned int DataID;
    unsigned int Timestamp_Nanoseconds;
    unsigned int Timestamp_Seconds;
    unsigned char Timestamp_SyncStatus;
    unsigned int EventDataQualifier;
    unsigned char ExtendedQualifier;
    unsigned char ObjectList_NumOfObjects;
    RadarObject object_array[50];
}RadarObjectList;


typedef struct _RadarBasicStatus
{
    float Longitudinal;
    float Lateral;
    float Vertical;
    float Yaw;
    float Pitch;
    unsigned char PlugOrientation;
    float Length;
    float Width;
    float Height;
    float Wheelbase;
    unsigned short MaximumDistance;
    unsigned char FrequencySlot;
    unsigned char CycleTime;
    unsigned char TimeSlot;
    unsigned char HCC;
    unsigned char Powersave_Standstill;
    unsigned int SensorIPAddress_0;
    unsigned int SensorIPAddress_1;
    unsigned char Configuration_counter;
    unsigned char Status_LongitudinalVelocity;
    unsigned char Status_LongitudinalAcceleration;
    unsigned char Status_LateralAcceleration;
    unsigned char Status_YawRate;
    unsigned char Status_SteeringAngle;
    unsigned char Status_DrivingDirection;
    unsigned char Status_CharacteristicSpeed;
    unsigned char Status_Radar;
}RadarBasicStatus;


typedef struct _VehicleAccLateral
{
    float AccelerationLateralErrAmp;
    unsigned char AccelerationLateralErrAmp_InvalidFlag;
    unsigned char  QualifierAccelerationLateral;
    float AccelerationLateral;
    unsigned char  AccelerationLateral_InvalidFlag;
    unsigned char  AccelerationLateralEventDataQualifier;
}VehicleAccLateral;


typedef struct _VehicleAccLongitudinal
{
    float AccelerationLongitudinalErrAmp;
    unsigned char AccelerationLongitudinalErrAmp_InvalidFlag;
    unsigned char QualifierAccelerationLongitudinal;
    float AccelerationLongitudinal;
    unsigned char AccelerationLongitudinal_InvalidFlag;
    unsigned char AccelerationLongitudinalEventDataQualifier;
}VehicleAccLongitudinal;


typedef struct _VehicleCharactericSpeed
{
    unsigned char CharacteristicSpeedErrAmp;
    unsigned char QualifierCharacteristicSpeed;
    unsigned char CharacteristicSpeed;
}VehicleCharactericSpeed;

typedef struct _VehicleDrivingDirection
{
    unsigned char DrivingDirectionUnconfirmed;
    unsigned char DrivingDirectionConfirmed;
}VehicleDrivingDirection;

typedef struct _VehicleSteeringAngle
{
    unsigned char QualifierSteeringAngleFrontAxle;
    float SteeringAngleFrontAxleErrAmp;
    unsigned char SteeringAngleFrontAxleErrAmp_InvalidFlag;
    float SteeringAngleFrontAxle;
    unsigned char SteeringAngleFrontAxle_InvalidFlag;
    unsigned char SteeringAngleFrontAxleEventDataQualifier;
}VehicleSteeringAngle;

typedef struct _VehicleVelocity
{
    unsigned char StatusVelocityNearStandstill;
    unsigned char QualifierVelocityVehicle;
    unsigned char VelocityVehicleEventDataQualifier;
    float VelocityVehicle;
    unsigned char VelocityVehicle_InvalidFlag;
}VehicleVelocity;

typedef struct _VehicleYawRate
{
    float YawRateErrAmp;
    unsigned char YawRateErrAmp_InvalidFlag;
    unsigned char QualifierYawRate;
    float YawRate;
    unsigned char YawRate_InvalidFlag;
    unsigned char YawRateEventDataQualifier;
}VehicleYawRate;



#endif 