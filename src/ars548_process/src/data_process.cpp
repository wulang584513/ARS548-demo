#include "data_process.h"

DataProcess::DataProcess()
{

}

DataProcess::~DataProcess()
{

}

bool DataProcess::processObjectListMessage(char *in,RadarObjectList *o_list)
{
    int num;
    int base_index;

    o_list->serviceID = cvt.byteToUshort(in);
    o_list->MethodID = cvt.byteToUshort(in + 2);
    if(o_list->MethodID != 329)
        return false;
    o_list->data_length = cvt.byteToUint(in + 4);
    if(o_list->data_length != (9401 - 8))
        return false;
    o_list->clientID = cvt.byteToUshort(in + 8);
    o_list->sessionID = cvt.byteToUshort(in + 10);
    o_list->protocol_version = (unsigned char)(in[12]);
    o_list->interface_version = (unsigned char)(in[13]);
    o_list->message_type = (unsigned char)(in[14]);
    o_list->return_code = (unsigned char)(in[15]);

    o_list->CRC = cvt.byteToUlong(in + 16);
    o_list->Length = cvt.byteToUint(in + 24);
    o_list->SQC = cvt.byteToUint(in + 28);
    o_list->DataID = cvt.byteToUint(in + 32);
    //if(o_list->DataID != 0x01010200)
    //    return false;
    o_list->Timestamp_Nanoseconds = cvt.byteToUint(in + 36);
    o_list->Timestamp_Seconds = cvt.byteToUint(in + 40);
    o_list->Timestamp_SyncStatus = (unsigned char)(in[44]);
    o_list->EventDataQualifier = cvt.byteToUint(in + 45);
    o_list->ExtendedQualifier = (unsigned char)(in[49]);
    o_list->ObjectList_NumOfObjects = (unsigned char)(in[50]);
    if(o_list->ObjectList_NumOfObjects>50)
        return false;
    for(num = 0; num<o_list->ObjectList_NumOfObjects; num++)
    {
        base_index = 51+num*187;
        o_list->object_array[num].u_StatusSensor = cvt.byteToUshort(in + base_index);
        o_list->object_array[num].u_ID = cvt.byteToUint(in + base_index+2);
        o_list->object_array[num].u_Age = cvt.byteToUshort(in + base_index+6);
        o_list->object_array[num].u_StatusMeasurement = (unsigned char)(in[base_index+8]);
        o_list->object_array[num].u_StatusMovement = (unsigned char)(in[base_index+9]);

        o_list->object_array[num].u_Position_InvalidFlags = cvt.byteToUshort(in + base_index+10);
        o_list->object_array[num].u_Position_Reference = (unsigned char)(in[base_index+12]);
        o_list->object_array[num].u_Position_X = cvt.byteToFloat(in + base_index +13);
        o_list->object_array[num].u_Position_X_STD = cvt.byteToFloat(in + base_index +17);
        o_list->object_array[num].u_Position_Y = cvt.byteToFloat(in + base_index +21);
        o_list->object_array[num].u_Position_Y_STD = cvt.byteToFloat(in + base_index +25);
        o_list->object_array[num].u_Position_Z = cvt.byteToFloat(in + base_index +29);
        o_list->object_array[num].u_Position_Z_STD = cvt.byteToFloat(in + base_index +33);
        o_list->object_array[num].u_Position_CovarianceXY = cvt.byteToFloat(in + base_index +37);
        o_list->object_array[num].u_Position_Orientation = cvt.byteToFloat(in + base_index +41);
        o_list->object_array[num].u_Position_Orientation_STD = cvt.byteToFloat(in + base_index +45);

        o_list->object_array[num].u_Existence_InvalidFlags = (unsigned char)(in[base_index+49]);
        o_list->object_array[num].u_Existence_Probability = cvt.byteToFloat(in + base_index +50);
        o_list->object_array[num].u_Existence_PPV = cvt.byteToFloat(in + base_index +54);

        o_list->object_array[num].u_Classification_Car = (unsigned char)(in[base_index+58]);
        o_list->object_array[num].u_Classification_Truck = (unsigned char)(in[base_index+59]);
        o_list->object_array[num].u_Classification_Motorcycle = (unsigned char)(in[base_index+60]);
        o_list->object_array[num].u_Classification_Bicycle = (unsigned char)(in[base_index+61]);
        o_list->object_array[num].u_Classification_Pedestrian = (unsigned char)(in[base_index+62]);
        o_list->object_array[num].u_Classification_Animal = (unsigned char)(in[base_index+63]);
        o_list->object_array[num].u_Classification_Hazard = (unsigned char)(in[base_index+64]);
        o_list->object_array[num].u_Classification_Unknown = (unsigned char)(in[base_index+65]);
        o_list->object_array[num].u_Classification_Overdrivable = (unsigned char)(in[base_index+66]);
        o_list->object_array[num].u_Classification_Underdrivable = (unsigned char)(in[base_index+67]);

        o_list->object_array[num].u_Dynamics_AbsVel_InvalidFlags = (unsigned char)(in[base_index+68]);
        o_list->object_array[num].f_Dynamics_AbsVel_X = cvt.byteToFloat(in + base_index +69);
        o_list->object_array[num].f_Dynamics_AbsVel_X_STD = cvt.byteToFloat(in + base_index +73);
        o_list->object_array[num].f_Dynamics_AbsVel_Y = cvt.byteToFloat(in + base_index +77);
        o_list->object_array[num].f_Dynamics_AbsVel_Y_STD = cvt.byteToFloat(in + base_index +81);
        o_list->object_array[num].f_Dynamics_AbsVel_CovarianceXY = cvt.byteToFloat(in + base_index +85);

        o_list->object_array[num].u_Dynamics_RelVel_InvalidFlags = (unsigned char)(in[base_index+89]);
        o_list->object_array[num].f_Dynamics_RelVel_X = cvt.byteToFloat(in + base_index +90);
        o_list->object_array[num].f_Dynamics_RelVel_X_STD = cvt.byteToFloat(in + base_index +94);
        o_list->object_array[num].f_Dynamics_RelVel_Y = cvt.byteToFloat(in + base_index +98);
        o_list->object_array[num].f_Dynamics_RelVel_Y_STD = cvt.byteToFloat(in + base_index +102);
        o_list->object_array[num].f_Dynamics_RelVel_CovarianceXY = cvt.byteToFloat(in + base_index +106);

        o_list->object_array[num].u_Dynamics_AbsAccel_InvalidFlags = (unsigned char)(in[base_index+110]);
        o_list->object_array[num].f_Dynamics_AbsAccel_X = cvt.byteToFloat(in + base_index +111);
        o_list->object_array[num].f_Dynamics_AbsAccel_X_STD = cvt.byteToFloat(in + base_index +115);
        o_list->object_array[num].f_Dynamics_AbsAccel_Y = cvt.byteToFloat(in + base_index +119);
        o_list->object_array[num].f_Dynamics_AbsAccel_Y_STD = cvt.byteToFloat(in + base_index +123);
        o_list->object_array[num].f_Dynamics_AbsAccel_CovarianceXY = cvt.byteToFloat(in + base_index +127);

        o_list->object_array[num].u_Dynamics_RelAccel_InvalidFlags = (unsigned char)(in[base_index+131]);
        o_list->object_array[num].f_Dynamics_RelAccel_X = cvt.byteToFloat(in + base_index +132);
        o_list->object_array[num].f_Dynamics_RelAccel_X_STD = cvt.byteToFloat(in + base_index +136);
        o_list->object_array[num].f_Dynamics_RelAccel_Y = cvt.byteToFloat(in + base_index +140);
        o_list->object_array[num].f_Dynamics_RelAccel_Y_STD = cvt.byteToFloat(in + base_index +144);
        o_list->object_array[num].f_Dynamics_RelAccel_CovarianceXY = cvt.byteToFloat(in + base_index +148);

        o_list->object_array[num].u_Dynamics_Orientation_InvalidFlags = (unsigned char)(in[base_index+152]);
        o_list->object_array[num].u_Dynamics_Orientation_Rate_Mean = cvt.byteToFloat(in + base_index +153);
        o_list->object_array[num].u_Dynamics_Orientation_Rate_STD = cvt.byteToFloat(in + base_index +157);

        o_list->object_array[num].u_Shape_Length_Status = cvt.byteToUint(in + base_index+161);
        o_list->object_array[num].u_Shape_Length_Edge_InvalidFlags = (unsigned char)(in[base_index+165]);
        o_list->object_array[num].u_Shape_Length_Edge_Mean = cvt.byteToFloat(in + base_index +166);
        o_list->object_array[num].u_Shape_Length_Edge_STD = cvt.byteToFloat(in + base_index +170);
        o_list->object_array[num].u_Shape_Width_Status = cvt.byteToUint(in + base_index+174);
        o_list->object_array[num].u_Shape_Width_Edge_InvalidFlags = (unsigned char)(in[base_index+178]);
        o_list->object_array[num].u_Shape_Width_Edge_Mean = cvt.byteToFloat(in + base_index +179);
        o_list->object_array[num].u_Shape_Width_Edge_STD = cvt.byteToFloat(in + base_index +183);

    }
    return true;
}


bool DataProcess::processDetectionListMessage(char *in,RadarDetectionList *d_list)
{
    float f_AzimuthAngle;
    float f_ElevationAngle;
    float f_Range;
    int num;
    int base_index;

    d_list->serviceID = cvt.byteToUshort(in);
    d_list->MethodID = cvt.byteToUshort(in + 2);
    if(d_list->MethodID != 336)
        return false;
    d_list->data_length = cvt.byteToUint(in + 4);
    if(d_list->data_length != (35336 - 8))
        return false;
    d_list->clientID = cvt.byteToUshort(in + 8);
    d_list->sessionID = cvt.byteToUshort(in + 10);
    d_list->protocol_version = (unsigned char)(in[12]);
    d_list->interface_version = (unsigned char)(in[13]);
    d_list->message_type = (unsigned char)(in[14]);
    d_list->return_code = (unsigned char)(in[15]);

    d_list->CRC = cvt.byteToUlong(in + 16);
    d_list->Length = cvt.byteToUint(in + 24);
    d_list->SQC = cvt.byteToUint(in + 28);
    d_list->DataID = cvt.byteToUint(in + 32);

    d_list->Timestamp_Nanoseconds = cvt.byteToUint(in + 36);
    d_list->Timestamp_Seconds = cvt.byteToUint(in + 40);
    d_list->Timestamp_SyncStatus = (unsigned char)(in[44]);;
    d_list->EventDataQualifier = cvt.byteToUint(in + 45);
    d_list->ExtendedQualifier = (unsigned char)(in[49]);
    d_list->Origin_InvalidFlags = cvt.byteToUshort(in + 50);

    d_list->Origin_Xpos = cvt.byteToFloat(in + 52);
    d_list->Origin_Xstd = cvt.byteToFloat(in + 56);
    d_list->Origin_Ypos = cvt.byteToFloat(in + 60);
    d_list->Origin_Ystd = cvt.byteToFloat(in + 64);
    d_list->Origin_Zpos = cvt.byteToFloat(in + 68);
    d_list->Origin_Zstd = cvt.byteToFloat(in + 72);

    d_list->Origin_Roll = cvt.byteToFloat(in + 76);
    d_list->Origin_Rollstd = cvt.byteToFloat(in + 80);
    d_list->Origin_Pitch = cvt.byteToFloat(in + 84);
    d_list->Origin_Pitchstd = cvt.byteToFloat(in + 88);
    d_list->Origin_Yaw = cvt.byteToFloat(in + 92);
    d_list->Origin_Yawstd = cvt.byteToFloat(in + 96);

    d_list->List_InvalidFlags = (unsigned char)(in[100]);

    d_list->List_RadVelDomain_Min = cvt.byteToFloat(in + 35301);
    d_list->List_RadVelDomain_Max = cvt.byteToFloat(in + 35305);
    d_list->List_NumOfDetections = cvt.byteToUint(in + 35309);
    d_list->Aln_AzimuthCorrection = cvt.byteToFloat(in + 35313);
    d_list->Aln_ElevationCorrection = cvt.byteToFloat(in + 35317);
    d_list->Aln_Status = (unsigned char)(in[35321]);

    for(num = 0; num< d_list->List_NumOfDetections;num++)
    {
        base_index = 101+num*44;

        f_AzimuthAngle = cvt.byteToFloat(in + base_index);
        f_ElevationAngle = cvt.byteToFloat(in + base_index +9);
        f_Range = cvt.byteToFloat(in + base_index +17);
        if((f_AzimuthAngle == 0.0 && f_ElevationAngle == 0.0)||f_Range == 0)
            break;
        d_list->detection_array[num].f_x = f_Range*cos(f_ElevationAngle)*cos(f_AzimuthAngle);
        d_list->detection_array[num].f_y = f_Range*cos(f_ElevationAngle)*sin(f_AzimuthAngle);
        d_list->detection_array[num].f_z = f_Range*sin(f_ElevationAngle);
        d_list->detection_array[num].u_InvalidFlags = (unsigned char)(in[base_index+8]);
        d_list->detection_array[num].f_RangeRate = cvt.byteToFloat(in + base_index +25);
        d_list->detection_array[num].f_RangeRateSTD = cvt.byteToFloat(in + base_index +29);
        d_list->detection_array[num].s_RCS = (signed char)(in[base_index+33]);
        d_list->detection_array[num].u_MeasurementID = cvt.byteToUshort(in + base_index +34);
        d_list->detection_array[num].u_PositivePredictiveValue = (unsigned char)(in[base_index+36]);
        d_list->detection_array[num].u_Classification = (unsigned char)(in[base_index+37]);
        d_list->detection_array[num].u_MultiTargetProbability = (unsigned char)(in[base_index+38]);
        d_list->detection_array[num].u_ObjectID = cvt.byteToUshort(in + base_index +39);
        d_list->detection_array[num].u_AmbiguityFlag = (unsigned char)(in[base_index+41]);
        d_list->detection_array[num].u_SortIndex = cvt.byteToUshort(in + base_index +42);
    }
    if(num< d_list->List_NumOfDetections)
        d_list->List_NumOfDetections = num;

    return true;
}


bool DataProcess::processBasicStatusMessage(char *in,RadarBasicStatus *basic_status)
{
    bool succeed = true;
    unsigned short service_ID;
    unsigned short method_ID;
    unsigned int length;

    service_ID  = cvt.byteToUshort(in);
    method_ID = cvt.byteToUshort(in + 2);
    if(method_ID != 380)
    {
        return false;
    }
    length = cvt.byteToUint(in + 4);
    if(length != (69 - 8))
    {
        return false;
    }

    basic_status->Longitudinal = cvt.byteToFloat(in + 8);
    basic_status->Lateral = cvt.byteToFloat(in + 12);
    basic_status->Vertical = cvt.byteToFloat(in + 16);
    basic_status->Yaw = cvt.byteToFloat(in + 20);
    basic_status->Pitch = cvt.byteToFloat(in + 24);
    basic_status->PlugOrientation = (unsigned char)(in[28]);
    basic_status->Length = cvt.byteToFloat(in + 29);
    basic_status->Width = cvt.byteToFloat(in + 33);
    basic_status->Height = cvt.byteToFloat(in + 37);
    basic_status->Wheelbase = cvt.byteToFloat(in + 41);
    basic_status->MaximumDistance = cvt.byteToUshort(in + 45);
    basic_status->FrequencySlot = (unsigned char)(in[47]);
    basic_status->CycleTime = (unsigned char)(in[48]);
    basic_status->TimeSlot = (unsigned char)(in[49]);
    basic_status->HCC = (unsigned char)(in[50]);
    basic_status->Powersave_Standstill = (unsigned char)(in[51]);
    basic_status->SensorIPAddress_0 = cvt.byteToUint(in + 52);
    basic_status->SensorIPAddress_1 = cvt.byteToUint(in + 56);
    basic_status->Configuration_counter = (unsigned char)(in[60]);
    basic_status->Status_LongitudinalVelocity = (unsigned char)(in[61]);
    basic_status->Status_LongitudinalAcceleration = (unsigned char)(in[62]);
    basic_status->Status_LateralAcceleration = (unsigned char)(in[63]);
    basic_status->Status_YawRate = (unsigned char)(in[64]);
    basic_status->Status_SteeringAngle = (unsigned char)(in[65]);
    basic_status->Status_DrivingDirection = (unsigned char)(in[66]);
    basic_status->Status_CharacteristicSpeed = (unsigned char)(in[67]);
    basic_status->Status_Radar = (unsigned char)(in[68]);

    return true;

}