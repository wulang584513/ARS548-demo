#ifndef DATA_PROCESS_H
#define DATA_PROCESS_H

#include "converttype.h"
#include "data_struct.h"
#include <math.h>

class DataProcess
{
public:
    DataProcess();
    ~DataProcess();

    bool processObjectListMessage(char *in,RadarObjectList *o_list);
    bool processDetectionListMessage(char *in,RadarDetectionList *d_list);
    bool processBasicStatusMessage(char *in,RadarBasicStatus *basic_status);

private:
    ConvertType cvt;

};







#endif 