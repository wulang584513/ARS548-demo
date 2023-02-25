#ifndef CONVERTTYPE_H
#define CONVERTTYPE_H

#include <iostream>
#include <cstdlib>

class ConvertType
{
public:
    ConvertType();
    ~ConvertType();

    unsigned short byteToUshort(char *x);
    unsigned int byteToUint(char *x);
    unsigned long long byteToUlong(char *x);
    signed short byteToSshort(char *x);
    signed int byteToSint(char *x);
    signed long long byteToSlong(char *x);
    float byteToFloat(char *x);
    double byteToDouble(char *x);
    void floatToByte(float in, char *x);
    void uIntToByte(unsigned int in, char *x);
    void uShortToByte(unsigned short in, char *x);


private:
    union Node_f{
        struct{
            char b[4];
        }t;
        float c;
    }Obj_f;

    union Node_d{
        struct{
            char b[8];
        }t;
        double c;
    }Obj_d;

    union Node_INT{
        struct{
            char b[4];
        }t;
        unsigned int c;
    }Obj_INT;

};

#endif // CONVERTTYPE_H
