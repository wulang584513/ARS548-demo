#include "converttype.h"

ConvertType::ConvertType()
{

}

unsigned short ConvertType::byteToUshort(char *x)
{
    unsigned short r;
    r = ((unsigned short)((*x)&0x00ff)<<8) | *(x+1);
    return r;
}

unsigned int ConvertType::byteToUint(char *x)
{
    unsigned int r;
    r = (((unsigned int)((*x)&0x000000ff))<<24) |(((unsigned int)((*(x+1))&0x000000ff))<<16)|(((unsigned int)((*(x+2))&0x000000ff))<<8)| (unsigned int)((*(x+3))&0x000000ff);
    return r;
}

unsigned long long ConvertType::byteToUlong(char *x)
{
    unsigned long long r;
    r = ((unsigned long long)((*x)&0x00000000000000ff)<<56) |((unsigned long long)((*(x+1))&0x00000000000000ff)<<48)|((unsigned long long)((*(x+2))&0x00000000000000ff)<<40)| ((unsigned long long)((*(x+3))&0x00000000000000ff)<<32)|\
            ((unsigned long long)((*(x+4))&0x00000000000000ff)<<24)|((unsigned long long)((*(x+5))&0x00000000000000ff)<<16)|((unsigned long long)((*(x+6))&0x00000000000000ff)<<8)|((unsigned long long)((*(x+7))&0x00000000000000ff));
    return r;
}

signed short ConvertType::byteToSshort(char *x)
{
    signed short r;
    r = (signed short)(((unsigned short)((*x)&0x00ff)<<8) | *(x+1));
    return r;
}

signed int ConvertType::byteToSint(char *x)
{
    signed int r;
    r = (signed int)(((unsigned int)((*x)&0x000000ff)<<24) |((unsigned int)((*(x+1))&0x000000ff)<<16)|((unsigned int)((*(x+2))&0x000000ff)<<8)| (unsigned int)((*(x+3))&0x000000ff));
    return r;
}

signed long long ConvertType::byteToSlong(char *x)
{
    signed long long r;
    r = (signed long long)(((unsigned long long)((*x)&0x00000000000000ff)<<56) |((unsigned long long)((*(x+1))&0x00000000000000ff)<<48)|((unsigned long long)((*(x+2))&0x00000000000000ff)<<40)| ((unsigned long long)((*(x+3))&0x00000000000000ff)<<32)|\
            ((unsigned long long)((*(x+4))&0x00000000000000ff)<<24)|((unsigned long long)((*(x+5))&0x00000000000000ff)<<16)|((unsigned long long)((*(x+6))&0x00000000000000ff)<<8)|((unsigned long long)((*(x+7))&0x00000000000000ff)));
    return r;
}

float ConvertType::byteToFloat(char *x)
{
    Obj_f.t.b[0]=*(x+3);
    Obj_f.t.b[1]=*(x+2);
    Obj_f.t.b[2]=*(x+1);
    Obj_f.t.b[3]=*(x);

    return Obj_f.c;
}

double ConvertType::byteToDouble(char *x)
{
    Obj_d.t.b[0]=*(x+7);
    Obj_d.t.b[1]=*(x+6);
    Obj_d.t.b[2]=*(x+5);
    Obj_d.t.b[3]=*(x+4);
    Obj_d.t.b[4]=*(x+3);
    Obj_d.t.b[5]=*(x+2);
    Obj_d.t.b[6]=*(x+1);
    Obj_d.t.b[7]=*(x);
    return Obj_d.c;
}

void ConvertType::floatToByte(float in, char *x)
{
    Obj_f.c = in;

    x[0]=Obj_f.t.b[3];
    x[1]=Obj_f.t.b[2];
    x[2]=Obj_f.t.b[1];
    x[3]=Obj_f.t.b[0];

}

void ConvertType::uIntToByte(unsigned int in, char *x)
{
    Obj_INT.c = in;

    x[0]=Obj_INT.t.b[3];
    x[1]=Obj_INT.t.b[2];
    x[2]=Obj_INT.t.b[1];
    x[3]=Obj_INT.t.b[0];
}

void ConvertType::uShortToByte(unsigned short in, char *x)
{
    x[0]=((in>>8)&0xff);
    x[1]=(in&0xff);
}

ConvertType::~ConvertType()
{

}
