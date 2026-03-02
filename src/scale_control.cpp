#include "scale_control.h"
#include <M5UnitUnifiedWEIGHT.h>

extern m5::unit::UnitWeightI2C scale;

float scale_get_raw()
{
    return 0; //return scale.getRaw();
}

void scale_tare()
{
    //scale.tare();
}

void scale_set_cal_factor(float factor)
{
    //scale.setScale(factor);
}
