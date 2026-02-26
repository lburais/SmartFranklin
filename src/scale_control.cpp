#include "scale_control.h"
#include <M5Unit-WeightI2C.h>

extern M5Unit::WeightI2C scale;

float scale_get_raw()
{
    return scale.getValue();
}

void scale_tare()
{
    scale.tare();
}

void scale_set_cal_factor(float factor)
{
    scale.setCalFactor(factor);
}
