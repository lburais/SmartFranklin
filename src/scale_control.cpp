#include "scale_control.h"
#include <M5Unit-MiniScale.h>

extern MiniScale scale;

float scale_get_raw()
{
    return scale.getWeight();
}

void scale_tare()
{
    scale.tare();
}

void scale_set_cal_factor(float factor)
{
    scale.setCalFactor(factor);
}
