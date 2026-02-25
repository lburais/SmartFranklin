#include "jbd_bms.h"

bool jbd_parse_frame(const uint8_t *data, size_t len, JbdFrame &out)
{
    if (len < 13) return false;
    if (data[0] != 0xDD) return false;
    uint8_t cmd = data[1];
    if (cmd != 0x03) return false;

    uint16_t mv = (data[4] << 8) | data[5];
    out.voltage = mv / 1000.0f;

    int16_t ma = (data[6] << 8) | data[7];
    out.current = ma / 1000.0f;

    out.soc = data[8];
    return true;
}
