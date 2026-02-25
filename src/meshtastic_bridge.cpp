#include "meshtastic_bridge.h"
#include <Meshtastic.h>

HardwareSerial MESHSERIAL(1);
Meshtastic meshtastic;
static String lastReceived;

void onMeshPacket(const meshtastic_MeshPacket *p)
{
    if (p->decoded.portnum == meshtastic_PortNum_TEXT_MESSAGE_APP) {
        lastReceived = String((const char*)p->decoded.payload.bytes,
                              p->decoded.payload.size);
    }
}

void meshtastic_bridge_init()
{
    MESHSERIAL.begin(115200, SERIAL_8N1, 33, 32);
    meshtastic.begin(&MESHSERIAL);
    meshtastic.onReceive(onMeshPacket);
}

void meshtastic_send_text(const String &msg)
{
    meshtastic.sendText(msg.c_str(), meshtastic_ChannelIndex_PRIMARY);
}

bool meshtastic_poll_received(String &out)
{
    meshtastic.loop();
    if (lastReceived.length()) {
        out = lastReceived;
        lastReceived = "";
        return true;
    }
    return false;
}
