#include "meshtastic_bridge.h"

#include <meshtastic/mesh.h>
#include <meshtastic/serial.h>

using namespace meshtastic;

HardwareSerial MESHSERIAL(1);
SerialDevice* meshDevice = nullptr;
Mesh* mesh = nullptr;

static String lastReceived;

void onMeshPacket(const MeshPacket& p)
{
    if (p.decoded.portnum == PortNum::TEXT_MESSAGE_APP) {
        lastReceived = String(
            reinterpret_cast<const char*>(p.decoded.payload.data()),
            p.decoded.payload.size()
        );
    }
}

void meshtastic_bridge_init()
{
    // UART pins for Meshtastic module (adjust if needed)
    MESHSERIAL.begin(115200, SERIAL_8N1, 33, 32);

    meshDevice = new SerialDevice(&MESHSERIAL);
    mesh = new Mesh(meshDevice);

    mesh->onReceive(onMeshPacket);
    mesh->begin();
}

void meshtastic_send_text(const String& msg)
{
    if (!mesh) return;
    mesh->sendText(msg.c_str(), ChannelIndex::PRIMARY);
}

bool meshtastic_poll_received(String& out)
{
    if (!mesh) return false;

    mesh->loop();

    if (lastReceived.length()) {
        out = lastReceived;
        lastReceived = "";
        return true;
    }
    return false;
}
