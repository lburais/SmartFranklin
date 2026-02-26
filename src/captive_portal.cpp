#include "captive_portal.h"
#include <DNSServer.h>
#include <WiFi.h>

static DNSServer dnsServer;

void captive_portal_start()
{
    dnsServer.start(53, "*", WiFi.softAPIP());
}
