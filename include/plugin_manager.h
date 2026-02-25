#pragma once
#include "plugin.h"
#include <vector>

void plugin_register(Plugin *p);
void plugin_setup_all();
void plugin_loop_all();
