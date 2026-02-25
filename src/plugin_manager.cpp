#include "plugin_manager.h"

static std::vector<Plugin*> plugins;

void plugin_register(Plugin *p) {
    plugins.push_back(p);
}

void plugin_setup_all() {
    for (auto *p : plugins) p->setup();
}

void plugin_loop_all() {
    for (auto *p : plugins) p->loop();
}
