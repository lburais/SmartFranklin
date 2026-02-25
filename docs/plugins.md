# Plugin System

Interface:
class Plugin {
public:
    virtual const char* name() = 0;
    virtual void setup() = 0;
    virtual void loop() = 0;
};

Register via plugin_register() and plugin_manager.
