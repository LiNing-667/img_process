//专门负责 Monitor 往 Pilot 板下发宏指令

#pragma once
#include "types.h"
#include <string>

class PilotCommunicator {
public:
    void sendDemoCommand(const std::string &demo_name, const Pose6D &pose);
};