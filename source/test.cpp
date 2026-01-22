#include "EliteCSRobot.h"
#include <iostream>

using namespace ROBOT;

int main()
{
    std::string rootDir = R"(C:\Users\GhFeng\Desktop\xuwei\gitcode\Elite_Robots_CS_Interface)";
    ELITE::EliteDriverConfig config;
    config.headless_mode = true;
    config.robot_ip = "192.168.1.200";
    config.script_file_path = rootDir + "/resources/external_control.script";

    EliteCSRobot robot(config);
    ConfigDict configDict;
    configDict["inputRecipePath"] = rootDir + "/resources/input_recipe.txt";
    configDict["outputRecipePath"] = rootDir + "/resources/output_recipe.txt";

    std::cout << "Test start\n";

    if (!robot.Connect(configDict))
    {
        std::cout << "Robot connet falied\n";
        return -1;
    }

    std::cout << "Robot connet successfully\n";

    RobotPosition pos;
    if (!robot.GetPosition(pos))
    {
        std::cout << "Get position failed\n";
        return -1;
    }

    pos.x += 0.2;
    if (!robot.MoveTo(pos.x, pos.y, pos.z, pos.rx, pos.ry, pos.rz))
    {
        std::cout << "Move to position failed\n";
        return -1;
    }

    std::cout << "Test finish\n";

    return 0;
}
