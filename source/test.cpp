#include "EliteCSRobot.h"
#include <iostream>

using namespace ROBOT;

int main()
{
    // ELITE::EliteDriverConfig config;
    // config.local_ip = "";
    // config.script_file_path = "../resources/external_control.script";

    // EliteCSRobot robot(config);
    // ConfigDict configDict;
    // configDict["inputRecipePath"] = std::string("../resources/input_recipe.txt");
    // configDict["outputRecipePath"] = std::string("../resources/output_recipe.txt");

    // if (!robot.Connect(configDict))
    // {
    //     std::cout << "Robot connet falied\n";
    //     return -1;
    // }

    std::cout << "Robot connet successfully\n";

    std::cout << "test\n";

    return 0;
}
