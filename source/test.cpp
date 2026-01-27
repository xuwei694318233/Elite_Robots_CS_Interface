#include "EliteCSRobot.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <functional>

using namespace ROBOT;
using namespace std::chrono_literals;

// --------------------------------------------------------
// Helper to print position
// --------------------------------------------------------
void PrintPosition(const RobotPosition &pos, const std::string &label = "Pos")
{
    std::cout << "[" << label << "] "
              << "X:" << std::fixed << std::setprecision(4) << pos.x << " "
              << "Y:" << pos.y << " "
              << "Z:" << pos.z << " "
              << "RX:" << pos.rx << " "
              << "RY:" << pos.ry << " "
              << "RZ:" << pos.rz << std::endl;
}

// --------------------------------------------------------
// Helper to safely print std::any
// --------------------------------------------------------
std::string PrintAny(const std::any &a)
{
    try
    {
        if (a.type() == typeid(std::string))
            return std::any_cast<std::string>(a);
        if (a.type() == typeid(const char *))
            return std::string(std::any_cast<const char *>(a));
        if (a.type() == typeid(bool))
            return std::any_cast<bool>(a) ? "True" : "False";
        return "Unknown Type";
    }
    catch (...)
    {
        return "Error";
    }
}

// --------------------------------------------------------
// Test Modules
// --------------------------------------------------------
bool TestConnectionDetailed(EliteCSRobot &robot)
{
    std::cout << "\n[TEST] Testing detailed connection info...\n";
    InfoDict info;
    robot.GetInfo(info);
    std::cout << "Robot Info: Brand=" << PrintAny(info["brand"])
              << ", Connected=" << PrintAny(info["connected"]) << std::endl;

    InfoDict connTest;
    robot.TestConnection(connTest);
    if (connTest.count("success") && std::any_cast<bool>(connTest["success"]))
    {
        std::cout << "[SUCCESS] Connection Test Passed" << std::endl;
        return true;
    }
    std::cerr << "[FAILED] Connection Test Failed" << std::endl;
    return false;
}

void TestBasicMotion(EliteCSRobot &robot, RobotPosition &currentPos)
{
    std::cout << "\n[TEST] Basic MoveTo (Z + 0.02m)\n";

    if (!robot.GetPosition(currentPos))
    {
        std::cerr << "Failed to get initial position\n";
        return;
    }
    PrintPosition(currentPos, "Start Pos");

    RobotPosition targetPos = currentPos;
    targetPos.z += 0.02; // Move up 2cm

    if (robot.MoveTo(targetPos.x, targetPos.y, targetPos.z, targetPos.rx, targetPos.ry, targetPos.rz))
    {
        // Wait for move to start
        std::this_thread::sleep_for(200ms);
        // Wait for move to complete
        while (robot.IsMoving())
        {
            std::cout << ".";
            std::this_thread::sleep_for(100ms);
        }
        std::cout << "\n[DONE] MoveTo complete" << std::endl;
        robot.GetPosition(currentPos); // 更新位置
    }
    else
    {
        std::cerr << "[FAILED] MoveTo command failed" << std::endl;
    }
}

void TestLinearMotion(EliteCSRobot &robot, RobotPosition &currentPos)
{
    std::cout << "\n[TEST] MoveLinear (Z - 0.02m)\n";

    RobotPosition startLinear = currentPos;
    RobotPosition endLinear = currentPos;
    endLinear.z -= 0.02; // Move down 2cm

    if (robot.MoveLinear(startLinear, endLinear, 0.1))
    {
        std::this_thread::sleep_for(200ms);
        while (robot.IsMoving())
        {
            std::cout << ".";
            std::this_thread::sleep_for(100ms);
        }
        std::cout << "\n[DONE] MoveLinear complete" << std::endl;
        robot.GetPosition(currentPos); // 更新位置
    }
}

void TestCircularMotion(EliteCSRobot &robot, RobotPosition &currentPos)
{
    std::cout << "\n[TEST] MoveCircular\n";

    RobotPosition viaPos = currentPos;
    viaPos.y += 0.05;
    viaPos.x += 0.02;
    RobotPosition endCirclePos = currentPos;
    endCirclePos.y += 0.10;

    if (robot.MoveCircular(viaPos, endCirclePos, 0.1))
    {
        std::this_thread::sleep_for(200ms);
        while (robot.IsMoving())
        {
            std::cout << ".";
            std::this_thread::sleep_for(100ms);
        }
        std::cout << "\n[DONE] MoveCircular complete" << std::endl;

        // Return to start
        std::cout << "Returning to previous start..." << std::endl;
        robot.MoveTo(currentPos.x, currentPos.y, currentPos.z, currentPos.rx, currentPos.ry, currentPos.rz);
        while (robot.IsMoving())
            std::this_thread::sleep_for(100ms);
    }
}

void TestJogging(EliteCSRobot &robot)
{
    std::cout << "\n[TEST] Jogging (+Z for 1s)\n";
    if (robot.StartJogging("+z"))
    {
        std::this_thread::sleep_for(1s);
        robot.StopJogging();
        std::cout << "[DONE] Jogging stopped" << std::endl;
        std::this_thread::sleep_for(1s); // Wait for deceleration
    }
}

void TestPathRecordingAndPlayback(EliteCSRobot &robot)
{
    std::cout << "\n[TEST] Path Recording & Playback\n";
    if (robot.StartPathRecording("TestPath"))
    {
        std::cout << "1. Recording point 1 (Current)...\n";
        robot.AddPathPoint(nullptr);

        std::cout << "2. Moving to point 2 (X+0.05)...\n";
        RobotPosition targetPos;
        robot.GetPosition(targetPos);
        targetPos.x += 0.05;
        robot.MoveTo(targetPos.x, targetPos.y, targetPos.z, targetPos.rx, targetPos.ry, targetPos.rz);
        while (robot.IsMoving())
            std::this_thread::sleep_for(50ms);
        robot.AddPathPoint(nullptr);

        std::cout << "3. Moving to point 3 (Return)...\n";
        targetPos.x -= 0.05;
        robot.MoveTo(targetPos.x, targetPos.y, targetPos.z, targetPos.rx, targetPos.ry, targetPos.rz);
        while (robot.IsMoving())
            std::this_thread::sleep_for(50ms);
        robot.AddPathPoint(nullptr);

        robot.StopPathRecording();
        std::cout << "Recording stopped.\n";
    }

    RobotPath path;
    if (robot.GetRecordedPath(path))
    {
        std::cout << "Recorded path '" << path.name << "' with " << path.points.size() << " points.\n";
        std::cout << "Playing back...\n";

        if (robot.PlayPath(path, 1))
        {
            std::this_thread::sleep_for(200ms);
            while (robot.IsPathPlaying())
            {
                std::cout << ">> Playing..." << std::endl;
                std::this_thread::sleep_for(500ms);
            }
            std::cout << "[DONE] Path playback complete\n";
        }
        else
        {
            std::cerr << "Failed to start playback" << std::endl;
        }
    }
}

void Testcallbacks(EliteCSRobot &robot)
{
    std::cout << "\n[TEST] Callbacks\n";
    int updateCount = 0;
    auto posCb = [&](const RobotPosition &p)
    { updateCount++; };

    robot.RegisterPositionCallback(posCb);
    std::cout << "Callback registered. Jogging small amount...\n";
    robot.JogMove("z", 0.05, 0.01); // Small jog step
    std::this_thread::sleep_for(1s);
    robot.UnregisterPositionCallback(posCb);

    std::cout << "Position callback triggered " << updateCount << " times.\n";
}

void TestMotionMode(EliteCSRobot &robot)
{
    std::cout << "\n[TEST] Motion Mode\n";
    robot.SetMotionMode(MotionMode::MANUAL);
    MotionMode mode;
    robot.GetMotionMode(mode);
    if (mode == MotionMode::MANUAL)
    {
        std::cout << "[SUCCESS] Set/Get Motion Mode (Manual) verified.\n";
    }
    robot.SetMotionMode(MotionMode::AUTOMATIC);
}

// --------------------------------------------------------
// Main Entry
// --------------------------------------------------------
int main()
{
    // 1. Configuration
    std::cout << "=== Elite Robot Interface Test Start ===\n";
    std::string rootDir = R"(C:\Users\GhFeng\Desktop\xuwei\gitcode\Elite_Robots_CS_Interface-main)";

    ELITE::EliteDriverConfig config;
    config.headless_mode = true;
    config.robot_ip = "192.168.1.200";
    config.script_file_path = rootDir + "/resources/external_control.script";

    std::cout << "Creating robot instance...\n";
    EliteCSRobot robot(config);

    ConfigDict configDict;
    configDict["inputRecipePath"] = rootDir + "/resources/input_recipe.txt";
    configDict["outputRecipePath"] = rootDir + "/resources/output_recipe.txt";

    // 2. Connection
    std::cout << "Connecting to robot at " << config.robot_ip << "...\n";
    if (!robot.Connect(configDict))
    {
        std::cerr << "[FAILED] Robot connect failed" << std::endl;
        return -1;
    }
    std::cout << "[SUCCESS] Robot connected\n";

    // 3. Execution Sequence
    RobotPosition currentPos;

    TestConnectionDetailed(robot);

    if (robot.GetPosition(currentPos))
    {
        TestBasicMotion(robot, currentPos);
        TestLinearMotion(robot, currentPos);
        TestCircularMotion(robot, currentPos);
        TestJogging(robot);
        TestPathRecordingAndPlayback(robot);
        Testcallbacks(robot);
        TestMotionMode(robot);
    }
    else
    {
        std::cerr << "Cannot proceed: Failed to get initial position.\n";
    }

    // 4. Disconnect
    std::cout << "\nDisconnecting...\n";
    robot.Disconnect();
    std::cout << "=== Test Finished ===\n";

    return 0;
}
