#include <gtest/gtest.h>
#include "EliteCSRobot.h"
#include <thread>
#include <chrono>
#include <cmath>

using namespace ROBOT;
using namespace std::chrono_literals;

static std::string s_robot_ip = "192.168.1.200";

class MotionTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        std::string resDir = "resources/";
        config.headless_mode = true;
        config.robot_ip = s_robot_ip;
        config.script_file_path = resDir + "external_control.script";

        robot = std::make_unique<EliteCSRobot>(config);

        connConfig["inputRecipePath"] = resDir + "input_recipe.txt";
        connConfig["outputRecipePath"] = resDir + "output_recipe.txt";

        // 运动测试前确保连接
        if (!robot->Connect(connConfig))
        {
            // 在 SetUp 中无法直接 FAIL，但会让后续 Test 失败
        }
    }

    void TearDown() override
    {
        if (robot && robot->IsConnected())
        {
            // 停止任何可能的运动
            robot->StopJogging();
            robot->Disconnect();
        }
    }

    void WaitForMotionStop(int timeout_ms = 5000)
    {
        auto start = std::chrono::steady_clock::now();
        while (robot->IsMoving())
        {
            std::this_thread::sleep_for(100ms);
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > timeout_ms)
            {
                break;
            }
        }
    }

    ELITE::EliteDriverConfig config;
    ConfigDict connConfig;
    std::unique_ptr<EliteCSRobot> robot;
};

// 1. MoveTo (关节/直线混合，通常是 movel)
TEST_F(MotionTest, MoveTo_Offsets)
{
    if (!robot->IsConnected())
        GTEST_SKIP() << "Robot not connected";

    RobotPosition startPos;
    ASSERT_TRUE(robot->GetPosition(startPos));

    // Z + 0.01m
    double targetZ = startPos.z + 0.01;
    bool req = robot->MoveTo(startPos.x, startPos.y, targetZ,
                             startPos.rx, startPos.ry, startPos.rz);
    EXPECT_TRUE(req);

    // 等待一会并检查 Z 轴变化
    std::this_thread::sleep_for(500ms); // 等待运动开始
    WaitForMotionStop();

    RobotPosition endPos;
    robot->GetPosition(endPos);
    EXPECT_NEAR(endPos.z, targetZ, 0.005);
}

// 2. MoveLinear (直线插补)
TEST_F(MotionTest, MoveLinear_Vector)
{
    if (!robot->IsConnected())
        GTEST_SKIP() << "Robot not connected";

    RobotPosition p1;
    ASSERT_TRUE(robot->GetPosition(p1));

    RobotPosition p2 = p1;
    p2.y += 0.02; // Y 轴移动 2cm

    // 调用 MoveLinear
    EXPECT_TRUE(robot->MoveLinear(p1, p2, 0.1)); // speed 0.1 m/s

    std::this_thread::sleep_for(500ms);
    WaitForMotionStop();

    RobotPosition current;
    robot->GetPosition(current);
    EXPECT_NEAR(current.y, p2.y, 0.005);
}

// 3. MoveCircular (圆弧插补)
TEST_F(MotionTest, MoveCircular_Arc)
{
    if (!robot->IsConnected())
        GTEST_SKIP() << "Robot not connected";

    RobotPosition start;
    ASSERT_TRUE(robot->GetPosition(start));

    // 定义一个微小的圆弧运动
    // Start -> Via -> End
    RobotPosition via = start;
    via.x += 0.01;
    via.y += 0.01;

    RobotPosition end = start;
    end.x += 0.02;
    end.y += 0.00;

    // speed 0.1 m/s
    EXPECT_TRUE(robot->MoveCircular(via, end, 0.1));

    std::this_thread::sleep_for(500ms);
    WaitForMotionStop();

    RobotPosition current;
    robot->GetPosition(current);
    // 粗略检查是否到达结束点附近
    EXPECT_NEAR(current.x, end.x, 0.005);
}

// 4. Jogging (点动)
TEST_F(MotionTest, Jogging_Z_Axis)
{
    if (!robot->IsConnected())
        GTEST_SKIP() << "Robot not connected";

    EXPECT_TRUE(robot->StartJogging("+z"));
    std::this_thread::sleep_for(500ms);
    EXPECT_TRUE(robot->StopJogging());
}

int main(int argc, char **argv)
{
    if (argc >= 2)
    {
        s_robot_ip = argv[1];
    }
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
