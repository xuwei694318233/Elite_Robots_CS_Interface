#include <gtest/gtest.h>
#include "EliteCSRobot.h"
#include <thread>
#include <chrono>

using namespace ROBOT;
using namespace std::chrono_literals;

static std::string s_robot_ip = "192.168.1.200";

class StateTest : public ::testing::Test
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
    }

    void TearDown() override
    {
        if (robot && robot->IsConnected())
        {
            robot->Disconnect();
        }
    }

    ELITE::EliteDriverConfig config;
    ConfigDict connConfig;
    std::unique_ptr<EliteCSRobot> robot;
};

// 1. 获取基本信息
TEST_F(StateTest, GetInfo)
{
    ASSERT_TRUE(robot->Connect(connConfig));

    InfoDict info;
    robot->GetInfo(info);

    EXPECT_EQ(std::any_cast<const char *>(info["brand"]), std::string("Elite"));
    EXPECT_EQ(std::any_cast<std::string>(info["ip"]), s_robot_ip);
    EXPECT_TRUE(std::any_cast<bool>(info["connected"]));
}

// 2. 获取位置信息
TEST_F(StateTest, GetPosition)
{
    ASSERT_TRUE(robot->Connect(connConfig));

    RobotPosition pos;
    EXPECT_TRUE(robot->GetPosition(pos));

    // 检查时间戳是否更新
    using clock = std::chrono::system_clock;
    double current_ts = std::chrono::duration<double>(clock::now().time_since_epoch()).count();

    // 时间戳误差应在合理范围内 (例如1分钟内，考虑系统时间偏差)
    EXPECT_NEAR(pos.timestamp, current_ts, 60.0);
}

// 3. 运动模式 Get/Set
TEST_F(StateTest, MotionModeLogic)
{
    // 此测试无需连接机器人
    robot->SetMotionMode(MotionMode::MANUAL);
    MotionMode m;
    robot->GetMotionMode(m);
    EXPECT_EQ(m, MotionMode::MANUAL);

    robot->SetMotionMode(MotionMode::AUTOMATIC);
    robot->GetMotionMode(m);
    EXPECT_EQ(m, MotionMode::AUTOMATIC);
}

// 4. 获取机器人状态枚举
TEST_F(StateTest, GetRobotState)
{
    ASSERT_TRUE(robot->Connect(connConfig));

    // 静止时应为 IDLE
    RobotState state = robot->GetState();
    EXPECT_EQ(state, RobotState::IDLE);
    EXPECT_FALSE(robot->IsMoving());
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
