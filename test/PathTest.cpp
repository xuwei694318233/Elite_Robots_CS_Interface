#include <gtest/gtest.h>
#include "EliteCSRobot.h"
#include <thread>
#include <chrono>

using namespace ROBOT;
using namespace std::chrono_literals;

static std::string s_robot_ip = "192.168.1.200";

class PathTest : public ::testing::Test
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

// 1. 录制功能完整流程
TEST_F(PathTest, RecordingCycle)
{
    ASSERT_TRUE(robot->Connect(connConfig));

    std::string pathName = "TestPath_Auto";

    // Start
    EXPECT_TRUE(robot->StartPathRecording(pathName));

    // Add Points
    EXPECT_TRUE(robot->AddPathPoint(nullptr)); // Add current

    RobotPosition pos = {0.1, -0.2, 0.3, 0, 3.14, 0, 0};
    PathPoint pp;
    pp.position = pos;
    EXPECT_TRUE(robot->AddPathPoint(&pp)); // Add virtual

    // Stop
    EXPECT_TRUE(robot->StopPathRecording());

    // Verify
    RobotPath recordedPath;
    EXPECT_TRUE(robot->GetRecordedPath(recordedPath));
    EXPECT_EQ(recordedPath.name, pathName);
    EXPECT_EQ(recordedPath.points.size(), 2);

    // Clear
    EXPECT_TRUE(robot->ClearRecordedPath());
    robot->GetRecordedPath(recordedPath);
    EXPECT_EQ(recordedPath.points.size(), 0);
}

// 2. 异常流程：未开始录制就添加点
TEST_F(PathTest, AddPointWithoutRecording)
{
    ASSERT_TRUE(robot->Connect(connConfig));
    EXPECT_FALSE(robot->AddPathPoint(nullptr));
}

// 3. 路径回放 (模拟/轻量级)
TEST_F(PathTest, PlayBackLogic_Smoke)
{
    if (!robot->IsConnected())
    {
        ASSERT_TRUE(robot->Connect(connConfig));
    }

    RobotPosition currentPos;
    ASSERT_TRUE(robot->GetPosition(currentPos));

    RobotPath path;
    path.name = "SmokeTestPath";

    // 构建一个只有 2 个点（原地不动）的路径，确保安全
    PathPoint p1;
    p1.position = currentPos;
    path.points.push_back(p1);
    path.points.push_back(p1);

    // 启动回放
    EXPECT_TRUE(robot->PlayPath(path, 1));

    // 检查状态
    std::this_thread::sleep_for(100ms);
    EXPECT_TRUE(robot->IsPathPlaying());

    // 强制停止
    EXPECT_TRUE(robot->StopPathPlayback());
    std::this_thread::sleep_for(200ms); // 等待线程退出
    EXPECT_FALSE(robot->IsPathPlaying());
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
