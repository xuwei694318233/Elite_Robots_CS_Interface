#include <gtest/gtest.h>
#include "EliteCSRobot.h"
#include <thread>
#include <chrono>

using namespace ROBOT;
using namespace std::chrono_literals;

static std::string s_robot_ip = "192.168.1.200";

class ConnectionTest : public ::testing::Test
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

// 1. 正常连接与断开
TEST_F(ConnectionTest, ConnectAndDisconnect)
{
    bool connected = robot->Connect(connConfig);
    ASSERT_TRUE(connected) << "Failed to connect to robot at " << s_robot_ip;
    EXPECT_TRUE(robot->IsConnected());

    bool disconnected = robot->Disconnect();
    EXPECT_TRUE(disconnected);
    EXPECT_FALSE(robot->IsConnected());
}

// 2. 连接测试接口 (TestConnection)
TEST_F(ConnectionTest, TestConnectionStatus)
{
    ASSERT_TRUE(robot->Connect(connConfig));

    InfoDict testRes;
    robot->TestConnection(testRes);

    ASSERT_TRUE(testRes.count("success"));
    EXPECT_TRUE(std::any_cast<bool>(testRes["success"]));
    EXPECT_TRUE(testRes.count("position"));
}

// 3. 重复连接测试
TEST_F(ConnectionTest, Reconnection)
{
    EXPECT_TRUE(robot->Connect(connConfig));
    EXPECT_TRUE(robot->Disconnect());

    std::this_thread::sleep_for(1s);

    EXPECT_TRUE(robot->Connect(connConfig));
    EXPECT_TRUE(robot->IsConnected());
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
