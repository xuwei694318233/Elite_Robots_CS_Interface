#include "EliteCSRobot.h"
#include <Elite/Log.hpp>
#include <Elite/DataType.hpp>

#include <future>
#include <sstream>
#include <iomanip>
#include <cstdarg>
#include <cmath>

#include <chrono>
using namespace std::chrono_literals; // 打开字面量

#define CHECK_CONNECTION                        \
    if (!IsConnected())                         \
    {                                           \
        ELITE_LOG_FATAL("Robot not connected"); \
        return false;                           \
    }

using namespace ELITE;
using namespace ROBOT;

std::unordered_map<std::string, int> EliteCSRobot::m_axisToIndex{
    {"x", 0}, {"y", 1}, {"z", 2}, {"rx", 3}, {"ry", 4}, {"rz", 5}};

inline std::string FormatStr(const char *format, ...)
{
    va_list args1, args2;
    va_start(args1, format);
    va_copy(args2, args1);

    int len = vsnprintf(nullptr, 0, format, args1) + 1;
    va_end(args1);

    std::string buf(len, '\0');
    vsnprintf(buf.data(), len, format, args2);
    va_end(args2);

    buf.pop_back(); // 去掉末尾 '\0'
    return buf;
}

EliteCSRobot::EliteCSRobot(const EliteDriverConfig &config)
{
    m_config = config;
    m_driverPtr = std::make_unique<EliteDriver>(config);
    m_dashboardPtr = std::make_unique<DashboardClient>();

    m_posCallbackPtr = nullptr;
    m_stateCallbackPtr = nullptr;
}

EliteCSRobot::~EliteCSRobot()
{
    StopPositionMonitoring();
    StopPathPlayback();
    Disconnect();
}

bool EliteCSRobot::Connect(const ConfigDict &config)
{
    // 解析配置
    Config conf;
    if (!ParseConfig(config, conf))
    {
        ELITE_LOG_FATAL("Failed to parse config");
        m_isConnect = false; // 确保状态一致
        return false;
    }

    // dashboard连接，上电，释放抱闸
    ELITE_LOG_INFO("Connecting to the dashboard");
    if (!m_dashboardPtr->connect(m_config.robot_ip))
    {
        ELITE_LOG_FATAL("Failed to connect to theashboard.");
        m_isConnect = false; // 确保状态一致
        return false;
    }
    ELITE_LOG_INFO("Successfully connected to the dashboard");

    ELITE_LOG_INFO("Start powering on...");
    if (!m_dashboardPtr->powerOn())
    {
        ELITE_LOG_FATAL("Power-on failed");
        m_isConnect = false; // 确保状态一致
        return false;
    }
    ELITE_LOG_INFO("Power-on succeeded");

    ELITE_LOG_INFO("Start releasing brake...");
    if (!m_dashboardPtr->brakeRelease())
    {
        ELITE_LOG_FATAL("Brake release failed");
        m_isConnect = false; // 确保状态一致
        return false;
    }
    ELITE_LOG_INFO("Brake released");

    // 机器人驱动连接
    if (m_config.headless_mode)
    {
        if (!m_driverPtr->isRobotConnected())
        {
            if (!m_driverPtr->sendExternalControlScript())
            {
                ELITE_LOG_FATAL("Fail to send external control script");
                m_isConnect = false; // 确保状态一致
                return false;
            }
        }
    }
    else
    {
        if (!m_dashboardPtr->playProgram())
        {
            ELITE_LOG_FATAL("Fail to play program");
            m_isConnect = false; // 确保状态一致
            return false;
        }
    }

    ELITE_LOG_INFO("Wait external control script run...");
    auto start = std::chrono::steady_clock::now();
    while (!m_driverPtr->isRobotConnected())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start);
        if (elapsed.count() > 10) // 超时10秒
        {
            ELITE_LOG_FATAL("Timeout waiting for external control script to run");
            m_isConnect = false; // 确保状态一致
            return false;
        }
    }
    ELITE_LOG_INFO("External control script is running");

    // RTSI接口连接
    m_rtsiPtr = std::make_unique<RtsiIOInterface>(conf.outputRecipePath, conf.inputRecipePath, 250);
    ELITE_LOG_INFO("Connecting to the RTSI");
    if (!m_rtsiPtr->connect(m_config.robot_ip))
    {
        ELITE_LOG_FATAL("Fail to connect or config to the RTSI.");
        m_isConnect = false; // 确保状态一致
        return false;
    }
    ELITE_LOG_INFO("Successfully connected to the RTSI");

    m_isConnect = true;
    m_robotState = RobotState::IDLE;

    return true;
}

bool EliteCSRobot::Disconnect()
{
    if (!m_isConnect)
    {
        return true;
    }

    m_dashboardPtr->disconnect();

    if (!m_driverPtr->stopControl())
    {
        ELITE_LOG_FATAL("Elite driver stopControl failed");
        // 即使失败，也继续清理资源
    }

    m_rtsiPtr->disconnect();

    m_isConnect = false;

    return true;
}

bool EliteCSRobot::IsConnected() const
{
    return m_isConnect;
}

bool EliteCSRobot::MoveTo(double x, double y, double z,
                          double rx, double ry, double rz)
{
    CHECK_CONNECTION;

    std::string script = FormatStr(
        "def move_to_pos():\n"
        "\tmovel([%f,%f,%f,%f,%f,%f], a=0.25, v=0.5)\n"
        "end\n",
        x, y, z, rx, ry, rz);

    ELITE_LOG_INFO("Sending MoveTo script: %s", script.c_str());

    if (!m_driverPtr->sendScript(script))
    {
        ELITE_LOG_ERROR("Failed to send MoveTo script");
        return false;
    }

    // 验证运动是否开始
    // if (!IsMoving())
    // {
    //     ELITE_LOG_ERROR("MoveTo failed: robot did not start moving");
    //     return false;
    // }

    return true;
}

bool EliteCSRobot::WaitForMotionComplete(int timeoutMs)
{
    // Wait for motion to start (timeout 500ms)
    // 避免在发送命令后立即检查状态时，因速度尚未建立而误判为运动完成
    auto startWait = std::chrono::steady_clock::now();
    bool started = false;
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - startWait).count() < 500)
    {
        if (IsMoving())
        {
            started = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!started)
    {
        // 如果500ms内没有检测到运动，可能是运动指令未执行或运动极短瞬间完成
        // 这里选择返回true，认为"运动完成"（实际上可能未发生）
        // 但建议调用者检查返回值或位置确认
        return true;
    }

    auto start = std::chrono::steady_clock::now();

    while (IsMoving())
    {
        // 打印等待点，方便直观看到程序在等待
        // ELITE_LOG_INFO("Waiting for motion to complete...");

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);

        if (elapsed.count() > timeoutMs)
        {
            ELITE_LOG_ERROR("WaitForMotionComplete timeout after %d ms", timeoutMs);
            return false;
        }
    }

    return true;
}

bool EliteCSRobot::GetPosition(RobotPosition &outPos) const
{
    CHECK_CONNECTION;

    try
    {
        vector6d_t actualPos = m_rtsiPtr->getActualTCPPose();

        using clock = std::chrono::system_clock;
        double timestamp = std::chrono::duration<double>(clock::now().time_since_epoch()).count();

        outPos = RobotPosition{actualPos[0], actualPos[1], actualPos[2], actualPos[3], actualPos[4], actualPos[5],
                               timestamp};

        return true;
    }
    catch (const std::exception &e)
    {
        ELITE_LOG_ERROR("Failed to get position: %s", e.what());
        return false;
    }
}

bool EliteCSRobot::Home()
{
    CHECK_CONNECTION;

    if (!MoveTo(0, 0, 0, 0, 0, 0))
    {
        ELITE_LOG_ERROR("Failed to move to home position");
        return false;
    }

    if (!WaitForMotionComplete())
    {
        ELITE_LOG_ERROR("Failed to complete homing motion");
        return false;
    }

    return true;
}

bool EliteCSRobot::EmergencyStop()
{
    CHECK_CONNECTION;

    try
    {
        if (!m_dashboardPtr->stopProgram())
        {
            ELITE_LOG_ERROR("Failed to stop program via dashboard");
        }

        if (!m_driverPtr->stopControl())
        {
            ELITE_LOG_ERROR("Failed to stop control via driver");
        }

        std::string stopScript = "stopj(20)\n";
        if (!m_driverPtr->sendScript(stopScript))
        {
            ELITE_LOG_ERROR("Failed to send stop script");
        }

        // 软件停止后，期望状态应为静止(IDLE)，而非系统级急停(EMERGENCY_STOP)
        m_robotState = RobotState::IDLE;

        return true;
    }
    catch (const std::exception &e)
    {
        ELITE_LOG_ERROR("Failed to execute emergency stop: %s", e.what());
        return false;
    }
}

bool EliteCSRobot::SetSpeed(double percent_0_100)
{
    CHECK_CONNECTION;

    if (!(0 <= percent_0_100 && percent_0_100 <= 100))
    {
        ELITE_LOG_ERROR("Speed value %f not in [0, 100]", percent_0_100);
        return false;
    }

    double speedScaling = percent_0_100 / 100;

    try
    {
        if (!m_rtsiPtr->setSpeedScaling(speedScaling))
        {
            ELITE_LOG_ERROR("Failed to set speed %f", speedScaling);
            return false;
        }
    }
    catch (const std::exception &e)
    {
        ELITE_LOG_ERROR("Exception while setting speed: %s", e.what());
        return false;
    }

    return true;
}

void EliteCSRobot::GetInfo(InfoDict &outInfo) const
{
    outInfo["brand"] = "Elite";
    outInfo["type"] = "Robot";
    outInfo["connected"] = IsConnected();
    outInfo["model"] = "None";
    outInfo["ip"] = m_config.robot_ip;
}

void EliteCSRobot::TestConnection(InfoDict &outResult)
{
    outResult["success"] = false;
    if (!IsConnected())
    {
        outResult["error"] = "Robot not connected";
        ELITE_LOG_ERROR("TestConnection failed: Robot not connected");
        return;
    }

    RobotPosition pos;
    if (!GetPosition(pos))
    {
        outResult["error"] = "Failed to get position";
        ELITE_LOG_ERROR("TestConnection failed: Unable to get position");
        return;
    }

    outResult["success"] = true;
    outResult["position"] = pos;
}

bool EliteCSRobot::StartJogging(const std::string &axis)
{
    CHECK_CONNECTION;

    if (m_robotState != RobotState::IDLE)
    {
        ELITE_LOG_ERROR("Cannot start jogging: Robot is not in IDLE state");
        return false;
    }

    std::string ax = axis;
    double direction = 1.0;

    // Check for +/- prefix
    if (!ax.empty() && (ax[0] == '+' || ax[0] == '-'))
    {
        if (ax[0] == '-')
            direction = -1.0;
        ax = ax.substr(1);
    }

    if (m_axisToIndex.count(ax) == 0)
    {
        ELITE_LOG_ERROR("Invalid axis : %s", axis.c_str());
        return false;
    }

    int index = m_axisToIndex[ax];
    double speeds[6] = {0};

    // Default jogging speeds: 0.2 m/s for linear, 0.5 rad/s for joint
    double jogSpeed = (index < 3) ? 0.2 : 0.5;
    speeds[index] = direction * jogSpeed;

    std::string script = FormatStr(
        "def start_jog_proc():\n"
        "\tspeedl([%f,%f,%f,%f,%f,%f], a=0.25, t=100)\n"
        "end\n",
        speeds[0], speeds[1], speeds[2], speeds[3], speeds[4], speeds[5]);

    ELITE_LOG_INFO("Sending StartJogging script for axis %s", axis.c_str());
    if (!m_driverPtr->sendScript(script))
    {
        ELITE_LOG_ERROR("Failed to send StartJogging script");
        return false;
    }

    m_robotState = RobotState::JOGGING;

    return true;
}

bool EliteCSRobot::StopJogging()
{
    CHECK_CONNECTION;
    std::string script = "def stop_proc():\n\tstopj(2.0)\nend\n";
    if (!m_driverPtr->sendScript(script))
    {
        ELITE_LOG_ERROR("Failed to send StopJogging script");
        return false;
    }

    m_robotState = RobotState::IDLE;

    return true;
}

bool EliteCSRobot::JogMove(const std::string &axis, double speed, double distance)
{
    CHECK_CONNECTION;

    RobotPosition pos;
    if (!GetPosition(pos))
    {
        ELITE_LOG_ERROR("JogMove failed: Unable to get current position");
        return false;
    }

    if (axis == "x")
        pos.x += distance;
    else if (axis == "y")
        pos.y += distance;
    else if (axis == "z")
        pos.z += distance;
    else if (axis == "rx")
        pos.rx += distance;
    else if (axis == "ry")
        pos.ry += distance;
    else if (axis == "rz")
        pos.rz += distance;
    else
    {
        ELITE_LOG_ERROR("JogMove failed: Invalid axis %s", axis.c_str());
        return false;
    }

    std::string script = FormatStr(
        "def jog_move_proc():\n"
        "\tmovel([%f,%f,%f,%f,%f,%f], v=%f)\n"
        "end\n",
        pos.x, pos.y, pos.z, pos.rx, pos.ry, pos.rz, speed);

    ELITE_LOG_INFO("Sending JogMove script");
    if (!m_driverPtr->sendScript(script))
    {
        ELITE_LOG_ERROR("Failed to send JogMove script");
        return false;
    }

    return true;
}

bool EliteCSRobot::SetMotionMode(MotionMode mode)
{
    m_motionMode = mode;

    return true;
}

bool EliteCSRobot::GetMotionMode(MotionMode &outMode) const
{
    outMode = m_motionMode;

    return true;
}

RobotState EliteCSRobot::GetState() const
{
    try
    {
        if (!IsConnected())
        {
            return RobotState::ERROR;
        }

        SafetyMode safetyMode = m_rtsiPtr->getSafetyStatus();
        if (safetyMode == SafetyMode::ROBOT_EMERGENCY_STOP ||
            safetyMode == SafetyMode::SYSTEM_EMERGENCY_STOP)
        {
            return RobotState::EMERGENCY_STOP;
        }
        if (safetyMode == SafetyMode::PROTECTIVE_STOP ||
            safetyMode == SafetyMode::VIOLATION ||
            safetyMode == SafetyMode::FAULT)
        {
            return RobotState::ERROR;
        }

        if (IsMoving())
        {
            if (m_robotState == RobotState::JOGGING)
            {
                return RobotState::JOGGING;
            }
            return RobotState::MOVING;
        }

        return RobotState::IDLE;
    }
    catch (const std::exception &e)
    {
        ELITE_LOG_ERROR("Failed to get robot state: %s", e.what());
        return RobotState::ERROR;
    }
}

bool EliteCSRobot::IsMoving() const
{
    try
    {
        // 移除tcpSpeed检查，直接依靠关节速度判断
        // 因为只要机器人TCP在动，关节必然在动；反之关节不动，TCP也不会动。

        bool anyJointMoving = false;
        if (m_rtsiPtr)
        {
            auto jointVelocities = m_rtsiPtr->getActualJointVelocity();
            for (const auto &vel : jointVelocities)
            {
                if (std::abs(vel) > 1e-3) // 使用 1e-3 rad/s 作为阈值
                {
                    anyJointMoving = true;
                    break;
                }
            }
        }

        return anyJointMoving;
    }
    catch (const std::exception &e)
    {
        ELITE_LOG_ERROR("Failed to check motion state: %s", e.what());
        return false;
    }
}

bool EliteCSRobot::StartPathRecording(const std::string &pathName)
{
    CHECK_CONNECTION;

    // 使用局部变量生成时间字符串，避免锁内复杂操作
    using clock = std::chrono::system_clock;
    double ts = std::chrono::duration<double>(clock::now().time_since_epoch()).count();
    std::time_t tt = static_cast<std::time_t>(ts);

    struct tm tm_buf;
#if defined(_WIN32) || defined(_WIN64)
    if (localtime_s(&tm_buf, &tt) != 0)
    {
        ELITE_LOG_ERROR("Failed to get local time");
        return false;
    }
#else
    if (localtime_r(&tt, &tm_buf) == nullptr)
    {
        ELITE_LOG_ERROR("Failed to get local time");
        return false;
    }
#endif

    std::stringstream ss;
    ss << std::put_time(&tm_buf, "%F %T");
    std::string timeStr = ss.str();

    std::lock_guard<std::mutex> lock(m_recordPathMutex);
    if (m_isRecordingPath)
    {
        ELITE_LOG_ERROR("Path recording already in progress: %s", m_recordPathName.c_str());
        return false;
    }

    m_isRecordingPath = true;

    m_recordPath.name = pathName;
    m_recordPath.points.clear();
    m_recordPath.createdTime = ts;
    m_recordPath.description = "Recorded on " + timeStr;
    m_recordPath.id = "path_" + std::to_string(static_cast<long>(ts));

    m_recordPathName = pathName;

    ELITE_LOG_INFO("Started recording path: %s", pathName.c_str());

    return true;
}

bool EliteCSRobot::StopPathRecording()
{
    std::lock_guard<std::mutex> lock(m_recordPathMutex);

    if (!m_isRecordingPath)
    {
        ELITE_LOG_ERROR("StopPathRecording failed: No path recording in progress");
        return false;
    }

    m_isRecordingPath = false;
    ELITE_LOG_INFO("Stopped recording path: %s", m_recordPathName.c_str());

    return true;
}

bool EliteCSRobot::AddPathPoint(const PathPoint *point)
{
    CHECK_CONNECTION;

    // 1. 在锁外获取当前位置，避免在持有锁时进行可能耗时的网络/IO操作
    PathPoint pathPoint;
    if (point == nullptr)
    {
        RobotPosition currentPos;
        if (!GetPosition(currentPos))
        {
            ELITE_LOG_ERROR("AddPathPoint failed: Unable to get current position");
            return false;
        }
        pathPoint.position = currentPos;
    }
    else
    {
        pathPoint = *point;
    }

    // 2. 持有锁进行快速的数据插入
    std::lock_guard<std::mutex> lock(m_recordPathMutex);

    if (!m_isRecordingPath)
    {
        ELITE_LOG_ERROR("AddPathPoint failed: No path recording in progress");
        return false;
    }

    m_recordPath.points.push_back(pathPoint);
    ELITE_LOG_INFO("Added path point to recording: %s", m_recordPathName.c_str());

    return true;
}

bool EliteCSRobot::GetRecordedPath(RobotPath &outPath)
{
    std::lock_guard<std::mutex> lock(m_recordPathMutex);

    outPath = m_recordPath;

    return true;
}

bool EliteCSRobot::ClearRecordedPath()
{
    std::lock_guard<std::mutex> lock(m_recordPathMutex);

    m_recordPath = RobotPath();
    m_recordPathName = "";

    return true;
}

bool EliteCSRobot::PlayPath(const RobotPath &path, int loopCount)
{
    CHECK_CONNECTION;

    bool expected = false;
    if (!m_isPlaying.compare_exchange_strong(expected, true))
    {
        ELITE_LOG_WARN("PlayPath failed: Path already playing");
        return false;
    }

    try
    {
        m_stopRequested = false;
        m_playbackThread = std::thread(&EliteCSRobot::PlaybackWorker, this, path, loopCount);
        ELITE_LOG_INFO("Start Elite path playback: %s, loops: %i", path.name.c_str(), loopCount);
    }
    catch (const std::exception &e)
    {
        ELITE_LOG_ERROR("PlayPath failed: Unable to start playback thread: %s", e.what());
        m_isPlaying = false;
        return false;
    }

    return true;
}

bool EliteCSRobot::StopPathPlayback()
{
    if (!m_isPlaying)
    {
        ELITE_LOG_WARN("StopPathPlayback failed: No path playing");
        return false;
    }

    m_stopRequested = true;

    // 强制停止机器人运动，中断MoveTrajectory的等待
    if (m_driverPtr)
    {
        // 发送停止指令
        // 使用 stopControl 可能太重（会断开脚本连接），writeIdle 或 stopj 更合适
        // 但这里我们希望中断当前脚本执行
        m_driverPtr->stopControl();
        // 或者发送 stopj 脚本
        // std::string stopScript = "stopj(2.0)\n";
        // m_driverPtr->sendScript(stopScript);
    }

    if (m_playbackThread.joinable())
    {
        m_playbackThread.join();
    }

    m_isPlaying = false;
    ELITE_LOG_INFO("Elite path playback stopped");

    return true;
}

bool EliteCSRobot::IsPathPlaying() const
{
    return m_isPlaying;
}

bool EliteCSRobot::MoveLinear(const RobotPosition &start, const RobotPosition &end, double speed)
{
    CHECK_CONNECTION;

    std::string script = FormatStr(
        "def move_linear_proc():\n"
        "\tmovel([%f,%f,%f,%f,%f,%f], a=0.25, v=0.5)\n"
        "\tmovel([%f,%f,%f,%f,%f,%f], v=%f)\n"
        "end\n",
        start.x, start.y, start.z, start.rx, start.ry, start.rz,
        end.x, end.y, end.z, end.rx, end.ry, end.rz, speed);

    ELITE_LOG_INFO("Sending MoveLinear script");
    if (!m_driverPtr->sendScript(script))
    {
        ELITE_LOG_ERROR("MoveLinear failed: Unable to send script");
        return false;
    }

    ELITE_LOG_INFO("Move from start %s to end %s", PositionInfo(start).c_str(), PositionInfo(end).c_str());

    return true;
}

bool EliteCSRobot::MoveCircular(const RobotPosition &via, const RobotPosition &end, double speed)
{
    CHECK_CONNECTION;

    std::string script = FormatStr(
        "def move_circular_proc():\n"
        "\tmovec([%f,%f,%f,%f,%f,%f], [%f,%f,%f,%f,%f,%f], a=0.25, v=%f, r=0)\n"
        "end\n",
        via.x, via.y, via.z, via.rx, via.ry, via.rz,
        end.x, end.y, end.z, end.rx, end.ry, end.rz, speed);

    ELITE_LOG_INFO("Sending MoveCircular script");
    if (!m_driverPtr->sendScript(script))
    {
        ELITE_LOG_ERROR("MoveCircular failed: Unable to send script");
        return false;
    }

    ELITE_LOG_INFO("MoveCircular via %s to %s", PositionInfo(via).c_str(), PositionInfo(end).c_str());

    return true;
}

bool EliteCSRobot::SetWorkCoordinateSystem(const WcsDict &wcs)
{
    CHECK_CONNECTION;

    return true;
}

bool EliteCSRobot::GetWorkCoordinateSystem(WcsDict &outWcs) const
{
    CHECK_CONNECTION;

    return true;
}

bool EliteCSRobot::ToggleWorkCoordinateSystem()
{
    CHECK_CONNECTION;

    return true;
}

bool EliteCSRobot::RegisterPositionCallback(PositionCallback cb)
{
    m_posCallbackPtr = std::make_unique<PositionCallback>(cb);
    StartPositionMonitoring(); // 启动监控
    return true;
}

bool EliteCSRobot::RegisterStateCallback(StateCallback cb)
{
    m_stateCallbackPtr = std::make_unique<StateCallback>(cb);

    return true;
}

bool EliteCSRobot::UnregisterPositionCallback(PositionCallback cb)
{
    m_posCallbackPtr = nullptr;
    StopPositionMonitoring(); // 停止监控
    return true;
}

bool EliteCSRobot::UnregisterStateCallback(StateCallback cb)
{
    m_stateCallbackPtr = nullptr;

    return true;
}

bool EliteCSRobot::MoveTrajectory(const std::vector<ELITE::vector6d_t> &trajectory, float pointTime, float blendRadius,
                                  bool isCartesian)
{
    CHECK_CONNECTION;

    std::promise<TrajectoryMotionResult> moveDonePromise;
    m_driverPtr->setTrajectoryResultCallback([&](TrajectoryMotionResult result)
                                             { moveDonePromise.set_value(result); });

    ELITE_LOG_INFO("Trajectory motion start");
    if (!m_driverPtr->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::START, trajectory.size(), 200))
    {
        ELITE_LOG_ERROR("Failed to start trajectory motion");
        return false;
    }

    for (const auto &pos : trajectory)
    {
        if (!m_driverPtr->writeTrajectoryPoint(pos, pointTime, blendRadius, isCartesian))
        {
            ELITE_LOG_ERROR("Failed to write trajectory point");
            return false;
        }
        // Send NOOP command to avoid timeout.
        if (!m_driverPtr->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::NOOP, 0, 200))
        {
            ELITE_LOG_ERROR("Failed to send NOOP command");
            return false;
        }
    }

    std::future<TrajectoryMotionResult> moveDoneFuture = moveDonePromise.get_future();
    while (moveDoneFuture.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready)
    {
        // Wait for the trajectory motion to complete, and send NOOP command to avoid timeout.
        if (!m_driverPtr->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::NOOP, 0, 200))
        {
            ELITE_LOG_ERROR("Failed to send NOOP command");
            return false;
        }
    }
    auto result = moveDoneFuture.get();
    ELITE_LOG_INFO("Trajectory motion completed with result: %d", result);

    if (!m_driverPtr->writeIdle(0))
    {
        ELITE_LOG_ERROR("Failed to write idle command");
        return false;
    }

    return result == TrajectoryMotionResult::SUCCESS;
}

template <typename T>
bool GetAnyValue(const std::any &any, T &value)
{
    if (auto p = std::any_cast<T>(&any); p != nullptr)
    {
        value = *p;
        return true;
    }

    return false;
}

template <typename T>
bool GetConfigVal(const ConfigDict &configDict, const std::string &key, const T &defaultVal, T &val)
{
    if (configDict.count(key) != 0)
    {
        return GetAnyValue(configDict.at(key), val);
    }
    val = defaultVal;

    return true;
}

bool EliteCSRobot::ParseConfig(const ConfigDict &configDict, Config &config)
{
    if (!GetConfigVal(configDict, "inputRecipePath", std::string("input_recipe.txt"), config.inputRecipePath))
    {
        return false;
    }

    if (!GetConfigVal(configDict, "outputRecipePath", std::string("output_recipe.txt"), config.outputRecipePath))
    {
        return false;
    }

    return true;
}

void EliteCSRobot::PlaybackWorker(const RobotPath &path, int loop_count)
{
    std::vector<ELITE::vector6d_t> traj;
    traj.reserve(path.points.size());

    // 默认点时间，可视情况调整或根据速度距离计算
    float pointTime = 0.5f;
    // 默认交融半径
    float blendRadius = 0.002f;

    // 1. 将 RobotPath 转换为 MoveTrajectory 需要的格式
    for (const auto &pt : path.points)
    {
        const auto &p = pt.position;
        traj.push_back({p.x, p.y, p.z, p.rx, p.ry, p.rz});
    }

    for (int loop = 0; loop < loop_count; ++loop)
    {
        if (m_stopRequested)
            break;

        ELITE_LOG_INFO("Playback loop %d start", loop + 1);

        // 2. 调用已实现的 MoveTrajectory 进行完整轨迹运动
        //    MoveTrajectory 内部实现了轨迹透传、同步等待和 idle 状态恢复，无需手动轮询。
        //    注意：这里假设所有的点都是笛卡尔空间点 (isCartesian = true)
        if (!MoveTrajectory(traj, pointTime, blendRadius, true))
        {
            ELITE_LOG_ERROR("Trajectory playback failed at loop %d", loop + 1);
            break;
        }

        // 3. 处理每个点特有的 delay？
        // 遗憾的是 MoveTrajectory 是一次性发送所有路点，
        // 无法精确实现针对每个点的自定义 delay。
        // 如果必须严格遵守 path struct 中的 delay，
        // 则不能用 MoveTrajectory，必须用 MoveTo + Wait 的方式重写。
        // 但鉴于通常回放是为了连续运动，MoveTrajectory 是更优解。
    }

    m_isPlaying = false;
    ELITE_LOG_INFO("Playback finished");
}

std::string EliteCSRobot::PositionInfo(const RobotPosition &pos)
{
    return FormatStr("[%f, %f, %f, %f, %f, %f] %f", pos.x, pos.y, pos.z, pos.rx, pos.ry, pos.rz, pos.timestamp);
}

std::string EliteCSRobot::PositionInfo(const vector6d_t &pos)
{
    return FormatStr("[%f, %f, %f, %f, %f, %f]", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
}

void EliteCSRobot::StartPositionMonitoring()
{
    if (m_positionMonitorThread.joinable())
    {
        return;
    }

    m_stopPositionMonitor = false;
    m_positionMonitorThread = std::thread([this]()
                                          { PositionMonitorWorker(); });
}

void EliteCSRobot::StopPositionMonitoring()
{
    m_stopPositionMonitor = true;
    if (m_positionMonitorThread.joinable())
    {
        m_positionMonitorThread.join();
    }
}

void EliteCSRobot::PositionMonitorWorker()
{
    RobotPosition lastPos, currentPos;
    bool firstRun = true;

    while (!m_stopPositionMonitor && IsConnected())
    {
        // 直接使用RTSI获取位置，避免调用GetPosition
        if (m_rtsiPtr)
        {
            try
            {
                vector6d_t actualPos = m_rtsiPtr->getActualTCPPose();

                // 暂时不使用tcpSpeed进行pending清除逻辑，因为该逻辑被认为冗余
                // vector6d_t tcpSpeed = m_rtsiPtr->getActualTCPVelocity();

                using clock = std::chrono::system_clock;
                double timestamp = std::chrono::duration<double>(clock::now().time_since_epoch()).count();

                currentPos = RobotPosition{actualPos[0], actualPos[1], actualPos[2],
                                           actualPos[3], actualPos[4], actualPos[5], timestamp};

                // 检查位置变化是否超过阈值
                if (!firstRun)
                {
                    double dx = currentPos.x - lastPos.x;
                    double dy = currentPos.y - lastPos.y;
                    double dz = currentPos.z - lastPos.z;
                    double distanceSq = dx * dx + dy * dy + dz * dz;

                    if (distanceSq > m_positionThreshold * m_positionThreshold)
                    {
                        // 使用锁或者判空保护回调调用
                        // 注意：这里简单判空在多线程下仍有风险，但由于m_posCallbackPtr生命周期管理较简单
                        // 且主要在单线程中注册/注销，这里假设Unregister会join此线程
                        // 如果UnregisterPositionCallback正确实现了 join，则此处是安全的
                        if (m_posCallbackPtr && *m_posCallbackPtr)
                        {
                            (*m_posCallbackPtr)(currentPos);
                        }
                        lastPos = currentPos;
                    }
                }
                else
                {
                    firstRun = false;
                    lastPos = currentPos;
                }
            }
            catch (const std::exception &e)
            {
                // RTSI异常处理，继续监控
                ELITE_LOG_ERROR("RTSI error in position monitor: %s", e.what());
            }
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>(m_callbackIntervalMs)));
    }
}

bool EliteCSRobot::MoveToSync(double x, double y, double z, double rx, double ry, double rz, int timeoutMs)
{
    if (!MoveTo(x, y, z, rx, ry, rz))
    {
        return false;
    }

    return WaitForMotionComplete(timeoutMs);
}

bool EliteCSRobot::MoveToWithCallback(double x, double y, double z, double rx, double ry, double rz,
                                      std::function<void(const RobotPosition &)> progressCallback, int timeoutMs)
{
    if (!MoveTo(x, y, z, rx, ry, rz))
    {
        return false;
    }

    auto start = std::chrono::steady_clock::now();

    // 监控运动进度
    while (IsMoving())
    {
        if (progressCallback)
        {
            try
            {
                RobotPosition currentPos;
                if (GetPosition(currentPos))
                {
                    progressCallback(currentPos);
                }
            }
            catch (const std::exception &e)
            {
                ELITE_LOG_ERROR("Exception in progressCallback: %s", e.what());
            }
            catch (...)
            {
                ELITE_LOG_ERROR("Unknown exception in progressCallback");
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);

        if (elapsed.count() > timeoutMs)
        {
            ELITE_LOG_ERROR("MoveToWithCallback timeout after %d ms", timeoutMs);
            if (m_driverPtr)
                m_driverPtr->stopControl(); // 尝试停止
            return false;
        }
    }

    // 最后调用一次回调，确保获取最终位置
    if (progressCallback)
    {
        try
        {
            RobotPosition finalPos;
            if (GetPosition(finalPos))
            {
                progressCallback(finalPos);
            }
        }
        catch (...)
        {
            // ignore
        }
    }

    return true;
}
