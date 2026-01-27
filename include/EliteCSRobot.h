/*********************************************************************************
 * @copyright :
 * @file      : EliteCSRobot.h
 * @brief     : Elite CS系列机器人驱动文件
 * @author    : 许伟
 * @date      : 2026/01/08
 *********************************************************************************/

#pragma once
#include "RobotInterface.h"
#include <memory>
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>

#include <Elite/DataType.hpp>
#include <Elite/EliteDriver.hpp>
#include <Elite/DashboardClient.hpp>
#include <Elite/PrimaryPortInterface.hpp>
#include <Elite/RtsiIOInterface.hpp>

#include "RobotGlobal.h"

namespace ROBOT
{

    /**
     * @brief Elite 系列机器人实现
     */
    class ROBOT_EXPORT EliteCSRobot final : public IRobot
    {
    public:
        EliteCSRobot(const ELITE::EliteDriverConfig &config);
        ~EliteCSRobot() override;

        /*--------  生命周期  --------*/
        bool Connect(const ConfigDict &config) override;
        bool Disconnect() override;
        bool IsConnected() const override;

        /*--------  基本运动  --------*/
        bool MoveTo(double x, double y, double z,
                    double rx = 0, double ry = 0, double rz = 0) override;
        bool GetPosition(RobotPosition &outPos) const override;
        bool Home() override;
        bool EmergencyStop() override;
        bool SetSpeed(double percent_0_100) override;
        void GetInfo(InfoDict &outInfo) const override;
        void TestConnection(InfoDict &outResult) override;

        /*--------  实时控制  --------*/
        bool StartJogging(const std::string &axis) override;
        bool StopJogging() override;
        bool JogMove(const std::string &axis,
                     double speed, double distance) override;
        bool SetMotionMode(MotionMode mode) override;
        bool GetMotionMode(MotionMode &outMode) const override;
        RobotState GetState() const override;
        bool IsMoving() const override;

        /*--------  路径记录  --------*/
        bool StartPathRecording(const std::string &pathName) override;
        bool StopPathRecording() override;
        bool AddPathPoint(const PathPoint *point = nullptr) override;
        bool GetRecordedPath(RobotPath &outPath) override;
        bool ClearRecordedPath() override;

        /*--------  路径回放  --------*/
        bool PlayPath(const RobotPath &path, int loopCount = 1) override;
        bool StopPathPlayback() override;
        bool IsPathPlaying() const override;

        /*--------  高级控制  --------*/
        bool MoveLinear(const RobotPosition &start,
                        const RobotPosition &end,
                        double speed) override;
        bool MoveCircular(const RobotPosition &via,
                          const RobotPosition &end,
                          double speed) override;
        bool SetWorkCoordinateSystem(const WcsDict &wcs) override;
        bool GetWorkCoordinateSystem(WcsDict &outWcs) const override;
        bool ToggleWorkCoordinateSystem() override;

        /*--------  回调注册  --------*/
        bool RegisterPositionCallback(PositionCallback cb) override;
        bool RegisterStateCallback(StateCallback cb) override;
        bool UnregisterPositionCallback(PositionCallback cb) override;
        bool UnregisterStateCallback(StateCallback cb) override;

    public:
        bool MoveTrajectory(const std::vector<ELITE::vector6d_t> &trajectory, float pointTime, float blendRadius,
                            bool isCartesian);

    private:
        struct Config
        {
            std::string outputRecipePath;
            std::string inputRecipePath;
        };

    private:
        bool ParseConfig(const ConfigDict &configDict, Config &config);
        void PlaybackWorker(const RobotPath &path, int loopCount);
        std::string PositionInfo(const RobotPosition &pos);
        std::string PositionInfo(const ELITE::vector6d_t &pos);

    private:
        std::unique_ptr<ELITE::EliteDriver> m_driverPtr;
        std::unique_ptr<ELITE::DashboardClient> m_dashboardPtr;
        std::unique_ptr<ELITE::RtsiIOInterface> m_rtsiPtr;
        ELITE::EliteDriverConfig m_config;

        bool m_isConnect;
        RobotState m_robotState;
        MotionMode m_motionMode;

        /*--------  路径记录  --------*/
        std::mutex m_recordPathMutex;
        bool m_isRecordingPath;
        RobotPath m_recordPath;
        std::string m_recordPathName;

        /*--------  路径回放  --------*/
        std::atomic<bool> m_isPlaying;
        std::atomic<bool> m_stopRequested;
        std::thread m_playbackThread;

        /*--------  回调注册  --------*/
        std::unique_ptr<PositionCallback> m_posCallbackPtr;
        std::unique_ptr<StateCallback> m_stateCallbackPtr;

        static std::unordered_map<std::string, int> m_axisToIndex;
    };

} // namespace ROBOT
