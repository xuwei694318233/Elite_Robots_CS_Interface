/*********************************************************************************
 * @copyright :
 * @file      : RobotInterface.h
 * @brief     : 机器人抽象接口头文件
 * @author    : 许伟
 * @date      : 2026/01/08
 *********************************************************************************/

#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <functional>
#include <any>

#include "RobotGlobal.h"

namespace ROBOT
{

    /*====================  数据结构  ====================*/
    struct RobotPosition
    {
        double x{}, y{}, z{};
        double rx{}, ry{}, rz{};
        double timestamp{};
    };

    struct PathPoint
    {
        RobotPosition position;
        double speed = 50.0;
        double delay = 0.0;
        std::string action;
    };

    struct RobotPath
    {
        std::string name;
        std::vector<PathPoint> points;
        double createdTime;
        std::string description;
        std::string id;
    };

    /*====================  枚举  ====================*/
    enum class RobotState
    {
        IDLE,
        MOVING,
        JOGGING,
        ERROR,
        EMERGENCY_STOP
    };

    enum class MotionMode
    {
        MANUAL,
        AUTOMATIC,
        JOG
    };

    /*====================  别名  ====================*/
    using ConfigDict = std::unordered_map<std::string, std::any>;
    using InfoDict = std::unordered_map<std::string, std::any>;
    using WcsDict = std::unordered_map<std::string, std::any>;

    using PositionCallback = std::function<void(const RobotPosition &)>;
    using StateCallback = std::function<void(RobotState)>;

    /*====================  抽象接口  ====================*/
    class ROBOT_EXPORT IRobot
    {
    public:
        virtual ~IRobot() = default;

        /*--------  生命周期  --------*/
        virtual bool Connect(const ConfigDict &config) = 0;
        virtual bool Disconnect() = 0;
        virtual bool IsConnected() const = 0;

        /*--------  基本运动  --------*/
        virtual bool MoveTo(double x, double y, double z,
                            double rx = 0, double ry = 0, double rz = 0) = 0;
        // false: 位置无效
        virtual bool GetPosition(RobotPosition &outPos) const = 0;
        virtual bool Home() = 0;
        virtual bool EmergencyStop() = 0;
        virtual bool SetSpeed(double percent_0_100) = 0;
        virtual void GetInfo(InfoDict &outInfo) const = 0;
        virtual void TestConnection(InfoDict &outResult) = 0;

        /*--------  实时控制  --------*/
        virtual bool StartJogging(const std::string &axis) = 0;
        virtual bool StopJogging() = 0;
        virtual bool JogMove(const std::string &axis,
                             double speed, double distance) = 0;
        virtual bool SetMotionMode(MotionMode mode) = 0;
        // false: 当前无有效模式
        virtual bool GetMotionMode(MotionMode &outMode) const = 0;
        virtual RobotState GetState() const = 0;
        virtual bool IsMoving() const = 0;

        /*--------  路径记录  --------*/
        virtual bool StartPathRecording(const std::string &pathName) = 0;
        virtual bool StopPathRecording() = 0;
        // nullptr: 记录当前位姿
        virtual bool AddPathPoint(const PathPoint *point = nullptr) = 0;
        // false: 当前无记录
        virtual bool GetRecordedPath(RobotPath &outPath) = 0;
        virtual bool ClearRecordedPath() = 0;

        /*--------  路径回放  --------*/
        virtual bool PlayPath(const RobotPath &path, int loopCount = 1) = 0;
        virtual bool StopPathPlayback() = 0;
        virtual bool IsPathPlaying() const = 0;

        /*--------  高级控制  --------*/
        virtual bool MoveLinear(const RobotPosition &start,
                                const RobotPosition &end,
                                double speed) = 0;
        virtual bool MoveCircular(const RobotPosition &via,
                                  const RobotPosition &end,
                                  double speed) = 0;
        virtual bool SetWorkCoordinateSystem(const WcsDict &wcs) = 0;
        // false: 当前无有效 WCS
        virtual bool GetWorkCoordinateSystem(WcsDict &outWcs) const = 0;
        virtual bool ToggleWorkCoordinateSystem() = 0;

        /*--------  回调注册  --------*/
        virtual bool RegisterPositionCallback(PositionCallback cb) = 0;
        virtual bool RegisterStateCallback(StateCallback cb) = 0;
        virtual bool UnregisterPositionCallback(PositionCallback cb) = 0;
        virtual bool UnregisterStateCallback(StateCallback cb) = 0;
    };

} // namespace ROBOT
