/*********************************************************************************
 * @copyright : 
 * @file      : RobotGlobal.h
 * @brief     : 机器人全局头文件
 * @author    : 许伟
 * @date      : 2026/01/08
 *********************************************************************************/

#pragma once

#if defined(_WIN32) || defined(_WIN64)
	#if defined(ROBOT_STATIC_LIBRARY)
        #define ROBOT_EXPORT
	#elif defined ROBOT_EXPORT_LIBRARY
		#define ROBOT_EXPORT __declspec(dllexport)
	#else
		#define ROBOT_EXPORT __declspec(dllimport)
	#endif
#elif defined(__linux) || defined(linux) || defined(__linux__) || defined(__APPLE__)
	#define ROBOT_EXPORT __attribute__((visibility("default")))
#endif
