#pragma once
#ifdef HAS_ROS
#include <ros/console.h>
#else
#define ROS_INFO_STREAM(...)
#define ROS_DEBUG_STREAM(...)
#define ROS_WARN(...)
#define ROS_ASSERT_MSG(...)
#define ROS_DEBUG(...)
#define ROS_INFO(...)
#define ROS_ERROR(...)
#define ROS_ASSERT(...)
#define ROS_BREAK(...)
#endif
