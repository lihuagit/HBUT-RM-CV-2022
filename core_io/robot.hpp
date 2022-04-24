/**
 * @file robot.hpp
 * @brief 
 * @author Lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2022-04-17
 * 
 */
#ifndef CVRM2022_ROBOT_HPP
#define CVRM2022_ROBOT_HPP

#include <cstdint>
#include <array>

enum class EnemyColor : uint8_t {
    // 敌方颜色
    RED = 0,
    BLUE = 1,
};

enum class ProgramMode : uint8_t {
    // 视觉模式
    AUTO_AIM = 1,       // 自瞄
    ANTIMISSLE = 2,     // 反导
    SMALL_ENERGY = 4,   // 小能量机关
    BIG_ENERGY = 8,     // 大能量机关
};

// 低5位发射标志位，高3位状态标识位
enum class ShootMode : uint8_t {
    // 射击标志位
    COMMON = 0,     // 普通模式
    DISTANT = 1,    // 远距离击打
    ANTITOP = 2,    // 反陀螺
    SWITCH = 4,     // 快速切换装甲板
    FOLLOW = 8,     // 跟随不发弹
    CRUISE = 16,    // 巡航
    EXIST_HERO = 32,// 英雄存在
};

enum class GameState : uint8_t {
    // 比赛模式
    SHOOT_NEAR_ONLY = 0,    // 仅射击近处
    SHOOT_FAR = 1,          // 允许远处射击
    COMMON = 255,           // 巡航
};

enum class Priority : uint8_t {
    CORE = 0,       // 核心优先级
    EMERGENCY = 1,  // 紧急情况
    NONE = 255,     // 无优先级
};

// 发送数据包
struct RobotCmd {
    uint8_t start = (unsigned)'s';
    float pitch_angle = 0;      // 单位：度
    float yaw_angle = 0;        // 单位：度
    float pitch_speed = 0;      // 单位：弧度/s
    float yaw_speed = 0;        // 单位：弧度/s
    uint8_t distance = 0;       // 计算公式 (int)(distance * 10)
    uint8_t lrc = 0;
    uint8_t end = (unsigned)'e';
} __attribute__((packed));


void background_robot_io_usb_auto_restart(const std::string &robot_usb_hid);

#endif //CVRM2022_ROBOT_HPP
