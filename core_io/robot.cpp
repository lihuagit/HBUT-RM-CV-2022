/**
 * @file robot.cpp
 * @brief 
 * @author Lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2022-04-17
 * 
 */
#include "robot.hpp"
#include <serial/serial.h>
#include <umt/umt.hpp>
#include <chrono>
#include <thread>
#include <iostream>
#include <bitset>
#include <iomanip>
#include <log.h>

using namespace serial;
using namespace std::chrono;

static bool debug = true;

void robot_cmd_loop(Serial &robot_port, const bool &required_stop, bool &is_ok) {
    /*
     * 将收到的数据统一发送出去
     */
    if (debug) std::cout << "============robot_cmd_loop===========\n";
    umt::Subscriber<RobotCmd> robot_cmd_sub("robot_cmd", 0);
    while (!required_stop) {
        try {
            auto robot_cmd = robot_cmd_sub.pop();
            // for(auto *ptr=(uint8_t*)&robot_cmd.priority; ptr < &robot_cmd.lrc; ptr++){
            //     robot_cmd.lrc += *ptr;
            // }
            try {
                robot_port.write((uint8_t *) &robot_cmd, sizeof(RobotCmd));
            } catch (SerialException &e) {
                LOGE("[ERROR] { %s }", e.what());
                is_ok = false;
                break;
            }
        } catch (umt::MessageError &e) {
            LOGE("robot_cmd_loop pop error");
            std::this_thread::sleep_for(100ms);
	    }
    }
}

bool robot_io_usb(const std::string &robot_usb_hid = "") {
    /*
     * 使用 ttl 模块发送数据
     */
    if (debug) std::cout << "============robot_io_usb===========\n";
    Serial robot_port;
    std::cout << "finding serial port" << std::endl;
    // 第一次运行程序是可以打印所有可用串口
    for (const auto &port_info : list_ports()) {
        std::cout << port_info.port << "|" << port_info.hardware_id << std::endl;
        if (port_info.hardware_id == robot_usb_hid) {
            robot_port.setPort(port_info.port);
            robot_port.setBaudrate(115200);  // 设置波特率
            auto timeout = Timeout::simpleTimeout(Timeout::max());
            robot_port.setTimeout(timeout);
            break;
        }
    }


    robot_port.open();
    if (!robot_port.isOpen()) {
        LOGE("[ERROR] (robot_io_usb): robot serial init fail!");
        return false;
    }
    LOGM("find serial port!");

    bool robot_cmd_required_stop = false;
    bool robot_cmd_is_ok = true;
    std::thread robot_cmd_thread(robot_cmd_loop, std::ref(robot_port),
                                 std::ref(robot_cmd_required_stop), std::ref(robot_cmd_is_ok));
    while(true){
        /**
         * @brief 后期视情况加入新功能
         */
    }
    robot_cmd_required_stop = true;
    robot_cmd_thread.join();

    return false;
}

void background_robot_io_usb_auto_restart(const std::string &robot_usb_hid = "") {
    if (debug) std::cout << "============background_robot_io_usb_auto_restart===========\n";
    std::thread([=]() {
        while (!robot_io_usb(robot_usb_hid)) {
            std::this_thread::sleep_for(500ms);
        }
    }).detach();
}

/*
 * umt 导出自定义结构体部分
 */

// UMT_EXPORT_OBJMANAGER_ALIAS(ShortRobotStatus, ShortRobotStatus, c) {
//     c.def_readwrite("enemy_color", &ShortRobotStatus::enemy_color);
//     c.def_readwrite("eject", &ShortRobotStatus::eject);
//     c.def_readwrite("target_id", &ShortRobotStatus::target_id);
// }

// UMT_EXPORT_OBJMANAGER_ALIAS(LongRobotStatus, LongRobotStatus, c) {
//     c.def_readwrite("program_mode", &LongRobotStatus::program_mode);
//     c.def_readwrite("robot_speed_mps", &LongRobotStatus::robot_speed_mps);
//     c.def_readwrite("cruise_speed", &LongRobotStatus::cruise_speed);
//     c.def_readwrite("enemy", &LongRobotStatus::enemy);
//     c.def_readwrite("game_state", &LongRobotStatus::game_state);
// }

// UMT_EXPORT_MESSAGE_ALIAS(RobotCmd, RobotCmd, c) {
//     c.def_readwrite("priority", &RobotCmd::priority);
//     c.def_readwrite("target_id", &RobotCmd::target_id);
//     c.def_readwrite("pitch_angle", &RobotCmd::pitch_angle);
//     c.def_readwrite("yaw_angle", &RobotCmd::yaw_angle);
//     c.def_readwrite("pitch_speed", &RobotCmd::pitch_speed);
//     c.def_readwrite("yaw_speed", &RobotCmd::yaw_speed);
//     c.def_readwrite("shoot_mode", &RobotCmd::shoot_mode);
// }

// PYBIND11_EMBEDDED_MODULE(RobotIO, m) {
//     namespace py = pybind11;
//     m.def("background_robot_io_4pin_auto_restart", background_robot_io_4pin_auto_restart,
//           py::arg("robot_port_name") = "");
//     m.def("background_robot_io_usb_auto_restart", background_robot_io_usb_auto_restart,
//           py::arg("robot_usb_hid") = "");
//     // 导出枚举类型
//     py::enum_<EnemyColor>(m ,"EnemyColor")
//             .value("RED", EnemyColor::RED)
//             .value("BLUE", EnemyColor::BLUE)
//             ;
//     py::enum_<ProgramMode>(m, "ProgramMode")
//             .value("AUTO_AIM", ProgramMode::AUTO_AIM)
//             .value("SMALL_ENERGY", ProgramMode::SMALL_ENERGY)
//             .value("BIG_ENERGY", ProgramMode::BIG_ENERGY)
//             ;
//     py::enum_<ShootMode>(m, "ShootMode")
//             .value("COMMON", ShootMode::COMMON)
//             .value("DISTANT", ShootMode::DISTANT)
//             .value("ANTITOP", ShootMode::ANTITOP)
//             .value("SWITCH", ShootMode::SWITCH)
//             .value("FOLLOW", ShootMode::FOLLOW)
//             .value("CRUISE", ShootMode::CRUISE)
//             ;
//     py::enum_<GameState>(m, "GameState")
//             .value("SHOOT_NEAR_ONLY", GameState::SHOOT_NEAR_ONLY)
//             .value("SHOOT_FAR", GameState::SHOOT_FAR)
//             .value("COMMON", GameState::COMMON)
//             ;
//     py::enum_<Priority>(m, "Priority")
//             .value("CORE", Priority::CORE)
//             .value("EMERGENCY", Priority::EMERGENCY)
//             .value("NONE", Priority::NONE)
//             ;
// }
