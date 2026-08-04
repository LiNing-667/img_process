/**
 * @file cmd_gateway.h
 * @brief 指令网关与接收线程 (写定的工作流在.cpp)
 */
#pragma once
#include <string>
#include "types.h"

void processTextCommand(const std::string &cmd_line);

// 视觉任务单槽提交协议 (解决 g_demo_task 多线程写竞争)：
// - task_try_submit:  外部/一次性来源（PC、终端、串口事件）使用；
//                     若队列中已有未消费任务则拒绝并返回 false，防止覆盖排队中的任务。
// - task_force_submit: 仅供 start 动作链等内部主流水线使用，可强制占用任务槽。
bool task_try_submit(const DemoTask &task);
void task_force_submit(const DemoTask &task);

namespace CommunicationManager
{
    void startThreads();
}