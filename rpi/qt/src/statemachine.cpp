#include "statemachine.h"
#include <QDebug>

void StateMachine::update_robot_state(uint16_t robot_id, const RobotState& state)
{
    QMutexLocker locker(&m_mutex); // Lock 획득 (스레드 안전)

    if (robot_id < robot_num) {
        robot_states[robot_id] = state;
        locker.unlock(); // UI 갱신 시그널 발생 전에 Lock 해제하여 Deadlock 방지
        emit robotStateChanged(robot_id);
    } else {
        qWarning() << "StateMachine::update_robot_state - Invalid robot_id:" << robot_id;
    }
}

void StateMachine::set_current_control_robot(uint16_t robot_id)
{
    QMutexLocker locker(&m_mutex);

    if (robot_id < robot_num) {
        if (current_control_robot != robot_id) {
            current_control_robot = robot_id;
            locker.unlock();
            emit controlRobotChanged(robot_id);
        }
    } else {
        qWarning() << "StateMachine::set_current_control_robot - Invalid robot_id:" << robot_id;
    }
}

uint16_t StateMachine::get_current_control_robot() const
{
    QMutexLocker locker(&m_mutex);
    return current_control_robot;
}

RobotState StateMachine::get_robot_state(uint16_t robot_id) const
{
    QMutexLocker locker(&m_mutex);
    
    if (robot_id < robot_num) {
        return robot_states[robot_id];
    } else {
        qWarning() << "StateMachine::get_robot_state - Invalid robot_id:" << robot_id;
        return RobotState{"", false, 0, 0, 0, 0, nullptr};
    }
}

void StateMachine::print_robot_states() const
{
    QMutexLocker locker(&m_mutex);

    qDebug() << "=== Current Robot States ===";
    qDebug() << "Current Control Target:" << current_control_robot;
    for (uint16_t i = 0; i < robot_num; ++i) {
        const auto& state = robot_states[i];
        qDebug() << "Robot" << i 
                 << "- Connected:" << state.connected 
                 << ", Pos: (" << state.pos_x << "," << state.pos_y << ")"
                 << ", Progress:" << state.mission_progress << "%"
                 << ", Battery:" << state.battery << "%";
    }
    qDebug() << "============================";
}

uint16_t StateMachine::get_robot_num() const
{
    return robot_num;
}
