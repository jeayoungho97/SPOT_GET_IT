#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <QObject>
#include <QMutex>
#include <QMutexLocker>
#include <vector>
#include <string>

extern "C" {
#include "shm_def.h"
}

typedef struct RobotState {
    std::string robot_name;
    bool connected;
    uint16_t pos_x;
    uint16_t pos_y;
    uint16_t mission_progress;
    uint16_t battery;
    SharedData* robot_shm_data;
} RobotState;

class StateMachine : public QObject {
    Q_OBJECT

private:
    uint16_t robot_num;
    uint16_t current_control_robot;
    std::vector<RobotState> robot_states;
    std::vector<std::string> event_img_path;
    std::vector<std::string> event_img_msg;
    mutable QMutex m_mutex;

public:
    explicit StateMachine(QObject *parent = nullptr){
        this->robot_num = 0;
        this->current_control_robot = 65535;
        this->event_img_path.clear();
        this->robot_states.clear();
    }

    ~StateMachine() override = default;

   


    ////////////////////////////////////////// event img path functions
    void add_event_img_path(const std::string& path,const std::string& msg){
        QMutexLocker locker(&m_mutex);
        this->event_img_path.push_back(path);
        this->event_img_msg.push_back(msg);
        locker.unlock();
        emit eventAdded(QString::fromStdString(msg));
    }
    
    std::string get_event_img_path(const uint16_t idx) const{
        QMutexLocker locker(&m_mutex);
        if(idx >= event_img_path.size()){
            return "";
        }
        return this->event_img_path[idx];
    }

    void clear_event_img_path(){
        QMutexLocker locker(&m_mutex);
        this->event_img_path.clear();
    }
    ////////////////////////////////////////// end event img path functions



    ////////////////////////////////////////// robot state functions
    void add_robot(const RobotState& state,SharedData* shm_ptr){
        QMutexLocker locker(&m_mutex);

        this->robot_states.push_back(state);
        this->robot_states[this->robot_num].robot_shm_data = shm_ptr;
        uint16_t new_id = this->robot_num;
        this->robot_num++;
        
        locker.unlock();
        emit robotAdded(new_id, QString::fromStdString(state.robot_name));
    }


    void set_current_control_robot(uint16_t robot_id);
    uint16_t get_current_control_robot() const;
    RobotState get_robot_state(uint16_t robot_id) const;
    void print_robot_states() const;
    
    uint16_t get_robot_num() const;
    void update_robot_state(uint16_t robot_id, const RobotState& state);

signals:
    // 상태가 업데이트되었을 때 UI 갱신을 위한 시그널
    void robotAdded(uint16_t robot_id, QString name);
    void robotStateChanged(uint16_t robot_id);
    void controlRobotChanged(uint16_t robot_id);
    void eventAdded(QString msg);
};

#endif // STATEMACHINE_H