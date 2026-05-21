/*
- 경로: spot_navigation/src/local_path_planner_node.cpp
- 역할:
    PathProgress, localization pose, LiDAR Perception 정보를 기반으로
    follower/controller가 따라갈 짧은 local path를 생성하는 노드

- Input:
    - /navigation/path_progress/spot_01 (Type: robot_interfaces/msg/PathProgress)
    - /localization/mock_pose 또는 /localization/pose (Type: robot_interfaces/msg/LocalizedRobotPose)
    - /perception/lidar/obstacle_model (Type: robot_interfaces/msg/ObstacleModel)
    - /perception/lidar/free_space_model (Type: robot_interfaces/msg/FreeSpaceModel)

- Output:
    - /navigation/local_path/spot_01 (Type: nav_msgs/msg/Path)
    - /navigation/local_planner_status/spot_01 (Type: robot_interfaces/msg/LocalPlannerStatus)

- 주요 기능:
    - PathProgress 입력 수신 여부와 유효성을 검사한다.
    - localization pose 입력 수신 여부와 유효성을 검사한다.
    - ObstacleModel / FreeSpaceModel 입력 수신 여부와 timeout을 검사한다.
    - 전방 장애물 위험 여부(front_danger/front_clear)를 판단한다.
    - FreeSpaceModel의 best heading과 clearance를 이용해 회피 가능 여부를 판단한다.
    - GLOBAL_SUB_GOAL / AVOIDANCE / REJOIN / BLOCKED / INVALID_INPUT / GLOBAL_GOAL_REACHED 상태를 처리한다.
    - 상태에 맞는 local path와 LocalPlannerStatus를 함께 publish한다.

- 좌표계:
    - PathProgress, localization pose, local path는 global_frame 기준이다.
    - 현재 프로젝트에서는 global_frame = "mission_map"으로 사용한다.
    - ObstacleModel / FreeSpaceModel은 base_link 기준 Perception 결과다.
    - FreeSpaceModel.best_heading_angle_rad는 base_link 기준 상대 heading이므로,
      AVOIDANCE path 생성 시 current_yaw를 더해 mission_map 기준 heading으로 변환한다.
*/

#include "spot_navigation/local_path_planner_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

namespace spot_navigation
{
    namespace
    {
        constexpr double kPi = 3.14159265358979323846;
        constexpr double kTwoPi = 2.0 * kPi;
        constexpr double kEpsilon = 1.0e-6;        
    }

    // 생성자
    LocalPathPlannerNode::LocalPathPlannerNode()
    : Node("local_path_planner_node")
    {   
        // ==================================
        // [1] YAML Parameter Declare & Read
        // ==================================
        loadParameters();

        // ================================
        // [2] Subscriber & Publisher 생성
        // ================================
        auto path_progress_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

        path_progress_sub_ =
            this->create_subscription<PathProgressMsg>(
                path_progress_topic_,
                path_progress_qos,
                std::bind(&LocalPathPlannerNode::pathProgressCallback, this, std::placeholders::_1)
            );
        
        auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        pose_sub_ =
            this->create_subscription<LocalizedPoseMsg>(
                localization_pose_topic_,
                pose_qos,
                std::bind(&LocalPathPlannerNode::poseCallback, this, std::placeholders::_1)
            );
        
        auto perception_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

        obstacle_model_sub_ =
            this->create_subscription<ObstacleModelMsg>(
                obstacle_model_topic_,
                perception_qos,
                std::bind(&LocalPathPlannerNode::obstacleModelCallback, this, std::placeholders::_1)
            );

        free_space_model_sub_ =
            this->create_subscription<FreeSpaceModelMsg>(
                free_space_model_topic_,
                perception_qos,
                std::bind(&LocalPathPlannerNode::freeSpaceModelCallback, this, std::placeholders::_1)
            );

        auto output_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

        local_path_pub_ =
            this->create_publisher<nav_msgs::msg::Path>(
                local_path_topic_,
                output_qos
            );
        
        local_planner_status_pub_ =
            this->create_publisher<LocalPlannerStatusMsg>(
                local_planner_status_topic_,
                output_qos
            );
        
        // ===================
        // [3] 주기 Timer 생성
        // ===================
        const auto publish_period =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(1.0 / publish_rate_hz_)
            );
        
        publish_timer_ =
            this->create_wall_timer(
                publish_period,
                std::bind(&LocalPathPlannerNode::publishLocalPlanning, this)
            );

        planner_state_ = LocalPlannerStatusMsg::IDLE;
        
        // 시작 로그
        RCLCPP_INFO(
            this->get_logger(),
            "local_path_planner_node started. path_progress=%s, pose=%s, obstacle=%s, free_space=%s, local_path=%s, status=%s, robot_id=%s, frame=%s",
            path_progress_topic_.c_str(),
            localization_pose_topic_.c_str(),
            obstacle_model_topic_.c_str(),
            free_space_model_topic_.c_str(),
            local_path_topic_.c_str(),
            local_planner_status_topic_.c_str(),
            robot_id_.c_str(),
            global_frame_.c_str()
        );
    }

    // Parameter load
    void LocalPathPlannerNode::loadParameters()
    {
        // =========================
        // [1] 공통 식별자 / 좌표계
        // =========================
        robot_id_ = this->declare_parameter<std::string>("robot_id", "spot_01");
        global_frame_ = this->declare_parameter<std::string>("global_frame", "mission_map");

        // =========================
        // [2] Input & Output Topic
        // =========================
        path_progress_topic_ = this->declare_parameter<std::string>(
            "path_progress_topic",
            "/navigation/path_progress/spot_01");

        localization_pose_topic_ = this->declare_parameter<std::string>(
            "localization_pose_topic",
            "/localization/mock_pose");

        obstacle_model_topic_ = this->declare_parameter<std::string>(
            "obstacle_model_topic",
            "/perception/lidar/obstacle_model");

        free_space_model_topic_ = this->declare_parameter<std::string>(
            "free_space_model_topic",
            "/perception/lidar/free_space_model");

        local_path_topic_ = this->declare_parameter<std::string>(
            "local_path_topic",
            "/navigation/local_path/spot_01");        

        local_planner_status_topic_ = this->declare_parameter<std::string>(
            "local_planner_status_topic",
            "/navigation/local_planner_status/spot_01");
        
        // =================
        // [3] publish 주기
        // =================
        publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 10.0);
        
        // ===========================
        // [4] Input Topic's Timeout
        // ===========================
        path_progress_timeout_sec_ = this->declare_parameter<double>(
            "path_progress_timeout_sec", 0.5);

        pose_timeout_sec_ = this->declare_parameter<double>(
            "pose_timeout_sec", 0.5);
        
        obstacle_model_timeout_sec_ = this->declare_parameter<double>(
            "obstacle_model_timeout_sec", 0.5);

        free_space_model_timeout_sec_ = this->declare_parameter<double>(
            "free_space_model_timeout_sec", 0.5);
        
        // ====================================
        // [5] Local Path Generation Parameter
        // ====================================
        local_path_spacing_m_ = this->declare_parameter<double>(
            "local_path_spacing_m", 0.10);

        max_local_path_length_m_ = this->declare_parameter<double>(
            "max_local_path_length_m", 1.50);

        local_target_tolerance_m_ = this->declare_parameter<double>(
            "local_target_tolerance_m", 0.20);

        use_path_progress_target_yaw_ = this->declare_parameter<bool>(
            "use_path_progress_target_yaw", true);
        
        // ================================
        // [6] Local Planner FSM threshold
        // ================================
        front_block_distance_m_ = this->declare_parameter<double>(
            "front_block_distance_m", 0.60);

        front_clear_distance_m_ = this->declare_parameter<double>(
            "front_clear_distance_m", 0.80);

        avoidance_min_clearance_m_ = this->declare_parameter<double>(
            "avoidance_min_clearance_m", 0.60);

        avoidance_horizon_m_ = this->declare_parameter<double>(
            "avoidance_horizon_m", 0.70);

        rejoin_tolerance_m_ = this->declare_parameter<double>(
            "rejoin_tolerance_m", 0.30);

        rejoin_heading_tolerance_rad_ = this->declare_parameter<double>(
            "rejoin_heading_tolerance_rad", 0.35);
    }

    // pathProgressCallback()
    void LocalPathPlannerNode::pathProgressCallback(const PathProgressMsg::SharedPtr msg)
    {
        latest_path_progress_   = msg;
        path_progress_received_ = true;

        /*
        timeout 판단은 메시지 header.stamp가 아니라 실제 수신 시각 기준으로 수행한다.

        이유:
            - mock 데이터, bag replay, PC-Jetson 시간 동기화 문제 상황에서
              header.stamp가 현재 ROS time과 다를 수 있다.
            - local planner 입장에서는 "마지막으로 실제 수신한 시각"이 timeout 판단에 더 안정적.*/
        
        last_path_progress_receive_time_ = this->now();
        last_path_progress_receive_time_initialized_ = true;        
    }

    // poseCallback()
    void LocalPathPlannerNode::poseCallback(const LocalizedPoseMsg::SharedPtr msg)
    {
        latest_pose_ = msg;
        pose_received_ = true;

        /*
        timeout 판단은 메시지 header.stamp가 아니라 실제 수신 시각 기준으로 수행한다.
        local path의 시작점은 최신 localization pose를 사용한다.*/

        last_pose_receive_time_ = this->now();
        last_pose_receive_time_initialized_ = true;
    }

    // obstacleModelCallback()
    void LocalPathPlannerNode::obstacleModelCallback(const ObstacleModelMsg::SharedPtr msg)
    {
        latest_obstacle_model_ = msg;
        obstacle_model_received_ = true;

        /*
        ObstacleModel은 front/left/right sector별 대표 obstacle cluster를 담는다.

        주의:
            - front.valid == false는 "전방 장애물이 없다"는 정상 데이터일 수 있다.
            - 따라서 front.valid가 false라고 해서 ObstacleModel 입력 자체를 invalid로 보면 안 된다.
            - 입력 valid 여부는 메시지 수신 여부와 timeout으로 판단한다.
        */
        last_obstacle_model_receive_time_ = this->now();
        last_obstacle_model_receive_time_initialized_ = true;
    }

    // freeSpaceModelCallback()
    void LocalPathPlannerNode::freeSpaceModelCallback(const FreeSpaceModelMsg::SharedPtr msg)
    {
        latest_free_space_model_ = msg;
        free_space_model_received_ = true;

        /*
        FreeSpaceModel은 selected/best gap에 대한 정보를 담는다.

        주의:
            - path_available == false는 "통과 가능한 gap이 없다"는 정상 데이터일 수 있다.
            - 따라서 path_available이 false라고 해서 FreeSpaceModel 입력 자체를 invalid로 보면 안 된다.
            - path_available=false는 이후 FSM에서 BLOCKED 판단으로 이어질 수 있다.
        */
        last_free_space_model_receive_time_ = this->now();
        last_free_space_model_receive_time_initialized_ = true;
    }

    // publishLocalPlanning(): Local path planning 주기 실행 함수
    void LocalPathPlannerNode::publishLocalPlanning()
    {
        const auto stamp = this->now();

        const bool path_progress_valid = isPathProgressValid(stamp);
        const bool pose_valid          = isPoseValid(stamp);

        // ====================================================================
        // [1] PathProgress / Pose 유효성 검사
        //  - PathProgress, Pose는 Local Path 생성을 위한 기본 입력
        //  - 둘 중 하나라도 유효하지 않으면 Perception 판단 X, INVALID_INPUT로 전이
        // ====================================================================
        if (!path_progress_valid || !pose_valid) {
            std::string reason = "invalid input";

            if (!path_progress_valid && !pose_valid) reason = "invalid path progress and pose";
            else if (!path_progress_valid) reason = "invalid path progress";
            else reason = "invalid pose";
            
            // 유효하지 않은 상황이면, INVALID_INPUT 상태 처리 함수 호출
            publishInvalidInput(stamp, path_progress_valid, pose_valid, reason);
            return;
        }

        const auto &progress = *latest_path_progress_;
        const auto &pose     = *latest_pose_;

        // ===========================================================
        // [2] 최종 goal 도착 검사
        //  - goal_reached는 모든 주행/회피 상태보다 우선시
        //  - 최종 목적지 도착 시, GLOBAL_GOAL_REACHED 상태 처리 함수 호출
        // ===========================================================
        if (progress.goal_reached) {
            // 현재 pose 유지용 hold path를 publish
            publishGlobalGoalReached(stamp, pose, progress);
            return;
        }

        // ==================================================================
        // [3] LiDAR Perception 입력 유효성 검사
        //  - obstacle_model.front.valid == false는 invalid X
        //    -> 전방 장애물이 없다는 정상 데이터
        //  - free_space_model.path_available == false도 invalid X 
        //    -> 통과 가능한 gap이 없다는 정상 데이터이며, 이후 BLOCKED 판단에 사용
        // ==================================================================
        const bool obstacle_model_valid   = isObstacleModelValid(stamp);
        const bool free_space_model_valid = isFreeSpaceModelValid(stamp);
        
        if (!obstacle_model_valid || !free_space_model_valid) {
            std::string reason = "invalid perception input";

            if (!obstacle_model_valid && !free_space_model_valid) reason = "invalid obstacle model and free space model";
            else if (!obstacle_model_valid) reason = "invalid obstacle model";
            else reason = "invalid free space model";

            publishInvalidInput(stamp, true, true, reason);
            return;
        }

        // ====================
        // [4] 판단 변수 계산
        // =====================
        
        // 최신 obstacle model, free space model 불러오기 
        const auto &obstacle_model   = *latest_obstacle_model_;
        const auto &free_space_model = *latest_free_space_model_;

        const bool front_danger  = isFrontDanger(obstacle_model);
        const bool front_clear   = isFrontClear(obstacle_model);
        const bool free_space_ok = isFreeSpaceAcceptableForMotion(free_space_model); 
        const bool rejoin_done   = isRejoinDone(progress);

        // ======================================================================
        // [5] Local Planner FSM
        //  - GLOBAL_SUB_GOAL:
        //     global path의 PathProgress target을 따라가는 정상 추종 상태
        //
        //  - AVOIDANCE:
        //     front_danger가 있고, free-space heading으로 회피 가능한 상태
        //
        //  - REJOIN:
        //     회피 후 global path로 복귀하는 상태
        //
        //  - BLOCKED:
        //     front_danger가 있지만 사용할 수 있는 free-space가 없는 상태
        //
        // 주의:
        //   planner_state_는 이전 cycle의 상태를 기억한다.
        //   이 값을 기준으로 AVOIDANCE 유지, REJOIN 유지, BLOCKED 유지 여부를 결정한다.
        // =======================================================================
        switch (planner_state_) {
            case LocalPlannerStatusMsg::IDLE:
            case LocalPlannerStatusMsg::INVALID_INPUT:
            case LocalPlannerStatusMsg::GLOBAL_GOAL_REACHED:
            case LocalPlannerStatusMsg::GLOBAL_SUB_GOAL:
            {
                if (!front_danger) publishGlobalSubGoalPath(stamp, pose, progress);
                else if (free_space_ok) publishAvoidancePath(stamp, pose, progress, obstacle_model, free_space_model);
                else {
                    publishBlocked(
                        stamp, pose, progress, obstacle_model, free_space_model,
                        "front obstacle danger and no acceptable free-space"
                    );
                }
                break;
            }

            case LocalPlannerStatusMsg::AVOIDANCE:
            {
                if (front_danger && free_space_ok) publishAvoidancePath(stamp, pose, progress, obstacle_model, free_space_model);
                else if (front_danger && !free_space_ok) {
                    publishBlocked(
                        stamp, pose, progress, obstacle_model, free_space_model,
                        "avoidance lost acceptable free-space"
                    );
                }
                else if (front_clear) {
                    publishRejoinPath(stamp, pose, progress, obstacle_model, free_space_model);
                }
                else {
                    /*
                    hysteresis 구간:
                    - front_danger == false
                    - front_clear == false

                    즉, 전방 장애물이 block distance에서는 벗어났지만
                    아직 clear distance 이상으로 충분히 멀어진 것은 아니다.
                    */
                    if (free_space_ok) {
                        publishAvoidancePath(stamp, pose, progress, obstacle_model, free_space_model);
                    } else {
                        publishBlocked(
                            stamp, pose, progress, obstacle_model, free_space_model,
                            "avoidance hysteresis zone and no acceptable free-space");
                    }                   
                }
                break;
            }

            case LocalPlannerStatusMsg::REJOIN:
            {
                if (front_danger && free_space_ok) publishAvoidancePath(stamp, pose, progress, obstacle_model, free_space_model);
                else if (front_danger && !free_space_ok) {
                    publishBlocked(
                        stamp, pose, progress, obstacle_model, free_space_model,
                        "rejoin interrupted by blocked obstacle"
                    );
                }
                else if (rejoin_done) {
                    publishGlobalSubGoalPath(stamp, pose, progress);
                }
                else {
                    publishRejoinPath(stamp, pose, progress, obstacle_model, free_space_model);
                }
                break;
            }

            case LocalPlannerStatusMsg::BLOCKED:
            {
                if (front_danger && !free_space_ok) {
                    publishBlocked(
                        stamp,
                        pose,
                        progress,
                        obstacle_model,
                        free_space_model,
                        "blocked waiting for acceptable free-space"
                    );
                }
                else if (front_danger && free_space_ok) {
                    publishAvoidancePath(stamp, pose, progress, obstacle_model, free_space_model);
                }
                else if (front_clear) {
                    publishRejoinPath(stamp, pose, progress, obstacle_model, free_space_model);
                }
                else {
                    publishBlocked(
                        stamp,
                        pose,
                        progress,
                        obstacle_model,
                        free_space_model,
                        "blocked hysteresis zone before front clear"
                    );
                }
                break;
            }

            default:
            {
                // GLOBAL_SUB_GOAL 상태 처리 함수 호출
                publishGlobalSubGoalPath(stamp, pose, progress);
                break;
            }
        }
    }

    // PathProgress 입력 유효성 검사 함수
    bool LocalPathPlannerNode::isPathProgressValid(const rclcpp::Time &now) const
    {
        if (!path_progress_received_ || !latest_path_progress_) return false;   // path progress 수신 불안정 -> false

        const auto &progress = *latest_path_progress_;

        if (progress.robot_id != robot_id_) return false;               // 로봇 ID 불일치 -> false
        if (progress.header.frame_id != global_frame_) return false;    // 환경 ID 불일치 -> false
    
        /*
        PathProgressTrackerNode가 판단한 입력 상태도 함께 확인한다.

        path_received / pose_received:
            - tracker가 global path와 localization pose를 수신했는지 여부

        path_valid / pose_valid:
            - tracker가 해당 입력을 계산 가능한 상태로 판단했는지 여부
        */        
        if (!progress.path_received) return false;
        if (!progress.pose_received) return false;
        if (!progress.path_valid) return false;
        if (!progress.pose_valid) return false;

        if (!isFinite(static_cast<double>(progress.target_x_m))) return false;
        if (!isFinite(static_cast<double>(progress.target_y_m))) return false;
        if (!isFinite(static_cast<double>(progress.target_yaw_rad))) return false;
        if (!isFinite(static_cast<double>(progress.target_heading_rad))) return false;
        if (!isFinite(static_cast<double>(progress.heading_error_rad))) return false;
        if (!isFinite(static_cast<double>(progress.yaw_error_rad))) return false;
        if (!isFinite(static_cast<double>(progress.distance_to_target_m))) return false;

        if (!last_path_progress_receive_time_initialized_) return false;

        const double age_sec = (now - last_path_progress_receive_time_).seconds();

        if (age_sec < 0.0) return false;
        if (age_sec > path_progress_timeout_sec_) return false;

        if (!isFinite(static_cast<double>(progress.distance_to_nearest_m))) return false;
        if (!isFinite(static_cast<double>(progress.distance_to_goal_m))) return false;

        return true;
    }

    // Localization Pose 입력 유효성 검사 함수
    bool LocalPathPlannerNode::isPoseValid(const rclcpp::Time &now) const
    {
        if (!pose_received_ || !latest_pose_) return false;

        const auto &pose = *latest_pose_;

        if (pose.robot_id != robot_id_) return false;
        if (pose.header.frame_id != global_frame_) return false;

        if (!isFinite(static_cast<double>(pose.x_m))) return false;
        if (!isFinite(static_cast<double>(pose.y_m))) return false;
        if (!isFinite(static_cast<double>(pose.z_m))) return false;
        if (!isFinite(static_cast<double>(pose.yaw_rad))) return false;

        if (!last_pose_receive_time_initialized_) return false;

        const double age_sec = (now - last_pose_receive_time_).seconds();

        if (age_sec < 0.0) return false;
        if (age_sec > pose_timeout_sec_) return false;

        return true;        
    }

    // Obstacle Model 입력 유효성 검사 함수
    bool LocalPathPlannerNode::isObstacleModelValid(const rclcpp::Time & now) const
    {
        /*
        이 함수는 ObstacleModel 메시지 자체의 수신 상태를 판단한다.

        중요:
            - obstacle_model.front.valid == false는 invalid가 아니다.
            - front.valid == false는 "front sector에 대표 obstacle이 없다"는 정상 데이터다.
            - 따라서 이 함수에서는 front/left/right valid 여부로 입력 유효성을 판단하지 않는다.
        */
        if (!obstacle_model_received_ || !latest_obstacle_model_) return false;
        if (!last_obstacle_model_receive_time_initialized_) return false;

        const double age_sec = (now - last_obstacle_model_receive_time_).seconds();

        if (age_sec < 0.0) return false;
        if (age_sec > obstacle_model_timeout_sec_) return false;

        const auto & obstacle_model = *latest_obstacle_model_;

        /*
        valid=true인 cluster에 대해서만 핵심 수치값의 finite 여부를 확인한다.
        valid=false인 cluster는 "해당 sector에 대표 장애물이 없음"이라는 정상 상태다.
        */
        const auto cluster_value_valid =
            [this](const auto & cluster) -> bool
            {
                if (!cluster.valid) {
                    return true;
                }

                if (!isFinite(static_cast<double>(cluster.nearest_distance_xy))) return false;
                if (!isFinite(static_cast<double>(cluster.azimuth_angle_rad))) return false;
                if (!isFinite(static_cast<double>(cluster.centroid_distance_xy))) return false;

                return true;
            };

        if (!cluster_value_valid(obstacle_model.front)) return false;
        if (!cluster_value_valid(obstacle_model.left)) return false;
        if (!cluster_value_valid(obstacle_model.right)) return false;

        return true;
    }
    
    // Free Space Model 입력 유효성 검사 함수
    bool LocalPathPlannerNode::isFreeSpaceModelValid(const rclcpp::Time & now) const
    {
        /*
        이 함수는 FreeSpaceModel 메시지 자체의 수신 상태를 판단한다.

        중요:
            - path_available == false는 invalid가 아니다.
            - path_available == false는 "현재 통과 가능한 gap이 없다"는 정상 데이터다.
            - 이 경우는 이후 FSM에서 free_space_ok=false로 해석되어 BLOCKED 조건에 사용된다.
        */
        if (!free_space_model_received_ || !latest_free_space_model_) return false;
        if (!last_free_space_model_receive_time_initialized_) return false;

        const double age_sec = (now - last_free_space_model_receive_time_).seconds();

        if (age_sec < 0.0) return false;
        if (age_sec > free_space_model_timeout_sec_) return false;

        const auto & free_space_model = *latest_free_space_model_;

        if (free_space_model.risk_level > FreeSpaceModelMsg::RISK_BLOCKED) return false;

        if (!isFinite(static_cast<double>(free_space_model.best_heading_angle_rad))) return false;
        if (!isFinite(static_cast<double>(free_space_model.best_clearance))) return false;
        if (!isFinite(static_cast<double>(free_space_model.best_score))) return false;

        return true;
    }    

    // 전방 위험 여부 판단 함수
    bool LocalPathPlannerNode::isFrontDanger(const ObstacleModelMsg &obstacle_model) const
    {
        /*
        front_danger 의미:
            front sector를 점유한 대표 cluster가 존재하고,
            그 cluster의 nearest_distance_xy가 front_block_distance_m_보다 가까운 상태.

        주의:
            front.valid == false이면 전방 대표 장애물이 없다는 뜻이므로 front_danger=false.
        */
        if (!obstacle_model.front.valid) return false;
        
        if (!isFinite(static_cast<double>(obstacle_model.front.nearest_distance_xy))) return false;

        return static_cast<double>(obstacle_model.front.nearest_distance_xy) < front_block_distance_m_;
    }

    // 전방 위험 해소 여부 판단 함수
    bool LocalPathPlannerNode::isFrontClear(const ObstacleModelMsg &obstacle_model) const
    {
        /*
        front_clear 의미:
            front sector 대표 cluster가 없거나,
            front 대표 cluster가 front_clear_distance_m_보다 멀어진 상태.

        front_block_distance_m_와 front_clear_distance_m_를 분리해 hysteresis를 만든다.
        */
        if (!obstacle_model.front.valid) return true;

        if (!isFinite(static_cast<double>(obstacle_model.front.nearest_distance_xy))) return false;

        return static_cast<double>(obstacle_model.front.nearest_distance_xy) > front_clear_distance_m_;
    }

    // FreeSpaceModel이 제안한 gap을 실제 회피 주행에 사용할 수 있는지 판단하는 함수
    bool LocalPathPlannerNode::isFreeSpaceAcceptableForMotion(
        const FreeSpaceModelMsg &free_space_model) const
    {
        /*
        free_space_acceptable_for_motion 의미:
            FreeSpaceModel이 selected/best gap을 제공했고,
            그 gap이 Local Planner가 실제 회피 path를 발행하기에 충분히 안전한 상태.

        path_available:
            후보 gap이 하나 이상 존재하는지 여부.

        risk_level:
            FreeSpaceModel 계층에서 판단한 공간 위험도.

        best_clearance:
            selected gap 안에서 가장 가까운 occupied cell까지의 최소 여유 거리.

        avoidance_min_clearance_m_:
            Local Planner 계층에서 실제 회피 path를 발행하기 위해 요구하는 최소 여유 거리.
        */
       
        if (!free_space_model.path_available) return false;
        if (free_space_model.risk_level == FreeSpaceModelMsg::RISK_UNKNOWN) return false;
        if (free_space_model.risk_level == FreeSpaceModelMsg::RISK_BLOCKED) return false;
        if (!isFinite(static_cast<double>(free_space_model.best_clearance))) return false;

        return static_cast<double>(free_space_model.best_clearance) >= avoidance_min_clearance_m_;
    }

    // REJOIN 완료 여부 판단 함수
    bool LocalPathPlannerNode::isRejoinDone(const PathProgressMsg &progress) const
    {
        /*
        REJOIN 완료 의미:
            회피 후 global path 근처로 충분히 복귀했거나,
            global path target 방향과 현재 yaw가 충분히 정렬된 상태.

        2차 MVP 초기 구현에서는 distance_ok || heading_ok를 사용한다.

        이유:
            - 초기 검증 단계에서 REJOIN 상태가 과도하게 오래 유지되는 것을 방지한다.
            - 실제 주행 로그에서 복귀가 너무 빨리 풀리면 추후 distance_ok && heading_ok로 강화할 수 있다.
        */
       
        const bool distance_ok =
            isFinite(static_cast<double>(progress.distance_to_nearest_m)) &&
            static_cast<double>(progress.distance_to_nearest_m) < rejoin_tolerance_m_;
        
        const bool heading_ok =
            isFinite(static_cast<double>(progress.heading_error_rad)) &&
            std::abs(static_cast<double>(progress.heading_error_rad)) < rejoin_heading_tolerance_rad_;
        
        return distance_ok || heading_ok;
    }

    // 기본 상태 메시지 생성 함수
    LocalPathPlannerNode::LocalPlannerStatusMsg LocalPathPlannerNode::makeBaseStatus(
        const rclcpp::Time &stamp, 
        bool path_progress_valid, 
        bool pose_valid) const
    {
        LocalPlannerStatusMsg status;

        status.header.stamp    = stamp;
        status.header.frame_id = global_frame_;
        status.robot_id        = robot_id_;

        // ==============
        // 입력 상태
        // ==============
        status.path_progress_received = path_progress_received_;
        status.path_progress_valid    = path_progress_valid;
        status.pose_received          = pose_received_;
        status.pose_valid             = pose_valid;
        
        // =========================
        // Local planner 상태 기본값
        // =========================
        status.planner_state        = LocalPlannerStatusMsg::INVALID_INPUT;
        status.local_path_available = false;
        status.stop_required        = true;
        status.blocked              = false;
        
        // =========================
        // Local target 기본값
        // =========================
        status.local_target_x_m     = 0.0F;
        status.local_target_y_m     = 0.0F;
        status.local_target_yaw_rad = 0.0F;

        status.selected_heading_rad       = 0.0F;
        status.distance_to_local_target_m = 0.0F;
        status.local_heading_error_rad    = 0.0F;
        status.local_yaw_error_rad        = 0.0F;
        status.local_target_reached       = false;        

        // =========================
        // Local path 정보 기본값
        // =========================
        status.local_path_length_m   = 0.0F;
        status.local_path_pose_count = 0U;

        // =========================
        // 판단 근거 기본값
        // =========================
        status.source_target_index      = 0U;
        status.used_global_sub_goal     = false;
        status.used_free_space_heading  = false;
        status.used_rejoin_target       = false;

        // =========================
        // Free-space 정보 기본값
        // =========================
        status.free_space_heading_rad = 0.0F;
        status.free_space_clearance_m = 0.0F;
        status.free_space_score       = 0.0F;

        status.reason = "";

        return status;
    }

    // 빈 Local path 메시지 생성 함수
    nav_msgs::msg::Path LocalPathPlannerNode::makeEmptyPath(const rclcpp::Time &stamp) const
    {
        nav_msgs::msg::Path path_msg;

        path_msg.header.stamp    = stamp;
        path_msg.header.frame_id = global_frame_;

        return path_msg;
    }

    // 현재 pose를 유지하는 hold path 생성 함수
    nav_msgs::msg::Path LocalPathPlannerNode::makeHoldLocalPath(
        const LocalizedPoseMsg &pose,
        const rclcpp::Time &stamp) const
    {
        nav_msgs::msg::Path path_msg;

        path_msg.header.stamp    = stamp;
        path_msg.header.frame_id = global_frame_;
        
        /*
        hold path는 현재 pose를 유지하라는 의미의 zero-length local path다.

        poses[0] = 현재 pose
        poses[1] = 현재 pose

        같은 pose를 2개 넣는 이유:
            - 일부 follower 구현은 path.poses.size() >= 2를 전제로 할 수 있다.
            - follower가 local path만 보더라도 목표 거리 0의 path로 해석할 수 있다.
        */
        const double x = static_cast<double>(pose.x_m);
        const double y = static_cast<double>(pose.y_m);
        const double z = static_cast<double>(pose.z_m);
        const double yaw = normalizeAngle(static_cast<double>(pose.yaw_rad));

        path_msg.poses.push_back(makePoseStamped(x, y, z, yaw, stamp));
        path_msg.poses.push_back(makePoseStamped(x, y, z, yaw, stamp));

        return path_msg;        
    }

    // 현재 pose에서 local target까지 직선 local path 생성 함수 (핵심)
    nav_msgs::msg::Path LocalPathPlannerNode::makeLocalPathToTarget(
        const LocalizedPoseMsg &pose,
        double target_x_m, double target_y_m, 
        double target_yaw_rad, double selected_heading_rad,
        const rclcpp::Time &stamp, double &path_length_m) const
    {
        nav_msgs::msg::Path path_msg;

        path_msg.header.stamp    = stamp;
        path_msg.header.frame_id = global_frame_;
    
        const double start_x = static_cast<double>(pose.x_m);
        const double start_y = static_cast<double>(pose.y_m);
        const double start_z = static_cast<double>(pose.z_m);

        const double normalized_selected_heading = normalizeAngle(selected_heading_rad);
        const double normalized_target_yaw = normalizeAngle(target_yaw_rad);

        const double distance_to_target =
            calculateDistance2D(start_x, start_y, target_x_m, target_y_m);
            
        /*
        target이 현재 pose와 거의 동일한 경우:
            - 진행용 path를 만들 필요가 없다.
            - follower 안정성을 위해 hold path와 동일하게 현재 pose 2개를 반환한다.
        */
        if (distance_to_target <= kEpsilon) {
            path_length_m = 0.0;

            path_msg.poses.push_back(
            makePoseStamped(start_x, start_y, start_z, normalized_selected_heading, stamp)
            );

            path_msg.poses.push_back(
            makePoseStamped(start_x, start_y, start_z, normalized_selected_heading, stamp)
            );

            return path_msg;
        }
        
        /*
        local path는 전체 target까지 무조건 길게 만들지 않고,
        max_local_path_length_m_ 이내의 짧은 path로 제한한다.

        예:
            - target이 3.0m 앞에 있음
            - max_local_path_length_m_ = 1.5m
            - 이번 cycle에서는 1.5m 길이까지만 path 생성
        */
        path_length_m = std::min(distance_to_target, max_local_path_length_m_); // local path 총 길이

        const bool reaches_target = distance_to_target <= max_local_path_length_m_ + kEpsilon;

        double end_x = target_x_m;
        double end_y = target_y_m;

        if (!reaches_target) {
            end_x = start_x + path_length_m * std::cos(normalized_selected_heading);
            end_y = start_y + path_length_m * std::sin(normalized_selected_heading);
        }

        /*
        segment_count:
            - local path를 몇 개의 작은 구간으로 나눌지 결정한다.
            - pose 개수는 segment_count + 1이다.
            - 최소 1개 segment를 보장해 시작 pose와 끝 pose가 모두 들어가도록 한다.
        */
        const std::uint32_t segment_count =
            std::max<std::uint32_t>(
                1U,
                static_cast<std::uint32_t>(std::ceil(path_length_m / local_path_spacing_m_))
            );

        for (std::uint32_t i = 0U; i <= segment_count; ++i) {
            const double ratio = static_cast<double>(i) / static_cast<double>(segment_count);

            const double x = start_x + ratio * (end_x - start_x);
            const double y = start_y + ratio * (end_y - start_y);

            /*
            yaw 정책:
            - 중간 pose는 selected_heading_rad를 사용한다.
            - local path가 실제 target까지 도달하는 경우, 마지막 pose만 target_yaw_rad를 사용할 수 있다.
            - target까지 도달하지 않고 max_local_path_length_m_에서 잘리는 경우,
                마지막 pose도 selected_heading_rad를 유지한다.
            */
            double yaw = normalized_selected_heading;

            if (reaches_target && i == segment_count) yaw = normalized_target_yaw;

            path_msg.poses.push_back(makePoseStamped(x, y, start_z, yaw, stamp));
        }

        return path_msg;
    }

    // PoseStamped 생성 함수: Stamp 시간의 Pose
    geometry_msgs::msg::PoseStamped LocalPathPlannerNode::makePoseStamped(
        double x_m, double y_m, double z_m,
        double yaw_rad, const rclcpp::Time &stamp) const
    {
        geometry_msgs::msg::PoseStamped pose_stamped;

        pose_stamped.header.stamp    = stamp;
        pose_stamped.header.frame_id = global_frame_;

        pose_stamped.pose.position.x = x_m;
        pose_stamped.pose.position.y = y_m;
        pose_stamped.pose.position.z = z_m;

        pose_stamped.pose.orientation = yawToQuaternion(yaw_rad);

        return pose_stamped;
    }

    // INVALID_INPUT 상태 처리 함수
    void LocalPathPlannerNode::publishInvalidInput(
        const rclcpp::Time &stamp,
        bool path_progress_valid,
        bool pose_valid,
        const std::string &reason)
    {
        planner_state_ = LocalPlannerStatusMsg::INVALID_INPUT;
  
        auto status = makeBaseStatus(stamp, path_progress_valid, pose_valid);

        status.planner_state = LocalPlannerStatusMsg::INVALID_INPUT;
        status.stop_required = true;
        status.blocked = false;
        status.reason = reason;

        nav_msgs::msg::Path path_msg;        

        /*
        pose가 유효하면 현재 위치를 유지하는 hold path를 publish한다.
        pose도 유효하지 않으면 hold path를 만들 수 없으므로 empty path를 publish한다.
        */
        if (pose_valid && latest_pose_) {
            const auto & pose = *latest_pose_;

            path_msg = makeHoldLocalPath(pose, stamp);

            status.local_path_available = true;

            status.local_target_x_m = pose.x_m;
            status.local_target_y_m = pose.y_m;
            status.local_target_yaw_rad = pose.yaw_rad;

            status.selected_heading_rad = pose.yaw_rad;
            status.distance_to_local_target_m = 0.0F;
            status.local_heading_error_rad = 0.0F;
            status.local_yaw_error_rad = 0.0F;
            status.local_target_reached = true;

            status.local_path_length_m = 0.0F;
            status.local_path_pose_count =
            static_cast<std::uint32_t>(path_msg.poses.size());
        } else {
            path_msg = makeEmptyPath(stamp);

            status.local_path_available = false;
            status.local_path_pose_count = 0U;
        }

        if (latest_path_progress_) {
            status.source_target_index = latest_path_progress_->target_index;
        }

        publishPathAndStatus(path_msg, status);

        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Local planner invalid input. reason=%s, path_progress_valid=%s, pose_valid=%s",
            reason.c_str(),
            path_progress_valid ? "true" : "false",
            pose_valid ? "true" : "false"
        );
    }

    // GLOBAL_GOAL_REACHED 상태 처리 함수
    void LocalPathPlannerNode::publishGlobalGoalReached(
        const rclcpp::Time &stamp,
        const LocalizedPoseMsg &pose,
        const PathProgressMsg &progress)
    {
        planner_state_ = LocalPlannerStatusMsg::GLOBAL_GOAL_REACHED;

        auto path_msg = makeHoldLocalPath(pose, stamp);
        auto status   = makeBaseStatus(stamp, true, true);

        status.planner_state        = LocalPlannerStatusMsg::GLOBAL_GOAL_REACHED;
        status.local_path_available = true;
        status.stop_required        = true;
        status.blocked              = false;

        status.local_target_x_m     = pose.x_m;
        status.local_target_y_m     = pose.y_m;
        status.local_target_yaw_rad = pose.yaw_rad;

        status.selected_heading_rad         = pose.yaw_rad;
        status.distance_to_local_target_m   = 0.0F;
        status.local_heading_error_rad      = 0.0F;
        status.local_yaw_error_rad          = 0.0F;
        status.local_target_reached         = true;

        status.local_path_length_m   = 0.0F;
        status.local_path_pose_count = static_cast<std::uint32_t>(path_msg.poses.size());

        status.source_target_index = progress.target_index;

        status.used_global_sub_goal    = false;
        status.used_free_space_heading = false;
        status.used_rejoin_target      = false;

        status.reason = "global goal reached, hold current pose";

        // Local path & status publish 함수 호출
        publishPathAndStatus(path_msg, status);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Global goal reached. Publish hold local path."
        );        
    }

    // GLOBAL_SUB_GOAL 상태 처리 함수
    void LocalPathPlannerNode::publishGlobalSubGoalPath(
        const rclcpp::Time & stamp,
        const LocalizedPoseMsg & pose,
        const PathProgressMsg & progress)
    {
        planner_state_ = LocalPlannerStatusMsg::GLOBAL_SUB_GOAL;

        const double target_x = static_cast<double>(progress.target_x_m);
        const double target_y = static_cast<double>(progress.target_y_m);

        const double selected_heading =
            normalizeAngle(static_cast<double>(progress.target_heading_rad));

        const double local_target_yaw =
            use_path_progress_target_yaw_ ?
            normalizeAngle(static_cast<double>(progress.target_yaw_rad)) :
            selected_heading;

        double local_path_length_m = 0.0;

        auto path_msg =
            makeLocalPathToTarget(
            pose,
            target_x,
            target_y,
            local_target_yaw,
            selected_heading,
            stamp,
            local_path_length_m
            );

        const double current_x   = static_cast<double>(pose.x_m);
        const double current_y   = static_cast<double>(pose.y_m);
        const double current_yaw = normalizeAngle(static_cast<double>(pose.yaw_rad));

        const double distance_to_local_target =
            calculateDistance2D(current_x, current_y, target_x, target_y);

        const double local_heading_error =
            normalizeAngle(selected_heading - current_yaw);

        const double local_yaw_error =
            normalizeAngle(local_target_yaw - current_yaw);

        auto status = makeBaseStatus(stamp, true, true);

        status.planner_state        = LocalPlannerStatusMsg::GLOBAL_SUB_GOAL;
        status.local_path_available = true;
        status.stop_required        = false;
        status.blocked              = false;

        status.local_target_x_m     = static_cast<float>(target_x);
        status.local_target_y_m     = static_cast<float>(target_y);
        status.local_target_yaw_rad = static_cast<float>(local_target_yaw);

        status.selected_heading_rad = static_cast<float>(selected_heading);
        status.distance_to_local_target_m =
            static_cast<float>(distance_to_local_target);
        status.local_heading_error_rad =
            static_cast<float>(local_heading_error);
        status.local_yaw_error_rad =
            static_cast<float>(local_yaw_error);

        status.local_target_reached =
            distance_to_local_target <= local_target_tolerance_m_;

        status.local_path_length_m = static_cast<float>(local_path_length_m);
        status.local_path_pose_count =
            static_cast<std::uint32_t>(path_msg.poses.size());

        status.source_target_index = progress.target_index;

        status.used_global_sub_goal     = true;
        status.used_free_space_heading  = false;
        status.used_rejoin_target       = false;

        status.free_space_heading_rad = 0.0F;
        status.free_space_clearance_m = 0.0F;
        status.free_space_score       = 0.0F;

        status.reason = "follow global sub-goal";

        publishPathAndStatus(path_msg, status);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "\nlocal path planner"
            "\n  state                    : GLOBAL_SUB_GOAL"
            "\n  source_target_index      : %u"
            "\n  distance_to_local_target : %.2f"
            "\n  local_path_length        : %.2f"
            "\n  local_path_pose_count    : %u"
            "\n  local_heading_error_rad  : %.2f"
            "\n  local_target_reached     : %s",
            status.source_target_index,
            distance_to_local_target,
            local_path_length_m,
            status.local_path_pose_count,
            local_heading_error,
            status.local_target_reached ? "true" : "false"
        );
    }

    // AVOIDANCE 상태 처리 함수
    void LocalPathPlannerNode::publishAvoidancePath(
        const rclcpp::Time &stamp,
        const LocalizedPoseMsg &pose,
        const PathProgressMsg &progress,
        const ObstacleModelMsg &obstacle_model,
        const FreeSpaceModelMsg &free_space_model)
    {
        planner_state_ = LocalPlannerStatusMsg::AVOIDANCE;

        const double current_x   = static_cast<double>(pose.x_m);
        const double current_y   = static_cast<double>(pose.y_m);
        const double current_yaw = normalizeAngle(static_cast<double>(pose.yaw_rad));
        
        /*
        FreeSpaceModel.best_heading_angle_rad:
            base_link 기준 상대 heading.
            0 rad = 현재 로봇 전방
            + rad = 좌측
            - rad = 우측

        local path는 mission_map 기준이므로,
        current_yaw를 더해 mission_map 기준 selected heading으로 변환한다.
        */
       
        const double free_space_heading = static_cast<double>(free_space_model.best_heading_angle_rad);
        const double selected_heading   = normalizeAngle(current_yaw + free_space_heading);

        /*
        회피 target:
            현재 pose에서 selected_heading 방향으로 avoidance_horizon_m_만큼 떨어진 점.

        주의:
            avoidance_horizon_m_는 "무조건 전방 0.7m"가 아니라,
            FreeSpaceModel이 선택한 회피 heading 방향으로 0.7m 앞이라는 뜻이다.
        */

        const double target_x   = current_x + avoidance_horizon_m_ * std::cos(selected_heading);
        const double target_y   = current_y + avoidance_horizon_m_ * std::sin(selected_heading);
        const double target_yaw = selected_heading;

        double local_path_length_m = 0.0;
        
        auto path_msg = makeLocalPathToTarget(
            pose, target_x, target_y, target_yaw,
            selected_heading, stamp, local_path_length_m);
        
        const double distance_to_local_target = calculateDistance2D(current_x, current_y, target_x, target_y);
        const double local_heading_error      = normalizeAngle(selected_heading - current_yaw);
        const double local_yaw_error          = normalizeAngle(target_yaw - current_yaw);

        auto status = makeBaseStatus(stamp, true, true);

        status.planner_state        = LocalPlannerStatusMsg::AVOIDANCE;
        status.local_path_available = true;
        status.stop_required        = false;
        status.blocked              = false;

        status.local_target_x_m     = static_cast<float>(target_x);
        status.local_target_y_m     = static_cast<float>(target_y);
        status.local_target_yaw_rad = static_cast<float>(target_yaw);

        status.selected_heading_rad       = static_cast<float>(selected_heading);
        status.distance_to_local_target_m = static_cast<float>(distance_to_local_target);
        status.local_heading_error_rad    = static_cast<float>(local_heading_error);
        status.local_yaw_error_rad        = static_cast<float>(local_yaw_error);

        status.local_target_reached = distance_to_local_target <= local_target_tolerance_m_;

        status.local_path_length_m   = static_cast<float>(local_path_length_m);
        status.local_path_pose_count = static_cast<std::uint32_t>(path_msg.poses.size());

        status.source_target_index = progress.target_index;

        status.used_global_sub_goal    = false;
        status.used_free_space_heading = true;
        status.used_rejoin_target      = false;

        status.free_space_heading_rad = static_cast<float>(free_space_heading);
        status.free_space_clearance_m = static_cast<float>(free_space_model.best_clearance);
        status.free_space_score       = static_cast<float>(free_space_model.best_score);

        status.reason = "avoid front obstacle using free-space heading";

        publishPathAndStatus(path_msg, status);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "\nlocal path planner"
            "\n  state                    : AVOIDANCE"
            "\n  front_distance           : %.2f"
            "\n  free_heading_rad         : %.2f"
            "\n  selected_heading_rad     : %.2f"
            "\n  best_clearance           : %.2f"
            "\n  best_score               : %.2f"
            "\n  local_path_length        : %.2f"
            "\n  local_path_pose_count    : %u",
            obstacle_model.front.valid ? obstacle_model.front.nearest_distance_xy : 0.0F,
            free_space_heading,
            selected_heading,
            free_space_model.best_clearance,
            free_space_model.best_score,
            local_path_length_m,
            status.local_path_pose_count
        );
    }

    // REJOIN 상태 처리 함수
    void LocalPathPlannerNode::publishRejoinPath(
        const rclcpp::Time & stamp,
        const LocalizedPoseMsg & pose,
        const PathProgressMsg & progress,
        const ObstacleModelMsg & obstacle_model,
        const FreeSpaceModelMsg & free_space_model)
    {
        (void)obstacle_model;

        planner_state_ = LocalPlannerStatusMsg::REJOIN;

        /*
        2차 MVP에서는 별도의 rejoin target을 새로 계산하지 않는다.

        이유:
            PathProgressTracker가 이미 현재 pose 기준 nearest_index와 target_index를 계산하고 있다.
            따라서 REJOIN 상태에서도 PathProgress target을 복귀 target으로 재사용한다.
        */
        const double target_x         = static_cast<double>(progress.target_x_m);
        const double target_y         = static_cast<double>(progress.target_y_m);
        const double selected_heading = normalizeAngle(static_cast<double>(progress.target_heading_rad));

        const double local_target_yaw =
            use_path_progress_target_yaw_ ?
            normalizeAngle(static_cast<double>(progress.target_yaw_rad)) :
            selected_heading;

        double local_path_length_m = 0.0;

        auto path_msg =
            makeLocalPathToTarget(
                pose, target_x, target_y, local_target_yaw,
                selected_heading, stamp, local_path_length_m);

        const double current_x   = static_cast<double>(pose.x_m);
        const double current_y   = static_cast<double>(pose.y_m);
        const double current_yaw = normalizeAngle(static_cast<double>(pose.yaw_rad));

        const double distance_to_local_target = calculateDistance2D(current_x, current_y, target_x, target_y);
        const double local_heading_error      = normalizeAngle(selected_heading - current_yaw);
        const double local_yaw_error          = normalizeAngle(local_target_yaw - current_yaw);

        auto status = makeBaseStatus(stamp, true, true);

        status.planner_state        = LocalPlannerStatusMsg::REJOIN;
        status.local_path_available = true;
        status.stop_required        = false;
        status.blocked              = false;

        status.local_target_x_m     = static_cast<float>(target_x);
        status.local_target_y_m     = static_cast<float>(target_y);
        status.local_target_yaw_rad = static_cast<float>(local_target_yaw);

        status.selected_heading_rad       = static_cast<float>(selected_heading);
        status.distance_to_local_target_m = static_cast<float>(distance_to_local_target);
        status.local_heading_error_rad    = static_cast<float>(local_heading_error);
        status.local_yaw_error_rad        = static_cast<float>(local_yaw_error);

        status.local_target_reached = distance_to_local_target <= local_target_tolerance_m_;

        status.local_path_length_m   = static_cast<float>(local_path_length_m);
        status.local_path_pose_count = static_cast<std::uint32_t>(path_msg.poses.size());

        status.source_target_index = progress.target_index;

        status.used_global_sub_goal    = false;
        status.used_free_space_heading = false;
        status.used_rejoin_target      = true;

        status.free_space_heading_rad = static_cast<float>(free_space_model.best_heading_angle_rad);
        status.free_space_clearance_m = static_cast<float>(free_space_model.best_clearance);
        status.free_space_score       = static_cast<float>(free_space_model.best_score);

        status.reason = "rejoin global path";

        publishPathAndStatus(path_msg, status);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "\nlocal path planner"
            "\n  state                    : REJOIN"
            "\n  source_target_index      : %u"
            "\n  distance_to_nearest      : %.2f"
            "\n  heading_error_rad        : %.2f"
            "\n  distance_to_local_target : %.2f"
            "\n  local_path_length        : %.2f"
            "\n  local_path_pose_count    : %u",
            progress.target_index,
            progress.distance_to_nearest_m,
            progress.heading_error_rad,
            distance_to_local_target,
            local_path_length_m,
            status.local_path_pose_count
        );
    }

    // BLOCKED 상태 처리 함수
    void LocalPathPlannerNode::publishBlocked(
        const rclcpp::Time & stamp,
        const LocalizedPoseMsg & pose,
        const PathProgressMsg & progress,
        const ObstacleModelMsg & obstacle_model,
        const FreeSpaceModelMsg & free_space_model,
        const std::string & reason)
    {
        planner_state_ = LocalPlannerStatusMsg::BLOCKED;

        /*
        BLOCKED 상태에서는 새로운 공간을 찾기 위해 움직이는 path를 생성하지 않는다.

        2차 MVP 정책:
            - 현재 위치를 유지하는 hold path publish
            - stop_required=true
            - blocked=true
            - 다음 timer cycle에서 Perception이 회복되면 AVOIDANCE 또는 REJOIN으로 전이 가능
        */
        auto path_msg = makeHoldLocalPath(pose, stamp);
        auto status = makeBaseStatus(stamp, true, true);

        status.planner_state        = LocalPlannerStatusMsg::BLOCKED;
        status.local_path_available = true;
        status.stop_required        = true;
        status.blocked              = true;

        status.local_target_x_m     = pose.x_m;
        status.local_target_y_m     = pose.y_m;
        status.local_target_yaw_rad = pose.yaw_rad;

        status.selected_heading_rad       = pose.yaw_rad;
        status.distance_to_local_target_m = 0.0F;
        status.local_heading_error_rad    = 0.0F;
        status.local_yaw_error_rad        = 0.0F;
        status.local_target_reached       = true;

        status.local_path_length_m   = 0.0F;
        status.local_path_pose_count = static_cast<std::uint32_t>(path_msg.poses.size());

        status.source_target_index = progress.target_index;

        status.used_global_sub_goal    = false;
        status.used_free_space_heading = false;
        status.used_rejoin_target      = false;

        status.free_space_heading_rad = static_cast<float>(free_space_model.best_heading_angle_rad);
        status.free_space_clearance_m = static_cast<float>(free_space_model.best_clearance);
        status.free_space_score       = static_cast<float>(free_space_model.best_score);

        status.reason = reason;

        publishPathAndStatus(path_msg, status);

        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "\nlocal path planner"
            "\n  state              : BLOCKED"
            "\n  reason             : %s"
            "\n  front_valid        : %s"
            "\n  front_distance     : %.2f"
            "\n  path_available     : %s"
            "\n  risk_level         : %u"
            "\n  best_clearance     : %.2f",
            reason.c_str(),
            obstacle_model.front.valid ? "true" : "false",
            obstacle_model.front.valid ? obstacle_model.front.nearest_distance_xy : 0.0F,
            free_space_model.path_available ? "true" : "false",
            free_space_model.risk_level,
            free_space_model.best_clearance
        );
    }

    // Local path & status publish 함수
    void LocalPathPlannerNode::publishPathAndStatus(
        const nav_msgs::msg::Path & path_msg,
        const LocalPlannerStatusMsg & status_msg)
    {
        local_path_pub_->publish(path_msg);
        local_planner_status_pub_->publish(status_msg);
    }

    // 2D 유클리드 거리 계산 함수
    double LocalPathPlannerNode::calculateDistance2D(
        double x1, double y1,
        double x2, double y2) const
    {
        return std::hypot(x2 - x1, y2 - y1);
    }

    // 각도 -pi ~ +pi 범위로 정규화 함수
    double LocalPathPlannerNode::normalizeAngle(double angle_rad) const
    {
        while (angle_rad > kPi) {
            angle_rad -= kTwoPi;
        }

        while (angle_rad < -kPi) {
            angle_rad += kTwoPi;
        }

        return angle_rad;
    }

    // yaw(rad) -> quaternion 변환 함수
    geometry_msgs::msg::Quaternion LocalPathPlannerNode::yawToQuaternion(double yaw_rad) const
    {
        geometry_msgs::msg::Quaternion q;

        const double half_yaw = 0.5 * yaw_rad;

        q.x = 0.0;
        q.y = 0.0;
        q.z = std::sin(half_yaw);
        q.w = std::cos(half_yaw);

        return q;
    }

    // NaN/inf 입력값 검사 함수
    bool LocalPathPlannerNode::isFinite(double value) const
    {
        return std::isfinite(value);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<spot_navigation::LocalPathPlannerNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}