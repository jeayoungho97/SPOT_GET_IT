std_msgs/Header header              # frame_id : mission_map

string robot_id

# 이번 cycle에서 유효한 local path를 생성했는지 여부
# - true  : /planning/local_path/spot_01에 따라갈 path가 존재
# - false : local path를 만들 수 없거나 입력 데이터가 부족한 상태
bool path_available                 

uint8 planner_status            # local path 생성 상태값

float32 target_x_m              # local path의 최종 target x 좌표
float32 target_y_m              # local path의 최종 target y 좌표

float32 selected_heading_rad    # local planner가 최종적으로 선택한 heading
float32 local_path_length_m     # 생성된 local path의 대략적인 길이

# global path의 lookahead target 기반 local path 생성 여부
bool used_global_sub_goal

# free_space_model.best_heading_angle_rad 기반 회피 local path 생성 여부
bool used_free_space_heading

# 전방 장애물 O && !free-space gap 기반 local path 생성에 실패했는지 여부.
bool blocked

float32 free_space_heading_rad  # free_space_model에서 받은 base_link 기준 best heading
float32 free_space_clearance_m  # 선택된 free-space 방향의 최소 clearance [m] 
float32 free_space_score        # free_space_model이 계산한 선택 gap의 score

string reason

uint8 IDLE=0                    # 초기 상태
uint8 GLOBAL_SUB_GOAL=1         # lookahead target 기반 local path 생성 상태
uint8 AVOIDANCE=2               # free space heading 기반 local path 생성 상태
uint8 REJOIN=3                  # 장애물 회피 후 global path 복귀하기 위한 local path 생성 상태
uint8 BLOCKED=4                 # 장애물 존재 + free space gap X 기반 local path 생성 실패한 상태
uint8 INVALID_INPUT=5           # planning 수행할 수 없는 상태 (ex: pose 없음, path_progress 없음, frame_id 불일치 등)