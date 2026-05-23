from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO


class SpotmicroTestCfg(LeggedRobotCfg):

    class env(LeggedRobotCfg.env):
        num_envs = 4096 
        num_observations = 47
        num_actions = 12

    class ik:
        # Same gait/IK model as robot_ws/src/control/locomotion_common.
        gait_period = 1.2
        duty_factor = 0.58
        phase_cmd_norm = 0.04
        blend_cmd_norm = 0.04

        body_height = [0.170, 0.170, 0.170, 0.170]
        step_height = [0.013, 0.013, 0.016, 0.016]
        default_foot_x = [-0.010, -0.010, -0.010, -0.010]
        default_foot_y = [0.0, 0.0, 0.0, 0.0]

        leg_origin_x = [0.093, 0.093, -0.093, -0.093]
        leg_origin_y = [0.036, -0.036, 0.036, -0.036]
        shoulder_sign = [1.0, -1.0, 1.0, -1.0]
        phase_offsets = [0.0, 0.5, 0.5, 0.0]

        max_stride_x = 0.070
        max_stride_y = 0.035
        upper_link_x = 0.0
        upper_link_z = 0.105
        lower_link = 0.130
        shoulder_y_gain = 1.0
        shoulder_limit = 0.16

        joint_min = [
            -0.548, -2.666, -0.100,
            -0.548, -2.666, -0.100,
            -0.548, -2.666, -0.100,
            -0.548, -2.666, -0.100,
        ]
        joint_max = [
            0.548, 1.548, 2.590,
            0.548, 1.548, 2.590,
            0.548, 1.548, 2.590,
            0.548, 1.548, 2.590,
        ]

    class terrain(LeggedRobotCfg.terrain):
        mesh_type = 'plane'
        curriculum = False
        measure_heights = False
        # Real feet are expected to be slipperier than the previous mu=1.0
        # fixed plane. Keep the plane itself moderate and randomize robot
        # shape friction below.
        static_friction = 0.65
        dynamic_friction = 0.50
        restitution = 0.0
        
    class init_state(LeggedRobotCfg.init_state):
        pos = [0.0, 0.0, 0.19]
        default_joint_angles = {
            'front_left_shoulder': 0.0,
            'front_left_leg': -0.9263791118902657,
            'front_left_foot': 1.5314088540972293,
            'front_right_shoulder': 0.0,
            'front_right_leg': -0.9263791118902657,
            'front_right_foot': 1.5314088540972293,
            'rear_left_shoulder': 0.0,
            'rear_left_leg': -0.9263791118902657,
            'rear_left_foot': 1.5314088540972293,
            'rear_right_shoulder': 0.0,
            'rear_right_leg': -0.9263791118902657,
            'rear_right_foot': 1.5314088540972293,
        }

    class control(LeggedRobotCfg.control):
        control_type = 'P'
        stiffness = {'shoulder': 15.0, 'leg': 10.0, 'foot': 10.0}
        damping = {'shoulder': 0.3, 'leg': 0.3, 'foot': 0.2}
        action_scale = 0.25
        decimation = 4

    class asset(LeggedRobotCfg.asset):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/spotmicro_test/urdf/spotmicro_test.urdf'
        name = 'spotmicro_test'
        foot_name = 'toe'
        penalize_contacts_on = [
            'leg_link', 'shoulder_link',
            'base_link', 'front_link', 'rear_link', 'battery_link',
        ]
        terminate_after_contacts_on = [
            'base_link', 'front_link', 'rear_link', 'battery_link',
        ]
        self_collisions = 1
        flip_visual_attachments = False
        collapse_fixed_joints = False

    class rewards(LeggedRobotCfg.rewards):
        class scales:
            tracking_lin_vel = 1.0
            tracking_ang_vel = 0.6
            termination = -20.0
            lin_vel_z = -2.0
            ang_vel_xy = -1.0
            orientation = -8.0
            torques = -0.0012
            dof_vel = -0.0005
            dof_acc = -2.5e-7
            action_rate = -0.07
            base_height = -1.5
            feet_air_time = 0.04
            dof_pos_limits = 0.0
            collision = -1.0
            trot_symmetry = 0.0
            no_stuck_feet = -0.14
            symmetric_gait = 0.0
            feet_clearance = 0.012
            swing_contact = -0.25
            trot_contact = 0.20
            tracking_ik = 0.45
            stand_still = -0.4
            tilt_recovery = 5.0
            ang_vel_xy_recovery = 0.8
        soft_dof_pos_limit = 0.9
        base_height_target = 0.175
        min_base_height = 0.13
        max_base_tilt_deg = 50.0
        recovery_min_height = 0.165
        recovery_reward_tilt_threshold_deg = 6.0
        recovery_relief_tilt_threshold_deg = 7.0
        recovery_gait_relief_scale = 0.25
        recovery_ik_relief_scale = 0.35
        transition_recovery_horizon_s = 0.75
        transition_recovery_initial_tilt_threshold_deg = 8.0
        tracking_sigma = 0.02
        tracking_sigma_ang_vel = 0.03
        swing_contact_grace_time = 0.02
        feet_clearance_min = 0.006
        feet_clearance_cap = 0.010

    class normalization(LeggedRobotCfg.normalization):
        class obs_scales:
            lin_vel = 2.0
            ang_vel = 0.25
            dof_pos = 1.0
            dof_vel = 0.05
            height_measurements = 5.0

    class noise(LeggedRobotCfg.noise):
        add_noise = True 
        noise_level = 1.0
        class noise_scales:
            dof_pos = 0.01
            dof_vel = 1.5
            lin_vel = 0.1
            ang_vel = 0.2
            gravity = 0.05
            height_measurements = 0.1

    class commands(LeggedRobotCfg.commands):
        curriculum = False
        max_curriculum = 1.0
        num_commands = 4
        resampling_time = 10.0
        heading_command = False
        command_deadband = 0.02
        class ranges:
            lin_vel_x = [-0.03, 0.15]
            lin_vel_y = [0.0, 0.0]
            ang_vel_yaw = [-0.20, 0.20]
            heading = [-3.14, 3.14]

    class domain_rand(LeggedRobotCfg.domain_rand):
        randomize_friction = True
        friction_range = [0.18, 1.05]
        randomize_base_mass = True
        added_mass_range = [-0.10, 0.30]
        randomize_base_com = True
        base_com_offset_x_range = [-0.055, 0.010]
        base_com_offset_y_range = [-0.012, 0.012]
        base_com_offset_z_range = [-0.012, 0.030]
        randomize_motor_strength = True
        motor_strength_range = [0.75, 1.10]
        randomize_pd_gains = True
        stiffness_scale_range = [0.65, 1.35]
        damping_scale_range = [0.70, 1.80]
        randomize_joint_obs_offset = True
        joint_obs_offset_range = [-0.025, 0.025]
        push_robots = True
        push_interval_s = 3
        max_push_vel_xy = 0.16
        max_push_ang_vel_xy = 0.75
        max_push_ang_vel_z = 0.20
        push_lin_vel_clip = 0.30
        push_ang_vel_xy_clip = 1.10
        push_ang_vel_z_clip = 0.35
        action_delay = True
        action_delay_range = [0, 3]
        recovery_roll_pitch_range_deg = 12.0
        recovery_lin_vel_xy_range = 0.14
        recovery_lin_vel_z_range = 0.04
        recovery_ang_vel_xy_range = 0.85
        recovery_ang_vel_z_range = 0.35


class SpotmicroTestCfgPPO(LeggedRobotCfgPPO):

    class algorithm(LeggedRobotCfgPPO.algorithm):
        entropy_coef = 0.005
        learning_rate = 1e-4

    class runner(LeggedRobotCfgPPO.runner):
        run_name = 'spotmicro_v6_1_5_slippery_rear_com_motor_dr'
        experiment_name = 'spotmicro_test'
        max_iterations = 1000
        save_interval = 100
        resume = True
        load_run = "May20_14-29-52_spotmicro_v6_1_3_stronger_transition_push"
        checkpoint = 4100
