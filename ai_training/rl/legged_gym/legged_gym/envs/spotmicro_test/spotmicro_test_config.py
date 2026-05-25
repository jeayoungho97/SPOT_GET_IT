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
        static_friction = 1.0
        dynamic_friction = 1.0
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
        recovery_action_scale = 0.35
        decimation = 4

    class recovery:
        enabled = True
        tilt_threshold_deg = 17.0
        full_tilt_deg = 27.0
        command_scale_enabled = True
        command_scale = 0.0
        phase_enabled = True
        phase_scale = 0.0
        phase_freeze = False
        action_scale_enabled = True

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
            orientation = -10.0
            torques = -0.001
            dof_vel = -0.0005
            dof_acc = -2.5e-7
            action_rate = -0.05
            base_height = -2.0
            feet_air_time = 0.04
            dof_pos_limits = 0.0
            collision = -1.0
            trot_symmetry = 0.0
            no_stuck_feet = -0.2
            symmetric_gait = 0.0
            feet_clearance = 0.03
            swing_contact = -0.45
            trot_contact = 0.35
            tracking_ik = 0.6
            stand_still = -0.4
            tilt_recovery = 4.0
            ang_vel_xy_recovery = 0.5
            recovery_stance_contact = 0.25
        soft_dof_pos_limit = 0.9
        base_height_target = 0.175
        min_base_height = 0.13
        max_base_tilt_deg = 50.0
        recovery_min_height = 0.165
        recovery_reward_tilt_threshold_deg = 15.0
        recovery_diagnostic_initial_tilt_threshold_deg = 12.0
        recovery_relief_tilt_threshold_deg = 17.0
        recovery_relief_full_tilt_deg = 27.0
        recovery_gait_relief_scale = 0.65
        recovery_ik_relief_scale = 0.65
        transition_recovery_horizon_s = 0.75
        transition_recovery_initial_tilt_threshold_deg = 12.0
        tracking_sigma = 0.02
        tracking_sigma_ang_vel = 0.03
        swing_contact_grace_time = 0.02
        feet_clearance_min = 0.020
        feet_clearance_cap = 0.030

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
        friction_range = [0.35, 1.10]
        randomize_base_mass = True
        added_mass_range = [-0.05, 0.15]
        randomize_base_com = True
        base_com_offset_x_range = [-0.020, 0.010]
        base_com_offset_y_range = [-0.008, 0.008]
        base_com_offset_z_range = [-0.008, 0.015]
        randomize_motor_strength = True
        motor_strength_range = [0.85, 1.10]
        randomize_pd_gains = False
        stiffness_scale_range = [1.0, 1.0]
        damping_scale_range = [1.0, 1.0]
        randomize_joint_obs_offset = False
        joint_obs_offset_range = [0.0, 0.0]
        push_robots = True
        push_interval_s = 4
        max_push_vel_xy = 0.18
        max_push_ang_vel_xy = 0.85
        max_push_ang_vel_z = 0.20
        push_lin_vel_clip = 0.30
        push_ang_vel_xy_clip = 1.20
        push_ang_vel_z_clip = 0.35
        transition_tilt_push = True
        transition_tilt_push_prob = 0.20
        transition_tilt_push_min_deg = 18.0
        transition_tilt_push_max_deg = 27.0
        transition_tilt_push_ang_vel_xy = 0.30
        transition_tilt_cmd_x_range = [0.05, 0.10]
        transition_tilt_zero_yaw_cmd = True
        action_delay = True
        action_delay_range = [1, 2]
        recovery_roll_pitch_range_deg = 30.0
        recovery_lin_vel_xy_range = 0.14
        recovery_lin_vel_z_range = 0.04
        recovery_ang_vel_xy_range = 0.90
        recovery_ang_vel_z_range = 0.35


class SpotmicroTestCfgPPO(LeggedRobotCfgPPO):

    class algorithm(LeggedRobotCfgPPO.algorithm):
        entropy_coef = 0.005
        learning_rate = 1e-4

    class runner(LeggedRobotCfgPPO.runner):
        run_name = 'spotmicro_v6_4_prefall_brace_mode'
        experiment_name = 'spotmicro_test'
        max_iterations = 500
        save_interval = 100
        resume = True
        load_run = "May25_16-41-52_spotmicro_v6_3_1_prefall_transition_tilt_sampler_soft"
        checkpoint = 8100
