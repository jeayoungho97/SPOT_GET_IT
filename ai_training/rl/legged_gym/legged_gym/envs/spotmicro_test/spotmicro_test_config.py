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

        body_height = [0.190, 0.190, 0.190, 0.190]
        step_height = [0.025, 0.025, 0.025, 0.025]
        default_foot_x = [-0.040, -0.040, -0.040, -0.040]
        default_foot_y = [0.052, -0.052, 0.052, -0.052]

        leg_origin_x = [0.093, 0.093, -0.093, -0.093]
        leg_origin_y = [0.036, -0.036, 0.036, -0.036]
        shoulder_sign = [1.0, -1.0, 1.0, -1.0]
        shoulder_offset_y = [0.052, -0.052, 0.052, -0.052]
        phase_offsets = [0.0, 0.5, 0.5, 0.0]

        max_stride_x = 0.085
        max_stride_y = 0.024
        soft_stride_limit = True
        upper_link_x = 0.010
        upper_link_z = 0.120
        lower_link = 0.115
        toe_radius = 0.015
        shoulder_y_gain = 1.0
        shoulder_limit = 0.548

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
        terrain_profile = 'flat'
        curriculum = False
        measure_heights = False
        horizontal_scale = 0.05
        vertical_scale = 0.005
        border_size = 8.0
        max_init_terrain_level = 1
        num_rows = 8
        num_cols = 12
        terrain_length = 6.0
        terrain_width = 6.0
        curriculum_use_full_range = True
        curriculum_move_up_distance = 1.10
        curriculum_move_down_command_scale = 0.25
        terrain_proportions = [0.50, 0.25, 0.25]
        spotmicro_slope_min = 0.02
        spotmicro_slope_max = 0.10
        spotmicro_rough_height_max = 0.005
        spotmicro_rolling_amp_max = 0.015
        spotmicro_rolling_wavelength_min = 0.65
        spotmicro_rolling_wavelength_max = 1.40
        spotmicro_terrain_platform_size = 0.7
        slope_treshold = 0.75
        static_friction = 1.0
        dynamic_friction = 1.0
        restitution = 0.0
        
    class init_state(LeggedRobotCfg.init_state):
        pos = [0.0, 0.0, 0.19]
        default_joint_angles = {
            'front_left_shoulder': 0.0,
            'front_left_leg': -0.9921237899157832,
            'front_left_foot': 1.4907337340120823,
            'front_right_shoulder': 0.0,
            'front_right_leg': -0.9921237899157832,
            'front_right_foot': 1.4907337340120823,
            'rear_left_shoulder': 0.0,
            'rear_left_leg': -0.9921237899157832,
            'rear_left_foot': 1.4907337340120823,
            'rear_right_shoulder': 0.0,
            'rear_right_leg': -0.9921237899157832,
            'rear_right_foot': 1.4907337340120823,
        }

    class control(LeggedRobotCfg.control):
        control_type = 'P'
        stiffness = {'shoulder': 15.0, 'leg': 10.0, 'foot': 10.0}
        damping = {'shoulder': 0.3, 'leg': 0.3, 'foot': 0.2}
        action_scale = 0.18
        recovery_action_scale = 0.18
        decimation = 4

    class recovery:
        enabled = False
        tilt_threshold_deg = 14.0
        full_tilt_deg = 25.0
        command_scale_enabled = False
        command_scale = 0.15
        phase_enabled = False
        phase_scale = 0.1
        phase_freeze = False
        action_scale_enabled = False

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
            tracking_lin_vel = 0.9
            tracking_ang_vel = 0.6
            termination = -60.0
            survival = 0.15
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
            trot_contact = 0.30
            tracking_ik = 0.8
            stand_still = -0.4
            tilt_recovery = 0.0
            ang_vel_xy_recovery = 0.0
        soft_dof_pos_limit = 0.9
        base_height_target = 0.190
        min_base_height = 0.13
        max_base_tilt_deg = 50.0
        recovery_min_height = 0.165
        recovery_reward_tilt_threshold_deg = 12.0
        recovery_diagnostic_initial_tilt_threshold_deg = 12.0
        recovery_relief_tilt_threshold_deg = 14.0
        recovery_relief_full_tilt_deg = 25.0
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
        randomize_friction = False
        friction_range = [1.0, 1.0]
        randomize_base_mass = False
        added_mass_range = [0.0, 0.0]
        randomize_base_com = False
        base_com_offset_x_range = [-0.020, 0.010]
        base_com_offset_y_range = [-0.008, 0.008]
        base_com_offset_z_range = [-0.008, 0.015]
        randomize_motor_strength = False
        motor_strength_range = [1.0, 1.0]
        randomize_pd_gains = False
        stiffness_scale_range = [1.0, 1.0]
        damping_scale_range = [1.0, 1.0]
        randomize_joint_obs_offset = False
        joint_obs_offset_range = [0.0, 0.0]
        push_robots = False
        push_interval_s = 4
        max_push_vel_xy = 0.18
        max_push_ang_vel_xy = 0.85
        max_push_ang_vel_z = 0.20
        push_lin_vel_clip = 0.30
        push_ang_vel_xy_clip = 1.20
        push_ang_vel_z_clip = 0.35
        transition_tilt_push = False
        transition_tilt_push_prob = 0.20
        transition_tilt_push_min_deg = 18.0
        transition_tilt_push_max_deg = 27.0
        transition_tilt_push_ang_vel_xy = 0.40
        transition_tilt_cmd_x_range = [0.05, 0.10]
        transition_tilt_zero_yaw_cmd = True
        action_delay = False
        action_delay_range = [0, 0]
        recovery_roll_pitch_range_deg = 0.0
        recovery_yaw_range_deg = 0.0
        recovery_lin_vel_xy_range = 0.0
        recovery_lin_vel_z_range = 0.0
        recovery_ang_vel_xy_range = 0.0
        recovery_ang_vel_z_range = 0.0


class SpotmicroTestCfgPPO(LeggedRobotCfgPPO):

    class algorithm(LeggedRobotCfgPPO.algorithm):
        entropy_coef = 0.005
        learning_rate = 1e-4

    class runner(LeggedRobotCfgPPO.runner):
        run_name = 'spotmicro_classic_ik_flat_gait_from_scratch'
        experiment_name = 'spotmicro_test'
        max_iterations = 4000
        save_interval = 100
        resume = False
        load_run = ""
        checkpoint = -1
