from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO


class SpotmicroTestCfg(LeggedRobotCfg):

    class env(LeggedRobotCfg.env):
        num_envs = 4096 
        num_observations = 47
        num_actions = 12

    class terrain(LeggedRobotCfg.terrain):
        mesh_type = 'plane'
        curriculum = False
        measure_heights = False

    class init_state(LeggedRobotCfg.init_state):
        pos = [0.0, 0.0, 0.23]
        default_joint_angles = {
            'front_left_shoulder': 0.0,
            'front_left_leg': -0.6,
            'front_left_foot': 1.1,
            'front_right_shoulder': 0.0,
            'front_right_leg': -0.6,
            'front_right_foot': 1.1,
            'rear_left_shoulder': 0.0,
            'rear_left_leg': -0.6,
            'rear_left_foot': 1.1,
            'rear_right_shoulder': 0.0,
            'rear_right_leg': -0.6,
            'rear_right_foot': 1.1,
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
        penalize_contacts_on = ['leg_link', 'shoulder_link', 'base_link']
        terminate_after_contacts_on = ['base_link']
        self_collisions = 1
        flip_visual_attachments = False
        collapse_fixed_joints = False

    class rewards(LeggedRobotCfg.rewards):
        class scales:
            tracking_lin_vel = 1.5
            tracking_ang_vel = 1.3 
            termination = -10.0
            lin_vel_z = -2.0
            ang_vel_xy = -0.4
            orientation = -6.0
            torques = -0.002
            dof_vel = -0.001
            dof_acc = -2.5e-7
            action_rate = -0.05
            feet_air_time = 0.0
            dof_pos_limits = 0.0
            collision = -1.0
            trot_symmetry = 0.0
            no_stuck_feet = 0.0
            symmetric_gait = 0.0
            feet_clearance = 0.0
            trot_contact = 0.5
            tracking_ik = 1.0
            stand_still = -0.5
        soft_dof_pos_limit = 0.9
        base_height_target = 0.206
        tracking_sigma = 0.1
        tracking_sigma_ang_vel = 0.05

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
        class ranges:
            lin_vel_x = [0.0, 0.4]
            lin_vel_y = [0.0, 0.0]
            ang_vel_yaw = [-0.4, 0.4]
            heading = [-3.14, 3.14]

    class domain_rand(LeggedRobotCfg.domain_rand):
        randomize_friction = True
        friction_range = [0.4, 1.2]
        randomize_base_mass = True
        added_mass_range = [-0.2, 0.2]
        push_robots = True
        push_interval_s = 15
        max_push_vel_xy = 0.2
        action_delay = True
        action_delay_range = [1, 2]


class SpotmicroTestCfgPPO(LeggedRobotCfgPPO):

    class algorithm(LeggedRobotCfgPPO.algorithm):
        entropy_coef = 0.01

    class runner(LeggedRobotCfgPPO.runner):
        run_name = 'spotmicro_v3_3_1_torque_improve'
        experiment_name = 'spotmicro_test'
        max_iterations = 1500
        save_interval = 100
