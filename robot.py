import pybullet as p
import numpy as np
import math
import config
from PIL import Image
from config import fov, aspect, near_plane, far_plane

class Robot:

    def __init__(self, args):

        if args.robot == "sawyer":
            self.base_start_position = config.base_start_position_sawyer
            self.base_start_orientation_q = p.getQuaternionFromEuler(config.base_start_orientation_e_sawyer)
            self.joint_start_positions = config.joint_start_positions_sawyer
            self.id = p.loadURDF("sawyer_robot/sawyer_description/urdf/sawyer.urdf", self.base_start_position, self.base_start_orientation_q, useFixedBase=True)
            self.robot = "sawyer"
            self.ee_index = config.ee_index_sawyer
            self.gripper_id = p.loadURDF("robotiq_2f_85/robotiq_2f_85.urdf", config.ee_start_position, p.getQuaternionFromEuler(config.ee_start_orientation_e))
            self.gripper_motor = config.robotiq_motor_joint
            p.createConstraint(self.id, self.ee_index, self.gripper_id, 0, jointType=p.JOINT_FIXED, jointAxis=[0, 0, 0], parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, -0.07], childFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]))
        elif args.robot == "franka":
            self.base_start_position = config.base_start_position_franka
            self.base_start_orientation_q = p.getQuaternionFromEuler(config.base_start_orientation_e_franka)
            self.joint_start_positions = config.joint_start_positions_franka
            self.id = p.loadURDF("franka_robot/panda.urdf", self.base_start_position, self.base_start_orientation_q, useFixedBase=True)
            self.robot = "franka"
            self.ee_index = config.ee_index_franka
        self.ee_start_position = config.ee_start_position
        self.ee_start_orientation_e = config.ee_start_orientation_e
        self.ee_current_position = config.ee_start_position
        self.ee_current_orientation_e = config.ee_start_orientation_e

        self.gripper_open = True
        self.trajectory_step = 1

        i = 0
        self.joint_indices = []
        for j in range(p.getNumJoints(self.id)):
            joint_type = p.getJointInfo(self.id, j)[2]
            if joint_type == p.JOINT_PRISMATIC or joint_type == p.JOINT_REVOLUTE:
                p.resetJointState(self.id, j, self.joint_start_positions[i])
                i += 1
                self.joint_indices.append(j)



    def move(self, env, ee_target_position, ee_target_orientation_e, gripper_open, is_trajectory):

        if self.robot == "sawyer":
            gripper1_index = self.gripper_motor
            gripper2_index = self.gripper_motor
            gripper_target_position = config.gripper_goal_position_open_sawyer if gripper_open else config.gripper_goal_position_closed_sawyer
            if is_trajectory:
                ee_target_position = list(ee_target_position)
                ee_target_position[2] -= config.gripper_depth_offset_sawyer
        elif self.robot == "franka":
            gripper1_index = 9
            gripper2_index = 10
            gripper_target_position = config.gripper_goal_position_open_franka if gripper_open else config.gripper_goal_position_closed_franka
            if is_trajectory:
                ee_target_position = list(ee_target_position)
                ee_target_position[2] -= config.gripper_depth_offset_franka

        min_joint_positions = [p.getJointInfo(self.id, i)[8] for i in range(p.getNumJoints(self.id)) if p.getJointInfo(self.id, i)[2] == p.JOINT_PRISMATIC or p.getJointInfo(self.id, i)[2] == p.JOINT_REVOLUTE]
        max_joint_positions = [p.getJointInfo(self.id, i)[9] for i in range(p.getNumJoints(self.id)) if p.getJointInfo(self.id, i)[2] == p.JOINT_PRISMATIC or p.getJointInfo(self.id, i)[2] == p.JOINT_REVOLUTE]
        joint_ranges = [abs(max_joint_position - min_joint_position) for min_joint_position, max_joint_position in zip(min_joint_positions, max_joint_positions)]
        rest_poses = list((np.array(max_joint_positions) + np.array(min_joint_positions)) / 2)

        ee_target_orientation_q = p.getQuaternionFromEuler(ee_target_orientation_e)

        ee_current_position = p.getLinkState(self.id, self.ee_index, computeForwardKinematics=True)[0]
        ee_current_orientation_q = p.getLinkState(self.id, self.ee_index, computeForwardKinematics=True)[1]
        ee_current_orientation_e = p.getEulerFromQuaternion(ee_current_orientation_q)
        if self.robot == "sawyer":
            gripper1_current_position = p.getJointState(self.gripper_id, gripper1_index)[0]
            gripper2_current_position = p.getJointState(self.gripper_id, gripper2_index)[0]
        elif self.robot == "franka":
            gripper1_current_position = p.getJointState(self.id, gripper1_index)[0]
            gripper2_current_position = p.getJointState(self.id, gripper2_index)[0]

        time_step = 0

        while (not (ee_current_position[0] <= ee_target_position[0] + config.margin_error and ee_current_position[0] >= ee_target_position[0] - config.margin_error and
                    ee_current_position[1] <= ee_target_position[1] + config.margin_error and ee_current_position[1] >= ee_target_position[1] - config.margin_error and
                    ee_current_position[2] <= ee_target_position[2] + config.margin_error and ee_current_position[2] >= ee_target_position[2] - config.margin_error and
                    ee_current_orientation_e[0] <= ee_target_orientation_e[0] + config.margin_error and ee_current_orientation_e[0] >= ee_target_orientation_e[0] - config.margin_error and
                    ee_current_orientation_e[1] <= ee_target_orientation_e[1] + config.margin_error and ee_current_orientation_e[1] >= ee_target_orientation_e[1] - config.margin_error and
                    ee_current_orientation_e[2] <= ee_target_orientation_e[2] + config.margin_error and ee_current_orientation_e[2] >= ee_target_orientation_e[2] - config.margin_error and
                    gripper1_current_position <= gripper_target_position + config.gripper_margin_error and gripper1_current_position >= gripper_target_position - config.gripper_margin_error and
                    gripper2_current_position <= gripper_target_position + config.gripper_margin_error and gripper2_current_position >= gripper_target_position - config.gripper_margin_error)):

            target_joint_positions = p.calculateInverseKinematics(self.id, self.ee_index, ee_target_position, targetOrientation=ee_target_orientation_q, lowerLimits=min_joint_positions, upperLimits=max_joint_positions, jointRanges=joint_ranges, restPoses=rest_poses, maxNumIterations=500)

            if self.robot == "sawyer":
                p.setJointMotorControlArray(self.id, self.joint_indices, p.POSITION_CONTROL, targetPositions=target_joint_positions, forces=[config.arm_movement_force_sawyer] * 8)
                current_joints = [p.getJointState(self.gripper_id, i)[0] for i in range(p.getNumJoints(self.gripper_id))]
                joint_idx = [6, 3, 8, 5, 10]
                target_joints = [current_joints[1], -current_joints[1], -current_joints[1], current_joints[1], current_joints[1]]
                p.setJointMotorControlArray(self.gripper_id, joint_idx, p.POSITION_CONTROL, target_joints, positionGains=np.ones(5))
                p.setJointMotorControl2(self.gripper_id, self.gripper_motor, p.POSITION_CONTROL, targetPosition=gripper_target_position, force=config.gripper_movement_force_sawyer)
            elif self.robot == "franka":
                p.setJointMotorControlArray(self.id, self.joint_indices[:-2], p.POSITION_CONTROL, targetPositions=target_joint_positions[:-2], forces=[config.arm_movement_force_franka] * 7)
                p.setJointMotorControl2(self.id, gripper1_index, p.POSITION_CONTROL, targetPosition=gripper_target_position, force=config.gripper_movement_force_franka)
                p.setJointMotorControl2(self.id, gripper2_index, p.POSITION_CONTROL, targetPosition=gripper_target_position, force=config.gripper_movement_force_franka)

            env.update()
            self.get_camera_image("head", env, save_camera_image=is_trajectory, rgb_image_path=config.rgb_image_trajectory_path.format(step=self.trajectory_step), depth_image_path=config.depth_image_trajectory_path.format(step=self.trajectory_step))
            if is_trajectory:
                self.trajectory_step += 1

            ee_current_position = p.getLinkState(self.id, self.ee_index, computeForwardKinematics=True)[0]
            ee_current_orientation_q = p.getLinkState(self.id, self.ee_index, computeForwardKinematics=True)[1]
            ee_current_orientation_e = p.getEulerFromQuaternion(ee_current_orientation_q)
            if self.robot == "sawyer":
                gripper1_new_position = p.getJointState(self.gripper_id, gripper1_index)[0]
                gripper2_new_position = p.getJointState(self.gripper_id, gripper2_index)[0]
            elif self.robot == "franka":
                gripper1_new_position = p.getJointState(self.id, gripper1_index)[0]
                gripper2_new_position = p.getJointState(self.id, gripper2_index)[0]

            self.ee_current_position = ee_current_position
            self.ee_current_orientation_e = ee_current_orientation_e
            self.gripper_open = gripper_open

            if ((ee_current_position[0] <= ee_target_position[0] + config.margin_error and ee_current_position[0] >= ee_target_position[0] - config.margin_error and
                ee_current_position[1] <= ee_target_position[1] + config.margin_error and ee_current_position[1] >= ee_target_position[1] - config.margin_error and
                ee_current_position[2] <= ee_target_position[2] + config.margin_error and ee_current_position[2] >= ee_target_position[2] - config.margin_error and
                ee_current_orientation_e[0] <= ee_target_orientation_e[0] + config.margin_error and ee_current_orientation_e[0] >= ee_target_orientation_e[0] - config.margin_error and
                ee_current_orientation_e[1] <= ee_target_orientation_e[1] + config.margin_error and ee_current_orientation_e[1] >= ee_target_orientation_e[1] - config.margin_error and
                ee_current_orientation_e[2] <= ee_target_orientation_e[2] + config.margin_error and ee_current_orientation_e[2] >= ee_target_orientation_e[2] - config.margin_error) and
                (not gripper_open) and
                math.isclose(gripper1_new_position, gripper1_current_position, rel_tol=config.rel_tol, abs_tol=config.abs_tol) and
                math.isclose(gripper2_new_position, gripper2_current_position, rel_tol=config.rel_tol, abs_tol=config.abs_tol)):
                break

            gripper1_current_position = gripper1_new_position
            gripper2_current_position = gripper2_new_position

            time_step += 1

            if is_trajectory:
                if time_step > 0:
                    break
            else:
                if time_step > 99:
                    break



    def get_camera_image(self, camera, env, save_camera_image, rgb_image_path, depth_image_path):

        if camera == "wrist":
            camera_position = list(p.getLinkState(self.id, self.ee_index, computeForwardKinematics=True)[0])
            if self.robot == "sawyer":
                camera_position[2] -= config.wrist_camera_offset_sawyer
            camera_orientation_q = p.getLinkState(self.id, self.ee_index, computeForwardKinematics=True)[1]
        elif camera == "head":
            camera_position = config.head_camera_position
            camera_orientation_q = p.getQuaternionFromEuler(config.head_camera_orientation_e)

        projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near_plane, far_plane)
        rotation_matrix = np.array(p.getMatrixFromQuaternion(camera_orientation_q)).reshape(3, 3)

        if camera == "wrist":
            init_camera_vector = [0, 0, 1]
            init_up_vector = [1, 0, 0]
        elif camera == "head":
            init_camera_vector = [0, 0, 1]
            init_up_vector = [-1, 0, 0]

        camera_vector = rotation_matrix.dot(init_camera_vector)
        up_vector = rotation_matrix.dot(init_up_vector)
        view_matrix = p.computeViewMatrix(camera_position, camera_position + camera_vector, up_vector)

        image = p.getCameraImage(config.image_width, config.image_height, viewMatrix=view_matrix, projectionMatrix=projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)

        rgb_buffer = image[2]
        depth_buffer = image[3]

        if save_camera_image:
            rgb_image = Image.fromarray(rgb_buffer)
            rgb_image.save(rgb_image_path)

            n = config.near_plane
            f = config.far_plane
            depth_array = 2 * n * f / (f + n - (2 * depth_buffer - 1.0) * (f - n))

            depth_array = np.clip(depth_array, 0, 1)
            depth_image = Image.fromarray(depth_array * 255)
            depth_image.convert("L").save(depth_image_path)

        return camera_position, camera_orientation_q
