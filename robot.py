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
        for j in range(p.getNumJoints(self.id)):
            joint_type = p.getJointInfo(self.id, j)[2]
            if joint_type == p.JOINT_PRISMATIC or joint_type == p.JOINT_REVOLUTE:
                p.resetJointState(self.id, j, self.joint_start_positions[i])
                i += 1



    def move(self, env, ee_target_position, ee_target_orientation_e, gripper_open, is_trajectory):

        if self.robot == "sawyer":
            gripper1_index = None
            gripper2_index = None
            gripper_target_position = config.gripper_goal_position_open_sawyer if gripper_open else config.gripper_goal_position_closed_sawyer
        elif self.robot == "franka":
            gripper1_index = 9
            gripper2_index = 10
            gripper_target_position = config.gripper_goal_position_open_franka if gripper_open else config.gripper_goal_position_closed_franka

        min_joint_positions = [p.getJointInfo(self.id, i)[8] for i in range(p.getNumJoints(self.id)) if p.getJointInfo(self.id, i)[2] == p.JOINT_PRISMATIC or p.getJointInfo(self.id, i)[2] == p.JOINT_REVOLUTE]
        max_joint_positions = [p.getJointInfo(self.id, i)[9] for i in range(p.getNumJoints(self.id)) if p.getJointInfo(self.id, i)[2] == p.JOINT_PRISMATIC or p.getJointInfo(self.id, i)[2] == p.JOINT_REVOLUTE]
        joint_ranges = [abs(max_joint_position - min_joint_position) for min_joint_position, max_joint_position in zip(min_joint_positions, max_joint_positions)]
        rest_poses = list((np.array(max_joint_positions) + np.array(min_joint_positions)) / 2)

        ee_target_orientation_q = p.getQuaternionFromEuler(ee_target_orientation_e)

        ee_current_position = p.getLinkState(self.id, self.ee_index, computeForwardKinematics=True)[0]
        ee_current_orientation_q = p.getLinkState(self.id, self.ee_index, computeForwardKinematics=True)[1]
        ee_current_orientation_e = p.getEulerFromQuaternion(ee_current_orientation_q)
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
                pass
            elif self.robot == "franka":
                p.setJointMotorControlArray(self.id, range(7), p.POSITION_CONTROL, targetPositions=target_joint_positions[:-2], forces=[config.arm_movement_force_franka] * 7)
                p.setJointMotorControl2(self.id, gripper1_index, p.POSITION_CONTROL, targetPosition=gripper_target_position, force=config.gripper_movement_force_franka)
                p.setJointMotorControl2(self.id, gripper2_index, p.POSITION_CONTROL, targetPosition=gripper_target_position, force=config.gripper_movement_force_franka)

            env.update()
            self.get_camera_image("head", env, save_camera_image=is_trajectory, rgb_image_path=config.rgb_image_trajectory_path.format(step=self.trajectory_step), depth_image_path=config.depth_image_trajectory_path.format(step=self.trajectory_step))
            if is_trajectory:
                self.trajectory_step += 1

            ee_current_position = p.getLinkState(self.id, self.ee_index, computeForwardKinematics=True)[0]
            ee_current_orientation_q = p.getLinkState(self.id, self.ee_index, computeForwardKinematics=True)[1]
            ee_current_orientation_e = p.getEulerFromQuaternion(ee_current_orientation_q)
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
            camera_position = p.getLinkState(self.id, self.ee_index, computeForwardKinematics=True)[0]
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
