import math
import random

# Simulation
control_dt = 1. / 240.
margin_error = 0.001
gripper_margin_error = 0.0001
joint_margin_error = 0.01
rel_tol = 1e-4
abs_tol = 0.0

# Robots
gripper_goal_position_open_sawyer = 0.2
gripper_goal_position_closed_sawyer = 1.0
arm_movement_force_sawyer = 5 * 240
gripper_movement_force_sawyer = 1000
ee_index_sawyer = 16

gripper_goal_position_open_franka = 0.04
gripper_goal_position_closed_franka = 0.0005
arm_movement_force_franka = 5 * 240
gripper_movement_force_franka = 1000
ee_index_franka = 11

robotiq_motor_joint = 1

# Environment
base_start_position_sawyer = [0.0, 0.0, 0.0]
base_start_orientation_e_sawyer = [0.0, 0.0, math.pi / 2]
joint_start_positions_sawyer = [-0.0304, -2.0563, -1.1631, -0.3829, 1.3152, 0.1496, 1.4462, -0.2288]
base_start_position_franka = [0.0, 0.0, 0.0]
base_start_orientation_e_franka = [0.0, 0.0, math.pi / 2]
joint_start_positions_franka = [0.0, 0.0, 0.0, -1.5708, 0.0, 1.8675, 0.0, 0.04, 0.04]

ee_start_position = [0.0, 0.6, 0.55]
ee_start_orientation_e = [0.0, math.pi, -math.pi / 2]

object_start_position = [random.uniform(-0.2, 0.2), random.uniform(0.4, 0.8), 0.1]
object_start_orientation_e = [0.0, 0.0, random.uniform(-math.pi, math.pi)]

global_scaling = 0.08

# Camera
fov, aspect, near_plane, far_plane = 60, 1.0, 0.01, 100
image_width = 256
image_height = 256

head_camera_position = [0.0, 1.2, 0.6]
head_camera_orientation_e = [0.0, 3 / 4.5 * math.pi, -math.pi / 2]

camera_distance = 0.8
camera_yaw = 225.0
camera_pitch = -30.0
camera_target_position = [0.0, 0.6, 0.3]

wrist_camera_offset_sawyer = 0.125

# Object grasping
point_cloud_top_surface_filter = 0.06
bounding_cube_depth_offset = 0.06
gripper_depth_offset_franka = 0.06
gripper_depth_offset_sawyer = -0.12

# Segmentation
segmentation_threshold = 0.2

# XMem configuration
xmem_config = {
    "top_k": 30,
    "mem_every": 5,
    "deep_update_every": -1,
    "enable_long_term": True,
    "enable_long_term_count_usage": True,
    "num_prototypes": 128,
    "min_mid_term_frames": 5,
    "max_mid_term_frames": 10,
    "max_long_term_elements": 10000,
}

xmem_visualise_every = 1
xmem_output_every = 1
xmem_lm_input_every = 20

# Multiprocessing
CAPTURE_IMAGES = 1
ADD_BOUNDING_CUBES = 2
ADD_TRAJECTORY_POINTS = 3
EXECUTE_TRAJECTORY = 4
OPEN_GRIPPER = 5
CLOSE_GRIPPER = 6
TASK_COMPLETED = 7
RESET_ENVIRONMENT = 8

# Paths
rgb_image_wrist_path = "./images/rgb_image_wrist.png"
depth_image_wrist_path = "./images/depth_image_wrist.png"
rgb_image_head_path = "./images/rgb_image_head.png"
depth_image_head_path = "./images/depth_image_head.png"
rgb_image_trajectory_path = "./images/trajectory/rgb_image_{step}.png"
depth_image_trajectory_path = "./images/trajectory/depth_image_{step}.png"
bounding_cube_mask_image_path = "./images/bounding_cube_mask_{object}_{mask}.png"

langsam_image_path = "./images/langsam_image_{object}.png"
xmem_input_path = "./images/xmem_input.png"
xmem_output_path = "./images/xmem_output_{step}.png"

# Output
OK = "\033[92m"
PROGRESS = "\033[93m"
FAIL = "\033[91m"
ENDC = "\033[0m"
