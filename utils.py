import pybullet as p
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import torch
import math
import config
from PIL import Image
from torchvision.utils import save_image
from shapely.geometry import MultiPoint, Polygon, polygon

def get_segmentation_mask(model_predictions, segmentation_threshold):

    masks = []

    for model_prediction in model_predictions:
        model_prediction_np = model_prediction.detach().cpu().numpy()
        segmentation_threshold = np.max(model_prediction_np) - segmentation_threshold * (np.max(model_prediction_np) - np.min(model_prediction_np))
        model_prediction[model_prediction < segmentation_threshold] = False
        model_prediction[model_prediction >= segmentation_threshold] = True
        masks.append(model_prediction)

    return masks



def get_max_contour(image, image_width, image_height):

    ret, thresh = cv.threshold(image, 127, 255, 0)
    contours, hierarchy = cv.findContours(thresh, 1, 2)

    contour_index = None
    max_length = 0
    for c, contour in enumerate(contours):
        contour_points = [(c, r) for r in range(image_height) for c in range(image_width) if cv.pointPolygonTest(contour, (c, r), measureDist=False) == 1]
        if len(contour_points) > max_length:
            contour_index = c
            max_length = len(contour_points)

    if contour_index is None:
        return None

    return contours[contour_index]



def get_intrinsics_extrinsics(image_height, camera_position, camera_orientation_q):

    fov = (config.fov / 360) * 2 * math.pi
    f_x = f_y = image_height / (2 * math.tan(fov / 2))
    K = np.array([[f_x, 0, 0], [0, f_y, 0], [0, 0, 1]])

    R = np.array(p.getMatrixFromQuaternion(camera_orientation_q)).reshape(3, 3)
    Rt = np.hstack((R, np.array(camera_position).reshape(3, 1)))
    Rt = np.vstack((Rt, np.array([0, 0, 0, 1])))

    return K, Rt



def save_xmem_image(masks):

    xmem_array = np.array(Image.open(config.xmem_input_path).convert("L"))
    xmem_array = np.unique(xmem_array, return_inverse=True)[1].reshape(xmem_array.shape)

    for mask in masks:
        mask_index = np.max(xmem_array) + 1
        xmem_array[mask.detach().cpu().numpy().astype(bool)] = mask_index

    xmem_array = xmem_array / np.max(xmem_array)

    save_image(torch.Tensor(xmem_array), config.xmem_input_path)



def get_bounding_cube_from_point_cloud(image, masks, depth_array, camera_position, camera_orientation_q, segmentation_count):

    image_width, image_height = image.size

    bounding_cubes = []
    bounding_cubes_orientations = []

    for i, mask in enumerate(masks):

        save_image(mask, config.bounding_cube_mask_image_path.format(object=segmentation_count, mask=i))
        mask_np = cv.imread(config.bounding_cube_mask_image_path.format(object=segmentation_count, mask=i), cv.IMREAD_GRAYSCALE)

        contour = get_max_contour(mask_np, image_width, image_height)

        if contour is not None:

            contour_pixel_points = [(c, r, depth_array[r][c]) for r in range(image_height) for c in range(image_width) if cv.pointPolygonTest(contour, (c, r), measureDist=False) == 1]
            contour_world_points = [get_world_point_world_frame(camera_position, camera_orientation_q, "head", image, pixel_point) for pixel_point in contour_pixel_points]
            max_z_coordinate = np.max(np.array(contour_world_points)[:, 2])
            min_z_coordinate = np.min(np.array(contour_world_points)[:, 2])
            top_surface_world_points = [world_point for world_point in contour_world_points if world_point[2] > max_z_coordinate - config.point_cloud_top_surface_filter]

            rect = MultiPoint([world_point[:2] for world_point in top_surface_world_points]).minimum_rotated_rectangle
            if isinstance(rect, Polygon):
                rect = polygon.orient(rect, sign=-1)
                box = rect.exterior.coords
                box = np.array(box[:-1])
                box_min_x = np.argmin(box[:, 0])
                box = np.roll(box, -box_min_x, axis=0)
                box_top = [list(point) + [max_z_coordinate] for point in box]
                box_btm = [list(point) + [min_z_coordinate] for point in box]
                box_top.append(list(np.mean(box_top, axis=0)))
                box_btm.append(list(np.mean(box_btm, axis=0)))
                bounding_cubes.append(box_top + box_btm)

                # Calculating rotation in world frame
                bounding_cubes_orientation_width = np.arctan2(box[1][1] - box[0][1], box[1][0] - box[0][0])
                bounding_cubes_orientation_length = np.arctan2(box[2][1] - box[1][1], box[2][0] - box[1][0])
                bounding_cubes_orientations.append([bounding_cubes_orientation_width, bounding_cubes_orientation_length])

    bounding_cubes = np.array(bounding_cubes)

    return bounding_cubes, bounding_cubes_orientations



def get_world_point_world_frame(camera_position, camera_orientation_q, camera, image, point):

    image_width, image_height = image.size

    K, Rt = get_intrinsics_extrinsics(image_height, camera_position, camera_orientation_q)

    pixel_point = np.array([[point[0] - (image_width / 2)], [(image_height / 2) - point[1]], [1.0]])

    if camera == "wrist":
        pixel_point = [pixel_point[1], pixel_point[0], pixel_point[2]]
    elif camera == "head":
        pixel_point = [-pixel_point[1], -pixel_point[0], pixel_point[2]]

    world_point_camera_frame = (np.linalg.inv(K) @ pixel_point) * point[2]
    world_point_world_frame = Rt @ np.vstack((world_point_camera_frame, np.array([1.0])))
    world_point_world_frame = world_point_world_frame.squeeze()[:-1]

    return world_point_world_frame
