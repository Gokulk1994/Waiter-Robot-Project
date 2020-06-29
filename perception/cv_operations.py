import cv2 as cv
from enumeration.enum_classes import ItemColor
import numpy as np


def get_hsv_values(color):
    if color == ItemColor.Red:
        hsv_color = [
            np.array([0, 120, 70]),
            np.array([10, 255, 255]),
            np.array([170, 120, 70]),
            np.array([180, 255, 255])
        ]
    elif color == ItemColor.Blue:
        pass
    elif color == ItemColor.White:
        pass
    elif color == ItemColor.Yellow:
        hsv_color = [
            np.array([20, 100, 100]),
            np.array([30, 255, 255]),
        ]
    elif color == ItemColor.Blue:
        pass
    else:
        print("HSV for given color is not available")
        return False
    return hsv_color


def show_images(image_list):
    if len(image_list) > 0:

        for i in range(len(image_list)):
            if len(image_list[i].shape) < 3:
                print(len(image_list[i].shape))
                image_list[i] = cv.cvtColor(image_list[i], cv.COLOR_GRAY2BGR)

        op_img = np.hstack([x for x in image_list])

        while True:
            cv.imshow('Perception', op_img)

            if cv.waitKey(1) & 0xFF == ord('q'):
                break


def get_image_mask(hsv, hsv_values):
    mask = cv.inRange(hsv, hsv_values[0], hsv_values[1])
    if len(hsv_values) == 4:
        mask2 = cv.inRange(hsv, hsv_values[2], hsv_values[3])
        mask += mask2
    return mask


class CV_Perception:
    def __init__(self, env, camera_name, focal_len=0.895, cam_w=320.0, cam_h=180.):

        self.S = env.S
        self.V = env.V
        self.C = env.C

        self.S.selectSensor(camera_name)
        [self.rgb, self.depth] = self.S.getImageAndDepth()
        # the focal length
        self.f = focal_len
        self.f = self.f * 360.0
        self.fxfypxpy = [self.f, self.f, cam_w, cam_h]
        self.cameraFrame = self.C.frame(camera_name)
        self.points = self.S.depthData2pointCloud(self.depth, self.fxfypxpy)
        self.V.recopyMeshes(self.C)
        self.V.setConfiguration(self.C)

    def set_point_clouds(self):
        [self.rgb, self.depth] = self.S.getImageAndDepth()
        self.points = self.S.depthData2pointCloud(self.depth, self.fxfypxpy)
        self.cameraFrame.setPointCloud(self.points, self.rgb)
        self.V.recopyMeshes(self.C)
        self.V.setConfiguration(self.C)

    def get_target_pos(self, color, show_img=True):
        [self.rgb, self.depth] = self.S.getImageAndDepth()
        self.points = self.S.depthData2pointCloud(self.depth, self.fxfypxpy)

        bgr = cv.cvtColor(self.rgb, cv.COLOR_RGB2BGR)
        hsv = cv.cvtColor(bgr, cv.COLOR_BGR2HSV)

        hsv_values = get_hsv_values(color)
        hsv_mask = get_image_mask(hsv, hsv_values)

        contours, hierarchy = cv.findContours(hsv_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        segmented_image = cv.drawContours(bgr, contours, -1, (0, 255, 0), 2)
        image_list = [bgr]

        if show_img:
            show_images(image_list)

        return self.get_mean_point_cloud(hsv_mask)

    def get_mean_point_cloud(self, mask):
        pc_coord = []
        obj_position = None

        if mask is not None:
            x, y = np.where(mask)
            full_depth = np.zeros(self.points.shape)

            for i, j in zip(x, y):
                full_depth[i][j] = self.points[i][j]
                pc_coord.append(self.points[i][j])

            if len(x) > 0:
                pc_coord = (np.array(pc_coord))
                mean_pc = np.mean(pc_coord, axis=0)
                self.cameraFrame.setPointCloud(full_depth, self.rgb)

                obj_position = self.cameraFrame.getRotationMatrix() @ mean_pc + self.cameraFrame.getPosition()

        return obj_position
