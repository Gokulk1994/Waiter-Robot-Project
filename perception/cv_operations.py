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
    elif color == ItemColor.Black:
        hsv_color = [
            np.array([0, 0, 0]),
            np.array([180, 255, 10]),
        ]
    elif color == ItemColor.White:
        pass
    elif color == ItemColor.Yellow:
        hsv_color = [
            np.array([20, 100, 100]),
            np.array([30, 255, 255]),
        ]
    elif color == ItemColor.Green:
        hsv_color = [np.array([50, 100, 100]),
                     np.array([70, 255, 255])]
    else:
        print("HSV for given color is not available")
        return False
    return hsv_color


def show_images(image_list):
    if len(image_list) > 0:

        for i in range(len(image_list)):
            if len(image_list[i].shape) < 3:
                image_list[i] = cv.cvtColor(image_list[i], cv.COLOR_GRAY2BGR)

        op_img = np.hstack([x for x in image_list])


        cv.imshow('Perception', op_img)
        cv.waitKey(0)


def get_image_mask(hsv, hsv_values, vertices):
    #blur = cv.GaussianBlur(hsv,(5,5),cv.BORDER_DEFAULT)
    mask = cv.inRange(hsv, hsv_values[0], hsv_values[1])
    if len(hsv_values) == 4:
        mask2 = cv.inRange(hsv, hsv_values[2], hsv_values[3])
        mask += mask2

    cv.imshow('without mask', mask)
    cv.waitKey(1)

    if vertices is not None:
        roi_mask = np.zeros_like(mask)
        roi_mask = cv.fillPoly(roi_mask, vertices, (255, 255, 255))

        cv.imshow('roi', roi_mask)
        cv.waitKey(1)

        mask = cv.bitwise_and(mask, roi_mask)

        cv.imshow('with roi mask', mask)
        cv.waitKey(1)

    return mask


def destroy_windows(self):
    cv.destroyAllWindows()


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

        if camera_name == "kitchen_camera":
            #self.vertices_1 = np.array([[(120, 260), (180, 55), (350, 55), (530, 260)]], dtype=np.int32)
            self.vertices_1 = np.array([[(325, 260), (280, 55), (350, 55), (530, 260)]], dtype=np.int32)
            self.vertices_2 = np.array([[(120, 260), (180, 55), (280, 55), (325, 260)]], dtype=np.int32)
        else:
            self.vertices_1 = None
            self.vertices_2 = None

    def set_point_clouds(self):
        [self.rgb, self.depth] = self.S.getImageAndDepth()
        self.points = self.S.depthData2pointCloud(self.depth, self.fxfypxpy)
        self.cameraFrame.setPointCloud(self.points, self.rgb)
        self.V.recopyMeshes(self.C)
        self.V.setConfiguration(self.C)

    def get_target_pos(self, color, position, show_img=True, ):
        [self.rgb, self.depth] = self.S.getImageAndDepth()
        self.points = self.S.depthData2pointCloud(self.depth, self.fxfypxpy)

        bgr = cv.cvtColor(self.rgb, cv.COLOR_RGB2BGR)
        hsv = cv.cvtColor(bgr, cv.COLOR_BGR2HSV)

        hsv_values = get_hsv_values(color)
        if position == 1:
            final_mask = get_image_mask(hsv, hsv_values, self.vertices_1)
        else:
            final_mask = get_image_mask(hsv, hsv_values, self.vertices_2)

        contours, hierarchy = cv.findContours(final_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        segmented_image = cv.drawContours(bgr, contours, -1, (0, 0, 255), 1)
        image_list = [segmented_image]

        if show_img:
            show_images(image_list)

        return self.get_mean_point_cloud(final_mask)

    def get_mean_point_cloud(self, mask):
        pc_coord = []
        obj_position = []

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

    def update_rgb_image(self):
        [rgb, depth] = self.S.getImageAndDepth()
        points = self.S.depthData2pointCloud(depth, self.fxfypxpy)
        self.cameraFrame.setPointCloud(points, rgb)
        self.V.recopyMeshes(self.C)
        self.V.setConfiguration(self.C)
