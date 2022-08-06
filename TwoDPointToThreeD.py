
import numpy as np
from numpy import sin, cos, tan
import cv2

# The transformation matrix of R is calculated by entering the left side of the transformation
def calc_R(pitch, yaw, roll):
    a = np.radians(pitch)
    b = np.radians(yaw)
    c = np.radians(roll)

    R_x = np.asarray([
        [1, 0, 0],
        [0, cos(a), -sin(a)],
        [0, sin(a), cos(a)],
    ])

    R_y = np.asarray([
        [cos(b), 0, sin(b)],
        [0, 1, 0],
        [-sin(b), 0, cos(b)],
    ])

    R_z = np.asarray([
        [cos(c), -sin(c), 0],
        [sin(c), cos(c), 0],
        [0, 0, 1],
    ])

    R = np.dot(R_z, np.dot(R_y, R_x))

    return R

# Calculated camera parameter K,
# fov_x is Cape View
# pixel is Resolution
def calc_K(fov_x, pixel_w, pixel_h, cx=None, cy=None):
    if cx is None:
        cx = pixel_w / 2.0
    if cy is None:
        cy = pixel_h / 2.0

    fx = 1.0 / (2.0 * tan(np.radians(fov_x) / 2.0)) * pixel_w
    fy = fx

    K = np.asarray([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1],
    ])

    return K

# 2D to 3D
# u ,v is the 2D poitn
# z is the z-axis
# R and t is the camera position
# K is the camera parameter
def convert_uvz_to_xyz(u, v, z, R, t, K):
    K_inv = np.linalg.inv(K)

    # in screen coord
    cs = np.asarray([u, v, 1])
    cs_ = cs * z

    # in camera coord
    cc = np.dot(K_inv, cs_)

    # in world coord
    cw = np.dot(R, cc) + t

    return cw

def convert_(x, y):
    pitch = 0
    yaw = 0
    roll = 0
    R = calc_R(pitch, yaw, roll)

    fov_x = 86
    pixel_w = 1912
    pixel_h = 1160
    K = calc_K(fov_x, pixel_w, pixel_h, cx=None, cy=None)

    # t is the Coordinates of camera
    t = np.array([1.0, 1.0, 0])
    z = 3
    return(convert_uvz_to_xyz(x, y, z, R, t, K))




def main_func(u = 1280, v = 960, z = 3):
    # u , v, z is the point of image
    #dummy paras

    def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            t_ = convert_(x, y)
            x3d = t_[0]
            y3d = t_[1]
            z3d = t_[2]
            xy = "[%d,%d],[%.2f,%.2f,%.2f]" % (x, y, x3d, y3d, z3d)
            print(xy)
            cv2.circle(img, (x, y), 1, (0, 0, 255), thickness=-1)
            cv2.putText(img, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,
                        1.0, (0, 0, 0), thickness=1)
            print(x, y)

    img = cv2.imread('test.jpg')
    cv2.namedWindow("image")
    cv2.setMouseCallback("image", on_EVENT_LBUTTONDOWN)
    cv2.imshow("image", img)
    cv2.waitKey(0)


if __name__ == '__main__':
    main_func()
