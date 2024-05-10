import numpy as np
class Total_value:
    def __init__(self,yaw,world_coords):
    	self.header = 0
    	self.tracking = 0
    	self.id = 0
    	self.armors_num = 0
    	self.vx = 0
    	self.vy = 0
    	self.v_yaw = 0
    	self.r1 = 0
    	self.r2 = 0
    	self.dz = 0
    	self.checksum = 0
    	self.yaw = yaw
    	self.x = world_coords[0][0]
    	self.y = world_coords[1][0]
    	self.z = world_coords[2][0]
    	
def pixel_to_world(u, v, d, K, R, t):
    """
    将像素坐标转换为世界坐标。

    参数:
    u, v : float
        像素坐标。
    d : float
        从相机到点的深度。
    K : np.ndarray
        相机内参矩阵（3x3）。
    R : np.ndarray
        从世界坐标到相机坐标的旋转矩阵（3x3）。
    t : np.ndarray
        从世界坐标到相机坐标的平移向量（3x1）。

    返回:
    np.ndarray
        点的世界坐标（3x1）。
    """
    # 创建像素的齐次坐标
    uv1 = np.array([[u], [v], [1]])  # 修改为列向量

    # 求相机内参矩阵的逆
    inv_K = np.linalg.inv(K)

    # 将像素坐标转换为相机坐标
    cam_coords = d * np.dot(inv_K, uv1)

    # 求外参的逆，用以从相机坐标转换为世界坐标
    inv_R = R.T  # 旋转矩阵的逆是其转置
    inv_t = -np.dot(inv_R, t)

    # 计算世界坐标
    world_coords = np.dot(inv_R, cam_coords) + inv_t

    return world_coords


import math


def calculate_yaw(u, v, d, K, R, t):
    """
    计算目标点的yaw值，即相机左右转动的角度。

    参数:
    u, v : float
        像素坐标。
    d : float
        从相机到点的深度。
    K : np.ndarray
        相机内参矩阵（3x3）。
    R : np.ndarray
        从世界坐标到相机坐标的旋转矩阵（3x3）。
    t : np.ndarray
        从世界坐标到相机坐标的平移向量（3x1）。

    返回:
    float
        目标点的yaw角度（单位：度）。
    """
    # 创建像素的齐次坐标
    uv1 = np.array([[u], [v], [1]])  # 列向量形式

    # 求相机内参矩阵的逆
    inv_K = np.linalg.inv(K)

    # 将像素坐标转换为相机坐标
    cam_coords = d * np.dot(inv_K, uv1)

    # 从相机坐标中提取x和z坐标
    x = cam_coords[0, 0]
    z = cam_coords[2, 0]

    # 计算yaw角度
    yaw_rad = math.atan2(x, z)
    yaw_deg = math.degrees(yaw_rad)  # 将弧度转换为度

    return yaw_deg


def get_fixed_depth(R_center, rect_center):  # 获得目标点的深度
    distance = 6.438 + 2.013  # 摄像头到能量机关的地面距离
    r_height = 2.385  # 能量机关R中心距地面的距离
    x = 0.8  # 摄像头离车底的高度 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    altitude_difference = x + 0.945  # 摄像头离地面的距离 !!!!!!!!!!!!!!!!!!!!!!!!!!
    R_to_rectcenter = 0.862  # R与目标点的实际距离!!!!!!!!!!!!!!!!!!!!!!!!!!

    dis = (abs(R_center[0] - rect_center[0]) ** 2 + abs(R_center[1] - rect_center[1]) ** 2) ** 0.5  # R与目标点的像素距离
    if rect_center[0] >= R_center[0] and rect_center[1] <= R_center[1]:
        height_diff = R_to_rectcenter * abs(R_center[1] - rect_center[1]) / dis
    # 第二象限
    if rect_center[0] <= R_center[0] and rect_center[1] <= R_center[1]:
        height_diff = R_to_rectcenter * abs(R_center[1] - rect_center[1]) / dis
    # 第三象限
    if rect_center[0] <= R_center[0] and rect_center[1] >= R_center[1]:
        height_diff = -(R_to_rectcenter * abs(R_center[1] - rect_center[1]) / dis)
    # 第四象限
    if rect_center[0] >= R_center[0] and rect_center[1] >= R_center[1]:
        height_diff = -(R_to_rectcenter * abs(R_center[1] - rect_center[1]) / dis)

    h = r_height + height_diff
    depth = (abs(h - altitude_difference) ** 2 + distance ** 2) ** 0.5  # 目标点的深度
    return depth


def Message_to_send(R_center, rect_center):
    # 示例用法
    # 相机内参矩阵
    K = np.array([[2351.57417, 0, 711.028],
                  [0, 2346.18899, 539.03119],
                  [0, 0, 1]])

    # 从世界坐标到相机坐标的旋转矩阵（3x3）
    R = np.array([[1., 0., 0.],
                  [0., 1., 0.],
                  [0., 0., 1.]])

    # 从世界坐标到相机坐标的平移向量（3x1）
    t = np.array([[0.],
                  [0.],
                  [0.]])
    d = get_fixed_depth(R_center, rect_center)
    # 像素坐标和深度
    u, v = 400, 300  # 示例值

    # 计算世界坐标
    world_coords = pixel_to_world(u, v, d, K, R, t)
    yaw = calculate_yaw(u, v, d, K, R, t)
    value = Total_value(yaw, world_coords)
    return value

    # print("世界坐标:", world_coords)

