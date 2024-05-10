import math
import cv2
import numpy as np
from utils.parameterUtils import Parameter

# 红色时用的漫水填充法
def floodfill(src):
    inv = cv2.bitwise_not(src)
    tmp = inv.copy()
    h, w = inv.shape[:2]
    mask = np.zeros((h + 2, w + 2), np.uint8)
    cv2.floodFill(inv, mask, (0, 0), 0)
    floodfilled_ = inv
    if cv2.countNonZero(floodfilled_) > h * w * 0.5:
        cv2.floodFill(tmp, mask, (w - 1, h - 1), 0)
        inv = tmp
    return floodfilled_


# 找到中心r,返回r的bbox
def findR(src):

    global rect_last

    contours, hierarchy = cv2.findContours(src, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    armors = []
    maxarea = 0
    num = 0
    ansnum = 0

    for i, contour in enumerate(contours):

        epsilon = 0.1*cv2.arcLength(contour,True) 
        approx = cv2.approxPolyDP(contour,epsilon,True)


        # 根据轮廓点数量筛选
        if  len(approx) >= 5 or len(approx) <= 3:
            continue

        # 找到没有父轮廓的轮廓
        if hierarchy[0][i][3] >= 0 and hierarchy[0][i][3] < len(approx):
            continue

        # 找没子轮廓的
        if hierarchy[0][i][2] >= 0 and hierarchy[0][i][2] < len(approx):
            continue

        rect = cv2.minAreaRect(approx)
        area = cv2.contourArea(approx)

        if 100 < area < 1000:
            width, height = rect[1]
            ratio = width / height
            if 0.5 < ratio < 2:
                num += 1
                if (area >= maxarea):
                    maxarea = area
                    ansnum = num - 1
                armors.append(rect)

    if (armors):

        # 获取装甲板的四个角点
        points = cv2.boxPoints(armors[ansnum])
        # 将点转换为易于操作的形式
        points = np.intp(points)

        # 寻找最左上角的点：即x最小和y最小的点
        top_left = points.min(axis=0)

        # 寻找最右下角的点：即x最大和y最大的点
        bottom_right = points.max(axis=0)

        rect = [top_left[0], top_left[1], bottom_right[0] - top_left[0], bottom_right[1] - top_left[1]]

        rect_last = rect

        return rect 
    
    else:
        # 如果没有检测到装甲板，则返回 None 
        print('lost track')
        return None


# 计算特征“叉”的质点（重心）
def calculate_center(contours, maxId):
    rect = cv2.moments(contours[maxId], False)
    rectmid = (rect['m10'] / rect['m00'], rect['m01'] / rect['m00'])
    return rectmid

# 计算扇叶bbox
def get_blade_rect(src, center):
    #重点
    multiple = 1.5  # 倍率,根据能量机关大小来调整!!!!!!!!!
 
    contours, hierarchy = cv2.findContours(src, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    maxArea = -1
    maxId = -1
    
    # cv2.imshow('test',img0)
    # cv2.waitKey(0)

    for i, contour in enumerate(contours): 
         
        area = cv2.contourArea(contour)
        lenth = cv2.arcLength(contour,True)

        # 面积排除噪声
        if area < 1000 or area > 8000:
            continue

        # 找到没有父轮廓的轮廓
        if hierarchy[0][i][3] >= 0 and hierarchy[0][i][3] < len(contours):
            continue

        # 找没子轮廓的
        if hierarchy[0][i][2] >= 0 and hierarchy[0][i][2] < len(contours):
            continue

        # 周长与面积比值筛选
        if area/lenth < 5 or area/lenth > 7:
            continue

        # 找面积最大的轮廓
        if maxArea <= area:
            maxArea = area
            maxId = i
 
        # 控制误差范围
        if area <= maxArea + 50 and area >= maxArea - 50:
            maxArea = area
            maxId = i

    if maxId != -1:
        rectMid = calculate_center(contours, maxId)
        # cv2.drawContours(img,contours,maxId,(0,255,0),2)

        # 计算目标点
        # 第一象限
        if rectMid[0] >= center[0] and rectMid[1] <= center[1]:
            target = (center[0] + (rectMid[0] - center[0]) * multiple, center[1] - (center[1] - rectMid[1]) * multiple)

        # 第二象限
        if rectMid[0] <= center[0] and rectMid[1] <= center[1]:
            target = (center[0] - (center[0] - rectMid[0]) * multiple, center[1] - (center[1] - rectMid[1]) * multiple)

        # 第三象限
        if rectMid[0] <= center[0] and rectMid[1] >= center[1]:
            target = (center[0] - (center[0] - rectMid[0]) * multiple, center[1] + (rectMid[1] - center[1]) * multiple)

        # 第四象限
        if rectMid[0] >= center[0] and rectMid[1] >= center[1]:
            target = (center[0] + (rectMid[0] - center[0]) * multiple, center[1] + (rectMid[1] - center[1]) * multiple)
        
        distance = math.sqrt((target[0] - center[0]) ** 2 + (target[1] - center[1]) ** 2)
        armorlength = distance * 0.57
        sinrect = abs(target[1] - center[1]) / distance
        cosrect = abs(target[0] - center[0]) / distance
        rectlength_half = math.sqrt((sinrect * armorlength) ** 2 + (cosrect * armorlength) ** 2) / 2
        rect = [int(target[0] - rectlength_half), int(target[1] - rectlength_half), int(rectlength_half * 2),
                int(rectlength_half * 2)]
    

        return rect
    
    else:
        return None


def start_tracker(frame, color):                      
    if frame is not None:

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #调hsv上下限
        lower_limit = np.array([0, 5, 220])
        upper_limit = np.array([85, 100, 255])

        # 根据上下限创建掩码
        mask = cv2.inRange(hsv, lower_limit, upper_limit)
        mask2 = cv2.inRange(hsv, lower_limit, upper_limit)

        #这里kernel系数要修正
        mask = cv2.dilate(mask, np.ones((5,5), np.uint8), iterations=1)
        mask2 = cv2.dilate(mask, np.ones((4,4), np.uint8), iterations=1)

        '''                               
        #如果无法正确得到hsv,那就洪水填充
        if color == 'red':
            mask = floodfill(mask)
        '''

        R_rect = findR(mask2)  # R的矩形bbox

        if R_rect:

            center = [R_rect[0] + R_rect[2] / 2, R_rect[1] + R_rect[3] / 2]  # R的中心坐标
            rect = get_blade_rect(mask, center)  # 扇叶的矩形bbox

        return R_rect, rect
    
    else:

        return None, None
