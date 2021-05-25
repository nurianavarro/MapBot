# -*- coding: utf-8 -*-
import numpy as np
import cv2
import matplotlib.pyplot as plt
import math
from scipy.spatial.distance import cdist
from PIL import Image, ImageFilter, ImageDraw
import open3d as o3d

f=284.21 #focal
T=152 #distancia entre camaras
window_size = 45
search_size = 100
perspectiveAngle=60
width=512 #img width
height=256 #img height

"""
Depth_map
"""

def disparity_to_depth_map(disparity_matrix, scale):

	disparity_matrix = disparity_matrix.astype(np.float64)
	disparity_matrix = (f/scale) * (T/scale) / disparity_matrix
	disparity_matrix = disparity_matrix.astype(np.uint16)
	return disparity_matrix
"""
Mapeado
"""
def point_cloud(depth_map, color_img, fx, fy, cx, cy, scale):

	fx = fx/scale
	fy = fy/scale
	cx = cx/scale
	cy = cy/scale

	shape = color_img.shape;
	h = shape[0]
	w = shape[1]
	
	img = o3d.geometry.Image(color_img.astype('uint8'))
	depth = o3d.geometry.Image(depth_map.astype('uint16'))

	rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(img, depth)

	o3d_pinhole = o3d.camera.PinholeCameraIntrinsic()
	o3d_pinhole.set_intrinsics(w, h, fx, fy, cx, cy)

	pcd_from_depth_map = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d_pinhole)
	pcd_from_depth_map.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
# 	o3d.visualization.draw_geometries([pcd_from_depth_map])
	return np.asarray(pcd_from_depth_map.points)


def match_clouds(x, y, alfa, points):
    
    alfa = math.radians(alfa)
    
    R = np.array([[math.cos(alfa), - math.sin(alfa)],
                  [math.sin(alfa), math.cos(alfa)]])
    
    p = np.transpose(R @ np.transpose(points[:,[0,2]]))
    
    points[:,0] = p[:,0] + x
    points[:,2] = p[:,1] + y
    
    return points

def mapper3d(imgL, imgR, mapbot, mapa):
    
    imgL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    imgR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)
    stereo = cv2.StereoBM_create(numDisparities=16, blockSize=7)
    disparity = stereo.compute(imgL,imgR)
    # plt.imshow(disparity,'gray')
    # plt.show()
    depth = disparity_to_depth_map(disparity, 2)
    points = point_cloud(depth, imgL, 5814.40964, 5814.40964, 214.87, 120.4, 2)
    if mapa.shape[0]>1:
        points = match_clouds(mapbot.x, mapbot.y, mapbot.mapeado, points)
    mapa = np.concatenate((mapa, points), axis=0)
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(mapa)
    o3d.visualization.draw_geometries([pcd])
    return mapa

    
def scattered_map(kp):
    fig = plt.figure(figsize = (10, 7))
    ax = plt.axes(projection ="3d")
    ax.scatter3D(kp[:,0], kp[:,1], kp[:,2], color = "green")
    plt.show()
    
    