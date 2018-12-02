import numpy as np
import math
import random
import cv2
import os, os.path
import sys
import time
import pandas as pd
from depth_image_processing import *
from obstacle import Obstacle
#from ros_publish import send_obstacle_data

def get_obstacles_with_plane(depth_frame, num_planes, num_points, dist_thresh, offset, visualize):
    obstacles = np.zeros(depth_frame.shape) #empty image that will store the locations of detected obstacles
    img = np.uint8(depth_frame) #some opencv functions require a byte image
    kernel = np.ones((3,3),np.uint8)
    _, foreground = cv2.threshold(depth_frame,0.5,1.5, cv2.THRESH_TOZERO)
    cropped = np.zeros(depth_frame.shape, np.float32)
    cropped[depth_frame.shape[0]//4:] = depth_frame[depth_frame.shape[0]//4:]
    xyz_arr = depthMatrixToPointCloudPos(cropped) #convert depth data to XYZ coordinates
    #print(fit_plane_optimize(xyz_arr.T))
    start_time = time.time()
    center, plane, theta = get_orientation(xyz_arr, num_planes, num_points)
    #center, plane, theta = fit_plane_optimize(reject_outliers(xyz_arr))
    CameraPosition['elevation'] = -theta
    center = applyCameraOrientation(center)
    plane = applyCameraOrientation(plane)
    xyz_arr = applyCameraMatrixOrientation(xyz_arr)
    plane_img = np.zeros(len(xyz_arr))
    plane_img[xyz_arr[:,2] > dist_thresh - center[2] - offset] = 1
    if visualize:
        #xyz_arr = xyz_arr[xyz_arr[:,2] > dist_thresh - center[2]]
        points = pd.DataFrame(xyz_arr, columns=['x', 'y', 'z']) #convert XYZ coordinates to a DataFrame for pyntcloud
        cloud = PyntCloud(points)

        lines = [
            {
                # X axis
                "color": "red",
                "vertices": [[0, 0, 0], [10, 0, 0]]
            },
            {
                # Y axis
                "color": "green",
                "vertices": [[0, 0, 0], [0, 10, 0]]
            },
            {
                # Z axis
                "color": "blue",
                "vertices": [[0, 0, 0], [0, 0, 10]]
            },
            {
                #Original norm of the plane
                "color": "pink",
                "vertices": [center.tolist(), plane.tolist()]
            }
        ]
        cloud.plot(cmap="cool", polylines=lines)

    plane_img = np.uint8(np.reshape(plane_img,(424,512)) * 255) #reshape to match depth data and convert to uint8
    plane_img = np.uint8((np.ones((424,512)) * 255) - plane_img) #invert img so pixel value corresponds to NOT ground plane
    ret, plane_img = cv2.threshold(plane_img,0,255,cv2.THRESH_BINARY) #filter points that are probaly not ground plane
    plane_img = cv2.subtract(img, plane_img)

    #noise removal
    kernel = np.ones((3,3),np.uint8)
    #opening = cv2.morphologyEx(plane_img ,cv2.MORPH_OPEN, kernel, iterations = 6) #erosion followed by dilation
    erode = cv2.erode(plane_img, kernel, iterations=4)
    _, thresh = cv2.threshold(erode,0,255,cv2.THRESH_BINARY)
    color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) #BGR image to draw labels on

    #begin contour detection
    image, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    color = cv2.drawContours(color, contours, -1, (0,255,0), 1)
    for cntr in contours:
        try:
            #calculate diamter of equivalent circle
            #this measurement is only used for checking if countours fit our bounds
            area = cv2.contourArea(cntr)
            equi_diameter = np.sqrt(4*area/np.pi)

            x,y,obj_length,obj_height = cv2.boundingRect(cntr)

            equi_diameter = obj_length #bounding rectangle gives a better approximation of diameter

            #Hardcoded Diameter Range in pixels
            LOW_DIAMETER_BOUND = 20
            HIGH_DIAMETER_BOUND = 200

            HIGH_MM_BOUND = 400

            HIGH_DISTANCE_BOUND = 6000
            #Original tolerances were 20 and 150

            if(equi_diameter > LOW_DIAMETER_BOUND and equi_diameter < HIGH_DIAMETER_BOUND):
                mask = np.zeros_like(img) #mask will contain the fitted and adjusted ellipse of a single obstacle
                ellipse = cv2.fitEllipse(cntr)
                rect = cv2.minAreaRect(cntr)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                mask = cv2.ellipse(mask,ellipse,(255,255,255),-1) #draw the fitted ellipse
                rows = mask.shape[0]
                cols = mask.shape[1]
                M = np.float32([[1,0,0],[0,1,equi_diameter/4]]) #shift mask down to match obstacle, not edge
                mask = cv2.warpAffine(mask,M,(cols,rows))
                mask = cv2.erode(mask, kernel, iterations=3) #erode the mask to remove background points
                img_fg = cv2.bitwise_and(depth_frame,depth_frame,mask = mask) #use the mask to isolate original depth values
                img_fg = cv2.medianBlur(img_fg,5) #median blur to further remove noise
                obstacles = cv2.add(np.float32(img_fg), np.float32(obstacles))

                mean_val = np.median(img_fg[img_fg.nonzero()]) #compute the non-zero average of obstacle depth values
                min_val, distance_to_object, min_loc, max_loc = cv2.minMaxLoc(img_fg)
                mm_diameter = (equi_diameter) * (1.0 / CameraParams['fx']) * mean_val #convert pixel diameter to mm


                moment = cv2.moments(cntr) #get the centroid of the obstacle using its moment
                cx = int(moment['m10']/moment['m00'])
                cy = int(moment['m01']/moment['m00'])

                if mean_val < HIGH_DISTANCE_BOUND and mm_diameter < HIGH_MM_BOUND: #kinect loses accuracy beyond 4.5m
                    coords = depthToPointCloudPos(cx, cy, mean_val) #convert obstacle depth to XYZ coordinate
                    #begin visualization
                    cv2.drawContours(color,[box],0,(0,0,255),1)
                    cv2.rectangle(color,(x,y),(x+obj_length,y+obj_height),(0,255,0),2)
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(color, "x" + str(coords[0]), (cx,cy+30), font, 0.4, (0, 0, 255), 1, cv2.LINE_AA)
                    cv2.putText(color, "y" + str(coords[1]), (cx,cy+45), font, 0.4, (0, 255, 0), 1, cv2.LINE_AA)
                    cv2.putText(color, "z" + str(mean_val), (cx,cy+60), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)
                    cv2.putText(color,"diameter = " + str(mm_diameter), (cx,cy + 15), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)

        except:
            print ("Failed to process image")
            print (sys.exc_info()[0])

    elapsed_time = time.time() - start_time
    #print("Frame took " + str(elapsed_time) + " seconds to process")
    return obstacles
