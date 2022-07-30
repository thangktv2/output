from re import X
import cv2 as cv
from matplotlib.pyplot import contour
import numpy as np
import serial
import time

from cv2 import aruco
from cv2 import bitwise_and
from cv2 import bitwise_xor
from serial import Serial
from threading import Thread

def communicateInit(cI_port):
    global ser
    ser = serial.Serial(cI_port, 115200, bytesize = 8, parity='N', stopbits = 1, xonxoff=1)    

def imagePreprocess(iP_frame):
    iP_morphologyKernel = np.array(
                           [[0, 0, 1, 0, 0],
                            [0, 1, 1, 1, 0],
                            [1, 1, 1, 1, 1],
                            [0, 1, 1, 1, 0],
                            [0, 0, 1, 0, 0]],
                            dtype=np.uint8)

    iP_HSV = cv.cvtColor(iP_frame, cv.COLOR_BGR2HSV)

    iP_lowerColorBoundary = np.array([90, 0, 0])
    iP_upperColorBoundary = np.array([130, 255, 255])
    iP_rawItemExtract = cv.inRange(iP_HSV,
                                   iP_lowerColorBoundary,
                                   iP_upperColorBoundary)

    iP_output = iP_rawItemExtract
    # Remove Noise from raw image
    iP_output = cv.morphologyEx(iP_rawItemExtract,
                                          cv.MORPH_OPEN,
                                          iP_morphologyKernel)
    return iP_output

def getItemCoordinate(gIC_frame):
    gIC_output = [0, 0, 0, 0, 0]
    gIC_minAreaThreshold = cV_gICsetMinThreshold
    gIC_maxAreaThreshold = cV_gICsetMaxThreshold                                       
    gIC_contours, gIC_hierarchy = cv.findContours(gIC_frame,
                                                 cv.RETR_TREE, 
                                                 cv.CHAIN_APPROX_SIMPLE)
    gIC_numberOfContours = len(gIC_contours)

    if gIC_numberOfContours is not None:
        for pic, gIC_contour in enumerate(gIC_contours):
            gIC_contourArea = cv.contourArea(gIC_contour)
            if (gIC_minAreaThreshold < gIC_contourArea < gIC_maxAreaThreshold):
                gIC_x, gIC_y, gIC_w, gIC_h = cv.boundingRect(gIC_contour)
                gIC_output = [1, gIC_x, gIC_y, gIC_w, gIC_h]
                return gIC_output
    return gIC_output

def controlMotor(cM_input, cM_delay):
    if cM_input == 1:
        ser.write(b"C1\n")
    else:
        ser.write(b"C0\n")
    time.sleep(cM_delay)

def setCapSize(sCS_cap, sCS_frameWidth, sCS_frameHeight):
    # Set cap size
    sCS_cap.set(cv.CAP_PROP_FRAME_WIDTH, sCS_frameWidth)
    sCS_cap.set(cv.CAP_PROP_FRAME_HEIGHT, sCS_frameHeight)
    
def setGlobalValue(sGV_cap,sGV_error):
    pi = np.pi
    sGV_sumInnerArea = 0
    sGV_sumOuterArea = 0
    

    # Declare config value for getItemCoordinate() function
    global cV_gICsetMinThreshold
    global cV_gICsetMaxThreshold
    
    # Declare config value for rawClassify() function
    global rC_minOutterAreaThreshold
    global rC_maxInnerAreaThreshold

    for frames in range (0,10):
        sGV_ret, sGV_frame = sGV_cap.read() # Read a frame from video/camera
        if sGV_ret == True:
            # Pre-process input image
            iP_output = imagePreprocess(sGV_frame)
            
            # Contours process
            returnArea = contoursProcess(iP_output,1)
            sGV_sumInnerArea = sGV_sumInnerArea + returnArea[0] # Sum area of inner circle
            sGV_sumOuterArea = sGV_sumOuterArea + returnArea[1] # Sum area of outer circle

    sGV_averageInnerArea = sGV_sumInnerArea/10 # Average area of inner circle
    print('Average inner area', sGV_averageInnerArea)
    sGV_averageOuterArea = sGV_sumOuterArea/10 # Average area of outer circle
    print('Average outer area', sGV_averageOuterArea)

    # Average Diameter (21.5mm vs 37mm)

    inner_dia = 2*np.sqrt(sGV_averageInnerArea/pi)
    print('Average inner diameter', inner_dia)
    outer_dia = 2*np.sqrt(sGV_averageOuterArea/pi)
    print('Average inner diameter', outer_dia)
    
    # Set output value for getItemCoordinate() function
    cV_gICsetMinThreshold = sGV_averageOuterArea*float(1-sGV_error)
    print('Min outer threshold', cV_gICsetMinThreshold)
    cV_gICsetMaxThreshold = sGV_averageOuterArea*float(1+sGV_error)
    print('Max outer threshold', cV_gICsetMaxThreshold)

def contoursProcess(cP_input,
                    cP_mode): # 1: Draw contours

    cP_noiseThreshold = 2000 # Threshold to remove noise contours
    cP_output = [0,0] # Inner, outer

    cP_contours,cP_hierarchy = cv.findContours(cP_input, cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)

    # Get the actual inner list of hierarchy descriptions
    cP_hierarchy = cP_hierarchy[0]

    # For each contour, draw contour and calculate area of 10 frames to get average value
    for cP_component in zip(cP_contours, cP_hierarchy):
        cP_currentContour = cP_component[0]
        cP_currentHierarchy = cP_component[1]

        cP_contourArea = cv.contourArea(cP_currentContour) # Calculate the contour area
        if cP_contourArea > cP_noiseThreshold: # Remove noise contour
            if cP_currentHierarchy[2] < 0: # The innermost child components
                cv.drawContours(cP_input, [cP_currentContour], 0, (0,255,0), 2)
                if cP_mode == 1:
                    cP_output[0] = cP_contourArea # Sum area of inner circle
            elif cP_currentHierarchy[3] < 0: # The outermost parent components
                cv.drawContours(cP_input, [cP_currentContour], 0, (0,255,0), 2)
                if cP_mode == 1:
                    cP_output[1] = cP_contourArea # Sum area of outer circle
    return cP_output

def houghTransform(hT_input):
    return