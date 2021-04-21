#!/usr/bin/env python3
# import the necessary packages

import cv2
import numpy as np
import math

import datetime
import time

from pathlib import Path
import pickle

from networktables import NetworkTables
import cscore as cs
import logging

from FrameReaderPi import CameraVideoStream

from threading import Thread

tvecXSaved = np.array([0])
tvecYSaved = np.array([0])
tvecZSaved = np.array([0])

# Calibration for camera is 1080p
mtx = None
dist = None

# 640 by 360
xFactor = 3
yFactor = 3

# init camera server and network tables
logging.basicConfig(level=logging.DEBUG)
ip = "10.28.34.2"
NetworkTables.initialize(server=ip)
sd = NetworkTables.getTable("SmartDashboard")

cs = cs.CameraServer.getInstance()

outputStream = cs.putVideo("vision", 640, 360)

# calibrated for 1080p
distortion_correction_file = Path("distortion_correction_pickle_ir_1080.p")
# check if we already created the calibration file with coefficients
if distortion_correction_file.is_file():
    # load the coefficients to undistort the camera image
    with open('distortion_correction_pickle_ir_1080.p', mode='rb') as f:
        calibration_file = pickle.load(f)
        mtx, dist = calibration_file['mtx'], calibration_file['dist']
else:
    print('Calibration does not exist.')

print("mtx: ", mtx)
print("dist: ", dist)

# Scale camera matrix to allow for different frame sizes
#fx
mtx[0,0] = mtx[0,0] / xFactor
#cx
mtx[0,2] = mtx[0,2] / xFactor
#fy
mtx[1,1] = mtx[1,1] / yFactor
#cy
mtx[1,2] = mtx[1,2] / yFactor

print("mtx:", mtx)

# Inch to Meter conversion
inToMConversion = 0.0254
# Real World points of vision target in inches
objectPoints = np.array([[-19.625 * inToMConversion ,0,0], [19.625 * inToMConversion,0,0], [(19.625-9.8051325845) * inToMConversion,-17 * inToMConversion,0], [(-19.625+9.8051325845) * inToMConversion,-17 * inToMConversion,0]], dtype=np.float32)
# Virtual World points of trihedron to show target pose
trihedron = np.array([[0,0,0],[12 * inToMConversion,0,0],[0,12 * inToMConversion,0],[0,0,12 * inToMConversion]], dtype=np.float32)

# Contour filtering constants 
# Expected = 2.3088235294
aspectRatioMin = 1.0
aspectRatioMax = 2.5

# Expected = 0.2206857965
solidityMin = 0.1
solidityMax = 0.4

polySides = 4

#Class to examine Frames per second of camera stream.
class FPS:
    def __init__(self):
        # store the start time, end time, and total number of frames
        # that were examined between the start and end intervals
        self._start = None
        self._end = None
        self._numFrames = 0

    def start(self):
        # start the timer
        self._start = datetime.datetime.now()
        return self

    def stop(self):
        # stop the timer
        self._end = datetime.datetime.now()

    def update(self):
        # increment the total number of frames examined during the
        # start and end intervals
        self._numFrames += 1

    def elapsed(self):
        # return the total number of seconds between the start and
        # end interval
        return (self._end - self._start).total_seconds()

    def fps(self):
        # compute the (approximate) frames per second
        return self._numFrames / self.elapsed()

cap = CameraVideoStream("/dev/video0", True).start()

# Gets the contours of the target
def getContours(frame):
    # Mask the target
    blur = cv2.blur(frame,(5,5), -1)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    # mask = cv2.inRange(hsv, np.array([50,50,40]), np.array([93,255,255]))
    mask = cv2.inRange(hsv, np.array([0,0,100]), np.array([255,80,255]))

    # Find contours of the target
    _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(blur, contours, -1, (0,255,255), 3)

    #outputStream.putFrame(blur)

    return blur, contours

# Filter the contours
def filterContours(frame, contours):
    if len(contours) > 0:
        #Sort contours by area size (biggest to smallest)
        cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)

        for cnt in cntsSorted:

            #cnt = cntsSorted[0]

            # Get moments of contour; mainly for centroid
            M = cv2.moments(cnt)
            # Get bounding rectangle
            x, y, w, h = cv2.boundingRect(cnt)
            # Get convex hull (bounding polygon on contour)
            hull = cv2.convexHull(cnt)
            # Calculate Contour area
            cntArea = cv2.contourArea(cnt)
            # calculate area of convex hull
            hullArea = cv2.contourArea(hull)

            # Aspect ratio of the target
            aspectRatio = w/h
            # Solidity of the target
            solidity = 0
            if hullArea!=0:
                solidity = cntArea/hullArea

            # Calculate the epsilon for the polygon
            # epsilon = 0.045*cv2.arcLength(hull, closed=True)
            # epsilon = 0.015*cv2.arcLength(hull, closed=True)
            epsilon = 0.02*cv2.arcLength(hull, closed=True)
            # Approx a polygon fit to the convex hull
            approx = cv2.approxPolyDP(hull, epsilon, True)

            cv2.drawContours(frame, [approx], -1, (255, 255, 0), 3)

            print("aspectRatio", aspectRatio)
            print("solidity", solidity)
            print("hullSides", len(approx))

            sd.putNumber("aspectRatio", aspectRatio)
            sd.putNumber("solidity", solidity)
            sd.putNumber("hullSides", len(approx))

            # Check if contour matches criteria
            if (aspectRatio>=aspectRatioMin and aspectRatio<=aspectRatioMax) and (solidity>=solidityMin and solidity<=solidityMax) and (len(approx)==polySides):
                print("TARGET DETECTED")

                return True, approx, frame

    return False, None, frame

def solvePNP(frame, approx):
    # Sort the corners
    approxSortedY = sorted(approx, key=lambda k: k[0][1])
    topCorners = [approxSortedY[0], approxSortedY[1]]
    bottomCorners = [approxSortedY[2], approxSortedY[3]]
    bottomCornersSortedX = sorted(bottomCorners, key=lambda k: k[0][0])
    topCornersSortedX = sorted(topCorners, key=lambda k: k[0][0])

    # Corners, top left, top right, bottom right, bottom left
    corners = np.array([topCornersSortedX[0], topCornersSortedX[1], bottomCornersSortedX[1], bottomCornersSortedX[0]],dtype=np.float32)

    cv2.line(frame, (corners[0][0][0],corners[0][0][1]), (corners[0][0][0],corners[0][0][1]), (255,0,0), 5)
    cv2.line(frame, (corners[1][0][0],corners[1][0][1]), (corners[1][0][0],corners[1][0][1]), (0,255,0), 5)
    cv2.line(frame, (corners[2][0][0],corners[2][0][1]), (corners[2][0][0],corners[2][0][1]), (0,0,255), 5)
    cv2.line(frame, (corners[3][0][0],corners[3][0][1]), (corners[3][0][0],corners[3][0][1]), (0,255,255), 5)

    # Get the pose of the vision taraget
    retval, rvec, tvec = cv2.solvePnP(objectPoints, corners, mtx, dist, cv2.SOLVEPNP_P3P)

    trihedronPoints, _ = cv2.projectPoints(trihedron, rvec, tvec, mtx, dist)

    sd.putNumber("tvec x", tvec[0][0])
    sd.putNumber("tvec y", tvec[1][0])
    sd.putNumber("tvec z", tvec[2][0])

    cv2.putText(frame, "tvec x: " + str("%.2f" %(tvec[0][0])), (40, 90), cv2.FONT_HERSHEY_COMPLEX, 0.5,
        (255, 255, 255))
    cv2.putText(frame, "tvec y: " + str("%.2f" %(tvec[1][0])), (40, 150), cv2.FONT_HERSHEY_COMPLEX, 0.5,
        (255, 255, 255))
    cv2.putText(frame, "tvec z: " + str("%.2f" %(tvec[2][0])), (40, 210), cv2.FONT_HERSHEY_COMPLEX, 0.5,
        (255, 255, 255))

    if ((trihedronPoints[0][0][0] <= 10000) and (trihedronPoints[0][0][0] >= -10000)) and ((trihedronPoints[0][0][1] <= 10000) and (trihedronPoints[0][0][1] >= -10000)) and ((trihedronPoints[1][0][0] <= 10000) and (trihedronPoints[1][0][0] >= -10000)) and ((trihedronPoints[1][0][1] <= 10000) and (trihedronPoints[1][0][1] >= -10000)) and ((trihedronPoints[2][0][0] <= 10000) and (trihedronPoints[2][0][0] >= -10000)) and ((trihedronPoints[2][0][1] <= 10000) and (trihedronPoints[2][0][1] >= -10000)) and ((trihedronPoints[3][0][0] <= 10000) and (trihedronPoints[3][0][0] >= -10000)) and ((trihedronPoints[3][0][1] <= 10000) and (trihedronPoints[3][0][1] >= -10000)):
        cv2.line(frame, (int(trihedronPoints[0][0][0]), int(trihedronPoints[0][0][1])), (int(trihedronPoints[1][0][0]), int(trihedronPoints[1][0][1])), (255,0,0), 3)
        cv2.line(frame, (int(trihedronPoints[0][0][0]), int(trihedronPoints[0][0][1])), (int(trihedronPoints[2][0][0]), int(trihedronPoints[2][0][1])), (0,255,0), 3)
        cv2.line(frame, (int(trihedronPoints[0][0][0]), int(trihedronPoints[0][0][1])), (int(trihedronPoints[3][0][0]), int(trihedronPoints[3][0][1])), (0,0,255), 3)

#fps = FPS().start()

while(True):

    frame, frameAquiredTime = cap.read()
    frame, contours = getContours(frame)
    retVal, approx, frame = filterContours(frame, contours)
    if retVal:
        solvePNP(frame, approx)
    outputStream.putFrame(frame)
    #Thread(target=findTape(contours, blur, (image_width/2)-0.5, (image_height/2)-0.5))

cap.release()