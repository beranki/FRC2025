import cv2
from apriltag import AprilTag
from visionInput import VisionInput
from config import *
import numpy as np
RES = (1280, 720)

tag_module = AprilTag("bw_cam_4v1")
input = VisionInput(AT_FOV, AT_INPUT_RES, AT_CAM_HEIGHT, AT_CAM_ANGLE, 0)

while True:
    
    # Example rvec from solvePnP
    # rvec = np.array([[0.1], [0.2], [0.3]])  # Example rotation vector
    # euler_angles = tag_module.rotation_vector_to_euler_angles(rvec)
    # print("Euler Angles (radians):", euler_angles)
    
    frame = input.getFrame()
    cv2.imshow("frame", frame)
    print("frame size", frame.shape)
    annotated_frame = frame.copy()
    tagData = tag_module.estimate_3d_pose(frame, annotated_frame, ARUCO_LENGTH_METERS)


    #print(tagData)
    
    #cv2.imshow('result', annotated_frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    