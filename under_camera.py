#python under_camera.py --K_Matrix calibration_matrix.npy --D_Coeff distortion_coefficients.npy --type DICT_5X5_100
#마커기준 - 빨강:x,Roll / 초록 y,pitch / 파랑 z,yaw

import numpy as np
import cv2
import sys, math
from utils import ARUCO_DICT
import argparse
import time

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
    
#변환행렬 -> 오일러 각  ( x , z 변환...?)
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


# cv2.rectangle(frame ,(520,15),(620,115),(0,0,0),1)
def arrow(frame , x ,y):
    if x>0 and y>0 :
        cv2.arrowedLine(frame , (620,15),(520,115),(0,0,255),3)  
    if x<0 and y>0 :
        cv2.arrowedLine(frame , (520,15),(620,115),(0,0,255),3)  
    if x<0 and y<0 :
        cv2.arrowedLine(frame , (520,115),(620,15),(0,0,255),3)  
    if x>0 and y<0 :
        cv2.arrowedLine(frame , (620,115),(520,15),(0,0,255),3)  
    
    return frame


def pose_esitmation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):

    # (x,y,z) -> (x,-y,-z) x축 기준 으로 180도 회전 -> 카메라와 마커의 관계를 나타냄
    R_flip  = np.zeros((3,3), dtype=np.float32)
    R_flip[0,0] = 1.0
    R_flip[1,1] =-1.0
    R_flip[2,2] =-1.0

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()


    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,
                            parameters=parameters)
    
    print("corners :", corners)
    print()
    
    if len(corners) > 0:
        for (markerCorner, markerID) in zip(corners, ids):
            reCorner = markerCorner.reshape((4,2))
            (topLeft, topRight, bottomRight, bottomLeft) = reCorner
            centerX = (topLeft[0] +topRight[0]+ bottomRight[0]+ bottomLeft[0])*0.25
            centerY = (topLeft[1] +topRight[1]+ bottomRight[1]+ bottomLeft[1])*0.25
            print("{} = centerX : {} centerY : {}".format(markerID ,centerX , centerY))
            print(markerCorner)

            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorner, 0.02, matrix_coefficients,
                                         distortion_coefficients)
        
            cv2.aruco.drawDetectedMarkers(frame, corners ,borderColor =(0,0,255)) 
        
            print("rvec : ",rvec)   #카메라 관점에서의 마커의 자세 
            print("tvec : ",tvec)   #카메라 프레임 에서 마커의 위치 
            print("distance:",int(tvec[0][0][2]*100))
            print()
            # Draw Axes
            cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.03)  
            
            #calivbration 과정에서 2.5 -> 0.025라 넣기에
            tvecChange = tvec[0][0]*100
            print("tvecChange: ",tvecChange)
            font = cv2.FONT_HERSHEY_PLAIN
            
            #카메라에서 마커 위치 표현  
            str_position = "Marker Position x=%.1f y=%.1f z=%.1f"%(tvecChange[0],tvecChange[1],tvecChange[2])
            cv2.putText(frame ,str_position, (0,30),font,0.8,(255,0,0) ,1,cv2.LINE_AA)


            R_ct = np.matrix(cv2.Rodrigues(rvec[0])[0])
            #.T 는 전치 행렬
            R_tc = R_ct.T

            roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)
            print()
            print("roll_marker:",roll_marker)
            print("roll_degee:",math.degrees(roll_marker))
            print("pitch_marker:",pitch_marker)
            print("pitch_degree:",math.degrees(pitch_marker))
            print("yaw_marker:",yaw_marker)
            print("yaw_degree:",math.degrees(yaw_marker))

            str_attitude = "MARKER Attitude r=%.0f  p=%.0f  y=%.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
                            math.degrees(yaw_marker))
            cv2.putText(frame, str_attitude, (0, 60), font, 0.8, (255, 0, 0), 1, cv2.LINE_AA)

            
            #마커 관점에서 카메라 위치 
            pos_camera = -R_tc * np.matrix(tvecChange).T
            str_position = "Camera Position x=%.1f y=%.1f z=%.1f"%(pos_camera[0],pos_camera[1],pos_camera[2])
            cv2.putText(frame ,str_position, (0,90),font,0.8,(255,0,0) ,1,cv2.LINE_AA)

            
            frame = arrow(frame , pos_camera[0],pos_camera[1])
            
            #R_flip을 정의한 이유
            #카메라 axis들과 마커의 axis들은 서로 반대로 되어있기떄문에 R_flip을 통해 같은 방향으로 돌려줌 
            roll_camera,pitch_camera,yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)

            str_attitude = "Camera Attitude r=%.0f  p=%.0f  y=%.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                            math.degrees(yaw_camera))
            cv2.putText(frame, str_attitude, (0, 120), font, 0.8, (255, 0, 0), 1, cv2.LINE_AA)

    return frame

if __name__ == '__main__':

    ap = argparse.ArgumentParser()
    ap.add_argument("-k", "--K_Matrix", required=True, help="Path to calibration matrix (numpy file)")
    ap.add_argument("-d", "--D_Coeff", required=True, help="Path to distortion coefficients (numpy file)")
    ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="Type of ArUCo tag to detect")
    args = vars(ap.parse_args())

    
    if ARUCO_DICT.get(args["type"], None) is None:
        print(f"ArUCo tag type '{args['type']}' is not supported")
        sys.exit(0)

    aruco_dict_type = ARUCO_DICT[args["type"]]
    calibration_matrix_path = args["K_Matrix"]
    distortion_coefficients_path = args["D_Coeff"]
    
    k = np.load(calibration_matrix_path)
    d = np.load(distortion_coefficients_path)

    video = cv2.VideoCapture(0)
    time.sleep(2.0)

    while True:
        ret, frame = video.read()

        if not ret:
            break
        
        height,width = frame.shape[:2]
        frameCenter = (width//2 , height//2)
        cv2.line(frame , (frameCenter[0],frameCenter[1]) , (frameCenter[0],frameCenter[1]+10), (0,255,0))
        cv2.line(frame , (frameCenter[0],frameCenter[1]) , (frameCenter[0]+10 , frameCenter[1]),(0,0,255))
        output = pose_esitmation(frame, aruco_dict_type, k, d)

        cv2.rectangle(frame ,(520,15),(620,115),(0,0,0),1)
        cv2.imshow('Estimated Pose', output)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    video.release()
    cv2.destroyAllWindows()