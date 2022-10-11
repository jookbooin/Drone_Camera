# Drone_Camera

## 변수 
rvec : 카메라 프레임 관점에서의 attitude

tvec : 카메라 프레임 관점에서의 position

(-R_ct)*tvec : 마커 관점에서의 카메라 position

R_flip : roll축 기준으로 180도 회전

rodriguez : 회전변환을 컴팩트하게 표시 

#### 사진


## 카메라 
마커 position x = tvec[0] , y = tvec[1] , z = tvec[2]

마커 attitude : roll_marker , pitch_marker , yaw_marker (라디안) -> math.degrees로 각도 변환 

## 마커 

카메라 position = pos_camera

카메라 position x : pos_camera[0] , y: pos_camera[1] , z : pos_camera[2]

카메라 attitude : roll_camera , pitch_camera , yaw_camera

마커 attitude , 카메라 attitude -> 서로 flip한 상태로 보기때문에 동일 

# 이후 보완
aruco 마커 좌표계 사용하려는 이유는 gcs에서 정밀한 착륙위치를 알기위해서 였음 

calibration 이라는 것은 이미지상에 기준점을 주는 것으로 보여 마커와의 거리측정은 잘 안나오는것 같다.. (내가 잘 못한것 일수도?) -> 거리는 qgc에서 구해야 겠다 

1.calibration 과정에서 Checker파일 이미지들 0.025m 로 넣어주었는데 렌즈와의 거리 , 중심점 사이의 거리 등등 부정확한 값들이 많이 나오는 듯함 
이후 렌즈와 보드사이의 거리를 최대한 가깝게해서 다시 calibration 해보아야 할 것 같다.

2.하방카메라가 보는 위치에 따라서 가야하는 방향을 표시하려 했는데 roll pitch yaw 각도에 제한 범위를 주지않아 많이 흔들리게 된다.

marker 밖에서는 1 ,2 ,3 ,4 분면이 잘 인식되는데 marker 내부에서는 사분면들이 반전된 상태로나오게 된다. 이것에 대한 탐구가 필요함 






