import cv2
import numpy as np
# 左相机内参
left_camera_matrix = np.array([[4.831141936994179e+02,1.320361003535064, 2.871134830632458e+02],
                                         [0., 4.826240791607009e+02,2.143208188223237e+02],
                                         [0., 0., 1.]])
# 左相机畸变系数:[k1, k2, p1, p2, k3]
left_distortion = np.array([[0.057855078928651, 0.072392787262100,-9.463585503719572e-04,
                                      -0.001788559134934, -0.281052952846398]])


# 右相机内参
right_camera_matrix = np.array([[4.852654323258022e+02, 0.827884392289984, 3.084508609903978e+02],
                                          [0., 4.850513463365957e+02, 2.320792259875037e+02],
                                          [0., 0., 1.]])
# 右相机畸变系数:[k1, k2, p1, p2, k3]
right_distortion = np.array([[0.038634393094003, 0.298149342616519, -0.001071435803553,
                                       -0.001777371854203,-0.920170095588780]])

 # 旋转关系向量
R =  np.array([[0.999906015888366, 7.911264139072465e-04, -0.013686983204966],
                           [-8.867392736930706e-04, 0.999975239217104, -0.006981020423669],
                           [0.013681121434893, 0.006992501104212, 0.999881958955452]])
T = np.array([[-57.187388360193914], [0.123976361604774], [0.214336536856105]]) # 平移关系向量

size = (640, 480) # 图像尺寸

# 进行立体更正
R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(left_camera_matrix, left_distortion,
                                                                  right_camera_matrix, right_distortion, size, R,
                                                                  T)
# 计算更正map
left_map1, left_map2 = cv2.initUndistortRectifyMap(left_camera_matrix, left_distortion, R1, P1, size, cv2.CV_16SC2)
right_map1, right_map2 = cv2.initUndistortRectifyMap(right_camera_matrix, right_distortion, R2, P2, size, cv2.CV_16SC2)