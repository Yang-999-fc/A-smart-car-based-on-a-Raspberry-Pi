import cv2
import numpy as np

def cat2images(limg, rimg):
    HEIGHT = limg.shape[0]
    WIDTH = limg.shape[1]
    imgcat = np.zeros((HEIGHT, WIDTH*2+20,3))
    imgcat[:,:WIDTH,:] = limg
    imgcat[:,-WIDTH:,:] = rimg
    for i in range(int(HEIGHT / 32)):
        imgcat[i*32,:,:] = 255
    return imgcat

left_image = cv2.imread("images/camera-L/left9.jpg")
right_image = cv2.imread("images/camera-R/right9.jpg")

imgcat_source = cat2images(left_image,right_image)
HEIGHT = left_image.shape[0]
WIDTH = left_image.shape[1]
cv2.imwrite('imgcat_source.jpg', imgcat_source )

camera_matrix0 = np.array([[4.831141936994179e+02,1.320361003535064, 2.871134830632458e+02],
                                         [0., 4.826240791607009e+02,2.143208188223237e+02],
                                         [0., 0., 1.]]
                        ) .reshape((3,3)) #即上文标定得到的 cameraMatrix1

distortion0 = np.array([0.057855078928651, 0.072392787262100,-9.463585503719572e-04,
                                      -0.001788559134934, -0.281052952846398]) #即上文标定得到的 distCoeffs1

camera_matrix1 = np.array([[4.852654323258022e+02, 0.827884392289984, 3.084508609903978e+02],
                                          [0., 4.850513463365957e+02, 2.320792259875037e+02],
                                          [0., 0., 1.]]
                        ) .reshape((3,3)) #即上文标定得到的 cameraMatrix2
distortion1 = np.array([0.038634393094003, 0.298149342616519, -0.001071435803553,
                                       -0.001777371854203,-0.920170095588780]) #即上文标定得到的 distCoeffs2

R = np.array([[0.999906015888366, 7.911264139072465e-04, -0.013686983204966],
                           [-8.867392736930706e-04, 0.999975239217104, -0.006981020423669],
                           [0.013681121434893, 0.006992501104212, 0.999881958955452]]
            ) #即上文标定得到的 R
T = np.array([[-60], [0.123976361604774], [0.214336536856105]]) #即上文标定得到的T


(R_l, R_r, P_l, P_r, Q, validPixROI1, validPixROI2) = \
    cv2.stereoRectify(camera_matrix0, distortion0, camera_matrix1, distortion1, np.array([WIDTH,HEIGHT]), R, T) #计算旋转矩阵和投影矩阵，调用 stereo Rectify()函数得到左右摄像机各旋转一半的旋转矩阵和平移向量

(map1, map2) = \
    cv2.initUndistortRectifyMap(camera_matrix0, distortion0, R_l, P_l, np.array([WIDTH,HEIGHT]), cv2.CV_32FC1) #计算校正查找映射表，求解图像的无畸变和修正转换关系，将结果以映射的形式表达

rect_left_image = cv2.remap(left_image, map1, map2, cv2.INTER_CUBIC) #重映射及进行校正

#左右图需要分别计算校正查找映射表以及重映射
(map1, map2) = \
    cv2.initUndistortRectifyMap(camera_matrix1, distortion1, R_r, P_r, np.array([WIDTH,HEIGHT]), cv2.CV_32FC1)

rect_right_image = cv2.remap(right_image, map1, map2, cv2.INTER_CUBIC)

imgcat_out = cat2images(rect_left_image,rect_right_image)
cv2.imwrite('imgcat_out.jpg', imgcat_out)
