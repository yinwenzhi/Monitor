# -*- coding: utf-8 -*-
"""
Spyder Editor
This is a temporary script file.
"""
 
import cv2
 
img = cv2.imread(r'C:\Users\Pictures\Camera Roll/test.jpg')
gray= cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
 
# one SIFT
#sift = cv2.xfeatures2d.SIFT_create()
#kp1 = sift.detect(gray,None)
#img1=cv2.drawKeypoints(gray,kp1,img)
#cv2.imshow("sift_Image", img1)
#cv2.imwrite(r'C:\Users\Pictures\Camera Roll/sift_test.jpg',img1)
 
#Compare four alogorithms
 
#Another SIFT
sift = cv2.xfeatures2d.SIFT_create()
(kps, descs) = sift.detectAndCompute(gray, None)
img1=cv2.drawKeypoints(gray, kps, img, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
cv2.imshow('SIFT_Algorithm', img1)
 
# SURF
surf = cv2.xfeatures2d.SURF_create()
(kps2, descs2) = surf.detectAndCompute(gray, None)
img2=cv2.drawKeypoints(gray, kps2, img, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
cv2.imshow('SURF_Algorithm', img2)
 
# FAST
fast = cv2.FastFeatureDetector_create()
kps3 = fast.detect(gray, None)
img3=cv2.drawKeypoints(gray, kps3, img, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
cv2.imshow('FAST_Algorithm', img3)
 
# ORB
orb = cv2.ORB_create()
kps4 = orb.detect(gray, None)
(kps4, des4) = orb.compute(gray, kps4)
img4=cv2.drawKeypoints(gray, kps4, img, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
cv2.imshow('ORB_Algorithm', img4)
