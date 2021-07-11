from picamera.array import PiRGBArray
from picamera import PiCamera

import cv2
import numpy as np
import math

Video_Width = 360
Video_Height = 480

camera = PiCamera()
camera.resolution = (Video_Width, Video_Height)
camera.framerate = 24
rawCapture = PiRGBArray(camera, size = (Video_Width, Video_Height))

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    frame2 = frame.array
    frame1 = frame.array
    
    
    ## 메인 프레임 영상처리 ##
    blur = cv2.GaussianBlur(frame1, (5,5), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    
    ## 화면 정 중앙 ##
    cv2.circle(frame1, (int(Video_Width/2),int(Video_Height/2)), 3, (0,0,0), 2) 
    
    ## 무인카트 색 지정 ##
    lower_red = np.array([135,82,162])
    upper_red = np.array([159,255,255])
    
    ## 제품구간 표시 ##
    cv2.line(frame1, (int(Video_Width/3),0), (int(Video_Width/3),Video_Height), (255,255,255), 1)
    cv2.line(frame1, (int(2*Video_Width/3),0), (int(2*Video_Width/3),Video_Height), (255,255,255), 1)
    cv2.line(frame1, (0,int(Video_Height/3)), (int(Video_Width/3),int(Video_Height/3)), (255,255,255), 1)
    cv2.line(frame1, (0,int(2*Video_Height/3)), (int(Video_Width/3),int(2*Video_Height/3)), (255,255,255), 1)
    cv2.line(frame1, (Video_Width,int(Video_Height/3)), (int(2*Video_Width/3),int(Video_Height/3)), (255,255,255), 1)
    cv2.line(frame1, (Video_Width,int(2*Video_Height/3)), (int(2*Video_Width/3),int(2*Video_Height/3)), (255,255,255), 1)
                  
    
    ## 컨투어 프레임 ##
    frame2 = cv2.GaussianBlur(frame2, (5,5), 0)
    frame2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)
    frame2 = cv2.inRange(frame2, lower_red, upper_red)
    
    ## 컨투어 추출 ##
    contours, _ = cv2.findContours(frame2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    global x1,x2,x3,x4, y1,y2,y3,y4
    x1,x2,x3,x4, y1,y2,y3,y4 = 1,1,1,1, 1,1,1,1
    
    global Cart_Location_X, Cart_Location_Y
    Cart_Location_X, Cart_Location_Y = 1, 1
    
    ## 무인카트 인식 구간 ##
    if len(contours) > 0:
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            if area > 2000:
                rect = cv2.minAreaRect(contours[i])
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                if 0 <= box[1][0] <= Video_Width and 0 <= box[3][0] <= Video_Width:
                    cv2.drawContours(frame1, [box], -1, (0,255,0), 3)
                    
                    ## 카트의 좌표 ##
                    x1, y1 = box[0]
                    x2, y2 = box[1]
                    x3, y3 = box[2]
                    x4, y4 = box[3]
                    
                    ## 카트의 좌표 표시 ##
                    cv2.circle(frame1, (x1,y1), 2, (0,0,255), 2)
                    cv2.circle(frame1, (x2,y2), 2, (255,0,0), 2)
                    cv2.circle(frame1, (x3,y3), 2, (0,255,255), 2)
                    cv2.circle(frame1, (x4,y4), 2, (0,128,255), 2)
                    
                    ## 카트의 좌표 ##
                    Cart_Location_X = int((x1+x3)/2)
                    Cart_Location_Y = int((y1+y3)/2)
                    
                    ## 카트의 좌표 표시 ##
                    cv2.putText(frame1, '('+str(Cart_Location_X)+','+str(Cart_Location_Y)+')',
                                (Cart_Location_X + 10, Cart_Location_Y + 10), cv2.FONT_HERSHEY_SIMPLEX,
                                1, (255,255,255), 2, cv2.LINE_AA)
                    
                    cv2.line(frame1, (Cart_Location_X,0), (Cart_Location_X,Video_Height), (0,0,0), 1)
                    cv2.line(frame1, (0,Cart_Location_Y), (Video_Width,Cart_Location_Y), (0,0,0), 1)
                    
                                        
                    ## 카트의 위치 표시 ##
                    if 0 <= Cart_Location_X <= int(Video_Width/3) : ### 좌측 물품(1,2,3) ###
                        if 0 <= Cart_Location_Y <= int(Video_Height/3) :
                            cv2.putText(frame1, 'Section 3', (int(Video_Width/3 + Video_Width/40),40)
                                        , cv2.FONT_HERSHEY_SIMPLEX, 1, (255,128,0), 2, cv2.LINE_AA)
                            cv2.line(frame1, (0,0), (int(Video_Width/3),0), (0,0,255), 1)
                            cv2.line(frame1, (0,0), (0,int(Video_Height/3)), (0,0,255), 1)
                            cv2.line(frame1, (int(Video_Width/3),0), (int(Video_Width/3),int(Video_Height/3)), (0,0,255), 1)
                            cv2.line(frame1, (0,int(Video_Height/3)), (int(Video_Width/3),int(Video_Height/3)), (0,0,255), 1)
                            
                        elif int(Video_Height/3) <= Cart_Location_Y <= int(2*Video_Height/3) :
                            cv2.putText(frame1, 'Section 2', (int(Video_Width/3 + Video_Width/40),40)
                                        , cv2.FONT_HERSHEY_SIMPLEX, 1, (255,128,0), 2, cv2.LINE_AA)
                            cv2.line(frame1, (0,int(Video_Height/3)), (int(Video_Width/3),int(Video_Height/3)), (0,0,255), 1)
                            cv2.line(frame1, (0,int(Video_Height/3)), (0,2*int(Video_Height/3)), (0,0,255), 1)
                            cv2.line(frame1, (int(Video_Width/3),int(Video_Height/3)), (int(Video_Width/3),2*int(Video_Height/3)), (0,0,255), 1)
                            cv2.line(frame1, (0,2*int(Video_Height/3)), (int(Video_Width/3),2*int(Video_Height/3)), (0,0,255), 1)
                            
                        else :
                            cv2.putText(frame1, 'Section 1', (int(Video_Width/3 + Video_Width/40),40)
                                        , cv2.FONT_HERSHEY_SIMPLEX, 1, (255,128,0), 2, cv2.LINE_AA)
                            cv2.line(frame1, (0,2*int(Video_Height/3)), (int(Video_Width/3),2*int(Video_Height/3)), (0,0,255), 1)
                            cv2.line(frame1, (0,2*int(Video_Height/3)), (0,int(Video_Height)), (0,0,255), 1)
                            cv2.line(frame1, (int(Video_Width/3),2*int(Video_Height/3)), (int(Video_Width/3),int(Video_Height)), (0,0,255), 1)
                            cv2.line(frame1, (0,int(Video_Height)), (int(Video_Width/3),int(Video_Height)), (0,0,255), 1)
                            
                    elif int(2*Video_Width/3) <= Cart_Location_X <= Video_Width : ### 우측 물품(4,5,6) ###
                        if 0 <= Cart_Location_Y <= int(Video_Height/3) :
                            cv2.putText(frame1, 'Section 4', (int(Video_Width/3 + Video_Width/40),40)
                                        , cv2.FONT_HERSHEY_SIMPLEX, 1, (255,128,0), 2, cv2.LINE_AA)
                            cv2.line(frame1, (2*int(Video_Width/3),0), (int(Video_Width),0), (0,0,255), 1)
                            cv2.line(frame1, (2*int(Video_Width/3),0), (2*int(Video_Width/3),int(Video_Height/3)), (0,0,255), 1)
                            cv2.line(frame1, (int(Video_Width),0), (int(Video_Width),int(Video_Height/3)), (0,0,255), 1)
                            cv2.line(frame1, (2*int(Video_Width/3),int(Video_Height/3)), (int(Video_Width),int(Video_Height/3)), (0,0,255), 1)
                            
                        elif int(Video_Height/3) <= Cart_Location_Y <= int(2*Video_Height/3) :
                            cv2.putText(frame1, 'Section 5', (int(Video_Width/3 + Video_Width/40),40)
                                        , cv2.FONT_HERSHEY_SIMPLEX, 1, (255,128,0), 2, cv2.LINE_AA)
                            cv2.line(frame1, (2*int(Video_Width/3),int(Video_Height/3)), (int(Video_Width),int(Video_Height/3)), (0,0,255), 1)
                            cv2.line(frame1, (2*int(Video_Width/3),int(Video_Height/3)), (2*int(Video_Width/3),2*int(Video_Height/3)), (0,0,255), 1)
                            cv2.line(frame1, (2*int(Video_Width/3),2*int(Video_Height/3)), (int(Video_Width),2*int(Video_Height/3)), (0,0,255), 1)
                            cv2.line(frame1, (int(Video_Width),int(Video_Height/3)), (int(Video_Width),2*int(Video_Height/3)), (0,0,255), 1)
                            
                        else :
                            cv2.putText(frame1, 'Section 6', (int(Video_Width/3 + Video_Width/40),40)
                                        , cv2.FONT_HERSHEY_SIMPLEX, 1, (255,128,0), 2, cv2.LINE_AA)
                            cv2.line(frame1, (2*int(Video_Width/3),2*int(Video_Height/3)), (2*int(Video_Width/3),int(Video_Height)), (0,0,255), 1)
                            cv2.line(frame1, (2*int(Video_Width/3),2*int(Video_Height/3)), (int(Video_Width),2*int(Video_Height/3)), (0,0,255), 1)
                            cv2.line(frame1, (2*int(Video_Width/3),int(Video_Height)), (int(Video_Width),int(Video_Height)), (0,0,255), 1)
                            cv2.line(frame1, (int(Video_Width),2*int(Video_Height/3)), (int(Video_Width),int(Video_Height)), (0,0,255), 1)
                            

    cv2.imshow("Frame1", frame1)
    
    rawCapture.truncate(0)
    
    if cv2.waitKey(1) > 0: break


rawCapture.truncate(0)
cv2.destroyAllWindows()
