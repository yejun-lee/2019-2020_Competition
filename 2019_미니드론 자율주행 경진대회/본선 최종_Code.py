from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import math
import threading
from time import sleep
from e_drone.drone import *
from e_drone.protocol import *



### 영상처리 구간 ###
def test1():
    
## 비디오 해상도 ##
    camera = PiCamera( )
    camera.resolution = (320,240)
    camera.framerate = 60
    rawCapture = PiRGBArray(camera, size=(320,240))

## 컨투어 범위 설정 ##
    global set1,set2
    set1 = 0
    set2 = 110
    

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        frame1 = frame.array    ## frame HSV 변환 ##
        blur = cv2.GaussianBlur(frame1,(5,5),0)
        hsv = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)
        cv2.circle(frame1,(160,120),6,(0,255,255),2) ## 화면 정 중앙 ##

    ## 트랙 색 지정 ##
        frame2 = frame1[set1:set2, 0:320]
        frame2 = cv2.GaussianBlur(frame2,(5,5),0)
        frame2 = cv2.cvtColor(frame2,cv2.COLOR_BGR2HSV)

        lower_blue = np.array([90,164,123])
        upper_blue = np.array([179,255,255])
        frame2 = cv2.inRange(frame2,lower_blue,upper_blue)

        contours1,_ = cv2.findContours(frame2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        global y2,angle1,angle2,x1,x2,x3,x4,y1,y3,y4
        x1,x2,x3,x4,y1,y2,y3,y4=1,1,1,1,1,1,1,1
        angle1,angle2=1,1

    ## 컨투어 범위 표시 ##
        cv2.line(frame1,(0,set1),(640,set1),(0,0,0),1)
        cv2.line(frame1,(0,set2),(640,set2),(0,0,0),1)

    ## 도착지점 색 지정 ##
        lower_green = np.array([69,95,99])
        upper_green = np.array([95,255,255])
        frame3 = cv2.inRange(hsv,lower_green,upper_green)
        global x21,x23,y23,y21
        contours2,_ = cv2.findContours(frame3, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        x21,x22,x23,x24,y21,y22,y23,y24=1,1,1,1,1,1,1,1

    ## 체크포인트 색 지정 ##
        lower_red = np.array([147,121,64])
        upper_red = np.array([179,255,255])
        frame4 = cv2.inRange(hsv,lower_red,upper_red)
        global x31,x33,y31,y33
        contours3,_ = cv2.findContours(frame4, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        x31,x32,x33,x34,y31,y32,y33,y34=1,1,1,1,1,1,1,1

    ### 트랙 인식 구간 ###
        if len(contours1)>0:
            for i in range(len(contours1)):
                area1 = cv2.contourArea(contours1[i])
                if area1>5000:
                    rect1 = cv2.minAreaRect(contours1[i])
                    box1 = cv2.boxPoints(rect1)
                    box1 = np.int0(box1)
                    if 0<=box1[1][0]<=320 and 0<=box1[3][0]<=320:
                        cv2.drawContours(frame1,[box1],-1,(0,255,0),3)
 
                        x1,y1 = box1[0]
                        x2,y2 = box1[1]
                        x3,y3 = box1[2]
                        x4,y4 = box1[3]

                        cv2.circle(frame1,(x1,y1),4,(0,0,255),2)
                        cv2.circle(frame1,(x2,y2),4,(255,0,0),2)
                        cv2.circle(frame1,(x3,y3),4,(0,255,255),2)
                        cv2.circle(frame1,(x4,y4),4,(0,128,255),2)

                        angle1 = int(math.atan((x1-x2)/(y1-y2))*180/math.pi)
                        angle2 = int(math.atan((x3-x2)/(y2-y3))*180/math.pi)

                ## 2번 점이 절반 위에 있을 때 ##
                        if int(y2)<int((set2-set1)/2):
                            cv2.line(frame1,(int((x2+x3)/2),int((y2+y3)/2)),(int((x1+x4)/2),int((y1+y4)/2)),(0,0,255),1)
                            cv2.putText(frame1, 'L'+str(angle1),(160,120),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2,cv2.LINE_AA)
                ## 2번 점이 절반 아래에 있을 때 ##
                        elif int(y2)>int((set2-set1)/2):
                            cv2.line(frame1,(int((x3+x4)/2),int((y3+y4)/2)),(int((x2+x1)/2),int((y2+y1)/2)),(0,0,255),1)
                            cv2.putText(frame1, 'R'+str(angle2),(160,120),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2,cv2.LINE_AA)

    ### 도착지점 인식 구간 ###
        if len(contours2)>0:
            for i in range(len(contours2)):
                area2 = cv2.contourArea(contours2[i])
                if area2 > 1000:
                    rect2 = cv2.minAreaRect(contours2[i])
                    box2 = cv2.boxPoints(rect2)
                    box2 = np.int0(box2)

                    cv2.drawContours(frame1,[box2],-1,(0,255,255),3)
                    
                    
                    x21, y21 = box2[0]
                    x23, y23 = box2[2]

                    cv2.circle(frame1,(int((x21+x23)/2),int((y21+y23)/2)),4,(255,255,255),2)
                    cv2.putText(frame1,'('+str(int((x21+x23)/2))+','+str(int((y21+y23)/2))+')',(int((x21+x23)/2)+10,int((y21+y23)/2)),cv2.FONT_HERSHEY_SCRIPT_SIMPLEX,1,(255,255,255),2,cv2.LINE_AA)

    ### 체크포인트 인식 구간 ###
        if len(contours3)>0:
            for i in range(len(contours3)):
                area3 = cv2.contourArea(contours3[i])
                if area3 > 1000:
                    rect3 = cv2.minAreaRect(contours3[i])
                    box3 = cv2.boxPoints(rect3)
                    box3 = np.int0(box3)

                    cv2.drawContours(frame1,[box3],-1,(0,255,255),3)

                    x31, y31 = box3[0]
                    x33, y33 = box3[2]

                    cv2.circle(frame1,(int((x31+x33)/2),int((y31+y33)/2)),4,(255,255,255),2)
                    cv2.putText(frame1,'('+str(int((x31+x33)/2))+','+str(int((y31+y33)/2))+')',(int((x31+x33)/2)+10,int((y31+y33)/2)),cv2.FONT_HERSHEY_SCRIPT_SIMPLEX,1,(255,255,255),2,cv2.LINE_AA)
                
    ### 변수 영역 ###
    # 트랙 넓이
        global patharea, greenarea, redarea
        patharea = int((math.sqrt(math.pow((x1-x2),2)+math.pow((y1-y2),2)))*(math.sqrt(math.pow((x2-x3),2)+math.pow((y2-y3),2))))
    # 초록 원 넓이
        greenarea = int((math.sqrt(math.pow((x21-x22),2)+math.pow((y21-y22),2)))*(math.sqrt(math.pow((x22-x23),2)+math.pow((y22-y23),2))))
    # 빨간 원 넓이
        redarea = int((math.sqrt(math.pow((x31-x32),2)+math.pow((y31-y32),2)))*(math.sqrt(math.pow((x32-x33),2)+math.pow((y32-y33),2))))
    # 트랙 중간지점과 거리
        global gab,gab2,gab1
        gab = int((x1+x3)/2)-160
    # 빨간 원과의 거리
        gab1 = int(math.sqrt(math.pow(160-int((x31+x33)/2),2)+math.pow(120-int((y31+y33)/2),2)))
    # 초록 원과의 거리
        gab2 = int(math.sqrt(math.pow(160-int((x21+x23)/2),2)+math.pow(120-int((y21+y23)/2),2)))

    ## 트랙 중간지점과 거리 표시 ##
        cv2.line(frame1,(int((x1+x3)/2),120),(160,120),(255,255,0),3)
        cv2.putText(frame1,"gab="+str(gab),(int((x1+x3)/2)+10,120-10),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),1,cv2.LINE_AA)

    
        if redarea>10000:
            cv2.putText(frame1,"Detected",(200,30),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),1,cv2.LINE_AA)
        if greenarea>10000:
            cv2.putText(frame1,"Detected",(200,60),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,0),1,cv2.LINE_AA)
        
        cv2.imshow("Frame",frame1)


        key = cv2.waitKey(1) & 0xFF

        rawCapture.truncate(0)

        if key == ord("q"):
            break



### 명령구간 ###
def test2():

    if __name__ == '__main__':

        drone = Drone()
        drone.open()
        drone.sendLightManual(DeviceType.Drone, 0xFF, 0)
        sleep(1);

        print("TakeOff")
        drone.sendTakeOff()
        for i in range(3, 0, -1):
            print("{0}".format(i))
            sleep(1)

        print("Hovering")
        for i in range(2, 0, -1):
            print("{0}".format(i))
            drone.sendControlWhile(0, 0, 0, 0, 1000)
            sleep(0.01)
        sleep(2)

        print("오른쪽 회전")
        drone.sendControlPosition16(0, 0, 0, 0,-15, 20)
        for i in range(2, 0, -1):
            print("{0}".format(i))
            sleep(0.5)
    
    ### 비행 영역 ###

    ## 각도 확인 ##
        while(True):
            
            if int(y2)>(int((set2-set1)/2)) and angle2>5 and angle2<90:
                print("오른쪽 회전")
                drone.sendControlPosition16(0, 0, 0,0, -6, 20)
                for i in range(2, 0, -1):
                    print("{0}".format(i))
                    sleep(0.5)
                    #sleep(0.3)
                
            elif int(y2)<int((set2-set1)/2) and angle1>5 and angle1<90:
                print("왼쪽 회전")
                drone.sendControlPosition16(0, 0, 0, 0,6, 20)
                for i in range(2, 0, -1):
                    print("{0}".format(i))
                    sleep(0.5)
                    #sleep(0.3)
            elif (angle1<=7 and angle2>=83) or (angle1>=83 and angle2<=7):

                ## 위치 확인 ##
                if gab<-40 and gab>-100:
                    print("오른쪽 이동")
                    drone.sendControlPosition16(0, -1, 0, 5, 0, 0)
                    for i in range(2, 0, -1):
                        print("{0}".format(i))
                        sleep(1)
                        #sleep(0.3)
                elif gab>35:
                    print("왼쪽 이동")
                    drone.sendControlPosition16(0, 1, 0, 5, 0, 0)
                    for i in range(2, 0, -1):
                        print("{0}".format(i))        
                        sleep(1)
                        #sleep(0.3)  
                elif gab>=-40 and gab<=35:
                    print("전진")
                    drone.sendControlPosition16(1 , 0, 0, 5, 0, 0)
                    for i in range(2, 0, -1):
                        print("{0}".format(i))
                        sleep(0.5)

                ## 체크포인트 인식구간 ##
                if abs(160-int((x31+x33)/2))<=30 and redarea>=250:
                    t3=threading.Thread(target=test3)
                    t3.start()
                    t3.join()


                ## 목적지 도착 시 ##
                if abs(160-int((x21+x23)/2))<=30 and greenarea>=300:
                    while True:
                        if (int((x21+x23)/2)-160)<-30:
                            print("오른쪽 이동")
                            drone.sendControlPosition16(0, -1, 0, 5, 0, 0)
                            for i in range(2, 0, -1):
                                print("{0}".format(i))
                                sleep(0.5)
                        elif (int((x21+x23)/2)-160)>30:
                            print("왼쪽 이동")
                            drone.sendControlPosition16(0, 1, 0, 5, 0, 0)
                            for i in range(2, 0, -1):
                                print("{0}".format(i))
                                sleep(0.5)
                        
                        if (120-int((y21+y23)/2))<=-10:
                            print("전진")
                            drone.sendControlPosition16(1 , 0, 0, 5, 0, 0)
                            for i in range(2, 0, -1):
                                print("{0}".format(i))
                                sleep(0.5)
                                
                        elif gab2<=30:
                            print("착지준비")
                            sleep(2)
                            print("전진")
                            drone.sendControlPosition16(2 , 0, 0, 5, 0, 0)
                            for i in range(2, 0, -1):
                                print("{0}".format(i))
                                sleep(0.5)
                            print("Landing")
                            drone.sendLanding()
                            for i in range(5, 0, -1):
                                print("{0}".format(i))
                                sleep(1)
                            drone.close()
                            break

### 착지구간 ###         
def test3():

    
    if __name__ == '__main__':

        drone = Drone()
        drone.open()

        if 60<gab1<=120:
            
            print("직선구간_전진")
            drone.sendControlPosition16(6,0,0,5,0,0)
            for i in range(2,0,-1):
                print("{0}".format(i))
                sleep(0.5)
            
        elif 0<=gab1<=60:
            print("2차_전진")
            drone.sendControlPosition16(3,0,0,5,0,0)
            for i in range(2,0,-1):
                print("{0}".format(i))
                sleep(0.5)  

            
           
        print("착지준비")
        sleep(2)
        print("Landing")
        drone.sendLanding()
        for i in range(4, 0, -1):
            print("{0}".format(i))
            sleep(1)
        drone.sendTakeOff()
        for i in range(3,0,-1):
            print("{0}".format(i))
            sleep(1)

        print("Hovering")
        for i in range(3, 0, -1):
            print("{0}".format(i))
            drone.sendControlWhile(0, 0, 0, 0, 1000)
            sleep(0.01)

        print("전진")
        drone.sendControlPosition16(1,0,0,5,0,0)
        for i in range(2,0,-1):
            print("{0}".format(i))
            sleep(0.5)

        print("오른쪽 회전")
        drone.sendControlPosition16(0, 0, 0,0, -5, 15)
        for i in range(2, 0, -1):
            print("{0}".format(i))
            sleep(0.5)

              
    
t1=threading.Thread(target=test1)
t2=threading.Thread(target=test2)
t3=threading.Thread(target=test3)
t1.start()
t2.start()
