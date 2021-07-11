import cv2

capture = cv2.VideoCapture(0)
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

while True:
    ret, frame = capture.read()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    #h = cv2.inRange(h, 0, 2)
    #orange = cv2.bitwise_and(hsv, hsv, mask = h)
    #orange = cv2.cvtColor(orange, cv2.COLOR_HSV2BGR)

    lower_red = cv2.inRange(hsv, (0, 100, 100), (5, 255, 255))
    
    upper_red = cv2.inRange(hsv, (170, 100, 100), (180, 255, 255))

    # (이미지1, 이미지1 비율, 이미지2, 이미지2 비율, 가중치)
    added_red = cv2.addWeighted(lower_red, 1.0, upper_red, 1.0, 0.0)

    red = cv2.bitwise_and(hsv, hsv, mask = added_red)
    red = cv2.cvtColor(red, cv2.COLOR_HSV2BGR)

    lower_blue = cv2.inRange(hsv, (100, 100, 100), (105, 255, 255))
    
    upper_blue = cv2.inRange(hsv, (145, 100, 100), (150, 255, 255))

    added_blue = cv2.addWeighted(lower_blue, 1.0, upper_blue, 1.0, 0.0)

    blue = cv2.bitwise_and(hsv, hsv, mask = added_blue)
    blue = cv2.cvtColor(blue, cv2.COLOR_HSV2BGR)

    
    # 빨간색 캐니
    gray = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(gray, 50, 200)

    # 파란색 캐니
    gray1 = cv2.cvtColor(blue, cv2.COLOR_BGR2GRAY)
    canny1 = cv2.Canny(gray1, 50, 200)
    
    cv2.imshow("VideoFrame", frame)
    #cv2.imshow("orange", orange)
    cv2.imshow("red", red)
    #cv2.imshow("h", h)
    #cv2.imshow("s", s)
    #cv2.imshow("v", v)
    cv2.imshow("red_canny", canny)
    cv2.imshow("blue", blue)
    cv2.imshow("blue_canny", canny1)
    
    if cv2.waitKey(1) > 0: break

capture.release()
cv2.destroyAllWindows()
