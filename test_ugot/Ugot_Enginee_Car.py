from ugot import ugot
import cv2
import numpy as np
import utlis
import time

got = ugot.UGOT()
got.initialize('10.220.5.228')
got.load_models(['human_pose'])
data_sound = 'beep-warning-6387.mp3'
#got.initialize('10.220.5.233')
servo_xoay = [51]
servo_gap1 = [52]
servo_gap2 = [53]
angle1 = 90
angle2 = -125
angle3 = -100
duration = 1000
#
got.turn_servo_angle(servo_xoay, angle1, 1000, True)
got.turn_servo_angle(servo_gap1,angle2, 1000, True)
got.turn_servo_angle(servo_gap2,angle3, 1000, True)

curveList = []
avgVal = 10
enb=False

Kp = 50
# Tốc độ cơ bản
base_speed = 60
prev_speedL = 0
prev_speedR = 0
got.open_camera()

# Điều khiển robot xuất phát, dừng lại bằng chuột
def capture_image(event, x, y, flags, param):
    global enb
    if event == cv2.EVENT_RBUTTONDOWN:
        enb=True
        # got.mecanum_motor_control(base_speed, base_speed, base_speed, base_speed)
        got.mecanum_move_xyz(0, base_speed, 0)
    if event == cv2.EVENT_MBUTTONDOWN:
        got.mecanum_stop()
        enb = False

#Nhận diện Lane
def getLaneCurve(img, display=2):
    imgCopy = img.copy()
    imgResult = img.copy()
    #### STEP 1
    imgThres = utlis.thresholding(img)

    #### STEP 2
    hT, wT, c = img.shape
    points = utlis.valTrackbars()
    imgWarp = utlis.warpImg(imgThres, points, wT, hT)
    imgWarpPoints = utlis.drawPoints(imgCopy, points)

    #### STEP 3
    middlePoint, imgHist = utlis.getHistogram(imgWarp, display=True, minPer=0.5, region=4)
    curveAveragePoint, imgHist = utlis.getHistogram(imgWarp, display=True, minPer=0.9)
    curveRaw = curveAveragePoint - middlePoint

    #### SETP 4
    curveList.append(curveRaw)
    if len(curveList) > avgVal:
        curveList.pop(0)
    curve = int(sum(curveList) / len(curveList))

    #### STEP 5
    if display != 0:
        imgInvWarp = utlis.warpImg(imgWarp, points, wT, hT, inv=True)
        imgInvWarp = cv2.cvtColor(imgInvWarp, cv2.COLOR_GRAY2BGR)
        imgInvWarp[0:hT // 3, 0:wT] = 0, 0, 0
        imgLaneColor = np.zeros_like(img)
        imgLaneColor[:] = 0, 255, 0
        imgLaneColor = cv2.bitwise_and(imgInvWarp, imgLaneColor)
        imgResult = cv2.addWeighted(imgResult, 1, imgLaneColor, 1, 0)
        midY = 450
        cv2.putText(imgResult, str(curve), (wT // 2 - 80, 85), cv2.FONT_HERSHEY_COMPLEX, 2, (255, 0, 255), 3)
        cv2.line(imgResult, (wT // 2, midY), (wT // 2 + (curve * 3), midY), (255, 0, 255), 5)
        cv2.line(imgResult, ((wT // 2 + (curve * 3)), midY - 25), (wT // 2 + (curve * 3), midY + 25), (0, 255, 0), 5)
        for x in range(-30, 30):
            w = wT // 20
            cv2.line(imgResult, (w * x + int(curve // 50), midY - 10),
                     (w * x + int(curve // 50), midY + 10), (0, 0, 255), 2)
    if display == 2:
        imgStacked = utlis.stackImages(0.7, ([img, imgWarpPoints, imgWarp],
                                             [imgHist, imgLaneColor, imgResult]))
        cv2.imshow('ImageStack', imgStacked)
        cv2.setMouseCallback("ImageStack", capture_image, imgStacked)
    elif display == 1:
        cv2.imshow('Resutlt', imgResult)
        cv2.setMouseCallback("Resutlt", capture_image, imgResult)
    #### NORMALIZATION
    curve = curve / 100
    if curve > 1: curve == 1
    if curve < -1: curve == -1

    return curve

#Nhận diện màu sắc trong khuôn hình vuông
def is_square(contour, epsilon=0.02):
    peri = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon * peri, True)
    return len(approx) == 4

def detect_large_red_square(frame, area_threshold):
    # Define the range for red color in HSV
    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 50])
    upper_red2 = np.array([180, 255, 255])

    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create masks for red color
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        if is_square(contour):
            x, y, w, h = cv2.boundingRect(contour)
            area = w * h
            if area > area_threshold:
                return True
    return False

def detect_large_green_square(frame, area_threshold):
    lower_green = np.array([35, 100, 100])
    upper_green = np.array([85, 255, 255])

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, lower_green, upper_green)

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        if is_square(contour):
            x, y, w, h = cv2.boundingRect(contour)
            area = w * h
            if area > area_threshold:
                return True
    return False

if __name__ == '__main__':

    try:
        intialTrackBarVals = [184, 80, 86, 240]
        utlis.initializeTrackbars(intialTrackBarVals)
        frameCounter = 0
        while True:
            # print(enb,prev_speedL)
            frame = got.read_camera_data()
            if frame is not None:
                nparr = np.frombuffer(frame, np.uint8)
                data = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                got.get_pose_total_info()
                if detect_large_red_square(data, 5000):
                    got.mecanum_stop()
                    print("Large red square detected")
                    got.turn_servo_angle(servo_gap1, -70, 1000, True)
                    got.turn_servo_angle(servo_gap2, -40, 1000, True)
                    got.turn_servo_angle(servo_xoay, 90, 1000, True)
                    time.sleep(1)
                    while True:
                        nparr = np.frombuffer(frame, np.uint8)
                        data = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                        arr = got.get_pose_total_info()
                        print(arr)
                        if not arr:
                            break
                        else:
                            got.play_sound('police_car_2', wait=True)
                            got.mecanum_stop()
                            time.sleep(0.2)

                    got.turn_servo_angle(servo_xoay, -120, 5000, True)
                    while True:
                        nparr = np.frombuffer(frame, np.uint8)
                        data = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                        arr = got.get_pose_total_info()
                        print(arr)
                        if not arr:
                            break
                        else:
                            got.play_sound('police_car_2', wait=True)
                            got.mecanum_stop()
                            time.sleep(0.2)

                    got.turn_servo_angle(servo_xoay, angle1, 1000, True)
                    got.turn_servo_angle(servo_gap1, angle2, 1000, True)
                    got.turn_servo_angle(servo_gap2, angle3, 1000, True)

                    got.mecanum_turn_speed_times(3,60,90,2)
                    got.mecanum_stop()
                    time.sleep(0.2)

                elif detect_large_green_square(data, 5000):
                    got.mecanum_stop()
                    enb = False
                    while True:
                        print(enb)

                else:
                    print("No large color square detected")
                img = cv2.resize(data, (480, 280))
                curve = getLaneCurve(img, display=2)
                correction = int(Kp * curve)
                # print(curve,correction)
                if enb is True:
                    got.mecanum_move_xyz(correction, base_speed, 0)
                cv2.waitKey(1)
                if cv2.waitKey(1) and 0xFF == ord('q'):
                    break
    except KeyboardInterrupt:
        print('-----KeyboardInterrupt')