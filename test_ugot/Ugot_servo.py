from ugot import ugot
import time
got = ugot.UGOT()
got.initialize('10.220.5.228')
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

got.turn_servo_angle(servo_gap1,-170, 2000, True)
got.turn_servo_angle(servo_gap2,-60, 2000, True)
got.turn_servo_angle(servo_xoay, 170, 3000, True)

got.turn_servo_angle(servo_xoay, -120, 3000, True)

got.turn_servo_angle(servo_xoay, angle1, 3000, True)
got.turn_servo_angle(servo_gap1,angle2, 2000, True)
got.turn_servo_angle(servo_gap2,angle3, 2000, True)
got.mechanical_clamp_release()
print('-----1:',got.mechanical_get_clamp_status())
time.sleep(1)
got.mechanical_clamp_close()
print('-----2:',got.mechanical_get_clamp_status())
while  True:
    print(got.read_servo_angle(servo_xoay),got.read_servo_angle(servo_gap1),got.read_servo_angle(servo_gap2))


