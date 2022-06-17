import cv2  # opencv kütüphanesi dahil etme
import numpy as np  # Numpy kütüphanesi dahil etme
import sys
import time

import pymavlink.mavlink
from numpy.core.numeric import tensordot

from pymavlink import mavutil

vid = cv2.VideoCapture(0)  # Kamera aktif hale gelir
za = 0.3
# Create the connection
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Choose a mode
mode = 'STABILIZE'

# Check if mode is available
if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(master.mode_mapping().keys()))
    sys.exit(1)

# Get mode ID
mode_id = master.mode_mapping()[mode]
# Set new mode
# master.mav.command_long_send(
#    master.target_system, master.target_component,
#    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
#    0, mode_id, 0, 0, 0, 0, 0) or:
# master.set_mode(mode_id) or:
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)
while True:
    # Wait for ACK command
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    ack_msg = ack_msg.to_dict()

    # Check if command in the same in `set_mode`
    if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
        continue

    # Print the ACK result !
    print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    break

# Arm
master.arducopter_arm()
master.motors_armed_wait()

frame_width = int(vid.get(3))
frame_height = int(vid.get(4))

size = (frame_width, frame_height)

# Below VideoWriter object will create
# a frame of above defined The output
# is stored in 'filename.avi' file.
result = cv2.VideoWriter('filename.avi',
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         10, size)

while 1:  # aşağıdaki işlemleri sonsuza tekrarlansın
    time.sleep(0.2)

    ret, frem = vid.read()  # Kamerdan gelen görüntü frem değişkene aktarılır
    frem = cv2.resize(frem, (640, 480))  #
    frem = cv2.blur(frem, (5, 5))  # img deişkenin bouyutları deiştir
    frem = cv2.flip(frem, -1)

    Hsv = cv2.cvtColor(frem, cv2.COLOR_BGR2HSV)  # BGR formatında gelen görüntüer HSV formatına çevrilir.

    red_lower = np.array([170, 50, 50])
    red_upper = np.array([180, 255, 255])

    mask = cv2.inRange(Hsv, red_lower, red_upper)  #
    # mask = cv2.erode(mask, None, iterations=2)  # Bizim rengleri işaretliyor
    # mask = cv2.dilate(mask, None, iterations=2)  # Bizim Renlerimizin genişliği alıyor


    _,contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #Hedefin Adresi bolma

    contours = sorted(contours, key = lambda x: cv2.contourArea(x), reverse=True) # buyuktan kucuğa sırala

    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        ((x, y), radios) = cv2.minEnclosingCircle(c)

        M = cv2.moments(c) #Momenti hisapla ve M değşken içinde sakla
        Merkez = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])) # Merkez Adresi merkez değişken içersinde sakla

    for cnt in contours:
        area = cv2.contourArea(cnt)

        if area > 250:

            (x, y, w, h) = cv2.boundingRect(cnt) # hedfin alani ve adresi diğişken içinde saklar
            cv2.rectangle(frem, (x, y), (x + w, y + h), (0, 255, 0), 2) # Hedefin atrafinda kara çizme
            cv2.drawContours(frem,cnt, 0, (0, 255, 0), 2)
            frem1 = frem[y:y+h , x:x+w]
            x_kor =str(x+w/2)
            y_kor = str(y+h/2)
            cv2.circle(frem, (int(x+w/2) ,int(y+h/2) ), 5, (0, 255, 0), -1)
            print("x Kordinatı : " + x_kor ) # Rengin bulundugu X Kordinati
            print("Y Kordinatı : " + y_kor ) # Rengin bulundugu Y Kordinati
        break


        if 0 < i[0] < 230 and 0 < i[1] < 170:  # ust sol
            master.mav.manual_control_send(
                master.target_system,
                200,
                -200,
                400,
                0,
                0)
            time.sleep(za)
        if 230 < i[0] < 410 and 0 < i[1] < 170:  # ust orta
            master.mav.manual_control_send(
                master.target_system,
                200,
                0,
                400,
                0,
                0)
            time.sleep(za)
        if 410 < i[0] < 630 and 0 < i[1] < 170:  # ust sag
            master.mav.manual_control_send(
                master.target_system,
                200,
                200,
                400,
                0,
                0)
            time.sleep(za)
        if 0 < i[0] < 230 and 170 < i[1] < 310:  # orat sol
            master.mav.manual_control_send(
                master.target_system,
                0,
                -200,
                400,
                0,
                0)
            time.sleep(za)

        if 230 < i[0] < 410 and 170 < i[1] < 310:  # orat orta
            master.mav.manual_control_send(
                master.target_system,
                0,
                0,
                450,
                0,
                0)
            time.sleep(0.3)

            if 230 < i[0] < 410 and 170 < i[1] < 310:
                if 230 < i[0] < 320 and 170 < i[1] < 240:
                    master.mav.manual_control_send(
                        master.target_system,
                        100,
                        -100,
                        0,
                        0,
                        0)
                    time.sleep(0.5)

                if 320 < i[0] < 410 and 170 < i[1] < 240:
                    master.mav.manual_control_send(
                        master.target_system,
                        100,
                        100,
                        0,
                        0,
                        0)
                    time.sleep(0.5)

                if 230 < i[0] < 320 and 240 < i[1] < 310:
                    master.mav.manual_control_send(
                        master.target_system,
                        -100,
                        -100,
                        0,
                        0,
                        0)
                    time.sleep(0.5)

                if 320 < i[0] < 410 and 240 < i[1] < 310:
                    master.mav.manual_control_send(
                        master.target_system,
                        -100,
                        100,
                        0,
                        0,
                        0)
                    time.sleep(0.5)

                master.mav.manual_control_send(
                    master.target_system,
                    0,
                    0,
                    0,
                    0,
                    0)
                time.sleep(15)
                master.mav.manual_control_send(
                    master.target_system,
                    -600,
                    0,
                    750,
                    0,
                    0)
                time.sleep(15)
                master.arducopter_disarm()
                master.motors_disarmed_wait()
                break

        if 410 < i[0] < 630 and 170 < i[1] < 310:  # orat sag
            master.mav.manual_control_send(
                master.target_system,
                0,
                200,
                400,
                0,
                0)
            time.sleep(za)

        if 0 < i[0] < 230 and 310 < i[1] < 480:  # asga sol
            master.mav.manual_control_send(
                master.target_system,
                -200,
                -200,
                400,
                0,
                0)
            time.sleep(za)
        if 230 < i[0] < 410 and 310 < i[1] < 480:  # asga orta
            master.mav.manual_control_send(
                master.target_system,
                -200,
                0,
                400,
                0,
                0)
            time.sleep(za)

        if 410 < i[0] < 630 and 310 < i[1] < 480:  # asga sag
            master.mav.manual_control_send(
                master.target_system,
                -200,
                200,
                400,
                0,
                0)
            time.sleep(za)

        if i[0] and i[1] is None:
            master.mav.manual_control_send(
                master.target_system,
                350,
                0,
                330,
                0,
                0)
            time.sleep(za)

    result.write(frem)

    if cv2.waitKey(1) == 27:
        break


def test():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)


vid.release()
cv2.destroyAllWindows()
result.release()
