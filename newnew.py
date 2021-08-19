import cv2  # opencv kütüphanesi dahil etme
import numpy as np  # Numpy kütüphanesi dahil etme
import sys
import time

import pymavlink.mavlink
from numpy.core.numeric import tensordot

from pymavlink import mavutil

vid = cv2.VideoCapture(0)  # Kamera aktif hale gelir
za = 0.3
x = 0
x1 = 230
x2 = 320
x3 = 410
x4 = 640
y = 0
y1 = 160
y2 = 240
y3 = 330
y4 = 480

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
    frem = cv2.blur(frem, (11, 11))  # img deişkenin bouyutları deiştir
    frem = cv2.flip(frem, -1)

    Hsv = cv2.cvtColor(frem, cv2.COLOR_BGR2HSV)  # BGR formatında gelen görüntüer HSV formatına çevrilir.

    red_lower = np.array([30, 60, 0])  # kırmızı rengın aralığı
    red_upper = np.array([100, 255, 120])  # kırmızı rengın aralığı

    mask = cv2.inRange(Hsv, red_lower, red_upper)  #
    # mask = cv2.erode(mask, None, iterations=2)  # Bizim rengleri işaretliyor
    # mask = cv2.dilate(mask, None, iterations=2)  # Bizim Renlerimizin genişliği alıyor

    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, mask.shape[0] / 4, param1=20, param2=10, minRadius=150,
                               maxRadius=300)  # çemberleri bul

    cv2.line(frem, (230, 0), (230, 480), (255, 0, 0), 1, 1)
    cv2.line(frem, (410, 0), (410, 480), (255, 0, 0), 1, 1)
    cv2.line(frem, (0, 170), (640, 170), (255, 0, 0), 1, 1)
    cv2.line(frem, (0, 310), (640, 310), (255, 0, 0), 1, 1)
    cv2.rectangle(frem, (230, 170), (410, 310), (0, 255, 0), 1, 1)
    cv2.line(frem, (230, 240), (410, 240), (0, 0, 255), 1, 1)
    cv2.line(frem, (320, 170), (320, 310), (0, 0, 255), 1, 1)

    if circles is not None:  # çember yoksa aşağıdaki kodları geç
        circles = np.uint16(np.around(circles))

        for i in circles[0, :]:
            cv2.circle(frem, (i[0], i[1]), i[2], (0, 255, 0), 2)  # istenilen kordinata çember oluştur
            cv2.circle(frem, (i[0], i[1]), 5, (0, 255, 0), -1)  # istenilen kordinata çember oluştur
            print("x Kordinatı : " + str(i[0]))  # Rengin bulundugu X Kordinati
            print("Y Kordinatı : " + str(i[1]))  # Rengin bulundugu Y Kordinati
            break

        if x < i[0] < x1 and y < i[1] < y1:  # ust sol
            master.mav.manual_control_send(
                master.target_system,
                200,
                -200,
                400,
                0,
                0)
            time.sleep(za)
        if x1 < i[0] < x3 and y < i[1] < y1:  # ust orta
            master.mav.manual_control_send(
                master.target_system,
                200,
                0,
                400,
                0,
                0)
            time.sleep(za)
        if x3 < i[0] < x4 and y < i[1] < y1:  # ust sag
            master.mav.manual_control_send(
                master.target_system,
                200,
                200,
                400,
                0,
                0)
            time.sleep(za)
        if x < i[0] < x1 and y1 < i[1] < y3:  # orat sol
            master.mav.manual_control_send(
                master.target_system,
                0,
                -200,
                400,
                0,
                0)
            time.sleep(za)

        if x1 < i[0] < x3 and y1 < i[1] < y3:  # orat orta
            master.mav.manual_control_send(
                master.target_system,
                0,
                0,
                450,
                0,
                0)
            time.sleep(0.3)

            if x1 < i[0] < x3 and y1 < i[1] < y3:
                if x1 < i[0] < x2 and y1 < i[1] < y2:
                    master.mav.manual_control_send(
                        master.target_system,
                        100,
                        -100,
                        0,
                        0,
                        0)
                    time.sleep(0.5)

                if x2 < i[0] < x3 and y1 < i[1] < y2:
                    master.mav.manual_control_send(
                        master.target_system,
                        100,
                        100,
                        0,
                        0,
                        0)
                    time.sleep(0.5)

                if x1 < i[0] < x2 and y2 < i[1] < y3:
                    master.mav.manual_control_send(
                        master.target_system,
                        -100,
                        -100,
                        0,
                        0,
                        0)
                    time.sleep(0.5)

                if x2 < i[0] < x3 and y2 < i[1] < y3:
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

        if x3 < i[0] < x4 and y1 < i[1] < y3:  # orat sag
            master.mav.manual_control_send(
                master.target_system,
                0,
                200,
                400,
                0,
                0)
            time.sleep(za)

        if x < i[0] < x1 and y3 < i[1] < y4:  # asga sol
            master.mav.manual_control_send(
                master.target_system,
                -200,
                -200,
                400,
                0,
                0)
            time.sleep(za)
        if x1 < i[0] < x3 and y3 < i[1] < y4:  # asga orta
            master.mav.manual_control_send(
                master.target_system,
                -200,
                0,
                400,
                0,
                0)
            time.sleep(za)

        if x3 < i[0] < x4 and y3 < i[1] < y4:  # asga sag
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
