import math          # <------
import cv2           # <------ kutuphaneler 
import numpy as np   # <------

def nothing(x):      # <------TrackBar ulusturmak icin bos bir fonksiyon 
    # any operation  # <-----
    pass             # <-----

cap = cv2.VideoCapture(0) # Kamera acmak icin 

cv2.namedWindow("TRACKBAR")  # <---
cv2.namedWindow("TRACKBAR2") # <--- TrackBarlarin Pencereleri ulusturulmasi
cv2.namedWindow("TRACKBAR3") # <--- 
cv2.namedWindow("TRACKBAR4") # <---

cv2.createTrackbar("L-H","TRACKBAR",0,30,nothing)            # <---
cv2.createTrackbar("L-S","TRACKBAR",100,255,nothing)         # <---
cv2.createTrackbar("L-V","TRACKBAR",153 ,255,nothing) #RED 1 # <--- Birinci TrackBar
cv2.createTrackbar("U-H","TRACKBAR",23,30,nothing)           # <--- 
cv2.createTrackbar("U-S","TRACKBAR",255,255,nothing)         # <---
cv2.createTrackbar("U-V","TRACKBAR",255,255,nothing)         # <---

cv2.createTrackbar("L-H_2", "TRACKBAR2", 105, 180, nothing)          # <---
cv2.createTrackbar("L-S_2", "TRACKBAR2", 144, 255, nothing)          # <---
cv2.createTrackbar("L-V_2", "TRACKBAR2", 143, 255, nothing)  # RED 2 # <--- Ikinci TrackBar
cv2.createTrackbar("U-H_2", "TRACKBAR2", 124, 180, nothing)          # <---
cv2.createTrackbar("U-S_2", "TRACKBAR2", 226, 255, nothing)          # <---
cv2.createTrackbar("U-V_2", "TRACKBAR2", 231, 255, nothing)          # <---

cv2.createTrackbar("L-H_3", "TRACKBAR3",122,360, nothing)            # <---
cv2.createTrackbar("L-S_3", "TRACKBAR3", 59, 255, nothing)           # <---
cv2.createTrackbar("L-V_3", "TRACKBAR3", 123, 255, nothing)  # RED 3 # <--- Ucuncu TrackBar
cv2.createTrackbar("U-H_3", "TRACKBAR3", 203,360, nothing)           # <---
cv2.createTrackbar("U-S_3", "TRACKBAR3", 233, 255, nothing)          # <---
cv2.createTrackbar("U-V_3", "TRACKBAR3", 255, 255, nothing)          # <---

cv2.createTrackbar("L-H_4", "TRACKBAR4", 114,345 , nothing)          # <---
cv2.createTrackbar("L-S_4", "TRACKBAR4", 96, 255, nothing)           # <---
cv2.createTrackbar("L-V_4", "TRACKBAR4", 255, 255, nothing)  # RED 4 # <--- Dorduncu TrackBar
cv2.createTrackbar("U-H_4", "TRACKBAR4", 102, 345, nothing)          # <---
cv2.createTrackbar("U-S_4", "TRACKBAR4", 255, 255, nothing)          # <---
cv2.createTrackbar("U-V_4", "TRACKBAR4", 255, 255, nothing)          # <---

while True: # <--------------------------------------------# While Dongusuu
    ret, frame = cap.read()                                # Frameleri okunmasi
    frame = cv2.flip(frame , 1)                            # Videonun eksenini ayarlanmasi icin 
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)            # RGB uzayindan HSV uzayina gecisi


    l_h = cv2.getTrackbarPos("L-H","TRACKBAR")             # <---
    l_s = cv2.getTrackbarPos("L-S", "TRACKBAR")            # <---
    l_v = cv2.getTrackbarPos("L-V", "TRACKBAR")            # <--- Birinci TrackBar Cagirilmasi
    u_h = cv2.getTrackbarPos("U-H", "TRACKBAR")  # 1       # <---
    u_s = cv2.getTrackbarPos("U-S", "TRACKBAR")            # <---
    u_v = cv2.getTrackbarPos("U-V", "TRACKBAR")            # <---

    l_h_2 = cv2.getTrackbarPos("L-H_2", "TRACKBAR2")       # <---
    l_s_2 = cv2.getTrackbarPos("L-S_2", "TRACKBAR2")       # <---
    l_v_2 = cv2.getTrackbarPos("L-V_2", "TRACKBAR2")  # 2  # <--- Ikinci TrackBar Cagirilmasi
    u_h_2 = cv2.getTrackbarPos("U-H_2", "TRACKBAR2")       # <---
    u_s_2 = cv2.getTrackbarPos("U-S_2", "TRACKBAR2")       # <---
    u_v_2 = cv2.getTrackbarPos("U-V_2", "TRACKBAR2")       # <---
 
    l_h_3 = cv2.getTrackbarPos("L-H_3", "TRACKBAR3")       # <---
    l_s_3 = cv2.getTrackbarPos("L-S_3", "TRACKBAR3")       # <---
    l_v_3 = cv2.getTrackbarPos("L-V_3", "TRACKBAR3")  # 3  # <--- Ucuncu TrackBar Cagirilmasi
    u_h_3 = cv2.getTrackbarPos("U-H_3", "TRACKBAR3")       # <---
    u_s_3 = cv2.getTrackbarPos("U-S_3", "TRACKBAR3")       # <---
    u_v_3 = cv2.getTrackbarPos("U-V_3", "TRACKBAR3")       # <---

    l_h_4 = cv2.getTrackbarPos("L-H_4", "TRACKBAR4")       # <---
    l_s_4 = cv2.getTrackbarPos("L-S_4", "TRACKBAR4")       # <---
    l_v_4 = cv2.getTrackbarPos("L-V_4", "TRACKBAR4")  # 4  # <--- Dorduncu TrackBar Cagirilmasi
    u_h_4 = cv2.getTrackbarPos("U-H_4", "TRACKBAR4")       # <---
    u_s_4 = cv2.getTrackbarPos("U-S_4", "TRACKBAR4")       # <---
    u_v_4 = cv2.getTrackbarPos("U-V_4", "TRACKBAR4")       # <---
 
    ########################### FIND RANGE ##############################################

    # 1 lower boundary RED color range values; Hue (0 - 30)
    lower1 = np.array([l_h, l_s, l_v])
    upper1 = np.array([u_h, u_s, u_v])
 
    # 2 upper boundary RED color range values; Hue (160 - 180)
    lower2 = np.array([l_h_2,l_s_2,l_v_2])
    upper2 = np.array([u_h_2,u_s_2,u_v_2])

    # 3 lower boundary RED color range values; Hue (160 - 360)    
    lower3 = np.array([l_h_3,l_s_3,l_v_3])
    upper3 = np.array([u_h_3,u_s_3,u_v_3])

    # 4 upper boundary RED color range values; Hue (160 - 345)
    lower4 = np.array([l_h_4,l_s_4,l_v_4])
    upper4 = np.array([u_h_4,u_s_4,u_v_4])


    ############################### STEP 1 DO MASK ####################################
    

   # kernel = np.ones((5, 5), np.uint8)          # <--- To creat black squer in mask
    lower_mask1 = cv2.inRange(hsv, lower1, upper1)  # <---
    upper_mask2 = cv2.inRange(hsv, lower2, upper2)  # <--- 1,2,3,4 Masklarini ulusturulmasi
    lower_mask3 = cv2.inRange(hsv, lower3, upper3)  # <---
    upper_mask4 = cv2.inRange(hsv, lower4, upper4)  # <---
    
    #lower_mask1 =  cv2.erode(lower_mask1, kernel)
    #upper_mask2 = cv2.erode(upper_mask2, kernel)
    #lower_mask3 = cv2.erode(lower_mask3, kernel)
    #upper_mask4 = cv2.erode(upper_mask4, kernel)


    mask_Fainal = lower_mask1 + upper_mask2 + lower_mask3 + upper_mask4 # <--- 4 masklerin Toplanmasi
    
    # mask_Fainal = cv2.erode(mask_Fainal, kernel)


    lower_mask1 = cv2.bitwise_and(frame, frame, mask=lower_mask1) # <---
    upper_mask2 = cv2.bitwise_and(frame, frame, mask=upper_mask2) # <--- Maskin Pencerelerinde bulmak istenilen renk yansmak icin 
    lower_mask3 = cv2.bitwise_and(frame, frame, mask=lower_mask3) # <--- ((Siyah Beyaz Pencere olmak yerine))
    upper_mask4 = cv2.bitwise_and(frame, frame, mask=upper_mask4) # <---
 
    ################################# STEP 2 FIND CONTOURS #############################

    contours, _ = cv2.findContours(mask_Fainal, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # <--- Nesnenin Cercevesini Bulmak

    detections = []


     ################################# STEP 3 DETECT THE SHAPS ############################



    for cnt in contours:            # <------ TO draw Contours 
        area = cv2.contourArea(cnt) # <------
        
        approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True) # <--- To do contours lines streat
        #print("approx", len(approx))
        x = approx.ravel()[0] # <--- Cizilen contourun tegitinin kordinati 
        y = approx.ravel()[1] # <---

        if area > 400:        # <--- Contourun alani 400 den fazla olunca asagidaki islemler gerciklessin
            #cv2.drawContours(frame, [approx], 0, (0, 255, 0), 1) # <--- contouru ciz
            x1, y1, w, h = cv2.boundingRect(cnt)                  # <--- To draw rectangle around shape
            #cv2.rectangle(frame, (x1, y1), (x1 + w, y1 + h), (0, 0, 255), 2) # <-- draw it
            detections.append([x1, y1, w, h]) # <--- Dortgenin kordinatini bir matrisin icinde koymak icin


            ###### FIND MOMENT TOFIND CENTER FOR SHAPS ##########

            M = cv2.moments(cnt) 
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(frame, "Y =", (cx + 200, cy + 10), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
            cv2.putText(frame, str(cy), (cx + 250, cy + 10), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
            cv2.putText(frame, "X =", (cx + 200, cy - 10), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
            cv2.putText(frame, str(cx), (cx + 250, cy - 10), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
            cv2.putText(frame, "Center", (cx + 200, cy - 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
            
            ################################## FIND TRIANGLE ###########################


            if len(approx) == 3:
                E = x1 + w  # Triangle's Base
                F = y1 + h

                Base = round((math.sqrt(((x1 - E) ** 2) + ((F - F) ** 2)))/4)  # Base Line
                R = round( (math.sqrt(((x1 - cx) ** 2) + ((F - y1) ** 2)))/4)  # Right Line
                T = round((math.sqrt(((E - cx) ** 2) + ((F - y1) ** 2)))/4)  # LeftLine
                

                '''print("Base =",Base)
                print("R =",R)
                print("T =",T)'''

                A_O_T = round(((Base * h) / 2))  # Area Of Triangle
                P_O_T = round((Base + R + T )) # Perimeter Of Triangle
                '''print("A_O_T =",A_O_T)
                print("P_O_T =",P_O_T)'''

                #cv2.drawContours(frame, [approx], 0, (0, 255, 0), 2)
                cv2.putText(frame, "Triangle", (cx + 200, y - 20), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
                cv2.putText(frame, "A_O_T--->", (cx + 200, cy + 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                cv2.putText(frame, str(A_O_T), (cx + 300, cy + 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                cv2.putText(frame, "mm2", (cx + 350, cy + 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                cv2.putText(frame, "P_O_T--->", (cx + 200, cy + 60), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                cv2.putText(frame, str(P_O_T), (cx + 300, cy + 60), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                cv2.putText(frame, "mm3", (cx + 350, cy + 60), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                # Base
                cv2.line(frame, (x1+40, F+8), (E-40,F+8), (0, 0, 255),2)
                cv2.putText(frame, "--->", (E-40,F+13), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                cv2.putText(frame, "<---", (x1-5,F+13), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                cv2.putText(frame, "Base = ",(cx-50,F+30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                cv2.putText(frame, str(Base), (cx+10,F+30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                cv2.putText(frame, "mm", (cx+30,F+30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                # T
                
                cv2.putText(frame, "T = ",(cx+70,cy), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                cv2.putText(frame, str(T), (cx+100, cy), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                cv2.putText(frame, "mm", (cx+120, cy), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                # R
                
                cv2.putText(frame, "R = ", (x1-40, cy), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                cv2.putText(frame, str(R), (x1-10,cy), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                cv2.putText(frame, "mm", (x1+10,cy), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                
               ###################### FIND RECTANGLE AND SQUARE #########################

            elif len(approx) == 4:
                #cv2.drawContours(frame, [cnt], 0, (0, 255, 0), 2)
                cv2.drawContours(frame, [approx], 0, (0, 255, 0), 2)
                
                
                E = x1 + w 
                F = y1 + h

                A = round((math.sqrt(((x1 - E) ** 2) + (( F- F) ** 2)))/4)
                B =round( (math.sqrt(((E - E) ** 2) + (( F-y1) ** 2)))/4)
                C = round((math.sqrt(((x1 - E) ** 2) + (( y1- y1) ** 2)))/4)
                D = round((math.sqrt(((x1 - x1) ** 2) + (( F- y1) ** 2)))/4)

                if A != B :
                    A_O_R = round((A* B))         # <--- Area Of Rectangle
                    P_O_R = round(((A + B) * 2))  # <--- Perimeter Of Rectangle
 
                    #print("A =",A)
                    #print("B =",B)
                    #print("C =",C)
                    #print("D =",D)
                    #print("A_O_R =",A_O_R)
                    #print("P_O_R =",P_O_R)
                    cv2.putText(frame, "A_O_R--->", (cx + 200, cy + 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                    cv2.putText(frame, str(A_O_R), (cx + 300, cy + 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                    cv2.putText(frame, "mm2", (cx + 350, cy + 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                    cv2.putText(frame, "P_O_R--->", (cx + 200, cy + 60), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                    cv2.putText(frame, str(P_O_R), (cx + 300, cy + 60), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                    cv2.putText(frame, "mm3", (cx + 350, cy + 60), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                    cv2.putText(frame, "Rectangle", (cx + 200, y - 20), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
                    # A
                    cv2.line(frame, (x1, F+8), (E,F+8), (0, 0, 255),2)
                    cv2.putText(frame, ">", (E-5,F+13), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, "<", (x1,F+13), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, "A = ", (cx-20,F+25), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, str(A), (cx+10,F+25), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, "mm", (cx+30,F+25), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    # B
                    cv2.line(frame, (E+10, F), (E+10,y1), (0, 0, 255),2)
                    cv2.putText(frame, "^", (E+4,y1+10), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, "B =", (E+20,cy), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, str(B), (E+55,cy), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, "mm", (E+75,cy), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    # c
                    cv2.line(frame, (E, y1-10), (x1,y1-10), (0, 0, 255),2)
                    cv2.putText(frame, ">", (E-10, y1-5), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, "<",(x1,y1-5), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, "C =", (cx-20,y1-20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, str(C), (cx+10,y1-20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, "mm", (cx+30,y1-20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    # D
                    cv2.line(frame, (x1-10 ,y1), (x1-10,F), (0, 0, 255),2)
                    cv2.putText(frame, "^",(x1-16 ,y1+10), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, "D =", (x1-80,cy), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, str(D), (x1-50,cy), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, "mm", (x1-30,cy), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                         
                    
                
                # Square

                elif A == B:
                    
                    A_O_S =round(A * A)  # Area Of Square
                    P_O_S =round(A * 4) # Perimeter Of Square
                    #print("A =",A)
                    #print("B =",B)
                    #print("C =",C)
                    #print("D =",D)
                    
                    #print("A_O_S =",A_O_S)
                    #print("P_O_S =",P_O_S)
                    cv2.putText(frame, "A_O_S--->", (cx + 200, cy + 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                    cv2.putText(frame, str(A_O_S), (cx + 300, cy + 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                    cv2.putText(frame, "mm2", (cx + 350, cy + 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                    cv2.putText(frame, "P_O_S--->", (cx + 200, cy + 60), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                    cv2.putText(frame, str(P_O_S), (cx + 300, cy + 60), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                    cv2.putText(frame, "mm3", (cx + 350, cy + 60), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                    cv2.putText(frame, "Square", (cx + 200, y - 20), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
                    

                     # A
                    cv2.line(frame, (x1, F+8), (E,F+8), (0, 0, 255),2)
                    cv2.putText(frame, ">", (E-5,F+13), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, "<", (x1,F+13), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, "A = ", (cx-20,F+25), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, str(A), (cx+10,F+25), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, "mm", (cx+30,F+25), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    # B
                    cv2.line(frame, (E+10, F), (E+10,y1), (0, 0, 255),2)
                    cv2.putText(frame, "^", (E+4,y1+10), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, "B =", (E+20,cy), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, str(B), (E+55,cy), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, "mm", (E+75,cy), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    # c
                    cv2.line(frame, (E, y1-10), (x1,y1-10), (0, 0, 255),2)
                    cv2.putText(frame, ">", (E-10, y1-5), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, "<",(x1,y1-5), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, "C =", (cx-20,y1-20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, str(C), (cx+10,y1-20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, "mm", (cx+30,y1-20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    # D
                    cv2.line(frame, (x1-10 ,y1), (x1-10,F), (0, 0, 255),2)
                    cv2.putText(frame, "^",(x1-16 ,y1+10), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, "D =", (x1-80,cy), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, str(D), (x1-50,cy), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                    cv2.putText(frame, "mm", (x1-30,cy), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                
                


            ##################### FIND CIRCLE #########################

            if 8 <= len(approx) < 9:
                #cv2.drawContours(frame, [approx], 0, (0, 255, 0), 2)
                
                E = x1 + w  # Triangle's Base
                F = y1 + h
                j = cx - x1
                r = round((math.sqrt(((cx - cx) ** 2) + (( cy - y1) ** 2)))/4)  # r
                
                pi = 22 / 7
                A_O_C = round((pi * ((r) ** 2)))  # Area Of Circle
                P_O_C = round((pi * r * 2))  # Perimeter Of Circle
                #print("j =", j)
                #print("r =", r)
                #print("pi =", pi)
                #print("A_O_C=", A_O_C)
                #print("P_O_C=", P_O_C)

                

                
                cv2.circle(frame, (cx, cy), j, (0, 255, 0), 2)

                cv2.putText(frame, "A_O_C--->", (cx + 200, cy + 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                cv2.putText(frame, str(A_O_C), (cx + 300, cy + 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                cv2.putText(frame, "mm2", (cx + 350, cy + 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                cv2.putText(frame, "P_O_C--->", (cx + 200, cy + 60), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                cv2.putText(frame, str(P_O_C), (cx + 300, cy + 60), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                cv2.putText(frame, "mm3", (cx + 350, cy + 60), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                cv2.putText(frame, "Circle", (cx + 200, y - 20), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)


                cv2.line(frame, (cx, cy), (E-50, cy), (0, 0, 255),2)
                cv2.putText(frame, "--->", (E-50,cy+5), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                cv2.putText(frame, "r = ",(cx+10, cy+20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                cv2.putText(frame, str(r), (cx+40,cy+20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
                cv2.putText(frame, "mm", (cx+65,cy+20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)

    cv2.imshow("Frame",frame)              # <------
    cv2.imshow("lower_mask1", lower_mask1) # <------
    cv2.imshow("upper_mask2", upper_mask2) # <------
    cv2.imshow("lower_mask3", lower_mask3) # <------ Frame ve Masklarin Pencereleri Acmak  Icin
    cv2.imshow("upper_mask4", upper_mask4) # <------
    
    cv2.imshow("mask_Fainal", mask_Fainal) # <------
    
    #cv2.imshow("kernel", kernel)          # <------



    key = cv2.waitKey(1)   # <------ Esc Tusuna basinca Pencereleri Kapatir
    if key == 27:
        break

#cap.release()
#cv2.destroyWindow()

