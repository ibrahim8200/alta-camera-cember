
import cv2 # opencv kütüphanesi dahil etme
import numpy as np  # Numpy kütüphanesi dahil etme



name = "video.avi"
cadoc = cv2.VideoWriter_fourcc('W ','M', 'V', '2')
fremRate=30
boyut = (640 , 480)
videoFileOutput = cv2.VideoWriter(name , cadoc, fremRate, boyut )


vid = cv2.VideoCapture("C:\\Users\\hh456\\OneDrive\\Desktop\\4.1.mp4") #Kamera aktif hale gelir
while 1:  #aşağıdaki işlemleri sonsuza tekrarlansın
    ret, frem = vid.read() #Kamerdan gelen görüntü frem değişkene aktarılır
    frem = cv2.resize(frem, (640, 480)) #
    frem = cv2.blur(frem, (11, 11))  # img deişkenin bouyutları deiştir



    Hsv = cv2.cvtColor(frem , cv2.COLOR_BGR2HSV)  #BGR formatında gelen görüntüer HSV formatına çevrilir.

    red_lower = np.array([90, 71, 110])  # kırmızı rengın aralığı
    red_upper = np.array([140, 120, 127])  # kırmızı rengın aralığı

    mask = cv2.inRange(Hsv, red_lower, red_upper) #
    #mask = cv2.erode(mask, None, iterations=2)  # Bizim rengleri işaretliyor
    #mask = cv2.dilate(mask, None, iterations=2)  # Bizim Renlerimizin genişliği alıyor

    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, mask.shape[0] / 4, param1=20, param2=10, minRadius=150,
                               maxRadius=300)    #çemberleri bul



    cv2.line(frem,(250,0),(250,480),(255,0,0),1,1)
    cv2.line(frem,(390,0),(390,480),(255,0,0),1,1)
    cv2.line(frem,(0,180),(640,180),(255,0,0),1,1)
    cv2.line(frem,(0,300),(640,300),(255,0,0),1,1)
    cv2.rectangle(frem,(250,180),(390,300),(0,255,0),1,1)

    if circles is None:
        cv2.putText(frem,"Cember Yok",(30,20),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,(100,100,00))
    if circles is not None: # çember yoksa aşağıdaki kodları geç
         circles = np.uint16(np.around(circles))

         for i in circles[0, :]:
             cv2.circle(frem, (i[0], i[1]), i[2], (0, 255, 0), 2) # istenilen kordinata çember oluştur
             cv2.circle(frem, (i[0], i[1]), 5, (0, 255, 0), -1)   # istenilen kordinata çember oluştur
             print("x Kordinatı : " + str(i[0]))  # Rengin bulundugu X Kordinati
             print("Y Kordinatı : " + str(i[1]))  # Rengin bulundugu Y Kordinati
             break

    cv2.imshow("video", frem) # fremleri göster
    cv2.imshow("mask", mask)   #kırmızı rengin algılaması göster

    videoFileOutput.write(frem)

    if cv2.waitKey(1) == 27:
        break

vid.release()
cv2.destroyAllWindows()
videoFileOutput.release()