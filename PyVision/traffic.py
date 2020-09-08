import cv2
import serial

ser=serial.Serial('/dev/ttyUSB0',115200)

csae_path="/home/pi/traffic_light_5/GREEN15.xml"
csae2_path="/home/pi/traffic_light_5/RED14.xml"
cap=cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)
print(cap.get(3),cap.get(4))                    #顯示目前畫質
greenCascade=cv2.CascadeClassifier(csae_path)   #建立綠燈分類器物件
redCascade=cv2.CascadeClassifier(csae2_path)    #建立紅燈分類器物件
while (True):
    _,image=cap.read()
    #更改影像大小
    imagename=cv2.resize(src=image,dsize=(int(image.shape[1]*0.6),int(image.shape[0]*0.6)))
    #綠燈分類器
    green = greenCascade.detectMultiScale(imagename,scaleFactor=1.1,minNeighbors=5,minSize=(1,1),flags=cv2.CASCADE_SCALE_IMAGE)
    #紅燈分類器
    red = redCascade.detectMultiScale(imagename,scaleFactor=1.1,minNeighbors=5,minSize=(1,1),flags=cv2.CASCADE_SCALE_IMAGE)
    
    for (x,y,w,h) in green:
        cv2.rectangle(imagename,(x,y),(x+w,y+h),(0,255,0),2)         #框住綠燈區塊
        cv2.putText(imagename,'go',(int(x+(w/2)),y-30),cv2.FONT_HERSHEY_COMPLEX,1,(255,0,255),thickness=1)   #於影像上增加文字 go
        print('green',end=' ')
        print(x,y,w,h)
        ser.write(b'g')

    for (x,y,w,h) in red:
        cv2.rectangle(imagename,(x,y),(x+w,y+h),(0,0,255),2)         #框住紅燈區塊
        cv2.putText(imagename,'stop',(int(x+(w/2)),y-30),cv2.FONT_HERSHEY_COMPLEX,1,(255,0,255),thickness=1)  #於影像上增加文字 stop
        print('red',end=' ')
        print(x,y,w,h)
        ser.write(b's')
    
    cv2.imshow("image",imagename)
    if cv2.waitKey(1) & 0xFF==ord('q'):
        break

ser.close()
cap.release()
cv2.destroyAllWindows()

