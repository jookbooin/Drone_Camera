import cv2

cap = cv2.VideoCapture(0)

i = 1
while(1):
    k = cv2.waitKey(1) & 0xFF 
    fileName = "Checker/checkerBoard" +str(i)+".jpg"
 
    ret, frame = cap.read()
 
    cv2.imshow("capture", frame)
    if  k == ord('s'):
        cv2.imwrite(fileName, frame)
        i+=1
    elif k == ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()