import numpy as np
import cv2

vc = cv2.VideoCapture(0)

while(True):
    ret, frame = vc.read()
    cv2.imshow("Framne", frame)

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

vc.release()
cv2.destroyAllWindows()