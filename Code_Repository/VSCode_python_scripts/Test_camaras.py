#test de camaras opencv
import cv2
print("OpenCV version:", cv2.__version__)

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: No se pudo abrir la webcam")
else:
    print("Webcam detectada correctamente!")
    while True:
        ret, frame = cap.read()
        if ret:
            cv2.imshow('Webcam Test', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    cap.release()
    cv2.destroyAllWindows()