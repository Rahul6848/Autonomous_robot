from ultralytics import YOLO
import cv2

# Load trained model
model = YOLO("yolov8n.pt")

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    results = model(frame)

    for r in results:
        for box in r.boxes:
            cls = int(box.cls[0])
            
            if cls == 0:
                print("LEFT")
            elif cls == 1:
                print("RIGHT")
            elif cls == 2:
                print("STOP")

    cv2.imshow("Frame", frame)

    if cv2.waitKey(1) == 27:
        break