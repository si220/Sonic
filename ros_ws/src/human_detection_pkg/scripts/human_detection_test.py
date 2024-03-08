#L515 RealSense
from ultralytics import YOLO
import cv2

model = YOLO('yolov8n.pt')

#8 is rgb cam
cap = cv2.VideoCapture(8)

ret = True
while ret:
    ret, frame = cap.read()

    if ret:
        frame = cv2.flip(frame, 1)

        #only draw on persons(which has class = 0)
        results = model.track(frame, persist=True, classes=[0])

        #frame_ = results[0].plot()
        #print(results[0].boxes)

        #draw boxes aroung humans
        boxes = results[0].boxes.xyxy
        boxes = boxes.numpy()
        for box in boxes:
            x1, y1, x2, y2 = box
            start_point = (int(x1), int(y1))
            end_point = (int(x2), int(y2))
            color = (0, 0, 255) #red box
            thickness = 8 
            cv2.rectangle(frame, start_point, end_point, color, thickness)

        cv2.imshow('frame', frame)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
