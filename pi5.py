import cv2
from ultralytics import YOLO
import serial
import time
import sys

# configuration
MODEL_NAME = 'yolov8n.pt'      # nano model
CONFIDENCE_THRESHOLD = 0.5     # minimum confidence to track
LOST_TARGET_THRESHOLD = 30     # frames to wait before resetting ID
SERIAL_PORT = '/dev/ttyAMA0'   # UART Pin on Pi 5 (GPIO 14 TX / 15 RX)
BAUD_RATE = 115200
CAMERA_ID = 0                  # 0 is usually the default Camera Module

# serial connect
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    ser.flush()
    print(f"[INFO] Serial connected on {SERIAL_PORT}")
except Exception as e:
    print(f"[WARNING] Serial connection failed: {e}")
    print("[INFO] Running in Vision-Only mode (No motor control)")
    ser = None

# load model
print(f"[INFO] Loading YOLOv8 Model: {MODEL_NAME}...")
model = YOLO(MODEL_NAME)

# camera setup
cap = cv2.VideoCapture(CAMERA_ID)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("[ERROR] Could not open Camera Module 3.")
    sys.exit()

# get frame dim
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
frame_center_x = frame_width // 2
frame_center_y = frame_height // 2

# tracking
target_id = None
lost_target_counter = 0

print("[INFO] System Ready. Press 'q' to quit.")

try:
    while True:
        success, frame = cap.read()
        if not success:
            print("[ERROR] Failed to read frame")
            break

        # run yolo
        # persist=True keeps the ID assigned to the same object
        results = model.track(frame, classes=0, conf=CONFIDENCE_THRESHOLD, persist=True, verbose=False)
        
        # manual draw for optimize performance
        annotated_frame = frame 
        
        target_found = False

        # if objects are detected
        if results[0].boxes.id is not None:
            boxes = results[0].boxes.xyxy.cpu().numpy().astype(int)
            ids = results[0].boxes.id.cpu().numpy().astype(int)

            # aquire mode: find a target if none
            if target_id is None:
                min_dist = float('inf')
                best_id = -1
                
                for i, box in enumerate(boxes):
                    # calculate dist from cent
                    cx = (box[0] + box[2]) // 2
                    cy = (box[1] + box[3]) // 2
                    dist = ((cx - frame_center_x)**2 + (cy - frame_center_y)**2)**0.5
                    
                    # draw all in red
                    cv2.rectangle(annotated_frame, (box[0], box[1]), (box[2], box[3]), (0, 0, 255), 2)
                    
                    if dist < min_dist:
                        min_dist = dist
                        best_id = ids[i]
                
                # lock to closest
                if best_id != -1:
                    target_id = best_id
                    lost_target_counter = 0
                    print(f"[TRACKING] Acquired Target ID: {target_id}")

            # tracking mode: follow locked ID
            if target_id is not None:
                for i, box in enumerate(boxes):
                    if ids[i] == target_id:
                        target_found = True
                        lost_target_counter = 0
                        
                        # get target cent
                        cx = (box[0] + box[2]) // 2
                        cy = (box[1] + box[3]) // 2
                        
                        # calculate error (pixel distance from cent)
                        # pan error: x difference
                        # tilt error: y difference
                        pan_error = frame_center_x - cx
                        tilt_error = frame_center_y - cy
                        
                        # draw target in green
                        cv2.rectangle(annotated_frame, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), 2)
                        cv2.circle(annotated_frame, (cx, cy), 5, (0, 255, 0), -1)
                        
                        # send to esp32
                        # Protocol: "P<pan_val>T<tilt_val>\n"
                        # Example: "P-45T120\n"
                        if ser is not None:
                            command = f"P{pan_error}T{tilt_error}\n"
                            ser.write(command.encode('utf-8'))
                        break
        
        # action when lost target
        if target_id is not None and not target_found:
            lost_target_counter += 1
            if lost_target_counter > LOST_TARGET_THRESHOLD:
                print(f"[TRACKING] Target ID {target_id} Lost. Resetting.")
                target_id = None
                lost_target_counter = 0
                # stop motors
                if ser is not None:
                    ser.write(b"P0T0\n")

        # draw crosshair
        cv2.line(annotated_frame, (frame_center_x, frame_center_y - 15), (frame_center_x, frame_center_y + 15), (255, 255, 0), 2)
        cv2.line(annotated_frame, (frame_center_x - 15, frame_center_y), (frame_center_x + 15, frame_center_y), (255, 255, 0), 2)

        # show output
        cv2.imshow("Gimbal Vision", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\n[INFO] Stopping...")

finally:
    cap.release()
    cv2.destroyAllWindows()
    if ser is not None:
        ser.close()
