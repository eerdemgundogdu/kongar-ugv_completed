import cv2
import numpy as np
import serial

# Setup serial
ser = serial.Serial('/dev/ttyUSB0', 9600)

# Setup camera
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)

    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Find largest contour
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)

        # Get center
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            
            cv2.circle(frame, (cX, cY), 7, (0, 255, 0), -1)
            cv2.putText(frame, f"Center: ({cX}, {cY})", (cX + 10, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Send commands
            if cX < frame.shape[1] // 3:
                print("Turn Left")
                ser.write(b"left\n")
            elif cX > 2 * frame.shape[1] // 3:
                print("Turn Right")
                ser.write(b"right\n")
            else:
                print("Move Forward")
                ser.write(b"forward\n")
        else:
            print("No line detected")
    
    cv2.imshow('Frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
