import cv2
import numpy as np

# Setup camera
cap = cv2.VideoCapture(0)

# PID constants
Kp = 0.5
Kd = 0.2
previous_error = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)

    # ROI - bottom half
    height, width = edges.shape
    roi = edges[int(height/2):height, 0:width]

    contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Find path center
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            
            cv2.circle(frame, (cX, cY), 7, (0, 255, 0), -1)
            cv2.putText(frame, f"Center: ({cX}, {cY})", (cX + 10, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Steering error
            error = cX - width // 2

            # PID calc
            steering_adjustment = Kp * error + Kd * (error - previous_error)
            previous_error = error
            
            # Motor control logic
            if steering_adjustment < 0:
                print("Turn Left")
            elif steering_adjustment > 0:
                print("Turn Right")
            else:
                print("Move Forward")

    cv2.imshow('Path Detection', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Obstacle detection (HSV)
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# Green mask
lower_bound = np.array([35, 100, 100])
upper_bound = np.array([85, 255, 255])
mask = cv2.inRange(hsv, lower_bound, upper_bound)

contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
for contour in contours:
    if cv2.contourArea(contour) > 500:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        print("Obstacle Detected!")

cv2.imshow("Obstacle Detection", mask)
cv2.imshow("Frame with Obstacles", frame)

edges = cv2.Canny(blurred, 50, 150)
lines = cv2.HoughLines(edges, 1, np.pi / 180, 100)

if lines is not None:
    for line in lines:
        rho, theta = line[0]
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        angle = np.degrees(theta)
        if angle < 45 or angle > 135:
            print("Slope detected!")

cap.release()
cv2.destroyAllWindows()
