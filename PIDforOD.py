class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.previous_error = 0
        self.integral = 0

    def update(self, error):
        """
        Update PID.
        """
        # Proportional
        proportional = self.Kp * error
        
        # Integral
        self.integral += error
        integral = self.Ki * self.integral
        
        # Derivative
        derivative = self.Kd * (error - self.previous_error)
        
        # Output
        output = proportional + integral + derivative
        
        self.previous_error = error
        
        return output

import cv2
import numpy as np
import serial

# Setup serial
ser = serial.Serial('/dev/ttyUSB0', 9600)

# Setup camera
cap = cv2.VideoCapture(0)

# PIDs
steering_pid = PID(0.5, 0.0, 0.2)
speed_pid = PID(1.0, 0.1, 0.5)

previous_error = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)

    # ROI
    height, width = edges.shape
    roi = edges[int(height/2):height, 0:width]

    contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Path logic
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)

        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            cv2.circle(frame, (cX, cY), 7, (0, 255, 0), -1)
            cv2.putText(frame, f"Center: ({cX}, {cY})", (cX + 10, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Steering error
            steering_error = cX - width // 2
            steering_adjustment = steering_pid.update(steering_error)

            # Speed logic
            speed_error = 0
            speed_adjustment = speed_pid.update(speed_error)

            # Steering control
            if steering_adjustment < 0:
                print("Turn Left")
                ser.write(b"left\n")
            elif steering_adjustment > 0:
                print("Turn Right")
                ser.write(b"right\n")
            else:
                print("Move Forward")
                ser.write(b"forward\n")

            # Speed control
            if speed_adjustment < 0:
                print("Reduce Speed")
                ser.write(b"slow\n")
            else:
                print("Maintain Speed")
                ser.write(b"normal\n")

    cv2.imshow("Frame", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
