import cv2
import numpy as np

def initialize_camera():
    camera = cv2.VideoCapture(0)
    if not camera.isOpened():
        print("Camera initialization failed!")
        return None
    
    # Set res
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    return camera

def process_image(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    return edges

def find_path_center(edge_image, width, height):
    roi = edge_image[int(height/2):height, 0:width]
    contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None
    
    largest_contour = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest_contour)
    if M["m00"] == 0:
        return None
    
    center_x = int(M["m10"] / M["m00"])
    center_y = int(M["m01"] / M["m00"]) + int(height/2)
    
    return (center_x, center_y)

def detect_obstacles(frame):
    """Detect obstacles"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Green range
    lower_green = np.array([35, 100, 100])
    upper_green = np.array([85, 255, 255])
    
    mask = cv2.inRange(hsv, lower_green, upper_green)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    obstacles = []
    for contour in contours:
        if cv2.contourArea(contour) > 500:  
            x, y, w, h = cv2.boundingRect(contour)
            obstacles.append((x, y, w, h))
    
    return obstacles, mask

def detect_slope(edge_image):
    """Detect slopes"""
    lines = cv2.HoughLines(edge_image, 1, np.pi / 180, 100)
    slope_detected = False
    
    if lines is not None:
        for line in lines:
            rho, theta = line[0]
            angle = np.degrees(theta)
            if angle < 45 or angle > 135:
                slope_detected = True
                break
    
    return slope_detected

def calculate_steering(center_x, frame_width, prev_error, kp=0.5, kd=0.2):
    """Calc steering PID"""
    error = center_x - frame_width // 2
    steering = kp * error + kd * (error - prev_error)
    return steering, error

def send_motor_command(steering, obstacles_detected, slope_detected):
    """Send motor cmds"""
    if obstacles_detected:
        print("OBSTACLE DETECTED - STOPPING")
        return "STOP"
    
    if slope_detected:
        print("SLOPE DETECTED - ADJUSTING SPEED")
        return "SLOW"
    
    if steering < -10:
        print("TURNING LEFT")
        return "LEFT"
    elif steering > 10:
        print("TURNING RIGHT")
        return "RIGHT"
    else:
        print("MOVING FORWARD")
        return "FORWARD"

def main():
    """Main loop"""
    cap = initialize_camera()
    if cap is None:
        return
    
    previous_error = 0
    Kp = 0.5
    Kd = 0.2
    
    print("Starting unmanned tank control system...")
    print("Press 'q' to quit")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break
        
        height, width = frame.shape[:2]   
        edges = process_image(frame)
        center = find_path_center(edges, width, height)
        obstacles, obstacle_mask = detect_obstacles(frame)
        
        # Detect slopes
        slope_detected = detect_slope(edges)
        
        if center:
            center_x, center_y = center
            cv2.circle(frame, (center_x, center_y), 7, (0, 255, 0), -1)
            cv2.putText(frame, f"Path Center", (center_x + 10, center_y - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            steering, error = calculate_steering(center_x, width, previous_error, Kp, Kd)
            previous_error = error
            
            
            cv2.putText(frame, f"Steering: {steering:.2f}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Draw obstacles
        obstacles_detected = len(obstacles) > 0
        for (x, y, w, h) in obstacles:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(frame, "OBSTACLE", (x, y - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # Display status
        status_y = 60
        cv2.putText(frame, f"Obstacles: {len(obstacles)}", (10, status_y), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        cv2.putText(frame, f"Slope: {'YES' if slope_detected else 'NO'}", (10, status_y + 30), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        command = send_motor_command(steering if center else 0, 
            obstacles_detected, slope_detected)
        
        cv2.putText(frame, f"Command: {command}", (10, status_y + 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        cv2.line(frame, (width // 2, 0), (width // 2, height), (255, 0, 0), 1)
        
        cv2.imshow('Tank Camera - Path Following', frame)
        cv2.imshow('Edge Detection', edges)
        cv2.imshow('Obstacle Mask', obstacle_mask)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    print("Tank control system stopped.")

if __name__ == "__main__":
    main()