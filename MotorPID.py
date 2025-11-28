import serial
import time
import gps

# PID Class
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

# Setup serial
ser = serial.Serial('/dev/ttyUSB0', 9600)

def send_motor_commands(motor1_speed, motor2_speed):
    # Send to Teensy
    command = f"{motor1_speed} {motor2_speed}\n"
    ser.write(command.encode())

# Init PID
Kp = 1.0
Ki = 0.1
Kd = 0.5
pid_controller = PID(Kp, Ki, Kd)

# Init GPS
gpsd = gps.gps(mode=gps.WATCH_ENABLE)

# Globals
desired_speed = 100
current_speed = 50

def get_current_position():
    # Get GPS pos
    report = gpsd.next()
    if report['class'] == 'TPV':
        if hasattr(report, 'lat') and hasattr(report, 'lon'):
            return report.lat, report.lon
    return None, None

def control_motor_speed():
    global current_speed
    
    # Calc error
    speed_error = desired_speed - current_speed
    
    # PID update
    motor_adjustment = pid_controller.update(speed_error)

    # Adjust speeds
    motor1_speed = desired_speed + motor_adjustment
    motor2_speed = desired_speed + motor_adjustment

    # Bounds
    motor1_speed = max(-100, min(100, motor1_speed))
    motor2_speed = max(-100, min(100, motor2_speed))

    # Send
    send_motor_commands(motor1_speed, motor2_speed)
    
    # Sim speed update
    current_speed = motor1_speed

def main():
    global desired_speed, current_speed

    while True:
        # Get pos
        lat, lon = get_current_position()

        if lat and lon:
            print(f"Current Position: Latitude: {lat}, Longitude: {lon}")
        
        # Control loop
        control_motor_speed()

        print(f"Desired Speed: {desired_speed}, Current Speed: {current_speed}")
        
        time.sleep(0.1)

if __name__ == "__main__":
    main()
