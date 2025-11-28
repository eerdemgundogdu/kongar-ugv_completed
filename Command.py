import serial
import time

# Setup serial
ser = serial.Serial('/dev/ttyUSB0', 9600)

def send_motor_commands(motor1_speed, motor2_speed):
    # Send speeds to Teensy
    command = f"{motor1_speed} {motor2_speed}\n"
    ser.write(command.encode())

def main():
    while True:
        # Calc speeds from sensors
        motor1_speed = 50
        motor2_speed = -50

        send_motor_commands(motor1_speed, motor2_speed)
        time.sleep(2)

        # Stop
        send_motor_commands(0, 0)
        time.sleep(2)

if __name__ == "__main__":
    main()
