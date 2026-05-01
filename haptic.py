import serial
import socket
import time

SERIAL_PORT = '/dev/cu.usbmodem103'
BAUD_RATE = 115200
UNITY_IP = '127.0.0.1'
UNITY_PORT = 5005
FEEDBACK_PORT = 5007
FLAT_VALUE = 250
BENT_VALUE = 100

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0)
time.sleep(2)
ser.flushInput()

send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.bind(('0.0.0.0', FEEDBACK_PORT))
recv_sock.setblocking(False)

print('Ready — hit Play in Unity and bend the sensor')

buffer = ''
wall_contact_flex = 0.0
is_resisting = False
last_servo_pwm = -1

while True:
    try:
        raw = ser.read(64).decode('utf-8', errors='ignore')
        buffer += raw
        while '\n' in buffer:
            line, buffer = buffer.split('\n', 1)
            line = line.strip()
            if line:
                val = float(line)
                flex = (FLAT_VALUE - val) / (FLAT_VALUE - BENT_VALUE)
                flex = max(0.0, min(1.0, flex))
                send_sock.sendto(str(flex).encode(), (UNITY_IP, UNITY_PORT))

                if is_resisting:
                    overshoot = max(0.0, flex - wall_contact_flex)
                    servo_pwm = int(1500 - (overshoot * 1000))
                    servo_pwm = max(1000, min(1500, servo_pwm))
                    if abs(servo_pwm - last_servo_pwm) > 5:
                        ser.write(f'SERVO:{servo_pwm}\n'.encode())
                        last_servo_pwm = servo_pwm
    except:
        pass

    try:
        data, _ = recv_sock.recvfrom(64)
        command = data.decode().strip()
        if command.startswith('RESIST:'):
            wall_contact_flex = float(command.split(':')[1])
            is_resisting = True
            ser.write(b'RESIST\n')
            last_servo_pwm = -1
            print(f'RESIST at flex {wall_contact_flex:.3f}')
        elif command == 'FREE':
            is_resisting = False
            last_servo_pwm = -1
            ser.write(b'FREE\n')
            print('FREE')
    except:
        pass
