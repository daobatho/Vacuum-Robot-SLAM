import serial
import time

# --- Cấu hình UART5
ser = serial.Serial("/dev/ttyAMA2", baudrate=115200, timeout=0.1)

def read_tfluna():
    while True:
        if ser.in_waiting >= 9:
            data = ser.read(9)
            ser.reset_input_buffer()
            if data[0] == 0x59 and data[1] == 0x59:
                distance = data[2] + data[3]*256
                strength = data[4] + data[5]*256
                temp = (data[6] + data[7]*256)/8 - 256
                return distance/100.0, strength, temp

try:
    while True:
        d, s, t = read_tfluna()
        print(f"Distance: {d:.2f} m, Strength: {s}, Temp: {t:.2f}°C")
        time.sleep(0.05)

except KeyboardInterrupt:
    ser.close()
    print("Stopped")
