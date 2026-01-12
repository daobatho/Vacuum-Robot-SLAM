import serial
import time

ser = serial.Serial(
    port='/dev/ttyAMA1',
    baudrate=115200,
    timeout=0.1
)

while True:
 #   ser.write(b'Hello UART\r\n')
  #  print("TX:", "Hello UART")
   # time.sleep(0.5)

    data = ser.readline()
    if data:
        print("RX:", data.decode(errors='ignore'))
