import serial
ser = serial.Serial('/dev/cu.usbmodem141301', 115200, timeout=1)
ser.write(b'\x09123456789\x26\x39\xf4\xcb')
while True:
  s = ser.read(1000)
  print(s)
