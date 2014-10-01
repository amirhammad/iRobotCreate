import serial

# def __init__():
port = serial.Serial("/dev/rfcomm1", baudrate=57600)
port.open()
port.write("\x80\x83")
port.write("\x91\x01\xff\x01\xff")

port.close()