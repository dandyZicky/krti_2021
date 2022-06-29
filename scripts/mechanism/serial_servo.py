# modul untuk integrasi program ino dengan python
import serial
import sys
import time

"""
 request :
    1. LED ON
    2. LED OFF
    3. LED BLINK
    4. DROP 
    5. DROP ALL
    6. LIDAR READ -> distance
    7. 
"""

class Mechanism():

    def __init__(self):
        self.name = "Arduino"
        self.ser = serial.Serial()
        self.ser.port = '/dev/ttyUSB0'
 
        self.ser.baudrate = 9600
        self.ser.timeout = 1

        # open port untuk write
        try:
            self.open_port()
        except serial.SerialException:
            print('Not connected to port')
        time.sleep(0.1)
        # self.ser = serial.Serial( port='/dev/ttyACM0' , baudrate=9600, timeout=1)

    def open_port(self):
        port_comm = self.ser.isOpen()
        if self.ser.port == None:
            print("Port not detected")
        else:
            if not self.ser.isOpen():
                self.ser.open()
                port_comm = self.ser.isOpen()
            print("Mechanism Port detected :", port_comm)

    def read_serial(self):
        serial_data = self.ser.readline().decode('utf-8')
        serial_data = str(serial_data)

        # parsing hilangkan tanda *...*
        serial_data = serial_data.split('*')
        recv_data = serial_data[0]
        return recv_data

    def drop_now(self, cmd):
        msg = self.parsing(cmd)
        print(f'SERVO: {msg}')

        try:
            self.ser.write(msg.encode())
        except:
            print("----eror----\nSerial data not sent")

    def parsing(self, cmd):
        self.start = '<'  # prefix
        self.end = '>'  # sufix
        self.kode = "S"  # kode parsing
        self.msg = ""  # fullmsg

        self.msg = self.start + cmd + self.end  # <S1>
        # print(self.msg)
        # future work , parsing dikodekan secara khusus sesuai aktuator agar responsive
        return self.msg

    def set_serial(self):
        pass

    def show_info(self):
        pass


if __name__ == "__main__":

    ser = Mechanism()
    # ser.drop_now('1')
    # for i in range(0, 1):
    #     ser.drop_now('1')
    #     time.sleep(2)
    #     ser.drop_now('2')
    #     time.sleep(2)
    #     ser.drop_now('3')
    #     time.sleep(2)
    #     i += 1
    # ser.drop_now('4') # open
    # ser.drop_now('5') # close
    try:
        msg = str(sys.argv[1])
    except IndexError:
        msg = '5'
    ser.drop_now(msg)