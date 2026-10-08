import serial
from nm3driver.nm3driver import Nm3

def get_address(): 
    with serial.Serial('/dev/ttyUSB0', 9600, 8, serial.PARITY_NONE, serial.STOPBITS_ONE, 0.1) as p:
        nm3 = Nm3(input_stream=p, output_stream=p)
        print('Address:', nm3.get_address())

def set_address(): 
    with serial.Serial('/dev/ttyUSB0', 9600, 8, serial.PARITY_NONE, serial.STOPBITS_ONE, 0.1) as p:
      nm3 = Nm3(input_stream=p, output_stream=p)
      new_addr = nm3.set_address(40)
      print('Address set to:', new_addr)

def main(): 
     get_address()
     set_address()

if __name__=="__main__":
    main()
    