 #!/usr/bin/env python3
# coding=UTF-8

import RPi.GPIO as GPIO
import serial


GPIO.setmode(GPIO.BCM)
ser = serial.Serial('/dev/ttyACM0', 115200)

input_pin_1 = 23
input_pin_2 = 24

datas=[0xff,0xff,0]


GPIO.setup(input_pin_1, GPIO.IN)
GPIO.setup(input_pin_2, GPIO.IN)


try: 
    while True:
        input_state_1 = GPIO.input(input_pin_1)
        input_state_2 = GPIO.input(input_pin_2)
        
        #发送第一次起飞指令
        if input_state_1==1:
            datas[2]=1
            print("第1次起飞")
            print(datas)
            ser.write(datas)
            
        
        #发送第二次起飞指令
        elif input_state_2==1:
            datas[2]=2
            ser.write(datas)
            print("第2次起飞")
        else:
            datas[2]=9
            ser.write(datas)
            print("no fly")
    
except KeyboardInterrupt:
    print("\n111")
finally:
    GPIO.cleanup()  

