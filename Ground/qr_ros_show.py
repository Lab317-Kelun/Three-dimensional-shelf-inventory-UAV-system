import cv2
import serial
import pyzbar.pyzbar as pyzbar
import numpy as np
import logging
import rospy
from std_msgs.msg import Int8
from fly_pkg.msg import pose
import threading
import os
import subprocess
# log information settings
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s: %(message)s')
code_detect_index=0
ser = serial.Serial("/dev/ttyUSB0", 115200)
ser.flushInput()  
# 初始化 ROS 节点:命名(唯一)
rospy.init_node("code_dection")
pub=rospy.Publisher("second_task_pos",pose,queue_size=10)
target_pos=pose()
array_key=["A3", "A2", "A1", "A4", "A5", "A6",
            "C6", "C5", "C4", "C1", "C2", "C3",
            "B1", "B2", "B3", "B6", "B5", "B4",
            "D4", "D5", "D6", "D3", "D2", "D1"]
array_num=np.array([])
array_pos = [( 0.25 ,0.75,1.35 ,0),(0.25,1.25 ,1.35 ,0), (0.25 ,1.75 ,1.35 ,0 ),(0.25 ,1.75 ,0.95, 0),( 0.25 ,1.25 ,0.95, 0),(0.25, 0.75 ,0.95 ,0),#  A3 A2 A1 A4 A5 A6 
            ( 2.2, 0.75 ,0.95 ,0),(2.2 ,1.25 ,0.95 ,0),(2.2 ,1.75 ,0.95 ,0),(2.2, 1.75 ,1.35 ,0),(2.2 ,1.25 ,1.35 ,0),( 2.2 ,0.75 ,1.35, 0),     #C6 C5 C4 C1 C2 C3           
            ( 1.25 ,0.75,1.35 ,180),(1.25 ,1.25 ,1.35 ,180),(1.25 ,1.75 ,1.35 ,180),(1.25 ,1.75 ,0.95 ,180),(1.25 ,1.25 ,0.95 ,180),(1.25 ,0.75 ,0.95 ,180),    # B1 B2 B3 B6 B5 B4
            ( 3.25, 0.75,0.95 ,180),(3.25 ,1.25 ,0.95 ,180),(3.25 ,1.75 ,0.95 ,180),(3.25 ,1.75 ,1.35 ,180),(3.25 ,1.25 ,1.35 ,180),(3.25 ,0.75 ,1.35 ,180)]   # D4 D5 D6 D3 D2 D1
second_task_start=0
second_code=0
rate = rospy.Rate(10)
num=0
target_num=0
first_code=0
camera_index=0
times=0
timess=0
flag = 0
def decodeDisplay(image):
    global num,target_num,second_task_start,first_code,array_num,camera_index
    barcodes = pyzbar.decode(image)
    i=len(barcodes)
    for barcode in barcodes:
        # 提取二维码的边界框的位置
        # 画出图像中条形码的边界框
        (x, y, w, h) = barcode.rect
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
        # 提取二维码数据为字节对象，所以如果我们想在输出图像上
        # 画出来，就需要先将它转换成字符串
        barcodeData = barcode.data.decode("utf-8")
        barcodeType = barcode.type
        # 绘出图像上条形码的数据和条形码类型
        text = "{} ({})".format(barcodeData, barcodeType)
        # cv2.putText(image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 3, color=(0, 255, 0), 
        # thickness=5)
        # 向终端打印条形码数据和条形码类型
        # print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
        data=int(barcodeData)
        if data in array_num:
            pass
        else:
            array_num=np.append(array_num,data)
        if (second_task_start>0):
            if first_code==0:
                num=0
                first_code=1
            if first_code==1:
                if num>0:
                    target_num=num
                    first_code=2
            if first_code==2:
                index=np.where(array_num==target_num)
                print(index)
                position=array_pos[index[0][0]]
                target_pos.x=position[0]
                target_pos.y=position[1]
                target_pos.z=position[2]
                target_pos.yaw=position[3]
                pub.publish(target_pos)
                if camera_index==0:
                    if target_pos.yaw==0:
                        os.system("echo 0 > /sys/class/gpio/PN.01/value")
                        os.system("echo 0 > /sys/class/gpio/PCC.04/value")
                    elif target_pos.yaw==180:
                        os.system("echo 1 > /sys/class/gpio/PN.01/value")
                        os.system("echo 1 > /sys/class/gpio/PCC.04/value")
                        os.system("echo 0 > /sys/class/gpio/PN.01/value")
                        os.system("echo 0 > /sys/class/gpio/PCC.04/value")
                        os.system()
                    camera_index=1
        num=data
        
        print(data)
    # if i==6:
    #     print("=============================")
    return image

def detect():
    #调用笔记本内置摄像头，所以参数为0，如果有其他的摄像头可以调整参数为1，2
    cap=cv2.VideoCapture(0)
    while not rospy.is_shutdown():
        #从摄像头读取图片
        sucess,img=cap.read()
        #转为灰度图片
        gray1=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        gray=gray1
        if code_detect_index or second_task_start:
            gray = decodeDisplay(gray1)
        #显示摄像头，背景是灰度。
        cv2.imshow("img",gray)
        #保持画面的持续。
        k=cv2.waitKey(1)
        if k == 27:
            #通过esc键退出摄像
            cv2.destroyAllWindows()
            break
    #关闭摄像头
    cap.release()
def data_send():
    global data
    global num,second_task_start,target_num
    while not rospy.is_shutdown():
        if num>0 and second_task_start==0:#任务1
            data=[0xff,0xff,num,0xaa]
            print("data",data)
            ser.write(data)
        if target_num>0 and second_task_start==1:#任务2
            data=[0xbb,0xbb,target_num,0xdd]
            print("second_data",data)
            ser.write(data)
        if target_num>0 and second_task_start==2:#任务2.2
            data=[0xcc,0xcc,1,0xee]
            print("third_data",data)
            ser.write(data)
        rate.sleep()
        
def doMsg(msg):
    global code_detect_index
    code_detect_index=msg.data

def data_recv():
    global second_task_start,times,timess,flag
    datas = [0xbb, 0xbb]
    start_data=[0xff, 0xff]
    step = 0
    start_step=0
    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            data = recv(ser)
            if step == 2:
                datas.append(data)
                if len(datas) == 4:
                    second_task_start = datas[2]
                    step = 0
                    #rospy.loginfo('num: %s', huo_num)
                    if second_task_start<160:
                        if second_task_start==1:
                            print("执行任务2")
                    datas = [0xbb, 0xbb]
            elif step == 1:
                if data == 0xbb:
                    step = 2
            elif step == 0:  
                if data == 0xbb:
                    step = 1
                    
            if start_step ==2:
                start_data.append(data)
                if len(start_data) == 4:
                #第一次飞行
                    if start_data[2]==1:
                        times+=1
                        if times==1:
                        # 发布消息
                            print("one fly")
                            flag = 1
                            subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', 'roslaunch fly_pkg main_CRAIC3.launch'])
                    elif start_data[2] == 2 and flag ==1:
                        timess+=1
                        if timess==1:
                            
                        # 发布消息
                            print("two fly")
                            subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', 'roslaunch fly_pkg main_CRAIC2.launch'])
                        
                    start_step = 0
                    start_data = [0xff, 0xff]
            elif start_step == 1:
                if data == 0xff:
                    start_step = 2
            elif start_step == 0:  
                if data == 0xff:
                    start_step = 1                                    

def recv(serial):
    while True:
        data = serial.read(1)
        data = int.from_bytes(data, byteorder='big')
        if data == '':
            continue
        else:
            break
        sleep(0.02)
    return data

def second_task(msg):
    global second_code,second_task_start
    second_code=msg.data
    if second_code==1:
        second_task_start=2

if __name__ == '__main__':

    t1=threading.Thread(target=detect)
    t1.start()
    t2=threading.Thread(target=data_send)
    t2.start()
    t3=threading.Thread(target=data_recv)
    t3.start()
    print("开始订阅")
    sub = rospy.Subscriber("code",Int8,doMsg,queue_size=10)
    sub2= rospy.Subscriber("code2",Int8,second_task,queue_size=10)
    rospy.spin()

