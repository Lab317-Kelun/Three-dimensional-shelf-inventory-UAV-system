# coding=UTF-8
import sys
import cv2
# from PySide6.QtWidgets import QApplication, QStackedWidget, QWidget, QVBoxLayout, QPushButton, QLabel, QLineEdit, QGridLayout
# from PySide6.QtCore import Slot, Qt, QTimer
# from PySide6.QtGui import QKeyEvent

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5 import QtCore, QtGui, QtWidgets
import RPi.GPIO as GPIO

import rospy
import serial
import numpy as np
import threading
import os
from time import sleep
font = cv2.FONT_HERSHEY_SIMPLEX  

GPIO.setmode(GPIO.BCM)
ser = serial.Serial('/dev/ttyACM0', 115200)
all_result_index=0
i=0
input_pin_1 = 23
input_pin_2 = 24
GPIO.setup(input_pin_1, GPIO.IN)
GPIO.setup(input_pin_2, GPIO.IN)
array_key=["A3", "A2", "A1", "A4", "A5", "A6",
            "C6", "C5", "C4", "C1", "C2", "C3",
            "B1", "B2", "B3", "B6", "B5", "B4",
            "D4", "D5", "D6", "D3", "D2", "D1"]
dict={"A3":0,"A2":0,"A1":0,"A4":0,"A5":0,"A6":0,
        "C6":0,"C5":0,"C4":0,"C1":0,"C2":0,"C3":0,
            "B1":0,"B2":0,"B3":0,"B6":0,"B5":0,"B4":0,
                "D4":0,"D5":0,"D6":0,"D3":0,"D2":0,"D1":0}   
dict2={'A3':(172,325),'A2':(172,247),'A1':(172,170) ,'A4':(172,170), 'A5':(172,247) ,'A6':(172,325), 
             'C3':(428,325) ,'C2':(428,247) ,'C1':(428,170), 'C4':(428,170) ,'C5 ':(428,247),'C6':(428,32),
             'B4 ':(212,325),'B5' :(212,247),'B6':(212,170),'B3':(212,170), 'B2':(212,247) ,'B1':(212,325),
             'D4':(468,325), 'D5':(468,247), 'D6':(468,170) ,'D3':(468,170) ,'D2':(468,247),'D1':(468,325)}
start = (53,440)
route_start=(96,390)
second_text=0
num_array=np.array([])  
switch_index=0
rospy.init_node('ground_task')
rate = rospy.Rate(10)   
target_num=0
symbol=1
datas=[255,255] 
send_data=[255,255]
class MainWindow(QWidget):
    def __init__(self):
        super(QWidget,self).__init__()
        self.setWindowTitle("无人机货物盘点系统")
        self.setGeometry(100, 100, 800, 600)

        self.stacked_widget = QStackedWidget()
        self.inventory_page = InventoryPage()
        self.target_page = TargetPage()
        self.stacked_widget.addWidget(self.inventory_page)
        self.stacked_widget.addWidget(self.target_page)

        self.layout = QVBoxLayout()       
        self.inventory_button = QPushButton("按1切换到遍历盘点，按2切换到指定盘点")
        self.layout.addWidget(self.inventory_button,alignment=Qt.AlignCenter)
        self.layout.addWidget(self.stacked_widget)
        self.setLayout(self.layout)

        #self.showFullScreen()
        self.show_inventory_page()
    
#@Slot()
    def show_inventory_page(self):
        global switch_index
        self.stacked_widget.setCurrentWidget(self.inventory_page)
        switch_index=0
    #@Slot()
    def show_target_page(self):
        global switch_index
        self.stacked_widget.setCurrentWidget(self.target_page)
        switch_index=1

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_1:
            self.show_inventory_page()
        elif event.key() == Qt.Key_2:
            self.show_target_page()
        elif self.stacked_widget.currentWidget() == self.inventory_page:
            if event.key() == Qt.Key_C:
                self.inventory_page.show_input_box()
                self.inventory_page.query_input.setFocus()
            elif event.key() == Qt.Key_Return:
                self.inventory_page.hide_input_box()
                self.inventory_page.query_input.clear()


# 遍历盘点界面
class InventoryPage(QWidget):
    def __init__(self):
        super(QWidget, self).__init__()

        self.stacked_widget = QStackedWidget()
        self.target_page = TargetPage()

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.title_button = QPushButton("遍历货物盘点界面")
        self.layout.addWidget(self.title_button)

        self.grid_layout = QGridLayout()
        self.labels = {}

        # 创建24个区域的标签
        sequence = ["A3", "A2", "A1", "A4", "A5", "A6",
                    "C6", "C5", "C4", "C1", "C2", "C3",
                    "B1", "B2", "B3", "B6", "B5", "B4",
                    "D4", "D5", "D6", "D3", "D2", "D1"]
        for i in range(24):
            row = i // 6
            col = i % 6
            qstr=sequence[i]+":未知编号"
            #label = QLabel(None,"%s:未知编号",sequence[i])
            label = QLabel(qstr)
            self.labels[i] = label
            self.grid_layout.addWidget(label, row, col)

        self.layout.addLayout(self.grid_layout)

        self.query_layout = QVBoxLayout()
        self.query_input = QLineEdit(self)
        self.query_input.setPlaceholderText('输入货物编号后按下回车查询')
        self.query_input.hide()  # 初始隐藏查询框
        self.query_layout.addWidget(self.query_input)

        self.query_button = QPushButton("按下C键进入货物位置查询")
        self.query_layout.addWidget(self.query_button)

        self.result_label = QLabel("查询结果：")
        self.query_layout.addWidget(self.result_label)
        self.layout.addLayout(self.query_layout)

        # 1秒更新一次
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_labels)
        self.timer.start(1000)

    def show_input_box(self):
        self.query_input.show()
        self.query_input.setFocus()
        self.query_input.returnPressed.connect(self.query_location)  # 输入数字后回车查询

    def hide_input_box(self):
        self.query_input.hide()

    #@Slot()
    def query_location(self):
        global array_key,dict
        print("hello")
        item_number = int(self.query_input.text())
        # print(number[item_number-1])
        location = get_key_by_value(dict,item_number,None)
        print(dict)
        # location = array_key[number[item_number-1]]
        strtext="查询结果 编号："+str(item_number)+"货物位置："+str(location)
        # 待写
        self.result_label.setText(strtext)
        #self.query_input.clear()
        self.hide_input_box()

    # #@Slot()
    # def show_target_page(self):
    #     self.stacked_widget.setCurrentWidget(self.target_page)

    # 定时唤起的或货物位置信息刷新
    def update_labels(self):
        for i in range(24):
            if number[i] > 0:
                new_text =array_key[i]+":"+str(number[i])
                self.labels[i].setText(new_text)

    # # 查询进入与退出
    # def keyPressEvent(self, event):
    #     if event.key() == Qt.Key_Return or event.key() == Qt.Key_Enter:
    #         self.query_input.show()
    #         self.query_input.setFocus()
    #     if event.key() == Qt.Key_2:
    #         self.show_target_page()
    #     elif event.key() == Qt.Key_Escape:
    #         self.query_input.hide()
    #         self.query_input.clear()


class TargetPage(QWidget):
    def __init__(self):
        super(QWidget,self).__init__()

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.title_button = QPushButton("指定货物盘点界面")
        # self.target_button.clicked.connect(self.show_target_page)
        self.layout.addWidget(self.title_button)

        #self.qr_label = QLabel("等待二维码识别结果...")
        #self.layout.addWidget(self.qr_label)

        self.path_label = QLabel("规划的定点盘点航线图：")
        self.layout.addWidget(self.path_label)

        self.img_label = QLabel()
        #img = create_image()
        #pixmap = convert_to_qt_format(img)
        #self.img_label.setPixmap(pixmap)
        #self.layout.addWidget(self.img_label)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_image)
        self.timer.start(1000)

    # 假设此函数由外部调用来更新二维码识别结果
    def update_qr_result(self, item_number):
        global target_num
        self.qr_label.setText("识别到的货物编号：%d",target_num)

    # 假设此函数由外部调用来更新目标货物位置
    def update_target_location(self, item_number, location):
        self.result_label.setText("货物 %d 位置 %d",item_number,location)
    
    def update_image(self):
        img = create_image()
        pixmap = convert_to_qt_format(img)
        self.img_label.setPixmap(pixmap)
        self.layout.addWidget(self.img_label)


# 此处画图
def create_image():
    global target_num,second_text
    img = np.ones((480, 640, 3), dtype=np.uint8)*255
    fly_map=draw_map(img)
    if target_num>0:
       image=show_route(fly_map,target_num)
    else:
        image=fly_map  
    if second_text:
        route=show_route(fly_map,target_num)
        image= show_text(route,target_num)
    #image=fly_map    
    return image

def draw_map(img):   
    starts1 = ( (192,128),(448,128))
    stops1 = ( (192,360),(448,360))
    # count_x = 640/500=1.28
    # count_y = 480/400=1.2
    for i in range(0,2,1):
        start = starts1[i] 
        stop = stops1[i]
        color=(0,0,0)
        thick=2
        img = cv2.line(img,start,stop,color,thick)
    starts_r =  (64,360)
    stops_r =  (128,420)
    cv2.rectangle(img,starts_r,stops_r,(0,0,0),-1)   
    circle =  (544,90)
    r = 30
    cv2.circle(img,circle,r,(0,0,0),-1)
    text_A='A'
    cv2.putText(img,text_A,(150,244),font,1,(0,0,0),2)
    text_B='B'
    cv2.putText(img,text_B,(230,244),font,1,(0,0,0),2)
    text_C='C'
    cv2.putText(img,text_C,(406,244),font,1,(0,0,0),2)
    text_D='D'
    cv2.putText(img,text_D,(486,244),font,1,(0,0,0),2)

    return img

def show_route(img,target_num):
    route_start=(96,390)
    target_A = get_key_by_value(dict,target_num,None)
    
    target_location=dict2[target_A]
    color=(255,0,0)
    thick=2
    start=route_start
    stop=(target_location[0],route_start[1])
    img = cv2.line(img,start,stop,color,thick)
    start=(target_location[0],route_start[1])
    stop=target_location
    img = cv2.line(img,start,stop,color,thick)
    text_A ="num: "+str(target_num)
    cv2.putText(img,text_A,(150,450),font,1,(0,0,255),2)
    return img

def show_text(img,target_num):
    target_A = get_key_by_value(dict,target_num,None)
    text_A ="num: "+str(target_num)+"  location: "+str(target_A)
    cv2.putText(img,text_A,(150,450),font,1,(0,0,255),2)
    ######################################################################
    os.system("echo 1 > /sys/class/gpio/gpio5/value")
    os.system("echo 0 > /sys/class/gpio/gpio5/value")
    return img


def convert_to_qt_format(img):
    # 将OpenCV图像转为Qt格式
    height, width, channel = img.shape
    bytes_per_line = 3 * width
    q_img = QImage(img.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()

    return QPixmap.fromImage(q_img)

def get_key_by_value(dictionary, value, default=None):
    for key, val in dictionary.items():
        if val == value:
            return key
    return default

def data_recv():
    step = 0
    second_step=0
    global huo_num,target_num,datas,second_text
    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            if switch_index==0:
                data = recv(ser)
                if step == 2:
                    datas.append(data)
                    if len(datas) == 4:
                        huo_num = datas[2]
                        step = 0
                        if huo_num<160:
                            save_numarr(huo_num)
                        datas = [255, 255]
                elif step == 1:
                    if data == 255:
                        step = 2
                elif step == 0:
                    if data == 255:
                        step = 1
            elif switch_index==1:
                data = recv(ser)
                if step == 2:
                    datas.append(data)
                    if len(datas) == 4:
                        target_num = datas[2]
                        step = 0
                        if target_num<160:
                            save_numarr(target_num)
                        datas = [187, 187]
                elif step == 1:
                    if data == 187:
                        step = 2
                elif step == 0:
                    if data == 187:
                        step = 1    
                #转动相机          
                if second_step==2:
                    datas.append(data)
                    if len(datas) == 4:
                        second_text = datas[2]
                        second_step = 0
                        datas = [204, 204]
                elif second_step == 1:
                    if data == 204:
                        second_step = 2
                elif second_step == 0:
                    if data == 204:
                        second_step = 1      

                          

#####################################


def recv(serial):
    while True:
        data = serial.read(1)
        data=ord(data)
        if data == '':
            continue
        else:
            break

    return data
def save_numarr(num):
    global num_array,all_result_index,number,i
    if num>0:
        #print(num)
        if num in num_array:
            pass
            #print("chongfu")
        else:
            os.system("echo 1 > /sys/class/gpio/gpio5/value")
            num_array=np.append(num_array,num)
            xuhao=len(num_array)-1
            key = array_key[xuhao]
            dict[key]=num
            number[i]=num
            i+=1
            text = "object pos: (货物编号为：{}, 位置坐标信息为{})".format(num, key)
            print(text)
            print("array:")
            print(num_array)
            os.system("echo 0 > /sys/class/gpio/gpio5/value")


    if len(num_array)==24 and all_result_index==0:
        print("all result:")
        all_result_index=1
        for i in range(24):
            text = "object pos: (货物编号为：{}, 位置坐标信息为{})".format(num_array[i], array_key[i])
            print(text)



def data_send():
    global data,send_data
    global switch_index
    global symbol
    while not rospy.is_shutdown():
        print(symbol)
        if symbol==1:
            if switch_index:        
                data=[0xbb,0xbb,switch_index,0xdd]
                print("second_data",data)
                ser.write(data)
            symbol=-symbol
        elif symbol==-1:
            input_state_1 = GPIO.input(input_pin_1)
            input_state_2 = GPIO.input(input_pin_2)            
            if input_state_1==1:
                send_data=[0xff,0xff,1,0]
                print("one fly")
                ser.write(send_data)
            elif input_state_2==1:
                send_data=[0xff,0xff,2,0]
                print("two fly")
                ser.write(send_data)
            else:
                send_data=[0xff,0xff,9,0]
                print("no_fly")
                ser.write(send_data)
            symbol=-symbol
        rate.sleep()
    GPIO.cleanup()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    # 按顺序写入扫描获得的货物编号即可刷新显示
    number = [0]*24
    #for i in range(24):
     #  number[i] = i+1

    t1=threading.Thread(target=data_recv)
    t1.start()
    t2=threading.Thread(target=data_send)
    t2.start()
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())

