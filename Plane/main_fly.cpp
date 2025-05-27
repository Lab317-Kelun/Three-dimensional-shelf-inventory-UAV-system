#include <ros/ros.h>
#include <fly_pkg/pos_srv.h>
#include <fly_pkg/cv_pos_srv.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#include <fly_pkg/robot_data.h>
#include <fly_pkg/vel_srv.h>
#include <fly_pkg/pose.h>


#define PI 3.1415926f
#define ON 1 //飞行模式
#define Vel_Land -1 //降落
#define NO 0
#define Ready 2  //进入待机模式
#define EEROR 3  //位置失灵
#define CV_task_down 4 //视觉纠正降落
#define Vel_circle 5    //速度穿越圆环模式
#define Vel_down 6    //速度下降模式
#define CV_task_circle 7 //视觉纠正圆环
#define Land -2 //降落


#define cv 0      //cv视觉
#define yolo 1    //yolo视觉

float x=0,y=0,z=0,yaw=0;
float X=0,Y=0,Z=0,YAW=0;

fly_pkg::pose pose_code;
void pose_cb(const fly_pkg::pose::ConstPtr &msg){
    pose_code = *msg;
    x = pose_code.x;
    y = pose_code.y;
    z = pose_code.z;
    yaw = pose_code.yaw;
    //ROS_INFO("X: %f, y: %f, z: %f, yaw: %f",x,y,z,yaw);
}

std_msgs::Int8 code_flag;
void cb_code(const std_msgs::Int8::ConstPtr &msg){
    ROS_INFO("进入回调成功");
    code_flag = *msg;
}

fly_pkg::robot_data H_pose;
void H_stop_cb(const fly_pkg::robot_data::ConstPtr &msg){
    H_pose = *msg;
}

fly_pkg::robot_data circle_pos;
void circle_cb(const fly_pkg::robot_data::ConstPtr &msg){
    circle_pos = *msg;
}


//位置信息订阅
geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	local_pos = *msg;
}
//状态订阅回调函数
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	current_state = *msg;
}

//飞行控制 pos_x 为x轴位置 yaw为偏航角 control_flag 为飞行控制指令
void fly_control(fly_pkg::pos_srv::Request& aim_pos,float pos_x,float pos_y,float pos_z,float yaw,short control_flag,
                    float angle_accuracy,float pos_accuracy,float cv_angle_accuracy,float cv_pos_accuracy,float cv_times){
    
    aim_pos.control_flag = control_flag;
    aim_pos.x   = pos_x;
    aim_pos.y   = pos_y;
    aim_pos.z   = pos_z;
    aim_pos.yaw = yaw;
    
    aim_pos.angle_accuracy = angle_accuracy;
    aim_pos.pos_accuracy = pos_accuracy;

    aim_pos.cv_angle_accuracy = cv_angle_accuracy;
    aim_pos.cv_pos_accuracy = cv_pos_accuracy;
}

//时间的单位是s 功能为到达目的地，并且保持一定的时间  times 为保持时间 hz为在此期间订阅话题的hz
void fly_control_with_time(fly_pkg::pos_srv& aim_pos,ros::ServiceClient &pos_client,float pos_x,float pos_y,float pos_z,
                            float yaw,short control_flag,float times,float hz,
                            float angle_accuracy,float pos_accuracy,float cv_angle_accuracy,float cv_pos_accuracy,float cv_times){
    ros::Time last_times = ros::Time::now();
    ros::Rate pos_rate(hz);
    fly_control(aim_pos.request, pos_x , pos_y , pos_z , yaw , control_flag,angle_accuracy,pos_accuracy,cv_angle_accuracy,cv_pos_accuracy,cv_times); 
    while(ros::ok()){         
        while(ros::ok()){
            if(pos_client.call(aim_pos))
                break;
        }
        if(control_flag==Vel_Land){  //降落模式
            if(!current_state.armed){ //等待模式切换完毕
                break;
            }
        }
        //处理持续时间
        else{
            if(aim_pos.response.arrive_flag){
                if(ros::Time::now() -last_times> ros::Duration(times)){
                    break;
                }
            else{
                last_times = ros::Time::now();
            }
            }
        }
        ros::spinOnce();
        pos_rate.sleep();
    }
}


//视觉飞行
void fly_cv_control_with_time(fly_pkg::cv_pos_srv& cv_aim_pos,ros::ServiceClient &pos_cv_client,
                            float cv_z,short cv_control_flag,short vision,float cv_times,float cv_hz,
                            float cv_angle_accuracy,float cv_pos_accuracy){

    ros::Time cv_last_times = ros::Time::now();
    ros::Rate cv_pos_rate(cv_hz);
    ros::spinOnce();
    cv_pos_rate.sleep();

    while(ros::ok()){         
        while(ros::ok()){

            cv_aim_pos.request.cv_control_flag = cv_control_flag;
            cv_aim_pos.request.cv_z = cv_z;
            cv_aim_pos.request.cv_angle_accuracy = cv_angle_accuracy;
            cv_aim_pos.request.cv_pos_accuracy = cv_pos_accuracy;
            //vision = 1 即为yolo 否则为cv
            if (vision){
                cv_aim_pos.request.cv_x = H_pose.image_x;
                cv_aim_pos.request.cv_y = H_pose.image_y;
            }
            else{
                cv_aim_pos.request.cv_x = circle_pos.image_x;
                cv_aim_pos.request.cv_y = circle_pos.image_y;
            }

            if(pos_cv_client.call(cv_aim_pos))
                break;
        }
        if(cv_control_flag==Vel_Land){  //降落模式
            if(!current_state.armed){ //等待模式切换完毕
                break;
            }
        }
        //处理持续时间
        else{
            if(cv_aim_pos.response.cv_arrive_flag){
                if(ros::Time::now() -cv_last_times> ros::Duration(cv_times)){
                    break;
                }
            else{
                cv_last_times = ros::Time::now();
            }
            }
        }
        ros::spinOnce();
        cv_pos_rate.sleep();
    }
}


//飞行控制 pos_x 为x轴位置 yaw为偏航角 control_flag 为飞行控制指令
void fly_vel_control(fly_pkg::vel_srv::Request& vel_pos,float pos_x,float pos_y,float pos_z,float yaw,short vel_control_flag,
                    float vel_angle_accuracy,float vel_pos_accuracy,float vel_x, float vel_y,float vel_z){
    
    vel_pos.vel_control_flag = vel_control_flag;
    vel_pos.x   = pos_x;
    vel_pos.y   = pos_y;
    vel_pos.z   = pos_z;

    vel_pos.vel_x = vel_x;
    vel_pos.vel_y = vel_y;
    vel_pos.vel_z = vel_z;

    vel_pos.vel_angle_accuracy = vel_angle_accuracy;
    vel_pos.vel_pos_accuracy = vel_pos_accuracy;

}

//时间的单位是s 功能为到达目的地，并且保持一定的时间  times 为保持时间 hz为在此期间订阅话题的hz
void fly_vel_control_with_time(fly_pkg::vel_srv& vel_pos,ros::ServiceClient &vel_client,float pos_x,float pos_y,float pos_z,float yaw,
                            float vel_x,float vel_y,float vel_z,
                            short vel_control_flag, float vel_times,float vel_hz,
                            float vel_angle_accuracy,float vel_pos_accuracy){
    ros::Time last_times = ros::Time::now();
    ros::Rate pos_rate(vel_hz);
    fly_vel_control(vel_pos.request, pos_x , pos_y , pos_z , yaw , vel_control_flag,vel_angle_accuracy,vel_pos_accuracy,vel_x,vel_y,vel_z); 
    while(ros::ok()){         
        while(ros::ok()){
            if(vel_client.call(vel_pos))
                break;
        }
        if(vel_control_flag==Vel_Land){  //降落模式
            if(!current_state.armed){ //等待模式切换完毕
                break;
            }
        }
        //处理持续时间
        else{
            if(vel_pos.response.vel_arrive_flag){
                if(ros::Time::now() -last_times> ros::Duration(vel_times)){
                    break;
                }
            else{
                last_times = ros::Time::now();
            }
            }
        }
        ros::spinOnce();
        pos_rate.sleep();
    }
}


//飞机进入待机模式 指令
void plane_to_ready(fly_pkg::pos_srv& aim_pos,ros::ServiceClient &pos_client){
    ros::Rate pos_rate(100);
    aim_pos.request.control_flag = Ready;
    while(ros::ok()){
        pos_client.call(aim_pos);
        if(current_state.mode == "OFFBOARD"&&current_state.armed){ //等待模式切换完毕
            break;
        }
        ros::spinOnce();
        pos_rate.sleep();
        X = x;
        Y = y;
        Z = z;
        YAW = yaw;
        ROS_INFO("X: %f Y: %f Z: %f, YAW: %f",X,Y,Z,YAW);
    }
}

void PID_revise(float x_kp,float x_ki,float x_kd,float x_outputMax,float x_outputMin,
                float y_kp,float y_ki,float y_kd,float y_outputMax,float y_outputMin,
                float z_kp,float z_ki,float z_kd,float z_outputMax,float z_outputMin,
                float yaw_kp,float yaw_ki,float yaw_kd,float yaw_outputMax,float yaw_outputMin,
                fly_pkg::pos_srv::Request& aim_pos){
    aim_pos.x_kp = x_kp;
    aim_pos.x_ki = x_ki;
    aim_pos.x_kd = x_kd;        
    aim_pos.x_outputMax = x_outputMax;
    aim_pos.x_outputMin = x_outputMin;

    aim_pos.y_kp = y_kp;
    aim_pos.y_ki = y_ki;
    aim_pos.y_kd = y_kd;        
    aim_pos.y_outputMax = y_outputMax;
    aim_pos.y_outputMin = y_outputMin;

    aim_pos.z_kp = z_kp;
    aim_pos.z_ki = z_ki;
    aim_pos.z_kd = z_kd;        
    aim_pos.z_outputMax = z_outputMax;
    aim_pos.z_outputMin = z_outputMin;

    aim_pos.yaw_kp = yaw_kp;
    aim_pos.yaw_ki = yaw_ki;
    aim_pos.yaw_kd = yaw_kd;        
    aim_pos.yaw_outputMax = yaw_outputMax;
    aim_pos.yaw_outputMin = yaw_outputMin;
}

void wait_time(float times,float hz,fly_pkg::pos_srv& aim_pos,ros::ServiceClient &pos_client){
    ros::Time last_times = ros::Time::now();
    while(ros::ok()){
        if(ros::Time::now() -last_times> ros::Duration(times)){
            break;
        }
        pos_client.call(aim_pos);
        ros::spinOnce();
        ros::Rate(hz).sleep();
    }
}


std_msgs::Int8 codes;

bool light_right_time(float time, ros::Publisher &pub_code_2){
    ros::Time last_times = ros::Time::now();
    while(ros::ok()){
        //二维码识别发送 并激光射击 右边
        if(ros::Time::now() -last_times> ros::Duration(1.0)){
            codes.data = 1;
            pub_code_2.publish(codes);

            system("echo 1 > /sys/class/gpio/PN.01/value");
            system("echo 0 > /sys/class/gpio/PCC.04/value");
            ros::Time last_timess = ros::Time::now();
            while(ros::ok()){
                if(ros::Time::now() -last_timess> ros::Duration(time)){            
                    codes.data = 0;
                    pub_code_2.publish(codes);
                    system("echo 0 > /sys/class/gpio/PN.01/value");
                    system("echo 0 > /sys/class/gpio/PCC.04/value");
                    return true;
                }
            }
        }
        ros::spinOnce();
        ros::Rate(100).sleep();
    }
    return false;
}


bool light_left_time(float time, ros::Publisher &pub_code_2){
    ros::Time last_times = ros::Time::now();
    while(ros::ok()){
        //二维码识别发送 并激光射击 左边
        if(ros::Time::now() -last_times> ros::Duration(1.0)){
            codes.data = 1;
            pub_code_2.publish(codes);

            system("echo 0 > /sys/class/gpio/PN.01/value");
            system("echo 1 > /sys/class/gpio/PCC.04/value");
            ros::Time last_timess = ros::Time::now();
            while(ros::ok()){
                if(ros::Time::now() -last_timess> ros::Duration(time)){            
                    codes.data = 0;
                    pub_code_2.publish(codes);
                    system("echo 0 > /sys/class/gpio/PN.01/value");
                    system("echo 0 > /sys/class/gpio/PCC.04/value");
                    return true;
                }
            }
        }
        ros::spinOnce();
        ros::Rate(100).sleep();
    }
    return false;
}


int main (int argc,char** argv){
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"main_task");
    ros::NodeHandle nh;
    //客户端
    ros::ServiceClient pos_client = nh.serviceClient<fly_pkg::pos_srv>("pos_control");
    ros::ServiceClient pos_cv_client = nh.serviceClient<fly_pkg::cv_pos_srv>("pos_cv_control");
    ros::ServiceClient vel_client = nh.serviceClient<fly_pkg::vel_srv>("vel_control");

    //订阅
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>  //接收无人机位置信息
		("mavros/local_position/pose", 10, local_pos_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>		//接收无人机模式状态
		("mavros/state", 10, state_cb);


    ros::Subscriber re_code = nh.subscribe<std_msgs::Int8>("code_s",10,cb_code);

    ros::Subscriber H_stop = nh.subscribe<fly_pkg::robot_data>("H_localtion",10,H_stop_cb);

    ros::Subscriber circle_sub = nh.subscribe<fly_pkg::robot_data>("cirque_location",10,circle_cb);

    ros::Subscriber sub_pose = nh.subscribe<fly_pkg::pose>("second_task_pos",10,pose_cb);

    ros::Publisher pub_code_2 = nh.advertise<std_msgs::Int8>("code2",10);

    fly_pkg::pos_srv aim_pos;
    fly_pkg::cv_pos_srv aim_cv_pos;
    fly_pkg::vel_srv vel_pos;

    ros::Rate pos_rate(100); //100hz

    std_msgs::Int8 code;

    //无用
    aim_pos.request.roll = 0;
    aim_pos.request.pitch = 0;
    
    float x_kp,x_ki,x_kd,x_speed_max,x_speed_min;
    float y_kp,y_ki,y_kd,y_speed_max,y_speed_min;
    float z_kp,z_ki,z_kd,z_speed_max,z_speed_min;
    float yaw_kp,yaw_ki,yaw_kd,yaw_speed_max,yaw_speed_min;
    float down;
    
    //PID初始参数读取
    nh.getParam("x_kp", x_kp);nh.getParam("x_ki", x_ki);nh.getParam("x_kd", x_kd);nh.getParam("x_speed_max", x_speed_max);nh.getParam("x_speed_min", x_speed_min);
    nh.getParam("y_kp", y_kp);nh.getParam("y_ki", y_ki);nh.getParam("y_kd", y_kd);nh.getParam("y_speed_max", y_speed_max);nh.getParam("y_speed_min", y_speed_min);
    nh.getParam("z_kp", z_kp);nh.getParam("z_ki", z_ki);nh.getParam("z_kd", z_kd);nh.getParam("z_speed_max", z_speed_max);nh.getParam("z_speed_min", z_speed_min);
    nh.getParam("yaw_kp", yaw_kp);nh.getParam("yaw_ki", yaw_ki);nh.getParam("yaw_kd", yaw_kd);nh.getParam("yaw_speed_max", yaw_speed_max);nh.getParam("yaw_speed_min", yaw_speed_min);
    
    ROS_INFO("--------------------------------PID-------------------------------------------");
    ROS_INFO("x_kp = %f   x_ki=%f  x_kd=%f x_speed_max=%f x_speed_min=%f ",x_kp,x_ki,x_kd,x_speed_max,x_speed_min);
    ROS_INFO("y_kp = %f   y_ki=%f  y_kd=%f y_speed_max=%f y_speed_min=%f ",y_kp,y_ki,y_kd,y_speed_max,y_speed_min);
    ROS_INFO("z_kp = %f   z_ki=%f  z_kd=%f z_speed_max=%f z_speed_min=%f ",z_kp,z_ki,z_kd,z_speed_max,z_speed_min);
    ROS_INFO("yaw_kp = %f   yaw_ki=%f  yaw_kd=%f yaw_speed_max=%f yaw_speed_min=%f ",yaw_kp,yaw_ki,yaw_kd,yaw_speed_max,yaw_speed_min);

    ROS_INFO("--------------------------------PID-------------------------------------------");
    ros::service::waitForService("pos_control");  //等待位置服务器开启
    /*-----------进入待机模式-----------------------------------------------------------------------------*/  
    plane_to_ready(aim_pos,pos_client);
    /*-----------PID参数修改 赋予至服务通信中-----------------------------------------------------------------------------*/ 
    PID_revise(x_kp,x_ki,x_kd,x_speed_max,x_speed_min,                  //x_kp x_ki x_kd x_speed_max x_speed_min
               y_kp,y_ki,y_kd,y_speed_max,y_speed_min,                //y_kp y_ki y_kd y_speed_max y_speed_min
               z_kp,z_ki,z_kd,z_speed_max,z_speed_min,                //z_kp z_ki z_kd z_speed_max z_speed_min
               yaw_kp,yaw_ki,yaw_kd,yaw_speed_max,yaw_speed_min,      //yaw_kp yaw_ki yaw_kd yaw_speed_max yaw_speed_min
               aim_pos.request);    
    
    //起飞
    /*-------------------------------------------------------------------------------------------------*/  
    fly_control_with_time(aim_pos,pos_client, 0 , 0 , 1.3 , 0*PI/180 , ON,0.75f,100, //x y z yaw model time hz
                          3*PI/180 , 0.15f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff
    /*-------------------------------------------------------------------------------------------------*/
    fly_control_with_time(aim_pos,pos_client, 0 , 0 , Z , 0*PI/180 , ON,0.25f,100, //x y z yaw model time hz
                          3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff

    //A区域  
    if( X < 0.75 ){
        /*-------------------------------------------------------------------------------------------------*/  
        fly_control_with_time(aim_pos,pos_client, 0 , Y , Z , 0*PI/180 , ON,0.5f,100, //x y z yaw model time hz
                          3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff
        /*-------------------------------------------------------------------------------------------------*/  
        fly_control_with_time(aim_pos,pos_client, X , Y , Z , 0*PI/180 , ON,0.5f,100, //x y z yaw model time hz
                          3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff
        //激光笔照射
        light_right_time(0.5,pub_code_2);
        /*-------------------------------------------------------------------------------------------------*/          
        fly_control_with_time(aim_pos,pos_client, 0 , Y , Z , 0*PI/180 , ON,0.5f,100, //x y z yaw model time hz
                          3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff 
        /*-------------------------------------------------------------------------------------------------*/          
        fly_control_with_time(aim_pos,pos_client, 0 , -0.25 , Z , 0*PI/180 , ON,0.5f,100, //x y z yaw model time hz
                          3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff
        /*-------------------------------------------------------------------------------------------------*/          
        fly_control_with_time(aim_pos,pos_client, 3.5 , -0.25 , Z , 0*PI/180 , ON,0.5f,100, //x y z yaw model time hz
                          3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff
        /*-------------------------------------------------------------------------------------------------*/          
        fly_control_with_time(aim_pos,pos_client, 3.5 , 2.45 , Z , 0*PI/180 , ON,0.5f,100, //x y z yaw model time hz
                          3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff                 
    }
    
    //B-C区域
    if(  X > 0.75 && X < 2.75 ) {
        //ROS_INFO("b");
        /*-------------------------------------------------------------------------------------------------*/  
        fly_control_with_time(aim_pos,pos_client, 0 , -0.25 , Z , 0*PI/180 , ON,0.5f,100, //x y z yaw model time hz
                          3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度
        /*-------------------------------------------------------------------------------------------------*/  
        fly_control_with_time(aim_pos,pos_client, 1.75, -0.25, Z , 0*PI/180 , ON,0.5f,100, //x y z yaw model time hz
                        3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff
        /*-------------------------------------------------------------------------------------------------*/  
        fly_control_with_time(aim_pos,pos_client, 1.75, Y, Z , 0*PI/180 , ON,0.5f,100, //x y z yaw model time hz
                        3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff 
        /*-------------------------------------------------------------------------------------------------*/          
        fly_control_with_time(aim_pos,pos_client, X, Y, Z , 0*PI/180 , ON,0.5f,100, //x y z yaw model time hz
                        3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff
        //激光笔照射
        if( X > 0.75 && X < 1.75){
            light_left_time(0.5,pub_code_2);  
        }
        else if(  X > 1.75 && X < 2.75){
            light_right_time(0.5,pub_code_2);  
        }
        /*-------------------------------------------------------------------------------------------------*/  
        fly_control_with_time(aim_pos,pos_client, 1.75, Y, Z , 0*PI/180 , ON,0.5f,100, //x y z yaw model time hz
                        3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff
        /*-------------------------------------------------------------------------------------------------*/  
        fly_control_with_time(aim_pos,pos_client, 1.75, -0.25, Z , 0*PI/180 , ON,0.5f,100, //x y z yaw model time hz
                        3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff 
        /*-------------------------------------------------------------------------------------------------*/  
        fly_control_with_time(aim_pos,pos_client, 3.5, -0.25, Z , 0*PI/180 , ON,0.5f,100, //x y z yaw model time hz
                        3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff
        /*-------------------------------------------------------------------------------------------------*/  
        fly_control_with_time(aim_pos,pos_client, 3.5, 2.45, Z, 0*PI/180 , ON,1.0f,100, //x y z yaw model time hz
                        3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff            
    }

    //D区域
    else if ( X > 2.75 ){
        //ROS_INFO("d");
        /*-------------------------------------------------------------------------------------------------*/  
        fly_control_with_time(aim_pos,pos_client, 0 , -0.25 , Z , 0*PI/180 , ON,0.5f,100, //x y z yaw model time hz
                          3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度
        /*-------------------------------------------------------------------------------------------------*/          
        fly_control_with_time(aim_pos,pos_client, X, -0.25, Z , 0*PI/180 , ON,0.5f,100, //x y z yaw model time hz
                        3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff
        /*-------------------------------------------------------------------------------------------------*/          
        fly_control_with_time(aim_pos,pos_client, X, Y, Z , 0*PI/180 , ON,0.5f,100, //x y z yaw model time hz
                        3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff
        //激光笔照射
        light_left_time(0.5,pub_code_2);
        /*-------------------------------------------------------------------------------------------------*/          
        fly_control_with_time(aim_pos,pos_client, 3.5, 2.45, Z , 0*PI/180 , ON,1.0f,100, //x y z yaw model time hz
                        3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff
    }


    /*-------------------------------------------------------------------------------------------------*/        
    fly_control_with_time(aim_pos,pos_client, 3.5 , 2.45, 0 , 0*PI/180 , Land,1.0f,100, //x y z yaw model time hz
                        3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff   
    /*-------------------------------------------------------------------------------------------------*/

    ROS_INFO("fly_task is close");
    
    while(ros::ok()){
        ros::spinOnce();
        pos_rate.sleep();
    }
}

