#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/Imu.h"
#include "joy_to_threster_pkgs/Therster.h"
#include <iostream>

bool flag = true;
bool control_mode = true;
int abs_num;
int data[10] = {1500,1500,1500,1500,1500,1500,1500,1500,0,0};
int num = 0;
int num_1 = 0;
double imu_data[9] = {0,0,0,0,0,0,0,0,0}; //current_R,P,Y //anguler_R,P,Y // linear X,Y,Z
double joy_data[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 
// double Kp[8] = {0,};
// double Kd[8] = {0,};
double error_curent[8] = {0,0,0,0,0,0,0,0};
double error_velocity[8] = {0,0,0,0,0,0,0,0};
int up_data[8] ={0,};
int roll_data[8] = {0,};
int pitch_data[8] = {0,};
double up_down_data[8]= {0,};
double servo_cnt[2] = {90,90};
int mode_change_button = 0;

struct Z {
    double Kp[8] = {0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1};
    double Kd[8] = {0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1};
    double cur[2] = {0,0};
    double ref[2] = {0,0};
    double error[2] = {0,0};
    double data[10] = {0,0,0,0,0,0,0,0,0,0};
    int rotate[8] = {1,1,1,1,1,1,1,1};
}Z;
struct R {
    double Kp[8] = {0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1};
    double Kd[8] = {0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1};
    double cur[2] = {0,0};
    double ref[2] = {0,0};
    double error[2] = {0,0};
    double data[10] = {0,0,0,0,0,0,0,0,0,0};
    int rotate[8] = {1,1,-1,-1,1,1,1,1};
    /*
    ////[2]//////[0]////
    ////////////////////
    ////////////////////
    ////[3]//////[1]////
    */
}R;
struct P {
    double Kp[8] = {0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1};
    double Kd[8] = {0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1};
    double cur[2] = {0,0};
    double ref[2] = {0,0};
    double error[2] = {0,0};
    double data[10] = {0,0,0,0,0,0,0,0,0,0};
    int rotate[8] = {-1,1,-1,1,1,1,1,1};
    /*
    ////[2]//////[0]////
    ////////////////////
    ////////////////////
    ////[3]//////[1]////
    */
}P;




/*
0 -> left nobe up&down
1 -> left nobe R&L

2 -> left + button up & down

4 -> left + button left & right


////////[2 + ]////////
///[4 +]//////[4 -]///
////////[2 - ]////////

6 -> right nobe up&down
7 -> right nobe R&L

8 -> X button
9 -> O button
10 -> ㅅ button (rectangle)
11 -> ㅁ button (triangle)

12 -> left triger button // cam_mode_button -> 누른 상태에서 카메라 제어
13 -> rght triger button // faster_mode_button -> 누른상태에서는 증분 정도가 달라짐
*/

int ROV_Control_Mode = 0; // 0 : safety , 1 : Joy_Control , 2 : PD_Control_base_Joy_control, 3: Cam_only_control
//4: all_control_mode
/*
0 : safety : kill all procese
1 : Joy_Control : Just Forward&Backward and UP&Down -> need to test threstor 
2 : PD_Control_base_Joy_control : to change ref Ang to use JoyStick
3 : Cam_only_control : Cotntrol Cam_Servo 
4 : all_control_mode : have PD_Control_bae_Joy_control_mode & Cam_Control 
*/

void IMUCallback(const sensor_msgs::Imu::ConstPtr& imu_msg){
    imu_data[0] = imu_msg->orientation.x;
    imu_data[1] = imu_msg->orientation.y;
    imu_data[2] = imu_msg->orientation.z;
    imu_data[3] = imu_msg->angular_velocity.x;
    imu_data[4] = imu_msg->angular_velocity.y;
    imu_data[5] = imu_msg->angular_velocity.z;
    imu_data[6] = imu_msg->linear_acceleration.x;
    imu_data[7] = imu_msg->linear_acceleration.y;
    imu_data[8] = imu_msg->linear_acceleration.z;

    R.cur[0] = imu_msg->orientation.x;
    R.cur[1] = imu_msg->angular_velocity.x;

    P.cur[0] = imu_msg->orientation.y;
    P.cur[1] = imu_msg->angular_velocity.y;

    Z.cur[1] = imu_msg->linear_acceleration.z;
    // printf("Subscribe_IMU_data");
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
    //X-> 0  O-> 1 ㅅ->2 ㅁ->3 // button msg
    // O : 모드 변경
    // X : 긴급 정지
    // ㅅ : cam 원점복귀
    // ㅁ :  
    // Joy 메시지에서 필요한 데이터 추출 (예시로 axes와 buttons의 첫 번째 값을 사용)
    // float axis_value = joy_msg->axes[7];
    // float axis_value_1 = joy_msg->axes[6];
    // int button_value = joy_msg->buttons[0];
    
    // float axis_forward = joy_msg->axes[1];
    // float axis_side = joy_msg->axes[0];
    // int boost_button_value = joy_msg->buttons[4];
    
    joy_data[0] = joy_msg->axes[1];
    joy_data[1] = joy_msg->axes[0];
    joy_data[2] = joy_msg->axes[7];
    joy_data[3] = 0;
    joy_data[4] = joy_msg->axes[6];
    joy_data[5] = 0;
    joy_data[6] = joy_msg->axes[4];
    joy_data[7] = joy_msg->axes[3];
    joy_data[8] = joy_msg->buttons[0];
    joy_data[9] = joy_msg->buttons[1];
    joy_data[10] = joy_msg->buttons[2];
    joy_data[11] = joy_msg->buttons[3];
    joy_data[12] = joy_msg->buttons[4];
    joy_data[13] = joy_msg->buttons[5];

    mode_change_button = joy_data[9];

    if(mode_change_button == 1){
        if(ROV_Control_Mode == 0){
            ROV_Control_Mode = 1;
            printf("change_MODE : Joy_control_mode");
        }
        else if(ROV_Control_Mode == 1){
            ROV_Control_Mode = 2;
            printf("change_MODE : PD_control_mode");

        }
        else if(ROV_Control_Mode == 2){

            ROV_Control_Mode = 3;
            printf("change_MODE : PD_base_joy_control");
        }
        else if(ROV_Control_Mode == 3){

            ROV_Control_Mode = 4;
            printf("change_MODE : ALL_in_one_mode");
        }
        else{
            ROV_Control_Mode = 0;
            printf("change_MODE : Safety_Mode");
        }
    }


    // printf("joy : %f", axis_value);
   
}



void main_control(int ms){

    if(ROV_Control_Mode == 0){ // 0 : safety
        printf("Safty_Mode \n");
        for(int i=0;i<8;i++){
            data[i] = 1500;
        }
        for(int i=0;i<2;i++){
            data[i+7] = 90;
        }
    }
    else if(ROV_Control_Mode == 1){ // 1 : Joy_Control
        printf("Joy_Control_Mode \n");    
        if(joy_data[0] > 0){
        for(int i=0;i<8;i++){
                data[i] = 1500;
            }
        data[0] = 1600;
        printf("1_threster\n");

        }
        else if(joy_data[0] < 0){
        for(int i=0;i<8;i++){
                data[i] = 1500;
            }
        data[1] = 1600;
        printf("2_threster\n");
        }
        else if(joy_data[1] > 0){
        for(int i=0;i<8;i++){
                data[i] = 1500;
            }
        data[2] = 1600;
        printf("3_threster\n");
        }
        else if(joy_data[1] < 0){
        for(int i=0;i<8;i++){
                data[i] = 1500;
            }
        data[3] = 1600;
        printf("4_threster\n");
            
        }
        else if(joy_data[6] > 0){
        for(int i=0;i<8;i++){
                data[i] = 1500;
            }
        data[4] = 1600;
        printf("5_threster\n");
            
        }
        else if(joy_data[6] < 0){
        for(int i=0;i<8;i++){
                data[i] = 1500;
            }
        data[5] = 1600;
        printf("6_threster\n");
            
        }
        else if(joy_data[7] > 0){
        for(int i=0;i<8;i++){
                data[i] = 1500;
            }
        data[6] = 1600;
        printf("7_threster\n");
            
        }
        else if(joy_data[7] < 0){
        for(int i=0;i<8;i++){
                data[i] = 1500;
            }

        data[7] = 1600;
        printf("8_threster\n");
            
        }
        else{
            for(int i=0;i<8;i++){
                data[i] = 1500;
            }
        }

    }
    else if(ROV_Control_Mode == 2){ // 2 : PD_Control_base_Joy_control

        printf("PD_Control_Mode\n");
        for(int i = 0; i <4; i++){
            Z.error[i] = Z.ref[i]-Z.cur[i];
            P.error[i] = P.ref[i]-P.cur[i];
            R.error[i] = R.ref[i]-R.cur[i];
        }
        for(int i = 0; i <4; i++){
            // Z.data[i] = Z.Kp[i] * Z.error[0] + Z.Kd[i] * Z.error[1];
            Z.data[i] = 1680;
            P.data[i] = P.Kp[i] * P.error[0] + P.Kd[i] * P.error[1];
            R.data[i] = R.Kp[i] * R.error[0] + R.Kd[i] * R.error[1];
            data[i] = Z.data[i] + R.rotate[i] * R.data[i] + P.rotate[i] * P.data[i] + up_down_data[i] * 400;
            if(data[i] > 1800){
                data[i] = 1800;
            }
            else if(data[i] <1200){
                data[i] = 1200;
            }
        }
        //0,1  L // R  ,6,7
        // data[4] = joy_data[0] * 200 + joy_data[1] * 100 + joy_data[7] * 50 + 1500;
        // data[5] = joy_data[0] * 200 - joy_data[1] * 100 + joy_data[7] * 50 + 1500;
        // data[6] = joy_data[0] * 200 - joy_data[1] * 100 - joy_data[7] * 50 + 1500;
        // data[7] = joy_data[0] * 200 + joy_data[1] * 100 - joy_data[7] * 50 + 1500; 
        //2 4 12 13
        if(joy_data[2] > 0){
            data[4] = 1500 + 250;
            data[5] = 1500 + 250;
            data[6] = 1500 + 250;
            data[7] = 1500 + 250;

        }
        else if(joy_data[2] < 0){
            data[4] = 1500 - 250;
            data[5] = 1500 - 250;
            data[6] = 1500 - 250;
            data[7] = 1500 - 250;

        }
        else if(joy_data[4] > 0){
            data[4] = 1500 + 250;
            data[5] = 1500 - 250;
            data[6] = 1500 - 250;
            data[7] = 1500 + 250;

        }else if(joy_data[4] < 0){
            data[4] = 1500 - 250;
            data[5] = 1500 + 250;
            data[6] = 1500 + 250;
            data[7] = 1500 - 250;

        }else if(joy_data[12] > 0){
            data[4] = 1500 + 50;
            data[5] = 1500 + 50;
            data[6] = 1500 - 50;
            data[7] = 1500 - 50;

        }else if(joy_data[13] > 0){
            data[4] = 1500 - 50;
            data[5] = 1500 - 50;
            data[6] = 1500 + 50;
            data[7] = 1500 + 50;

        }
        else{
            data[4] = 1500;
            data[5] = 1500;
            data[6] = 1500;
            data[7] = 1500;
        }
        

        
    }
    else if(ROV_Control_Mode == 3){ // 3 : Cam_only_control
        printf("Only_Cam_control\n");
        if(joy_data[0] > 0.05 && servo_cnt[0] < 70 ){
            servo_cnt[0] = servo_cnt[0] + 5;

            if(servo_cnt[0] == 80){
                printf("servo_Pitch_MAX");
            }
        }
        else if(joy_data[0] < -0.05 && servo_cnt[0] > -70){
            servo_cnt[0] = servo_cnt[0] - 5;

            if(servo_cnt[0] == -80){
                printf("servo_Pitch_MIN");
            }
        }


        if(joy_data[1] > 0.05 && servo_cnt[1] < 80 ){
            servo_cnt[1] = servo_cnt[1] + 5;
            if(servo_cnt[0] == 80){
                printf("servo_Roll_MAX");
            }
        }
        else if(joy_data[1] < -0.05 && servo_cnt[1] > -70){
            servo_cnt[1] = servo_cnt[1] - 5;
            if(servo_cnt[0] == -80){
                printf("servo_Roll_MIN");
            }

        }

        data[8] = servo_cnt[0] + 90;
        data[9] = servo_cnt[1] + 90;

        usleep(100000);
        

    }
    else if(ROV_Control_Mode == 4){ // 4 : all_control_mode
        printf("all_control_mode\n");
    }
    else{
        printf("Error_over_load_Control_Mode\n press [O] button\n");
    }
    
    //send_Data
    for(int i = 0; i < 8 ; i++){
        if(data[i] >= 1525 || data[i]<= 1475){
            data[i] = data[i];
        }
        else if(data[i] < 1525 || data[i]>= 1500){
            data[i] = 1525;
        }
        else{
            data[i] = 1475;
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "joy_to_therster_node");
    ros::NodeHandle nh;

    // Joy 메시지 구독자 선언
    joy_to_threster_pkgs::Therster therster_msg;
    
    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1000, &joyCallback);

    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/IMU", 1000, &IMUCallback);

    ros::Publisher therster_pub = nh.advertise<joy_to_threster_pkgs::Therster>("/therster_topic", 1000);
    ros::Rate loop_rate(100);
    int i = 0;
    while (ros::ok())
   {
     /**
      * This is a message object. You stuff it with data, and then publish it.
      */

    // printf("while_run\n");
    while(true){
        therster_msg.Therster[i] = data[i];
        // printf("data [%d]: %d \n",i ,data[i]);
        i = i + 1;
        if(i == 9){
            i = 0;
            break;
        }
    }
    // for(int i=0;i=9;i++){
    //     therster_msg.Therster[i] = data[i];
    //     printf("data [%d]: %d ",i ,data[i]);
    // }
        main_control(10);
        therster_pub.publish(therster_msg);
     /**
      * The publish() function is how you send messages. The parameter
      * is the message object. The type of this object must agree with the type
      * given as a template parameter to the advertise<>() call, as was done
      * in the constructor above.
      */
 
     ros::spinOnce();
 
     loop_rate.sleep();

   }
 
    return 0;
}