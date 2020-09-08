#include<Servo.h>                        //導入伺服馬達函式庫
#include <Wire.h>
#include <Adafruit_MotorShield.h>


Servo servo1;                           //建立Servo物件
Adafruit_MotorShield AFMS = Adafruit_MotorShield();   //創建直流馬達物件，使用默認的I2C地址

Adafruit_DCMotor *myMotor = AFMS.getMotor(1);   //使用 M1 來控制直流馬達

int stop_start=0;          //遇障礙物停止狀態:無                                 //////////////////////////////////////////////////////////////////////////////////////////////////
//伺服馬達區-----------------------------------------------------------
int cmd1;                 //最終角度
int mid=94;  //85         //前輪中間馬達角度
int right=mid+35;//    130//前輪最右邊馬達角度
int left=mid-50;  //40    //前輪最左邊馬達角度
int mid_left=mid-20;    //-35  //中左值           
int mid_right=mid+32;  //+28    //中右值           
//PID區----------------------------------------------------------------
int error;                //與50的誤差       
int integral=0;           //積分值           
int derivative;           //微分值           
int last_error=0;         //上次的誤差      
int PID1;                 //運算的PID值      
//紅外線測距區---------------------------------------------------------
int stop_step=0;          //停止距離次數
int slow_step=0;          //放慢距離速度
int go_step=0;            //重啟車子次數
//車子狀態-------------------------------------------------------------
int car_stop=0;              //停車狀態=0,開車狀態=1  
//車子啟動--------------------------------------------------------
int action=0;                   //是否在開啟狀態
//直流馬達區-----------------------------------------------------------
int Speed_pwm=0;                //後輪pwm值

unsigned long times;            //舊時間
unsigned long newtime;          //新時間
int AM1=7;             //相位A腳位
int BM1=8;             //相位B腳位
int valA=0;            //A的累積值
int valB=0;            //B的累積值
int flagA=0;           //A的狀態
int flagB=0;           //B的狀態
int d_time=7;          //週期時間

float Normal_speed_min=0.85; //0.8     0.95極限   正常轉速最小值
float Normal_speed_max=0.86; //0.81    0.96極限   正常轉速最大值
 
float Slow_speed_min=0.5;//0.5     0.58    放慢轉速最小值
float Slow_speed_max=0.51;//0.6      0.6     放慢轉速最大值

float Rotating_speed_min=Normal_speed_min;        //現在轉速最小值 
float Rotating_speed_max=Normal_speed_max;        //現在轉速最大值
//前進停止---------------------------------------------------------
void Forward(int pwm){          //前進
  myMotor->run(FORWARD);   //前進
  myMotor->setSpeed(pwm);  //速度，範圍 0~255 
  }
void Stop(){                    //停止
  myMotor->run(RELEASE);   //停止
}
//測距-----------------------------------------------------------
int Distance(){
  float volts = analogRead(A0)*(5./1024.);    //將 0~1024 轉換成 0~5
  int distance = 13*pow(volts, -1);           //計算距離
  return distance;                            //回傳距離
  }
//設定------------------------------------------------------------------
void setup() {
Serial.begin(115200);     //傳輸速率
servo1.attach(9);        //伺服馬達腳位 
pinMode(AM1,INPUT);      //脈衝腳位1 
pinMode(BM1,INPUT);      //脈衝腳位2 
pinMode(13,OUTPUT);       //轉彎提示燈腳位
digitalWrite(13,LOW);     //轉彎提示燈關閉
cmd1=0;                   //初始化
AFMS.begin();   //使用直流馬達物件
servo1.write(right);     //---------------------------------------------
delay(500);              //知道arduino有正常運作 

servo1.write(mid);
delay(500); 

servo1.write(left);
delay(500);

servo1.write(mid);
delay(500);              //----------------------------------------------

}

void loop() {
delay(3);
//計算車子速度------------------------------------------------------------
  if(car_stop==1){          //停車狀態=0,開車狀態=1
  newtime=times=millis();
  while((newtime-times)<d_time){
    if(digitalRead(AM1)==HIGH && flagA==0){      
      valA++;
      flagA=1;
      }    
    if(digitalRead(AM1)==LOW && flagA==1){
      valA++;
      flagA=0;
      }
    if(digitalRead(BM1)==HIGH && flagB==0){
      valB++;
      flagB=1;
      }
    if(digitalRead(BM1)==LOW && flagB==1){
      valB++;
      flagB=0;
      }
      newtime=millis();
    }
    double r=(valA+valB)/(1.98*d_time);      //Rotating speed
  
  if(r<Rotating_speed_min){               //轉速小於最小轉速
    Speed_pwm=Speed_pwm+1;                //pwm上升
    }
  else if(r>Rotating_speed_max){          //轉速大於最大轉速
    Speed_pwm=Speed_pwm-2;                //pwm下降
    }
    if(Speed_pwm>255){                    //如果pwm大於255
      Speed_pwm=255;                      //pwm等於255
      }
    if(Speed_pwm<0){                      //如果pwm小於0
      Speed_pwm=0;                        //pwm等於0
      }
  Forward(Speed_pwm);                     //前進
  valA=valB=0;                            //脈衝累積值重新計算
  }

//測距區--------------------------------------------------------------
int lon=Distance();
//lon=30;
if(action==1 && car_stop==0 && (lon>27 || lon==0)){          //stop=0,go=1  如果在停車狀態下障礙物離開 
  go_step=go_step+1;
  if(go_step>=500){
    car_stop=1;                 //停車狀態=0,開車狀態=1
    go_step=0;                  //重新計算
    stop_start=0;     ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }
  }
  if(action==1 && lon>=1 && lon<=20){              //當前方物品距離1~20之間 //////////////////////////////////////////////////
    slow_step=0;
    stop_step=stop_step+1;            
    if(stop_step>=10){                 //重複確認5次 stop      
      car_stop=0;                     //停車狀態=0,開車狀態=1
      Stop();
      stop_start=1;       ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////   
      }    
    }  

  else if(car_stop==1 && lon>20 && lon<=27){       //stop=0,go=1   當前方物品距離20~27之間
    stop_step=0;
    slow_step=slow_step+1;
    if(slow_step>=10){                              //重複確認5次 slow down
      Rotating_speed_min=Slow_speed_min;            //更改標準:放慢
      Rotating_speed_max=Slow_speed_max;            //更改標準:放慢
      car_stop=1;                                //停車狀態=0,開車狀態=1
      }
    }
//一般開車------------------------------------------------------------
else{
  slow_step=0;                  //計算停車的次數歸零
  stop_step=0;                  //計算減速的次數歸零 
  if(Serial.available() && stop_start==0){        //Serial.available >=1 的時候會執行，Serial.available >=1等於有資料傳入
  int cmd=Serial.read();         //接受到的值
  
  if (action==0){               //如果全停狀態後第一次接收到訊息
      action=1;                   //python結束前不會再跑這個判斷式
      car_stop=1;                 //停車狀態=0,開車狀態=1  車子開始行走
      Rotating_speed_min=Normal_speed_min;         //更改標準:正常              
      Rotating_speed_max=Normal_speed_max;         //更改標準:正常
      }
     if (cmd=='m'){                //all stop(python結束時)
      Stop();
      servo1.write(mid);
      delay(500);
      servo1.write(right);
      delay(500);
      servo1.write(mid);
      delay(500);
      servo1.write(left);
      delay(500);
      servo1.write(mid);
      delay(1000);
      digitalWrite(13,LOW);      //提示燈關閉
      action=0;                  //重新等待開始
      car_stop=0;                //停車狀態=0,開車狀態=1 //車子停止    
  }
     else if(cmd=='n'){          //紅燈
        servo1.write(mid);
        car_stop=0;                  //停車狀態=0,開車狀態=1
        action=2;                    //紅燈的等待
        Stop();
        delay(1000);  
      }
  
     else if(action==2 && cmd=='l'){      //綠燈    
        servo1.write(mid);
        car_stop=1;                 //停車狀態=0,開車狀態=1
        Rotating_speed_min=0.35;      //更改標準:放慢   較特殊 
        Rotating_speed_max=0.45;      //更改標準:放慢   較特殊
        action=1;                   //變回一般開車
    }
    
    else if(cmd=='e'){              //十字路口右轉                             
      car_stop=1;                         //停車狀態=0,開車狀態=1
      Rotating_speed_min=Slow_speed_min;      //更改標準:放慢         
      Rotating_speed_max=Slow_speed_max;      //更改標準:放慢                 
      for(int i=mid;i<=right;i+=((right-mid)/10)){
      if(i>right){
        i=right;
        }
      servo1.write(i);
      delay(40);   //30
      }
      }
    
    else if(cmd=='f'){                   //十字路口直走             
      car_stop=1;                        //停車狀態=0,開車狀態=1
      Rotating_speed_min=Slow_speed_min;      //更改標準:放慢         
      Rotating_speed_max=Slow_speed_max;      //更改標準:放慢         
      servo1.write(mid);
      delay(50);                                    
      }
    
    else if(cmd=='g'){                     //十字路口左轉                 
      car_stop=1;                          //停車狀態=0,開車狀態=1
      Rotating_speed_min=Slow_speed_min;      //更改標準:放慢         
      Rotating_speed_max=Slow_speed_max;      //更改標準:放慢                    
      for(int i=mid;i>=left;i-=((mid-left)/10)){
      if(i<left){
        i=left;
        } 
      servo1.write(i);
      delay(50);   //35
      }  
      }
  
    else if(cmd=='j'){                 //進入十字路口時  
      car_stop=1;                       //停車狀態=0,開車狀態=1
      servo1.write(mid);
      delay(100);
      Rotating_speed_min=Slow_speed_min;      //更改標準:放慢  較特殊0.59 
      Rotating_speed_max=Slow_speed_max;      //更改標準:放慢  較特殊0.69 
      digitalWrite(13,HIGH);               //提示燈開啟
      servo1.write(mid);      
      }
  
    else if(cmd=='r'){                //當十字路口直走時，右邊線消失，往右靠
      car_stop=1;                      //停車狀態=0,開車狀態=1
      servo1.write(mid+20);          
      Rotating_speed_min=Slow_speed_min;    //更改標準:放慢     
      Rotating_speed_max=Slow_speed_max;    //更改標準:放慢                        
      }
    else if(cmd=='s'){                //當十字路口直走時，左邊線消失，往左靠
      car_stop=1;                      //停車狀態=0,開車狀態=1
      servo1.write(mid-20);            
      Rotating_speed_min=Slow_speed_min;        //更改標準:放慢 
      Rotating_speed_max=Slow_speed_max;        //更改標準:放慢                      
      }  
    
   else {
    digitalWrite(13,LOW);             //提示燈關閉    
    error=cmd-50;                     //誤差   
  
    if(cmd==0 || cmd==100){           //當在最左或最右時不計算PID      
      error=0;
      derivative=0;
      integral=0;
      }
    else{  
    derivative=error-last_error;       //這一次誤差減上一次誤差  
    integral=integral+error;           //前面的誤差全部加總   
    }
  
    PID1=cmd+0*integral+2*derivative;   //計算PD控制
    
    last_error=error;                //將前一次誤差保留     
    
    if(PID1>100){
      PID1=100;
      }
    if(PID1<0){
      PID1=0;
      }  
  
    if(PID1>25 && PID1<=50){                   //中右轉轉換  
    cmd1=map(PID1,26,50,mid_right,mid);
    }
    if(PID1>=0 && PID1<=25){                 
    cmd1=map(PID1,0,25,right,mid_right);
    }
    
    if(PID1>=50 && PID1<75){                   //中左轉轉換  
    cmd1=map(PID1,50,74,mid,mid_left); 
      }
    
    if(PID1>=75 && PID1<=100){
    cmd1=map(PID1,75,100,mid_left,left);
    }
  
    car_stop=1;                             //stop=0,go=1  //正常運作下
    servo1.write(cmd1);                               
    Rotating_speed_min=Normal_speed_min;         //更改標準:正常              
    Rotating_speed_max=Normal_speed_max;         //更改標準:正常
  
   if(PID1<=25 || PID1>=75){
    Rotating_speed_min=Slow_speed_min;           //更改標準:放慢            
    Rotating_speed_max=Slow_speed_max;           //更改標準:放慢
     }
       
   }
  }
 }
}
