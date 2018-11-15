 //  智能小车超声波避障实验(有舵机)
//  程序中电脑打印数值部分都被屏蔽了，打印会影响小车遇到障碍物的反应速度
//  调试时可以打开屏蔽内容Serial.print，打印测到的距离
//  本实验控制速度的pwm值和延时均有调节，但还是配合实际情况，实际电量调节数值
//=============================================================================
#include <Servo.h> 

#include <LiquidCrystal.h> //申明1602液晶的函数库 
//LiquidCrystal lcd(12,11,10,9,8,7,6,5,4,3,2);   //8数据口模式连线声明
LiquidCrystal lcd(11,10,9,8,7,6); //4数据口模式连线声明 

#include <IRremote.h>//包含红外库
int RECV_PIN = A0;//端口声明
IRrecv irrecv(RECV_PIN);
decode_results results;//结构声明

int on = 0;//标志位
unsigned long last = millis();

#include <dht11.h>
dht11 DHT11;
#define DHT11PIN 13

int Distance = 0;

int Echo = A1;  // Echo回声脚(P2.0) 舵机超声波
int Trig =A2;  //  Trig 触发脚(P2.1)
int EchoPin=A8;
int TrigPin=A9;

int Front_Distance = 0;
int Left_Distance = 0;
int Right_Distance = 0;

int sensorTouch=A6;//定义按键 A0 接口
int beep=A7;//定义蜂鸣器 A1 接口

int servopin=2;//设置舵机驱动脚到数字口2
int myangle;//定义角度变量
int pulsewidth;//定义脉宽变量
int val;
//================================================================
long run_car = 0x00FF18E7;//按键2
long back_car = 0x00FF4AB5;//按键8
long left_car = 0x00FF10EF;//按键4
long right_car = 0x00FF5AA5;//按键6
long stop_car = 0x00FF38C7;//按键5
long left_turn = 0x00ff30CF;//按键1
long right_turn = 0x00FF7A85;//按键3
//================================================================
long genzong =0x00FF42BD;//按键7 
long bizhang = 0x00FF52AD;//按键9
long xunji = 0x00FF6897;//按键0
long hwbizhang=0x00FFA25D;//按键红色开关

const int SensorRight = A14;     //右循迹红外传感器(P3.2 OUT1)
const int SensorLeft = A15;      //左循迹红外传感器(P3.3 OUT2)

const int SensorLeft_2 =47 ;     //左红外传感器(P3.4 OUT3)
const int SensorRight_2 =22 ;    //右红外传感器(P3.5 OUT4)
const int Sensor =41 ;    //中间红外传感器(P3.5 OUT


unsigned long starttime;
unsigned long stoptime;
unsigned long looptime;

long x;
float cm; //距离

int ledPin = 44;                // Connect LED to pin 13
int switcher = 45;                 // Connect Tilt sensor to Pin3

int Pin0 = 30;
int Pin1 = 31;
int Pin2 = 32;
int Pin3 = 33;
int Pin4 = 50;
int Pin5 = 51;
int Pin6 = 52;
int Pin7 = 53;
int _step =0 ;
int stepperSpeed = 10;//电机转速,1ms一步

int SL;    //左循迹红外传感器状态
int SR;    //右循迹红外传感器状态
int SL_2;    //左红外传感器状态
int SR_2;    //右红外传感器状态
int S;

int sw = 5;//定义碰撞开关引脚
int peng = 0;

void setup()
{
  Serial.begin(9600);     // 初始化串口
  //初始化电机驱动IO为输出方式
//  pinMode(Left_motor_go,OUTPUT); // PIN 5 (PWM)
//  pinMode(Left_motor_back,OUTPUT); // PIN 9 (PWM)
//  pinMode(Right_motor_go,OUTPUT);// PIN 6 (PWM) 
//  pinMode(Right_motor_back,OUTPUT);// PIN 10 (PWM)
  pinMode(sensorTouch,INPUT);//定义按键接口为输入接口
  pinMode(beep,OUTPUT);
  pinMode(SensorRight, INPUT); //定义右循迹红外传感器为输入
  pinMode(SensorLeft, INPUT); //定义左循迹红外传感器为输入
  pinMode(SensorRight_2, INPUT); //定义右红外传感器为输入
  pinMode(SensorLeft_2, INPUT); //定义左红外传感器为输入
  //初始化超声波引脚
  pinMode(Echo, INPUT);    // 定义超声波输入脚
  pinMode(Trig, OUTPUT);   // 定义超声波输出脚
  pinMode(EchoPin, INPUT);    // 定义超声波输入脚
  pinMode(TrigPin, OUTPUT);
  lcd.begin(16,2);      //初始化1602液晶工作                       模式
  //定义1602液晶显示范围为2行16列字符  
  pinMode(servopin,OUTPUT);//设定舵机接口为输出接口
  pinMode(ledPin, OUTPUT);      // Set digital pin 13 to output mode
  pinMode(switcher, INPUT);       // Set digital pin 3 to input mode
  pinMode(sw,INPUT);
//  //初始化电机驱动IO为输出方式
//  pinMode(13, OUTPUT);////端口模式，输出
  irrecv.enableIRIn(); // Start the receiver
}
//=======================智能小车的基本动作=========================

void run()     // 前进
{for(int i=1;i<10;i++){
  up1();
  up2();
}
}

void run(int time)     // 前进
{for(int i=1;i<10;i++){
  up1();
up2();
}
  delay(time * 100);   //执行时间，可以调整  
}

void brake(int time)  //刹车，停车
{
      digitalWrite(Pin0, LOW);
      digitalWrite(Pin1, LOW);
      digitalWrite(Pin2, LOW);
      digitalWrite(Pin3, LOW);
      digitalWrite(Pin4, LOW);
      digitalWrite(Pin5, LOW);
      digitalWrite(Pin6, LOW);
      digitalWrite(Pin7, LOW);
      delay(time * 100);//执行时间，可以调整  
}

void brake()  //刹车，停车
{
      digitalWrite(Pin0, LOW);
      digitalWrite(Pin1, LOW);
      digitalWrite(Pin2, LOW);
      digitalWrite(Pin3, LOW);
      digitalWrite(Pin4, LOW);
      digitalWrite(Pin5, LOW);
      digitalWrite(Pin6, LOW);
      digitalWrite(Pin7, LOW);
}

void left(int time)         //左转(左轮不动，右轮前进)
{for(int i=1;i<10;i++){
  up2();
}
  delay(time * 100);	//执行时间，可以调整  
}

void left()         //左转(左轮不动，右轮前进)
{for(int i=1;i<10;i++){
  up2();
}
}

void spin_left(int time)         //左转(左轮后退，右轮前进)
{for(int i=1;i<10;i++){
  up2();

down1();}
  delay(time * 100);	//执行时间，可以调整  
}

void spin_left()         //左转(左轮后退，右轮前进)
{for(int i=1;i<10;i++){
  up2();
  down1();
}
}

void right(int time)
{for(int i=1;i<10;i++){
  up1();}
  delay(time * 100);	//执行时间，可以调整  
}

void right()        //右转(右轮不动，左轮前进)
{for(int i=1;i<10;i++){
  up1();
}
}

void spin_right(int time)        //右转(右轮后退，左轮前进)
{for(int i=1;i<10;i++){
  up1();
  down2();
}
  delay(time * 100);	//执行时间，可以调整    
}

void spin_right()        //右转(右轮后退，左轮前进)
{for(int i=1;i<10;i++){
  up1();
  down2();
}
}

void back(int time)          //后退
{
  for(int i=1;i<10;i++){
  down1();
  down2();
    //执行时间，可以调整  
  }
    delay(time * 100); 
}

void back()          //后退
{  for(int i=1;i<10;i++){
  down1();
  down2();
}
}

//==========================================================

void keysacn()//按键扫描
{
       for(int i=0;i<80;i++)//输出一个频率的声音
        {
          digitalWrite(beep,HIGH);//发声音
          delay(1);//延时1ms
          digitalWrite(beep,LOW);//不发声音
          delay(1);//延时ms
        }
}

float Distance_test()   // 量出前方距离 
{
digitalWrite(Trig, LOW); //低高低电平发一个短时间脉冲去TrigPin
delayMicroseconds(2);
digitalWrite(Trig, HIGH);
delayMicroseconds(10);
digitalWrite(Trig, LOW);

cm = pulseIn(Echo, HIGH) / 58.0; //将回波时间换算成cm
cm = (int(cm * 100.0)) / 100.0; //保留两位小数
Serial.print(cm); //用串口监视器。在arduino IDE的tool的serial monitor那里可以看到监视器。上面

//是send，发送数据，下面是数据print的内容（这是串口发送数据出去的内容，这里就用这个来看收到的数

//据）。rx的串口的接收端，tx是发送端。
Serial.print("cm");
Serial.println(cm);
 return cm;
}  

float Distance_test1()   // 量出前方距离 
{
digitalWrite(TrigPin, LOW); //低高低电平发一个短时间脉冲去TrigPin
delayMicroseconds(2);
digitalWrite(TrigPin, HIGH);
delayMicroseconds(10);
digitalWrite(TrigPin, LOW);

cm = pulseIn(EchoPin, HIGH) / 58.0; //将回波时间换算成cm
cm = (int(cm * 100.0)) / 100.0; //保留两位小数
Serial.print(cm); //用串口监视器。在arduino IDE的tool的serial monitor那里可以看到监视器。上面

//是send，发送数据，下面是数据print的内容（这是串口发送数据出去的内容，这里就用这个来看收到的数

//据）。rx的串口的接收端，tx是发送端。
Serial.print("cm");
 return cm;
}  

void Distance_display(int Distance,int looptime)//显示距离
{
//  if((2<Distance)&(Distance<400))
//  {
    lcd.home();        //把光标移回左上角，即从头开始输出   
    lcd.print(" Distance: ");       //显示
    lcd.print(Distance);       //显示距离
    lcd.print("cm");          //显示
    lcd.setCursor(0,2);   //把光标定位在第2行，第6列
    lcd.print(" Time: ");
    lcd.print(looptime);
    lcd.print("s");
    
  
//  }
//  else
//  {
//    lcd.home();        //把光标移回左上角，即从头开始输出  
//    lcd.print("!!! Out of range");       //显示
//  }
//  delay(50);
  lcd.clear();
}

void Distance_display(int Distance, int humidity,int temperature)
//
//  Serial.print("Temperature (oC): ");
//  Serial.println((float)DHT11.temperature, 2); )//显示距离
{
//  if((2<Distance)&(Distance<400))
//  {
    lcd.home();        //把光标移回左上角，即从头开始输出   
    lcd.print(" Distance: ");       //显示
    lcd.print(Distance);       //显示距离
    lcd.print("cm");          //显示
    lcd.setCursor(0,2);   //把光标定位在第2行，第6列
    lcd.print(" T&H: ");
    lcd.print((float)DHT11.temperature,1);
    lcd.print(" ");
    lcd.print((float)DHT11.humidity,1);
    
  
//  }
//  else
//  {
//    lcd.home();        //把光标移回左上角，即从头开始输出  
//    lcd.print("!!! Out of range");       //显示
//  }
//  delay(50);
//  lcd.clear();
}

void Distance_display(int Distance)//显示距离
{
//  if((2<Distance)&(Distance<400))
//  {
    lcd.home();        //把光标移回左上角，即从头开始输出   
    lcd.print(" Distance: ");       //显示
    lcd.print(Distance);       //显示距离
    lcd.print("cm");          //显示
//    lcd.setCursor(0,2);   //把光标定位在第2行，第6列
//    lcd.print(" Time: ");
//    lcd.print(looptime);
    
  
//  }
//  else
//  {
//    lcd.home();        //把光标移回左上角，即从头开始输出  
//    lcd.print("!!! Out of range");       //显示
//  }
//  delay(50);
  lcd.clear();
}

void servopulse(int servopin,int myangle)/*定义一个脉冲函数，用来模拟方式产生PWM值*/
{
  pulsewidth=(myangle*11)+500;//将角度转化为500-2480 的脉宽值
  digitalWrite(servopin,HIGH);//将舵机接口电平置高
  delayMicroseconds(pulsewidth);//延时脉宽值的微秒数
  digitalWrite(servopin,LOW);//将舵机接口电平置低
  delay(20-pulsewidth/1000);//延时周期内剩余时间
}

void front_detection()
{
  //此处循环次数减少，为了增加小车遇到障碍物的反应速度
  for(int i=0;i<=5;i++) //产生PWM个数，等效延时以保证能转到响应角度
  {
    servopulse(servopin,90);//模拟产生PWM
  }
  Front_Distance = Distance_test();
  //Serial.print("Front_Distance:");      //输出距离（单位：厘米）
 // Serial.println(Front_Distance);         //显示距离
 //Distance_display(Front_Distance);
}

void left_detection()
{
  for(int i=0;i<=15;i++) //产生PWM个数，等效延时以保证能转到响应角度
  {
    servopulse(servopin,175);//模拟产生PWM
  }
  Left_Distance = Distance_test();
  //Serial.print("Left_Distance:");      //输出距离（单位：厘米）
  //Serial.println(Left_Distance);         //显示距离
}

void right_detection()
{
  for(int i=0;i<=15;i++) //产生PWM个数，等效延时以保证能转到响应角度
  {
    servopulse(servopin,0);//模拟产生PWM
  }
  Right_Distance = Distance_test();
  //Serial.print("Right_Distance:");      //输出距离（单位：厘米）
  //Serial.println(Right_Distance);         //显示距离
}
//===========================================================

void dump(decode_results *results)
{
  int count = results->rawlen;
  if (results->decode_type == UNKNOWN) 
  {
    //Serial.println("Could not decode message");
    brake();
  } 
}

void down2()
  {
   
    switch(_step){
    case 0:
    //stepperSpeed++;
      digitalWrite(Pin0, LOW);
      digitalWrite(Pin1, LOW);
      digitalWrite(Pin2, LOW);
      digitalWrite(Pin3, HIGH);//32A
    break;
    case 1:
      digitalWrite(Pin0, LOW);
      digitalWrite(Pin1, HIGH);//10B
      digitalWrite(Pin2, LOW);
      digitalWrite(Pin3, HIGH);
    break;
    case 2:
      digitalWrite(Pin0, LOW);
      digitalWrite(Pin1, HIGH);
      digitalWrite(Pin2, LOW);
      digitalWrite(Pin3, LOW);
    break;
    case 3:
      digitalWrite(Pin0, LOW);
      digitalWrite(Pin1, HIGH);
      digitalWrite(Pin2, HIGH);
      digitalWrite(Pin3, LOW);
    break;
    case 4:
      digitalWrite(Pin0, LOW);
      digitalWrite(Pin1, LOW);
      digitalWrite(Pin2, HIGH);
      digitalWrite(Pin3, LOW);
    break;
    case 5:
      digitalWrite(Pin0, HIGH);
      digitalWrite(Pin1, LOW);
      digitalWrite(Pin2, HIGH);
      digitalWrite(Pin3, LOW);
    break;
      case 6:
      digitalWrite(Pin0, HIGH);
      digitalWrite(Pin1, LOW);
      digitalWrite(Pin2, LOW);
      digitalWrite(Pin3, LOW);
    break;
    case 7:
      digitalWrite(Pin0, HIGH);
      digitalWrite(Pin1, LOW);
      digitalWrite(Pin2, LOW);
      digitalWrite(Pin3, HIGH);
    break;
    default:
      digitalWrite(Pin0, LOW);
      digitalWrite(Pin1, LOW);
      digitalWrite(Pin2, LOW);
      digitalWrite(Pin3, LOW);
    break;
  }
   _step++;
 
  if(_step>7){    _step=0;  }

  delay(stepperSpeed);
  } 
    
     
    
  void up2()
{ 
  
  switch(_step){
    case 0:
    //stepperSpeed++;
      digitalWrite(Pin0, HIGH);
      digitalWrite(Pin1, LOW);
      digitalWrite(Pin2, LOW);
      digitalWrite(Pin3, LOW);//32A
    break;
    case 1:
      digitalWrite(Pin0, HIGH);
      digitalWrite(Pin1, LOW);//10B
      digitalWrite(Pin2, HIGH);
      digitalWrite(Pin3, LOW);
    break;
    case 2:
      digitalWrite(Pin0, LOW);
      digitalWrite(Pin1, LOW);
      digitalWrite(Pin2, HIGH);
      digitalWrite(Pin3, LOW);
    break;
    case 3:
      digitalWrite(Pin0, LOW);
      digitalWrite(Pin1, HIGH);
      digitalWrite(Pin2, HIGH);
      digitalWrite(Pin3, LOW);
    break;
    case 4:
      digitalWrite(Pin0, LOW);
      digitalWrite(Pin1, HIGH);
      digitalWrite(Pin2, LOW);
      digitalWrite(Pin3, LOW);
    break;
    case 5:
      digitalWrite(Pin0, LOW);
      digitalWrite(Pin1, HIGH);
      digitalWrite(Pin2, LOW);
      digitalWrite(Pin3, HIGH);
    break;
      case 6:
      digitalWrite(Pin0, LOW);
      digitalWrite(Pin1, LOW);
      digitalWrite(Pin2, LOW);
      digitalWrite(Pin3, HIGH);
    break;
    case 7:
      digitalWrite(Pin0, HIGH);
      digitalWrite(Pin1, LOW);
      digitalWrite(Pin2, LOW);
      digitalWrite(Pin3, HIGH);
    break;
    default:
      digitalWrite(Pin0, LOW);
      digitalWrite(Pin1, LOW);
      digitalWrite(Pin2, LOW);
      digitalWrite(Pin3, LOW);
    break;
  }
    _step++;
 
  if(_step>7){    _step=0;  }
 
  delay(stepperSpeed);

}

void up1()
  {
   
    switch(_step){
    case 0:
    //stepperSpeed++;
      digitalWrite(Pin4, LOW);
      digitalWrite(Pin5, LOW);
      digitalWrite(Pin6, LOW);
      digitalWrite(Pin7, HIGH);//32A
    break;
    case 1:
      digitalWrite(Pin4, LOW);
      digitalWrite(Pin5, HIGH);//10B
      digitalWrite(Pin6, LOW);
      digitalWrite(Pin7, HIGH);
    break;
    case 2:
      digitalWrite(Pin4, LOW);
      digitalWrite(Pin5, HIGH);
      digitalWrite(Pin6, LOW);
      digitalWrite(Pin7, LOW);
    break;
    case 3:
      digitalWrite(Pin4, LOW);
      digitalWrite(Pin5, HIGH);
      digitalWrite(Pin6, HIGH);
      digitalWrite(Pin7, LOW);
    break;
    case 4:
      digitalWrite(Pin4, LOW);
      digitalWrite(Pin5, LOW);
      digitalWrite(Pin6, HIGH);
      digitalWrite(Pin7, LOW);
    break;
    case 5:
      digitalWrite(Pin4, HIGH);
      digitalWrite(Pin5, LOW);
      digitalWrite(Pin6, HIGH);
      digitalWrite(Pin7, LOW);
    break;
      case 6:
      digitalWrite(Pin4, HIGH);
      digitalWrite(Pin5, LOW);
      digitalWrite(Pin6, LOW);
      digitalWrite(Pin7, LOW);
    break;
    case 7:
      digitalWrite(Pin4, HIGH);
      digitalWrite(Pin5, LOW);
      digitalWrite(Pin6, LOW);
      digitalWrite(Pin7, HIGH);
    break;
    default:
      digitalWrite(Pin4, LOW);
      digitalWrite(Pin5, LOW);
      digitalWrite(Pin6, LOW);
      digitalWrite(Pin7, LOW);
    break;
  }
   _step++;
 
  if(_step>7){    _step=0;  }

  delay(stepperSpeed);
  } 
    
     
    
  void down1()
{ 
  
  switch(_step){
    case 0:
    //stepperSpeed++;
      digitalWrite(Pin4, HIGH);
      digitalWrite(Pin5, LOW);
      digitalWrite(Pin6, LOW);
      digitalWrite(Pin7, LOW);//32A
    break;
    case 1:
      digitalWrite(Pin4, HIGH);
      digitalWrite(Pin5, LOW);//10B
      digitalWrite(Pin6, HIGH);
      digitalWrite(Pin7, LOW);
    break;
    case 2:
      digitalWrite(Pin4, LOW);
      digitalWrite(Pin5, LOW);
      digitalWrite(Pin6, HIGH);
      digitalWrite(Pin7, LOW);
    break;
    case 3:
      digitalWrite(Pin4, LOW);
      digitalWrite(Pin5, HIGH);
      digitalWrite(Pin6, HIGH);
      digitalWrite(Pin7, LOW);
    break;
    case 4:
      digitalWrite(Pin4, LOW);
      digitalWrite(Pin5, HIGH);
      digitalWrite(Pin6, LOW);
      digitalWrite(Pin7, LOW);
    break;
    case 5:
      digitalWrite(Pin4, LOW);
      digitalWrite(Pin5, HIGH);
      digitalWrite(Pin6, LOW);
      digitalWrite(Pin7, HIGH);
    break;
      case 6:
      digitalWrite(Pin4, LOW);
      digitalWrite(Pin5, LOW);
      digitalWrite(Pin6, LOW);
      digitalWrite(Pin7, HIGH);
    break;
    case 7:
      digitalWrite(Pin4, HIGH);
      digitalWrite(Pin5, LOW);
      digitalWrite(Pin6, LOW);
      digitalWrite(Pin7, HIGH);
    break;
    default:
      digitalWrite(Pin4, LOW);
      digitalWrite(Pin5, LOW);
      digitalWrite(Pin6, LOW);
      digitalWrite(Pin7, LOW);
    break;
  }
    _step++;
 
  if(_step>7){    _step=0;  }
 
  delay(stepperSpeed);

}

void pengzhuang(){
   val = digitalRead(sw);    //读传感器信息
   if(LOW == val)
   { 
    Serial.println("DIDIDIDIDIDIDIDI");
//    digitalWrite(13,HIGH);
   for(int j=0;j<5;j++)
       {keysacn();      } 

   }
}

void loop()
{  
  int chk = DHT11.read(DHT11PIN);
  Serial.print("Read sensor: ");
//  switch (chk)
//  {
//    case DHTLIB_OK: 
//                Serial.println("OK"); 
//                break;
//    case DHTLIB_ERROR_CHECKSUM: 
//                Serial.println("Checksum error"); 
//                break;
//    case DHTLIB_ERROR_TIMEOUT: 
//                Serial.println("Time out error"); 
//                break;
//    default: 
//                Serial.println("Unknown error"); 
//                break;
//  }
  Serial.print("Humidity (%): ");
  Serial.println((float)DHT11.humidity, 2);

  Serial.print("Temperature (oC): ");
  Serial.println((float)DHT11.temperature, 2);
//  delay(100);
  
  x=Distance_test1();
  Distance_display(x,DHT11.humidity,DHT11.temperature); 
     
  if (digitalRead(switcher) == HIGH) //Read sensor value
    {digitalWrite(ledPin, HIGH); // Turn on LED when the sensor is tilted
    Serial.println("翻车");
//    for(int j=0;j<50;j++)
       {keysacn();      } 
    }
  else
    digitalWrite(ledPin, LOW);    // Turn off LED when the sensor is not triggered
  
   if(digitalRead(sensorTouch) == LOW ){ //触摸传感器
    
      keysacn();   
      brake(5);   //停车
  for(int i=1;i<200;i++){
  down1();
  down2();
    //执行时间，可以调整  
  }
//    back(10); //后退1s
   
      brake(5); //停止0.5s
        for(int i=1;i<200;i++){
  up1();
  up2();
    //执行时间，可以调整  
  }
//      run(10);//前进1s

      brake(5); //停止0.5s
      for(int i=1;i<200;i++){
  up2();
}
//      left(10);//向左转1s
for(int i=1;i<200;i++){
  up1();
}
//      right(10);//向右转1s
for(int i=1;i<200;i++){
  down1();
  up2();
}
      spin_left(20);//向左旋转2s
   for(int i=1;i<200;i++){
  down2();
  up1();
}
//      spin_right(20);//向右旋转2s
      brake(5);   //停车    
      }

  if (irrecv.decode(&results)) //调用库函数：解码
  { 
 
    // If it's been at least 1/4 second since the last
    // IR received, toggle the relay
    if (millis() - last > 250) //确定接收到信号
    {
      Serial.println("jiema"); 
      on = !on;//标志位置反
      dump(&results);//解码红外信号
    }
    
    if (results.value == bizhang )//按键2
{ 
// keysacn();	   //调用按键扫描函数
  while(1)
  {      pengzhuang();
    Serial.println("bizhang");   
    front_detection();//测量前方距离
    Distance_display(Front_Distance);
    if(Front_Distance < 32)//当遇到障碍物时
    {
      back(2);//后退减速 
      brake(2);//停下来做测距
      left_detection();//测量左边距障碍物距离
      Distance_display(Left_Distance);//液晶屏显示距离
      right_detection();//测量右边距障碍物距离
      Distance_display(Right_Distance);//液晶屏显示距离
      if((Left_Distance < 35 ) &&( Right_Distance < 35 ))//当左右两侧均有障碍物靠得比较近
        spin_left(0.7);//旋转掉头
      else if(Left_Distance > Right_Distance)//左边比右边空旷
      {      
        left(3);//左转
        brake(1);//刹车，稳定方向
      }
      else//右边比左边空旷
      {
        right(3);//右转
        brake(1);//刹车，稳定方向
      }
    }
    else
    {
      run(); //无障碍物，直行     
    }
  } 
}
 if (results.value == genzong ){    
  Serial.println("genzong"); 
  while(1)
  { 
     pengzhuang();
//    Serial.println("Obj detected at:");
    //有信号为LOW  没有信号为HIGH
//    x=Distance_test();
//    Distance_display(x); 
//    front_detection();//测量前方距离
//    Distance_display(Front_Distance);//液晶屏显示距离
    SR_2 = digitalRead(SensorRight_2);
    SL_2 = digitalRead(SensorLeft_2);
    S=digitalRead(Sensor);
    x=Distance_test1();
//    Distance_display(x,looptime); 
    if((SL_2==LOW && SR_2==HIGH && S==HIGH)||(SL_2 == HIGH && SR_2==LOW && S==HIGH)||(SL_2 == HIGH && SR_2==HIGH && S==LOW)||(SL_2 == LOW && SR_2==HIGH && S==LOW)||(SL_2 == HIGH && SR_2==LOW && S==LOW))
    { 
    starttime = millis();
    if (SL_2 == HIGH && SR_2==HIGH && S==LOW)
      {
        run();   //调用前进函数
      }
    else if ((SL_2 == HIGH && SR_2==LOW && S==HIGH)||(SL_2 == HIGH && SR_2==LOW && S==LOW))// 右边探测到有障碍物，有信号返回，向右转 
      {right();
//      Serial.print("RIGHT");
      }
    else if ((SL_2==LOW && SR_2==HIGH && S==HIGH)||(SL_2 == LOW && SR_2==HIGH && S==LOW)) //左边探测到有障碍物，有信号返回，向左转  
      {left();}
      stoptime = millis();
      looptime = stoptime - starttime+looptime;   
    }
    else // 没有障碍物，停
    {
      brake();
      Serial.println(looptime*0.22); 
      }
  }
 }

if (results.value == hwbizhang ){
    while(1)
  {     pengzhuang();
    //有信号为LOW  没有信号为HIGH
    SR_2 = digitalRead(SensorRight_2);
    SL_2 = digitalRead(SensorLeft_2);
    S=digitalRead(Sensor);
    if (SL_2==HIGH&&SR_2==HIGH&&S==HIGH)
      run();
        //调用前进函数
    else if ((SL_2 == HIGH && SR_2==LOW && S==HIGH)||(SL_2 == HIGH && SR_2==LOW && S==LOW))// 右边探测到有障碍物，有信号返回，向左转 
        left();
    else if ((SL_2==LOW && SR_2==HIGH && S==HIGH)||(SL_2 == LOW && SR_2==HIGH && S==LOW)) //左边探测到有障碍物，有信号返回，向右转  
      right();
    else // 都是有障碍物, 后退
    {
      back(4.5);//后退
      spin_right(4.5);//有旋转，调整方向
    }
  }
  }


if (results.value == xunji ){
  while(1)
  {     
  //有信号为LOW  没有信号为HIGH
  SR = digitalRead(SensorRight);//有信号表明在白色区域，车子底板上L3亮；没信号表明压在黑线上，车子底板上L3灭
  SL = digitalRead(SensorLeft);//有信号表明在白色区域，车子底板上L2亮；没信号表明压在黑线上，车子底板上L2灭
  if (SL == LOW&&SR==LOW)
    run();   //调用前进函数
  else if (SL == HIGH & SR == LOW)// 左循迹红外传感器,检测到信号，车子向右偏离轨道，向左转 
    left();
  else if (SR == HIGH & SL == LOW) // 右循迹红外传感器,检测到信号，车子向左偏离轨道，向右转  
    right();
  else // 都是白色, 停止
  brake();
  }
  }
 
 if (results.value == run_car )//按键2
      run();//前进
    if (results.value == back_car )//按键8
      back();//后退
    if (results.value == left_car )//按键4
      left();//左转
    if (results.value == right_car )//按键6
      right();//右转
    if (results.value == stop_car )//按键5
      {
       brake();//停车
//       front_detection();
//        Distance_test(); 
       Distance_display(Distance_test()); }
    if (results.value == left_turn )//按键1
      spin_left();//左旋转
    if (results.value == right_turn )//按键3
      spin_right();//右旋转
 last = millis();   
 irrecv.resume(); // 接收下一个值
 
}
}



  
