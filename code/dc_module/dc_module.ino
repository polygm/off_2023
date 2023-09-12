/*
 * DC 모듈 코드
 * version 0.0.1
*/
#include "PinChangeInterrupt.h" // 라이브러리 추가 설치 필요

#define BeepPin 12 // 부저

// L298n 모터연결
#define DC1_1 4 // IN1
#define DC1_2 5 // IN2, PWM
#define DC2_1 6 // IN3, PWM
#define DC2_2 7 // IN4

#define BTN1 8 
#define BTN2 9 
#define BTN3 10 
#define BTN4 11 

void setup() {
  // 전원 입력시 삑 (1번)
  pinMode(BeepPin, OUTPUT);
  beep(150); delay(150); // 삐빅


}

void beep(int time)
{
  // 초기화1 : 내부 LED
  pinMode(LED_BUILTIN, OUTPUT);

  // 초기화2 : 비프 소리
  digitalWrite(BeepPin, HIGH);
  delay(time);
  digitalWrite(BeepPin, LOW);

  //초기화3 : 시리얼
  // 시리얼 초기화
  Serial.begin(115200);
  Serial.println("Module On");

  // 초기화4. DC모터 초기화
  pinMode(DC1_1, OUTPUT);
  pinMode(DC1_2, OUTPUT);
  pinMode(DC2_1, OUTPUT);
  pinMode(DC2_2, OUTPUT);


  // 초기화5. 버튼스위치
  pinMode(BTN1, INPUT);
  attachPCINT( digitalPinToPCINT(BTN1), button1_press, FALLING);

  pinMode(BTN2, INPUT);
  attachPCINT( digitalPinToPCINT(BTN2), button2_press, FALLING);

  pinMode(BTN3, INPUT);
  attachPCINT( digitalPinToPCINT(BTN3), button3_press, FALLING);

  pinMode(BTN4, INPUT);
  attachPCINT( digitalPinToPCINT(BTN4), button4_press, FALLING);


}

int btn1_status = 0;
void button1_press() {
  int temp = digitalRead(BTN1);
  btn1_status = 1;
  Serial.println("btn1 pressed");
}

int btn2_status = 0;
void button2_press() {
  int temp = digitalRead(BTN2);
  btn2_status = 1;
  Serial.println("btn2 pressed");
}

int btn3_status = 0;
void button3_press() {
  int temp = digitalRead(BTN3);
  btn3_status = 1;
  Serial.println("btn3 pressed");
}

int btn4_status = 0;
void button4_press() {
  int temp = digitalRead(BTN4);
  btn4_status = 1;
  Serial.println("btn4 pressed");
}

/**
 * 내부 LED 깜빡임 (초기값 1초)
*/
void builtin_led_blank(int time=1000) {
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(time);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(time);                      // wait for a second
}

// 모터속도 지정
int m1_speed = 255, m2_speed=255;
char mode;
void loop() {
  builtin_led_blank(500); // 0.5초 깜빡임

  if(btn1_status == 1) {
    M1_CW_step();
  }

  if(btn2_status == 1) {
    M1_CCW_step();
  }

  if(btn3_status == 1) {
    M2_CW_step();
  }

  if(btn4_status == 1) {
    M2_CCW_step();
  }

  char cmd;
  
  if(Serial.available()){
    cmd = Serial.read();

    if(cmd == 10) { 
      // lf code (10)
    } else if( cmd >= 48 && cmd <= 57) {
      if(mode == 112) { //pwm set
        m1_speed = m1_speed * 10 + (cmd - 48); 
        m2_speed = m1_speed;
        Serial.print("speed is ");
        Serial.println(m1_speed);
      }
      
      
      Serial.print(cmd);
    } else if(cmd == 97) { //a
      // 모터1 (정회전)
      M1_stop();
      M1_CW();
    } else if(cmd == 98) { //b
      // 모터1 (역회전)
      M1_stop();
      M1_CCW();
    } else if(cmd == 99) { //c
      // 모터2 (정회전)
      M2_stop();
      M2_CW();
    } else if(cmd == 100) { //d
      // 모터2 (역회전)
      M2_stop();
      M2_CCW();
    } else if(cmd == 115) { //s
      // 모터2 (역회전)
      M1_stop();
      M2_stop();
    } else if(cmd == 112) { //p
      // pwm 속도조절
      mode = cmd;
      m1_speed = 0;
      Serial.println("pwn speed set");
    } else if(cmd == 109) { // m(109) 값 전달 받은 경우 이동
      //motor_move(received_pos);
      //received_pos = 0;
    } else if(cmd == 100) { // d
      //Serial.print("max distance = ");
      //Serial.println(motor_stop_max);
    } else if(cmd == 99) { // c
      //Serial.print("current position = ");
      //Serial.println(motor_position);
    }
  }
}

// 누르는 동작
void M1_CW_step() {
  int temp;
  M1_CW();

  while(btn1_status) {
    temp = digitalRead(BTN1);
    if(temp == 1) {
      M1_stop();
      btn1_status = 0;
    }
  }
}

void M1_CCW_step() {
  int temp;
  M1_CCW();

  while(btn2_status) {
    temp = digitalRead(BTN2);
    if(temp == 1) {
      M1_stop();
      btn2_status = 0;
    }
  }
}

void M2_CW_step() {
  int temp;
  M2_CW();

  while(btn3_status) {
    temp = digitalRead(BTN3);
    if(temp == 1) {
      M2_stop();
      btn3_status = 0;
    }
  }
}

void M2_CCW_step() {
  int temp;
  M2_CCW();

  while(btn4_status) {
    temp = digitalRead(BTN4);
    if(temp == 1) {
      M2_stop();
      btn4_status = 0;
    }
  }
}


// Motor1 정방향 회전
void M1_CW() {
  // 1, 0

  // digitalWrite(DC1_1, HIGH);
  // PWM (펄스폭)
  //# analogWrite(DC1_1,m1_speed);
  //# digitalWrite(DC1_2, LOW);
  // 정방향, 0값 최대속도
  digitalWrite(DC1_1, HIGH);
  int speed = 255 - m1_speed;
  analogWrite(DC1_2,speed);
}

// Motor1 역방향 회전
void M1_CCW()
{
  // 0, 1
  digitalWrite(DC1_1, LOW); 
  //digitalWrite(DC1_2, HIGH);
  //역방향, 255가 최대 속도
  analogWrite(DC1_2,m1_speed);
}

void M2_CW() {
  // 1, 0
  //digitalWrite(DC2_1, HIGH);
  //digitalWrite(DC2_2, LOW);

  digitalWrite(DC2_2, HIGH);
  int speed = 255 - m2_speed;
  analogWrite(DC2_1,speed);

}


void M2_CCW()
{
  // 0, 1
  digitalWrite(DC2_2, LOW); 
  // digitalWrite(DC2_2, HIGH);
  analogWrite(DC2_1,m2_speed);
}

// Motor1 멈춤
void M1_stop()
{
  // 0,0 
  digitalWrite(DC1_1, LOW);
  digitalWrite(DC1_2, LOW);
}

void M2_stop()
{
  // 0,0 
  digitalWrite(DC2_1, LOW);
  digitalWrite(DC2_2, LOW);
}
