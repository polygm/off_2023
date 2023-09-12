/*
 * DC 모듈 코드
 * version 0.0.1
*/
#include "PinChangeInterrupt.h" // 라이브러리 추가 설치 필요

#define BeepPin 12 // 부저

// L298n 모터연결
#define DC1_1 4 // IN1
#define DC1_2 5 // IN2
#define DC2_1 6 // IN3
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
  Serial.print(temp);

  if(temp == 0) {
    // 토글
    if(btn1_status== 1) {
      btn1_status = 0;
    } else {
      btn1_status = 1;
    }
    Serial.println("btn1 pressed");
  }
}

int btn2_status = 0;
void button2_press() {
  int temp = digitalRead(BTN2);
  if(temp == 0) {
    // 토글
    if(btn2_status== 1) {
      btn2_status = 0;
    } else {
      btn2_status = 1;
    }
    Serial.println("btn2 pressed");
  }
}

int btn3_status = 0;
void button3_press() {
  int temp = digitalRead(BTN3);
  if(temp == 0) {
    // 토글
    if(btn3_status== 1) {
      btn3_status = 0;
    } else {
      btn3_status = 1;
    }
    Serial.println("btn3 pressed");
  }
}

int btn4_status = 0;
void button4_press() {
  int temp = digitalRead(BTN4);
  if(temp == 0) {
    // 토글
    if(btn4_status== 1) {
      btn4_status = 0;
    } else {
      btn4_status = 1;
    }
    Serial.println("btn4 pressed");
  }
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

void loop() {
  builtin_led_blank(500); // 0.5초 깜빡임
  //Serial.print("btn1=");
  //Serial.print(btn1_status);
  //Serial.print("btn2=");
  //Serial.println(btn2_status);


  if(btn1_status == 1) {
    Serial.println("Motor1_CW");
    M1_stop();
    M1_CW();
  } else {
    if(btn2_status == 0) {
      Serial.println("Motor1_STOP");
      M1_stop();
    }
  }


  if(btn2_status == 1) {
    Serial.println("Motor1_CCW");
    M1_stop();
    M1_CCW();
  } else {
    if(btn1_status == 0) {
      Serial.println("Motor1_STOP");
      M1_stop();
    }
  }

  
  if(btn3_status == 1) {
    Serial.println("Motor2_CW");
    M2_stop();
    M2_CW();
  } else {
    if(btn4_status == 0) {
      Serial.println("Motor2_STOP");
      M2_stop();
    }
  }


  if(btn4_status == 1) {
    Serial.println("Motor2_CCW");
    M2_stop();
    M2_CCW();
  } else {
    if(btn3_status == 0) {
      Serial.println("Motor2_STOP");
      M2_stop();
    }
  }






}

// Motor1 정방향 회전
void M1_CW() {
  // 1, 0
  digitalWrite(DC1_1, HIGH);
  digitalWrite(DC1_2, LOW);
}

void M2_CW() {
  // 1, 0
  digitalWrite(DC2_1, HIGH);
  digitalWrite(DC2_2, LOW);
}

// Motor1 역방향 회전
void M1_CCW()
{
  // 0, 1
  digitalWrite(DC1_1, LOW); 
  digitalWrite(DC1_2, HIGH);
}

void M2_CCW()
{
  // 0, 1
  digitalWrite(DC2_1, LOW); 
  digitalWrite(DC2_2, HIGH);
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
