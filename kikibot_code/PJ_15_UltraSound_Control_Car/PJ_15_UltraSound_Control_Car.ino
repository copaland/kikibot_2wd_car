// ESP32 WROOM-32E 2WD 자동차 초음파 센서 제어 코드 (개선된 버전)
// 하드웨어: ESP32, DRV8837 모터드라이버
// LED: 좌측 4번, 우측 13번 핀
// 모터 Enable: 32번, 26번 핀

#include <Arduino.h>

// 핀 정의
#define TRIG_PIN 18        // 초음파 센서 트리거 핀
#define ECHO_PIN 19        // 초음파 센서 에코 핀

#define MOTOR1_DIR 33      // 모터1 방향 제어
#define MOTOR1_PWM 25      // 모터1 속도 제어 (PWM)
#define MOTOR2_DIR 27      // 모터2 방향 제어  
#define MOTOR2_PWM 14      // 모터2 속도 제어 (PWM)

#define MOTOR1_EN 32       // 모터1 활성화
#define MOTOR2_EN 26       // 모터2 활성화

#define LED_LEFT 4         // 좌측 LED
#define LED_RIGHT 13       // 우측 LED

// PWM 채널 정의
#define PWM_CHANNEL_1 1
#define PWM_CHANNEL_2 2
#define PWM_FREQ 1200
#define PWM_RESOLUTION 8

// 제어 변수
#define OBSTACLE_THRESHOLD 20    // 장애물 감지 거리 (cm)
#define RECHECK_THRESHOLD 25     // 재확인 거리 (cm)
#define NORMAL_SPEED 220         // 일반 주행 속도
#define TURN_SPEED_HIGH 220      // 회전 시 빠른 바퀴 속도
#define TURN_SPEED_LOW 50        // 회전 시 느린 바퀴 속도

long distance = 0;
long recheckDistance = 0;

// 초음파 센서 거리 측정 함수
float checkDistance() {
  // 트리거 핀을 LOW로 초기화
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // 10마이크로초 동안 HIGH 신호 전송
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // 에코 핀에서 신호 수신 시간 측정
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms 타임아웃
  
  // 거리 계산 (음속: 343m/s, 왕복 시간이므로 /2)
  float calculatedDistance = duration * 0.034 / 2;
  
  // 센서 오류 처리 (0 또는 비정상적으로 큰 값)
  if (duration == 0 || calculatedDistance > 400) {
    return -1; // 오류 시 -1 반환
  }
  
  delay(50); // 센서 안정화 대기
  return calculatedDistance;
}

// 모터 제어 함수들
void setMotorSpeed(int motor1Speed, int motor2Speed, bool motor1Dir, bool motor2Dir) {
  // 모터 방향 설정
  digitalWrite(MOTOR1_DIR, motor1Dir);
  digitalWrite(MOTOR2_DIR, motor2Dir);
  
  // 모터 속도 설정
  ledcWrite(PWM_CHANNEL_1, motor1Speed);
  ledcWrite(PWM_CHANNEL_2, motor2Speed);
}

void stopMotors() {
  setMotorSpeed(0, 0, LOW, LOW);
  
  // LED 끄기
  digitalWrite(LED_LEFT, LOW);
  digitalWrite(LED_RIGHT, LOW);
}

void moveForward(int speed = NORMAL_SPEED) {
  setMotorSpeed(speed, speed, LOW, LOW);
  
  // 전진 시 양쪽 LED 켜기
  digitalWrite(LED_LEFT, HIGH);
  digitalWrite(LED_RIGHT, HIGH);
}

void moveBackward(int speed = NORMAL_SPEED) {
  setMotorSpeed(speed, speed, HIGH, HIGH);
  
  // 후진 시 양쪽 LED 켜기
  digitalWrite(LED_LEFT, HIGH);
  digitalWrite(LED_RIGHT, HIGH);
}

void turnRight() {
  // 우회전: 좌측 모터 빠르게, 우측 모터 느리게
  setMotorSpeed(TURN_SPEED_HIGH, TURN_SPEED_LOW, LOW, HIGH);
  
  // 우회전 시 우측 LED만 켜기
  digitalWrite(LED_LEFT, LOW);
  digitalWrite(LED_RIGHT, HIGH);
}

void turnLeft() {
  // 좌회전: 우측 모터 빠르게, 좌측 모터 느리게
  setMotorSpeed(TURN_SPEED_LOW, TURN_SPEED_HIGH, HIGH, LOW);
  
  // 좌회전 시 좌측 LED만 켜기
  digitalWrite(LED_LEFT, HIGH);
  digitalWrite(LED_RIGHT, LOW);
}

void spinRight(int speed = TURN_SPEED_HIGH) {
  // 제자리 우회전: 좌측 전진, 우측 후진
  setMotorSpeed(speed, speed, LOW, HIGH);
  
  // 제자리 회전 시 우측 LED만 켜기
  digitalWrite(LED_LEFT, LOW);
  digitalWrite(LED_RIGHT, HIGH);
}

void spinLeft(int speed = TURN_SPEED_HIGH) {
  // 제자리 좌회전: 좌측 후진, 우측 전진
  setMotorSpeed(speed, speed, HIGH, LOW);
  
  // 제자리 회전 시 좌측 LED만 켜기
  digitalWrite(LED_LEFT, HIGH);
  digitalWrite(LED_RIGHT, LOW);
}

void setup() {
  // 시리얼 통신 초기화 (디버깅용)
  Serial.begin(115200);
  Serial.println("ESP32 2WD 자동차 초음파 센서 제어 시작");
  
  // 초음파 센서 핀 설정
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // 모터 제어 핀 설정
  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR2_DIR, OUTPUT);
  
  // PWM 설정
  ledcSetup(PWM_CHANNEL_1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR1_PWM, PWM_CHANNEL_1);
  
  ledcSetup(PWM_CHANNEL_2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR2_PWM, PWM_CHANNEL_2);
  
  // 모터 활성화 핀 설정
  pinMode(MOTOR1_EN, OUTPUT);
  pinMode(MOTOR2_EN, OUTPUT);
  digitalWrite(MOTOR1_EN, HIGH);
  digitalWrite(MOTOR2_EN, HIGH);
  
  // LED 핀 설정
  pinMode(LED_LEFT, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);
  
  // 초기화 완료 표시 (LED 깜빡임)
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_LEFT, HIGH);
    digitalWrite(LED_RIGHT, HIGH);
    delay(200);
    digitalWrite(LED_LEFT, LOW);
    digitalWrite(LED_RIGHT, LOW);
    delay(200);
  }
  
  Serial.println("초기화 완료");
}

void loop() {
  // 거리 측정
  distance = checkDistance();
  
  // 센서 오류 처리
  if (distance < 0) {
    Serial.println("센서 오류 - 정지");
    stopMotors();
    delay(500);
    return;
  }
  
  Serial.print("측정 거리: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  // 장애물 감지 로직
  if (distance < OBSTACLE_THRESHOLD) {
    Serial.println("장애물 감지 - 정지 후 회피");
    
    // 일시 정지
    stopMotors();
    delay(300);
    
    // 거리 재확인
    recheckDistance = checkDistance();
    Serial.print("재확인 거리: ");
    Serial.print(recheckDistance);
    Serial.println(" cm");
    
    if (recheckDistance > 0 && recheckDistance < RECHECK_THRESHOLD) {
      Serial.println("회피 기동 시작 - 우회전");
      
      // 우회전으로 장애물 회피
      turnRight();
      delay(800); // 회전 시간 증가
      
      // 잠시 전진 후 다시 확인
      moveForward();
      delay(500);
    } else {
      Serial.println("장애물 해제 - 전진 재개");
    }
  } else {
    // 정상 전진
    Serial.println("전진");
    moveForward();
  }
  
  // 루프 지연
  delay(100);
}
