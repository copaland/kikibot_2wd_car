# kikibot_2wd_car

## 무선 조종 방법

#### 1. 컴파일러 다운로드 설치
[kidsblock 다운로드](https://wiki.kidsbits.cc/projects/KidsBlock/en/latest/download/)

KiKiBot_Block 폴더의 코드 다운로드  
PJ_8 초음파 센서 장애물 회피 주행  
PJ_13 WIFI 앱 조종  

[PJ_13_WiFi_Motor_Control.sb3](https://github.com/copaland/kikibot_2wd_car/blob/main/KiKiBot_Block/PJ_13_WiFi_Motor_Control.sb3)  

[구글 플레이 스토어 앱 다운로드](https://play.google.com/store/apps/details?id=com.keyestudio.beetlecar&hl=en)

[애플 앱스토어 앱 다운로드](https://apps.apple.com/ee/app/beetlebot/id1601167393)

<img src="https://github.com/copaland/kikibot_2wd_car/blob/main/mit_app/kikibot-img.png" title="2WD CAR CODING ROBOT" alt="KIKIBOT"></img><br/>
<img src="https://github.com/copaland/kikibot_2wd_car/blob/main/mit_app/esp32_kikibot2.jpg" title="2WD CAR CODING ROBOT" alt="KIKIBOT"></img><br/>

const int MOTOR1_PWM_PIN = 25;        // Motor 1 PWM  
const int MOTOR1_DIR_PIN = 33;        // Motor 1 Direction  
const int MOTOR1_EN_PIN = 32;         // Motor 1 Enable  

const int MOTOR2_PWM_PIN = 14;        // Motor 2 PWM   
const int MOTOR2_DIR_PIN = 27;        // Motor 2 Direction  
const int MOTOR2_EN_PIN = 26;         // Motor 2 Enable  

const int LED_LEFT_PIN = 4;           // Left LED  
const int LED_RIGHT_PIN = 13;         // Right LED  
const int BUTTON_PIN = 5;             // Push Button Pullup  
