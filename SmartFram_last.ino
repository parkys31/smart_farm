//#include <ArduinoBLE.h>
#include <SoftwareSerial.h>    // 시리얼 통신 라이브러리
#include "DHT.h"               // 온습도 센서 라이브러리
#include "FastLED.h"           // led 라이브러리
#include <Servo.h>             // 서보모터 라이브러리
#include <LiquidCrystal_I2C.h> // lcd 라이브러리
#include <Wire.h>              // lcd 출력용 라이브러리

// 온습도센서
#define DHTPIN 7           // Signal핀과 연결된 디지털 핀 넘버
#define DHTTYPE DHT11      // DHT11 사용 명시
DHT dht(DHTPIN, DHTTYPE);  // 사용핀넘버 타입 등록
Servo window_motor;        // 서보모터 사용을 위한 객체생성
// 창문 핀
#define WINDOW_PIN 10
// 발열패드 온도센서
#define HEAT_DHT 9
DHT heat_dht(HEAT_DHT, DHTTYPE);

// 조도센서
#define LIGHTSENSOR A7

// LED 제어
#define LED_DATA1 4
#define LED_DATA2 5
#define LED_DATA3 6
// LED 하나당 광원갯수
const int NUM_LEDS = 24;
#define BRIGHTNESS 64
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define UPDATES_PER_SECOND 100
// Define the array of leds
CRGB leds[NUM_LEDS];
#define UPDATES_PER_SECOND 100

// LCD
LiquidCrystal_I2C lcd(0x27,20,4); // 접근주소: 0x3F or 0x27
int pirState = LOW;
int pir_val = 0;

//// 온습도센서 변수 선언
float hum = 0;
float temp = 0;
float lastHum = 0;
float lastTemp = 0;
float lastDustDensity = 0;
unsigned int lastPPMValue = 0;

//// 발열패드 온도
float heat_temp = 0;

// 적외선 센서
#define PIR 3

// 쿨링팬
#define FAN A0

// 물수위센서
#define WATERLEVEL A5
int waterVal = 0;         // 물수위데이터
int smVal = 0;            // 토양수분데이터
int smpercent =0 ;        // 토양수분퍼센티지

// 워터펌프 속도
#define WATERMOTOR A1

// 워터펌프 방향
#define WATERDIR 2

// 토양수분센서
#define SMOISTURE A2
String soil_grade = "";

//CO2센서
#define CO2_RXD 13
#define CO2_TXD 11
unsigned char Send_data[4] = {0x11, 0x01, 0x01, 0xED};
unsigned char Receive_Buff[8];
unsigned char recv_cnt = 0;
unsigned int PPM_Value = 0;
int old_temp = 0;
// SoftwareSerial BTSerial(BT_RXD, BT_TXD);
SoftwareSerial CO2Serial(CO2_RXD, CO2_TXD);

// 릴레이 제어
#define RELAY1 22 // 발열패드
#define RELAY2 23 // 쿨링팬
#define RELAY3 24 // 워터펌프


// 창문과 쿨링팬 제어를 위한 변수선언

// 서보모터 각도
int window_open = 0;
// 쿨링팬 속도
int MaxSpeed = 255;
// 온도 고정값 저장
float tempold = 26;
// 블루투스 통신 연결 성공여부
char c=""; 
// 버튼제어를 위한 메세지 저장 변수 (블루투스)
char bt="";
// 미세먼지 센서 핀 정의
const int dustPin = A11;  // 미세먼지 아날로그 핀 번호 sensor 5 pin
// 미세먼지 A4에서 읽어 오는 값 저장
float dustVal = 0;     
// 미세먼지 값 변환후 보여주는 값 
float dust=0;  
// 전압변환
float calcVoltage = 0;  
// 미세먼지 vled선 
const int dustLedPower = 30;  // 미세먼지 vled sensor 3 pin 
int delayTime = 280;
int delayTime2 = 40;
float offTime = 9680;
// 기능별 플래그
// LED플래그
int led_flag = 0;
// 창문 플래그
int window_flag = 0;
// 쿨링팬 플래그
int fan_flag = 0;
// 발열패드 릴레이플래그(RELAY1)
int heat_flag = 0;
// 워터모터(펌프) 플래그
int waterm_flag = 0;
// 강제 on/off flag
int all_flag = 0;


void setup() {
  
  Serial.begin(9600);  // PC에서 모니터링하기 위한 시리얼 통신 시작
  //BTSerial.begin(9600);
  Serial1.begin(9600); // 블루투스 시리얼 통신
  // 온습도센서
  dht.begin();
  heat_dht.begin();
  // 통신시작시
  Serial.println("Bluetooth initial");
  // CO2센서
  pinMode(CO2_TXD, OUTPUT);
  pinMode(CO2_RXD, INPUT);
  CO2Serial.begin(9600);
  // 미세먼지센서
  pinMode(dustLedPower, OUTPUT);                         
  // LED
  FastLED.addLeds<LED_TYPE, LED_DATA1, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.addLeds<LED_TYPE, LED_DATA2, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.addLeds<LED_TYPE, LED_DATA3, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  // 쿨링팬
  pinMode(FAN, OUTPUT);
  // 발열패드 릴레이
  pinMode(RELAY1, OUTPUT);
  // 쿨링팬1 릴레이
  pinMode(RELAY2, OUTPUT);
  // 워터펌프 릴레이
  pinMode(RELAY3, OUTPUT);
  // 워터펌프 방향
  pinMode(WATERDIR, OUTPUT); 
  pinMode(PIR, INPUT);
  // 창문
  // window_motor.attach(WINDOW_PIN);
  lcd.init();
}

void loop() {
  // 시리얼 통신 연결 여부 체크
  while (Serial.available()) {
    c = Serial.read();
    Serial1.print(bt);
    //Serial1.print(c);
  }
 
  // 블루투스 버튼 작동 스위치
  while (Serial1.available()) {
    bt = Serial1.read();
    switch(bt){
    // LED 버튼 제어
    case '1' : led_flag = 1;  break;    // LED ON
    case '2' : led_flag = 0;  break;    // LED OFF
    // 창문 버튼 제어
    case '3' : window_flag = 1; break;  // 창문 ON
    case '4' : window_flag = 0; break;  // 창문 OFF
    // 쿨링팬 버튼 제어
    case '5' : fan_flag = 1;  break;   // 쿨링팬 ON
    case '6' : fan_flag = 0;  break;   // 쿨링팬 OFF
    // 창문&팬 제어
    case '7' : window_flag = 1; fan_flag = 1; break; // 창문 팬 ON
    case '8' : window_flag = 0; fan_flag = 0; break; // 창문 팬 OFF
    // 발열패드 버튼 제어   
    case '9' : heat_flag = 1; break;  // 발열패드 ON
    case 'o' : heat_flag = 0; break;  // 발열패드 OFF
    //워터펌프(모터)제어
    case 'm' : waterm_flag = 1;  break;  // ON
    case 'n' : waterm_flag = 0;  break;  // OFF
    //강제 컨트롤
    case 'q' : all_flag = 1; break; // 수동제어 ON
    case 'w' : all_flag = 0; break; // 자동제어 ON
    }
  }

  // CO2 센서
  Send_CMD();
  while(CO2Serial.available()){
    Receive_Buff[recv_cnt++] = CO2Serial.read();
    if(recv_cnt == 8){
      recv_cnt = 0;
      break;
    }
  }
  if (Checksum_cal() == Receive_Buff[7]){
    PPM_Value = Receive_Buff[3] << 8 | Receive_Buff[4];
    Serial.write("PPM : ");
    Serial.println(PPM_Value);
  }
  else {
    Serial.write("CHECKSUM Error");
  }

  // 물공급을 위한 변수선언
  waterVal = analogRead(WATERLEVEL);         // 물수위데이터
  smVal = analogRead(SMOISTURE);             // 토양수분데이터
  smpercent = map(smVal, 670, 210, 100, 0); // 토양수분퍼센티지
  if(smVal < 200) {
    soil_grade = "부족";
  }
  else if(smVal >= 500){
    soil_grade = "적정";
  }
  Serial.println(c);

  //실내 온습도
  hum = dht.readHumidity();     // 습도 읽기
  temp = dht.readTemperature(); // 온도 읽기
  heat_temp = heat_dht.readTemperature();
  /*if(isnan(hum) || isnan(temp))       // isnan 숫자 이외에 것들 true, 숫자 false 값 함수 like 예외처리
  {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  */
  delay(10);
  // 미세먼지센서를 위한 변수 선언
  digitalWrite(dustLedPower, LOW); // power on the LED
  delayMicroseconds(delayTime);

  dustVal=analogRead(dustPin); // read the dust value via pin 5 on the sensor
  calcVoltage = dustVal * (5.0 / 1024); 
  
  delayMicroseconds(delayTime2);

  digitalWrite(dustLedPower, HIGH); // turn the LED off
  delayMicroseconds(offTime);

  delay(1000);

  
  dust = (0.17 * calcVoltage - 0.1)*1000; 
  Serial.print("Dust Density [ug/m3]: ");
  Serial.println(dust);

  // 현재 온도값 저장
  if (temp > 27.00){
    tempold = temp;
  }
  else if (temp < 25.00){
    tempold = temp;
  }

   // 적외선 센서가 사람을 인식할 경우 lcd화면을 키고 끄는 조건
  pir_val = digitalRead(PIR);
  if(pir_val == HIGH) {
    lcd.backlight();
    if(pirState == LOW){
      //Serial.println("Welcome!");
      pirState = HIGH;
    }
  }
  else {
    lcd.noBacklight();
    if(pirState == HIGH){
      //Serial.println("Welcome!");
      pirState = LOW;
    }
  }

  // LCD 화면 출력
  //if (hum != lastHum || temp != lastTemp || PPMValue != lastPPMValue || dustDensity != lastDustDensity){
	  printLcd();
  //}
  // 결과 시리얼 출력
  
  Serial.print("토양습도 : ");
  Serial.print(soil_grade);
  Serial.print("%\t");
  Serial.print("물수위 : ");
  Serial.print(waterVal);
  Serial.print("ml\t");
  Serial.print("습도 : ");
  Serial.print(lastHum);
  Serial.print("%\t");
  Serial.print("온도 : ");
  Serial.print(lastTemp);
  Serial.print("C\t");
  Serial.print("미세먼지 : ");
  Serial.print(dust);
  Serial.print("ug/m3\t");
  Serial.print("CO2 : ");
  Serial.println(PPM_Value);
  Serial.println("{\"hum\":"+String(hum)+",\"temp\":"+String(temp)+",\"dust\":"+String(dust)+",\"CO2\":"+String(PPM_Value)+",\water\":"+String(waterVal)+",\"soil\":"+(soil_grade)+"}");
  //Serial.println(" ug/m3");
  
  // 블루투스 앱 인벤터 화면 출력용 데이터 송신
  //"{\"hum\":" = {"hum":
  Serial1.print("{\"hum\":"+String(hum)+"}");
  delay(100);
  Serial1.print("{\"temp\":"+String(temp)+"}");
  delay(100);
  Serial1.print("{\"dust\":"+String(dust)+"}");
  delay(100);
  Serial1.print("{\"co2\":"+String(PPM_Value)+"}");
  delay(100);
  Serial1.print("{\"water\":"+String(waterVal)+"}");
  delay(100);
  Serial1.print("{\"smoisture\":"+soil_grade+"}");
  delay(100);
  Serial1.print("{\"all_flag\":"+String(all_flag)+"}");
  delay(100);
  Serial1.print("{\"led_flag\":"+String(led_flag)+"}");
  delay(100);
  Serial1.print("{\"window_flag\":"+String(window_flag)+"}");
  delay(100);
  Serial1.print("{\"fan_flag\":"+String(fan_flag)+"}");
  delay(100);
  Serial1.print("{\"heat_flag\":"+String(heat_flag)+"}");
  delay(100);
  Serial1.print("{\"waterm_flag\":"+String(waterm_flag)+"}");
  delay(200);

  // 농장 자동화&수동화
  if (all_flag == 0){
    auto_farm();
  } else {
    manual_farm();
  }

}

// 미세먼지 함수
/*float pulse2ugm3(unsigned long pulse) {
  float value = (pulse - 1400) / 14.0;                // pulse에 -1400을 하고 14.0을 나누어 value에 저장합니다.
  if (value > 300) {                                  // value가 300보다 크면
    value = 0;                                        // value이 0으로 저장합니다.
  }
  return value;                                       // value 값을 반환합니다.
}
*/
//co2함수
void Send_CMD(void){
  unsigned int i;
  for (i = 0; i < 4; i++){
    CO2Serial.write(Send_data[i]);
    delay(1);
  }
}

//co2함수
unsigned char Checksum_cal(void){
  unsigned char count, SUM = 0;
  for (count = 0; count < 7; count++){
    SUM += Receive_Buff[count];
  }
  return 256 - SUM;
}

// led 출력 함수
void show_led(int R, int G, int B){
  for (int i = 0; i < NUM_LEDS; i++){
      leds[i] = CRGB(R,G,B);
  }
  FastLED.show();
}

// LCD 화면 출력 함수
void printLcd(){
  lastTemp = temp;
  lastHum = hum;
  lastPPMValue = PPM_Value;
  lastDustDensity = dust;
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("HUM  : ");
  lcd.print(hum);
  lcd.print("%");

  lcd.setCursor(0, 1);
  lcd.print("TEMP : ");
  lcd.print(temp);
  lcd.print("C");

  lcd.setCursor(0, 2);
  lcd.print("Co2  : ");
  lcd.print(lastPPMValue);
  lcd.print("ppm");

  lcd.setCursor(0, 3);
  lcd.print("Dust : ");
  lcd.print(lastDustDensity);
  lcd.print("ug/m3");
}

// 자동 제어
void auto_farm() {
    Serial.println("자동 제어 ON");
  // 미세먼지 센서 조건부 팬 기동
  // 창문 팬 제어
  if(dust >= 100 || tempold > 27.00){
    if (window_flag != 1) {
      window_flag = 1;
      fan_flag = 1;
      Serial.println("창문 제어 ON");
      digitalWrite(RELAY2, LOW);
      digitalWrite(FAN, HIGH);
      window_motor.attach(WINDOW_PIN);
      for (window_open = window_open; window_open <= 160; window_open += 1){
        window_motor.write(window_open);
        Serial.println(window_open);
        delay(15);

      }
      window_motor.detach();
    }
  }
  else {
    if (window_flag != 0){
      window_flag = 0;
      fan_flag = 0;
      Serial.println("창문 제어 OFF");
      digitalWrite(RELAY2, HIGH);
      digitalWrite(FAN,LOW);
      window_motor.attach(WINDOW_PIN);
      for (window_open = window_open; window_open >= 0; window_open -= 1){
        window_motor.write(window_open);
        delay(15);
      }
      window_motor.detach();
    }
   
  }

  // LED 제어
  int lightVal = analogRead(LIGHTSENSOR); // 조도센서 데이터 저장
  //조도센서 값이 100미만이면 LED를 켠다
  if(lightVal > 300) {
    led_flag = 1;
    Serial.println("LED 제어 ON");
    show_led(255,105,180);
  } 
  else if (lightVal < 300 ) {
    led_flag = 0;
    Serial.println("LED 제어 OFF");
    show_led(0,0,0);
  }

  // 워터펌프제어
  if (smVal < 200){
    waterm_flag = 1;
    Serial.println("워터펌프 제어 ON");
    //정방향으로 100/255의 속도로 회전
    // 워터펌프 릴레이 ON
    digitalWrite(RELAY3, HIGH);
    digitalWrite(WATERDIR, HIGH);   // Motor A 방향설정 (정방향)
    analogWrite(WATERMOTOR, 200);     // Motor A 속도조절 (0~255)
    delay(3000);
    digitalWrite(WATERDIR, LOW);
    analogWrite(WATERMOTOR, 0);
    delay(100);
  }
  else {
    waterm_flag = 0;
    Serial.println("워터펌프 제어 OFF");
    // 워터펌프 릴레이 OFF
    digitalWrite(RELAY3, LOW);
    analogWrite(WATERMOTOR, 0);
    digitalWrite(WATERDIR, LOW);
  } 

  // 발열패드 릴레이 제어
  // 발열패드 ON
  if (tempold < 21.00 && heat_temp < 25.00){
    heat_flag = 1;
    Serial.println("발열패드 제어 ON");
    digitalWrite(RELAY1, LOW);
  }
  // 발열패드 OFF
  else if (tempold > 27.00 || heat_temp > 30.00) {
    heat_flag = 0;
    Serial.println("발열패드 제어 OFF");
    digitalWrite(RELAY1, HIGH);
  }
}

// 수동 제어
void manual_farm() {
    Serial.println("수동 제어 ON");
  // 창문 제어
  if (window_flag == 1){
    Serial.println("창문 제어 ON");
    window_motor.attach(WINDOW_PIN);
    for (window_open = window_open; window_open < 160; window_open += 1){
      window_motor.write(window_open);
      delay(15);
    }
    window_motor.detach();
  }else if (window_flag == 0) {
    Serial.println("창문 제어 OFF");
    window_motor.attach(WINDOW_PIN);
    for (window_open = window_open; window_open > 0; window_open -= 1){
      window_motor.write(window_open);
      delay(15);
    }
    window_motor.detach();
  }
  // 팬 제어
  if (fan_flag == 1){
    Serial.println("팬 제어 ON");
    digitalWrite(RELAY2, LOW);
    digitalWrite(FAN, HIGH);
  }
  else if (fan_flag == 0){
    Serial.println("팬 제어 OFF");
    digitalWrite(RELAY2, HIGH);
    digitalWrite(FAN,LOW);
  }
  // led 작동
  if(led_flag == 1) {
    Serial.println("LED 제어 ON");
    show_led(255,105,180);
  } 
  else if (led_flag == 0) {
    Serial.println("LED 제어 OFF");
    show_led(0,0,0);
  }
  
  

  // 워터펌프제어
  if (waterm_flag == 1){
    if (smVal > 200){
      waterm_flag = 0;
    }
    Serial.println("워터펌프 제어 ON");
    //정방향으로 100/255의 속도로 회전
    // 워터펌프 릴레이 ON
    digitalWrite(RELAY3, HIGH);
    digitalWrite(WATERDIR, HIGH);   // Motor A 방향설정 (정방향)
    analogWrite(WATERMOTOR, 200);     // Motor A 속도조절 (0~255)
    delay(3000);
    digitalWrite(WATERDIR, LOW);
    analogWrite(WATERMOTOR, 0);
    delay(100);
  }
  else if (smVal < 500){
    Serial.println("워터펌프 제어 OFF");
    // 워터펌프 릴레이 OFF
    digitalWrite(RELAY3, LOW);
    analogWrite(WATERMOTOR, 0);
    digitalWrite(WATERDIR, LOW);
  }

  // 발열패드 릴레이 제어
  // 발열패드 ON
  if (heat_flag == 1){
    Serial.println("발열패드 제어 ON");
    digitalWrite(RELAY1, LOW);
  }
  // 발열패드 OFF
  else if (heat_flag == 0) {
    Serial.println("발열패드 제어 OFF");
    digitalWrite(RELAY1, HIGH);
  }
}