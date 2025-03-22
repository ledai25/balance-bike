#include <Wire.h>
#include <SoftwareSerial.h>

#define MPU6050  0x68         

SoftwareSerial JDY_31 (6, 5); //RX-->D5, TX-->D6 Giao tiếp với module Bluetooth

// PID 
float Kp = 178.0;//170.0 
float Ki = 680.0;//3.0 680.0
float Kd = 1.8;//1.7
float f = 0.2;// 0.2
float gcb = 0.0050;// 0.0050

int pid_output = 0;                 // Đầu ra pid
volatile int pwm = 330;             
int l = 0, r = 0; 
int dir = 11;                       // Điều khiển hướng tiến - lùi DC
int activate = 4;                   // Chân kích hoạt động cơ
int cw_ccw  = 8;                    // Điều khiển hướng quay BLDC
int servoPin = 3;                   // Chân servo
int pwmNidec = 9;                   // pwm Nidec
int pwmDCMotor = 10;                // pwm Motor DC
int count = 0; 
int PPR = 100;// 100 xung/vòng
int countInterrupt = 0;
int prevCount = 0;
const int encA = 2;
const int encB = 7;

float error;
float setpoint = 0;
float integral = 0.0;
float pid_input = 0; 
float derivative;    
float prev_error = 0.0;
float loop_timer = 6000; // 6ms
float pwm_motor = 0.0;           

float Angle, angle_accX, angleFilt = 0.0;
float dt = 0.0;
float tcb = 0;
int16_t gyro_Z; 
float gyroZFilt = 0;
float alpha = 0.37;
float al;
int32_t motor_speed = 0.0;

//int32_t motor_speed; 
volatile int dem = 12; // giá trị giữa cho servo
volatile int demInterrupt = 0;

unsigned long lastTimer = 0, prevTimer = 0;
unsigned long stTimer;

int16_t accX, accY, accZ, gyroY, gyroZ;
int16_t GyY_offset = 0;
int16_t GyZ_offset = 0;
int32_t GyY_offset_sum = 0;
int32_t GyZ_offset_sum = 0;
int32_t accX_sum = 0, accY_sum = 0;
int16_t accX_error, accY_error, gyro_errorY, gyro_errorZ;
int16_t accX_offset = 0, accY_offset = 0;
   
bool exe = false, goc_2 = false, sta = false;
char Blt;

void setup() {    
  Serial.begin(115200);   
  JDY_31.begin(9600);

  // Tạo xung pwm 30Khz cho động cơ Nidec
  // reset thanh ghi
  TCCR1A = 0;
  TCCR1B = 0;

  // Fast PWM, TOP = ICR1
  TCCR1A |= (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11);
  TCCR1B |= (1<<WGM13) | (1<<WGM12) | (1<<CS10); // prescaler = 1

  OCR1A = 510; // giá trị ban đầu
  OCR1B = 0;

  ICR1 = 532; // 799 = 20khz  532 = 30khz
  // Công thức tính TOP = (16,000,000/(prescaler * tần số mong muốn)) - 1
  // Công thức tính tần số = 16,000,000/(prescaler * (TOP + 1))
    
  // Sử dụng Timer 2 để tạo xung pwm cho servo 
  TCCR2A = 0;
  TCCR2B = 0;
  // Chế độ Phase-Correct PWM, TOP = OCR2A
  TCCR2A |= (1<< COM2B1) | (1<<WGM20);
  TCCR2B |= (1<< WGM22) | (1<<CS22) | (1<<CS21) | (1<<CS20);// Prescaler = 1024

  OCR2A = 157; ///  Tính giá trị TOP = 0.02 * 16000000 / (2 * 1024) = 156  (0.02s = 20ms) Tần số 50hz  
  OCR2B = 12; // giá trị ban đầu (ở giữa)

  // TOP = 156 sẽ cho tần số chính xác 50hz, chu kỳ 19998us
  // TOP = 157 sẽ cho chu kỳ chính xác 20us, tần số 49.78hz

  // Tính tần số   1 / 0,02 = 50  và 1 là 1 giây hoặc 1000 / 20 = 50              

  pinMode(activate, OUTPUT);
  pinMode(cw_ccw, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(servoPin, OUTPUT);
  pinMode(pwmNidec, OUTPUT);
  pinMode(pwmDCMotor, OUTPUT);
  pinMode(encA, INPUT);
  pinMode(encB, INPUT);
  pinMode(13, OUTPUT);

  //Thiết lập ngắt   
  attachInterrupt(digitalPinToInterrupt(encA), ai0, RISING);                      

  digitalWrite(activate, LOW);
    
  delay(100);
  Wire.begin();
      
  Wire.beginTransmission(MPU6050);
  Wire.write(0x6B);                   // 0x6B - địa chỉ thanh ghi nguồn
  Wire.write(0);
  Wire.endTransmission(true);
     
  ///////////Chỉ định tỉ lệ đầu ra/////////// 
  //Chia tỷ lệ đầu ra cảm biến      
  // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
  // 0 = 250rad/s, 1 = 500rad/s, 2 = 1000rad/s, 3 = 2000rad/s   
  Wire.beginTransmission(MPU6050);
  Wire.write(0x1C);                    // 0x1C - địa chỉ cảm biến gia tốc 
  Wire.write(0 << 3); // 2g
  Wire.endTransmission(true);
    
  Wire.beginTransmission(MPU6050);
  Wire.write(0x1B);                    // 0x1B - địa chỉ con quay hồi chuyển 
  Wire.write(1 << 3); // 500rad/s
  Wire.endTransmission(true);   

  // Tính lỗi IMU   
  for(int i=0; i < 700; i++) {    
  count++; 
  Wire.beginTransmission(MPU6050);                                        
  Wire.write(0x47);                                                       
  Wire.endTransmission();                                                        
  Wire.requestFrom(0x68, 2, true); 
               
  gyro_errorZ = Wire.read() << 8 | Wire.read();       //Kết hợp hai byte để tạo thành một số nguyên

  GyZ_offset_sum += (gyro_errorZ / 65.5);

  if(count == 700) sta = true;
  delay(4); 
  }/*
  // hãy đặt MPU6050 nằm ngang với mặt bàn để lấy giá trị đúng!
  if (sta) {
    for (int f=0; f < 700; f++) {
      Wire.beginTransmission(MPU6050);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU6050, 4, true);

      accX_error = Wire.read() << 8 | Wire.read();
      accY_error = Wire.read() << 8 | Wire.read();

      accX_sum += accX_error;
      accY_sum += accY_error;
  
      delay(4);
    }
  }
  accX_offset = accX_sum / 700;
  accY_offset = accY_sum / 700;*/
  GyZ_offset = GyZ_offset_sum / 700;

  digitalWrite(13, HIGH);
  delay(150);
  digitalWrite(13,LOW);
  delay(150);
  digitalWrite(13, HIGH);
  delay(150);
  digitalWrite(13,LOW);
  // in giá trị lỗi lên Serial
  //Serial.print(accX_offset);
  //Serial.print(",");
  //Serial.print(accY_offset);
  //Serial.print(",");
  Serial.println(GyZ_offset);
  delay(300);  
}
void loop() {
  stTimer = micros();
  if(stTimer - lastTimer >= loop_timer) {
  dt = (float)(loop_timer / 1.0e6);
  noInterrupts();
  countInterrupt = demInterrupt;
  interrupts();

  // Tính RPM
  //float angular_velocity_Filter = alpha * angular_velocity + (1 - alpha) * angular_velocity_Filter;
  float vci = (countInterrupt - prevCount) / dt;
  prevCount = countInterrupt;
  float rpm = (float)(vci/PPR)*60.0;// Tính RPM
  // Tính vận tốc góc
  float angular_velocity = (float)(2 * PI * rpm) / 60;// đơn vị radian/s
  //float angular_velocity_rad_to_deg = angular_velocity * 180 / PI;// đơn vị độ/s
  DKH();         
  angleMPU();      
  if(exe) 
  {            
    digitalWrite(activate, HIGH); // Kích hoạt động cơ

    //angleFilt = alpha * Angle + (1-alpha) * angleFilt;

    error = setpoint - tcb - Angle;

    integral += error * dt;
    //integral = constrain(integral, -60, 60);
  
    derivative = (error - prev_error) / dt;       
  
    pid_output = constrain(Kp * error + Kd * derivative + Ki * integral + f * motor_speed, -510, 510);
    motor_speed += angular_velocity;
    prev_error = error;

    if(pid_output < 8 && pid_output > -8)pid_output = 0;
  
    /*if(setpoint == 0) {
      if(pid_output < 0) tcb += gcb;
      if(pid_output > 0) tcb -= gcb;
    }*/

    if(pid_output < 0) {
      tcb += gcb;
      if(tcb == 3)
      {tcb = 3;}
    }
    if(pid_output > 0) {
      tcb -= gcb;
      if(tcb == -3)
      {tcb = -3;}
    }
    Motor_control(pid_output);
    //Serial.print(setpoint);
    //Serial.print(",");
    //Serial.println(Angle);
                                   
  }else {
    pid_output = 0;
    integral = 0.0;
    prev_error = 0;
    motor_speed = 0;
    tcb = 0.0;
    Motor_control(0); 
    digitalWrite(activate, LOW); 
  }
  lastTimer = stTimer;
  }  
}

void Motor_control(int pwm) {
  if (pwm <= 0) {
    digitalWrite(cw_ccw, LOW);  
    pwm = abs(pwm);       
  } else {
    digitalWrite(cw_ccw, HIGH);  
  }
  OCR1A = map(pwm, 0, 510, 510, 0);
}

void stop() {
  digitalWrite(dir, LOW);
  OCR1B = 0;
}
void phiatruoc(volatile int pwm) {
  digitalWrite(dir, HIGH);
  OCR1B = pwm; 
}
void  phiasau(volatile int pwm) {
  digitalWrite(dir, LOW);
  OCR1B = pwm;
}

void DKH() { // Điều Khiển Hướng
  if (JDY_31.available() > 0){
    Blt = JDY_31.read();
    switch (Blt) { 
        case 'F':
            phiatruoc(pwm);
            break;
        case 'B':
            phiasau(pwm);
            break;
        case 'R':
            l = 1;
            break;
        case 'L':
            r = 1;
            break;
        case 'S':
            //pwm = 0;
            l = 0;
            r = 0;
            stop();
            break;
// chỉnh tốc độ
        case '1':
            pwm = 0; //0 += 64
            break;
        case '2':
            pwm = 300;   
            break;
        case '3':
            pwm = 338;
            break;
        case '4':
            pwm = 379;
            break;
        case '5':
            pwm = 415;    
            break;
        case '6':
            pwm = 440;
            break;
        case '7':
            pwm = 470;
            break;
        case '8':
            pwm = 500;
            break;
        case '9':
            pwm = 530;
            break;
            /*
        case 'q':
            pwm = 255;
            break;*/
    }
  }

  if (l == 1) {
    (stTimer - prevTimer >= 1e5) ? dem++,prevTimer = stTimer : dem = dem;
  }
  if (r == 1) {
    (stTimer - prevTimer >= 1e5) ? dem--,prevTimer = stTimer : dem = dem;
  }

  dem = constrain(dem, 4, 20);

  OCR2B = dem;
}
void angleMPU() { // Tính toán góc
  
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);
    
  accX = Wire.read() << 8 | Wire.read();
  accY = Wire.read() << 8 | Wire.read();
  accZ = Wire.read() << 8 | Wire.read();
    
  Wire.beginTransmission(MPU6050);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true); 
     
  gyroZ = (Wire.read() << 8 | Wire.read()) / 65.5;

  // Bị trễ giư liệu góc là do viết như này AcX -= accX_offset; AcY -= accY_offset;
  // loop_timer có ảnh hưởng đến độ trễ góc
  
  accX -= 1730;//1730
  accY -= -394;//-394
  gyroZ -= GyZ_offset;
    
  Angle += gyroZ * dt;

  angle_accX = atan2(accY, -accX) * 180 / PI; // Tính góc gia tốc

  Angle = Angle * 0.996 + angle_accX * 0.004; // Kết hợp data gyro và accel

  if (!goc_2) {
    if (Angle > 0.3 || Angle < -0.3) {
      goc_2 = true;
    } else {
      goc_2 = false;
    }
  }
  if (abs(Angle) < 0.3 && goc_2) exe = true;   
  if (abs(Angle) > 15) exe = false;
  //Serial.println(Angle);        
}
void ai0() {
// ai0 được kích hoạt nếu digitalPin 2 chuyển từ THẤP lên CAO
// Kiểm tra chân 7 để xác định hướng
if(digitalRead(encB) == LOW) {
  demInterrupt++;
  }else{
  demInterrupt--;
  }
}