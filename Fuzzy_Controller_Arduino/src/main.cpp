#include <Arduino.h>
#include <Fuzzy.h>
#include <NewPing.h>
// ==== CÁC CHÂN ĐIỀU KHIỂN ĐỘNG CƠ ====
#define PWM_R_PIN 5
#define IN1R_PIN 2
#define IN2R_PIN 3
//
#define PWM_L_PIN 6
#define IN1L_PIN 4
#define IN2L_PIN 7

// ==== CÁC CHÂN CẢM BIẾN HR04 ====
#define TRIG_FRONT A0
#define ECHO_FRONT A1
#define TRIG_BACK  A2
#define ECHO_BACK  A3
#define MAX_DISTANCE 400  // cm
// ==== KHỞI TẠO 2 ĐỐI TƯỢNG SONAR ====
NewPing sonarFront(TRIG_FRONT, ECHO_FRONT, MAX_DISTANCE);
NewPing sonarBack(TRIG_BACK, ECHO_BACK, MAX_DISTANCE);

// ==== CÁC CHÂN ENCODER ====
#define ENC_L_A A4
#define ENC_L_B A5
#define ENC_R_A 8
#define ENC_R_B 10
long encoderCountL = 0;
long encoderCountR = 0;
// Encoder JGB37-520 
const int PPR = 330;
const int CPR = PPR*4; 


// ==== BIẾN TOÀN CỤC ====
//khoang cach
float phi = 0.0; // góc nghiêng của robot so với tường (rad)
//NewPing
volatile bool isPingingFront = true;
unsigned long lastPingTime = 0;
const unsigned long PING_INTERVAL_MS = 30; // Chu kỳ xử lý khoảng cách (50ms
//cảm biến khoảng cách
float distanceFront_cm = 0.0;
volatile float temp_distanceFront_cm = 0.0;
float distanceBack_cm = 0.0;
volatile float temp_distanceBack_cm = 0.0;
float distance_real_cm = 0.0;
float distance_real_m = 0.0;
float D_SETPOINT = 0.4; // (mét)
//omega
float denta_omegaR = 0.0;
float denta_omegaL = 0.0;  
//vR
float vR_SetPoint = 0.0;
float vR = 0.0;
float ek_vR = 0.0;
float ek_1_vR = 0.0;
float uk_vR = 0.0;
float uk_1_vR = 0.0;
float interval_e_vR = 0.0;
//vL
float vL_SetPoint = 0.0; 
float vL = 0.0;
float ek_vL = 0.0;
float ek_1_vL = 0.0;
float uk_vL = 0.0;
float uk_1_vL = 0.0;
float interval_e_vL = 0.0;
//Thoi gian
float dt = 0.005;
//PID parameters
float Kp_vR = 15.0;
float Ki_vR = 0.5;
float Kd_vR = 0.02;
float Kp_vL = 15.0;
float Ki_vL = 0.5;
float Kd_vL = 0.02;
//PWM
volatile int pwm_trai = 0;
volatile int pwm_phai = 0;
//Uart
String uartBuffer = "";
float timedelay = 0.0;
// =======================================================================
// ======================= HẰNG SỐ VẬT LÝ CỦA ROBOT ======================
// =======================================================================
const float BAN_KINH_BANH_XE = 0.0325; // (r) 3.25cm (32.5mm)
const float KHOANG_CACH_BANH_XE = 0.18; // (L_track) 
const float OMEGA_MAX_MOTOR = 26.18; // (rad/s) 250RPM@12V

// =======================================================================
// ======================= HẰNG SỐ ĐIỀU KHIỂN ============================
// =======================================================================
const float V_ROBOT_BASE = 0.6; // (m/s) Tốc độ dài đi thẳng
//const float K1 = 1.0 / 3.0; //  e  (m)
//const float K2 = 1.0 /1.0; //  dot_e (m/s)
const float KU_ROBOT_OMEGA = 3.5; // (RAD/s) Hệ số khuếch đại đầu ra điều khiển góc quay robot
const float K_e_distance = 1.0/2.0; // (m)
const float K_e_angle = 1.0/(PI/6.0); // (rad)
//========================CÁC HÀM NGẮT ĐỌC ENCODER========================
ISR(PCINT1_vect) { // A4–A5
  //int A = digitalRead(ENC_L_A);
 // int B = digitalRead(ENC_L_B);
 // if (A == B) encoderCountL++;
  //else encoderCountL--;
  encoderCountL++;
}
ISR(PCINT0_vect) { // pin 8–13
  //int A = digitalRead(ENC_R_A);
  //int B = digitalRead(ENC_R_B);
  //if (A == B) encoderCountR++;
  //else encoderCountR--;
  encoderCountR++;
}
//=====================================CÁC HÀM TÍNH TOÁN==================================
//HÀM TÍNH VẬN TỐC 
float readWheelSpeed(long &count, float dt_v) {
  long c;
  noInterrupts();
  c = count;
  count = 0;
  interrupts();
  float rev = (float)c / CPR;
  float rad = rev * 2.0 * PI;
  float omega = rad / dt_v;
  return omega * BAN_KINH_BANH_XE; // m/s
}
//HÀM TÍNH KHOẢNG CÁCH THỰC TẾ
float calculateDistance_1(float d1_cm, float d2_cm) {
  d1_cm = d1_cm + 8.5; // Hiệu chỉnh khoảng cách cảm biến
  d2_cm = d2_cm + 8.5; // Hiệu chỉnh khoảng cách cảm biến
  phi = atan((d2_cm - d1_cm)/(20.0)); // góc nghiêng của robot so với tường (rad));
  phi = constrain(phi, -(PI/6.0), (PI/6.0)); // Giới hạn góc phi trong khoảng -30° đến 30°
  float d_cm = (d1_cm + d2_cm) * cos(phi) / 2.0;
  return d_cm;
} 
///
float calculateDistance_2(float d1_cm, float d2_cm) {
  d1_cm = d1_cm + 10.0; // Hiệu chỉnh khoảng cách cảm biến
  d2_cm = d2_cm + 10.0; // Hiệu chỉnh khoảng
  float d_cm = (d1_cm + d2_cm) / 2.0;
  return d_cm;
}
//Bộ lọc Kalman cho khoảng cách
float Q1 = 0.1;   // nhiễu của mô hình
float R1 = 3;     // nhiễu của cảm biến
float x1 = 0;     // giá trị ước lượng
float P1 = 1;     // sai số ban đầu

float kalman_update_d1(float z1)
{
    // Prediction
    float x1_pred = x1;
    float P1_pred = P1 + Q1;
    // Kalman gain
    float K1 = P1_pred / (P1_pred + R1);

    // Update
    x1 = x1_pred + K1 * (z1 - x1_pred);
    P1 = (1 - K1) * P1_pred;

    return x1;
}
float Q2 = 0.1;   // nhiễu của mô hình
float R2 = 3;     // nhiễu của cảm biến
float x2 = 0;     // giá trị ước lượng
float P2 = 1;     // sai số ban đầu
float kalman_update_d2(float z2)
{
    // Prediction
    float x2_pred = x2;
    float P2_pred = P2 + Q2;
    // Kalman gain
    float K2 = P2_pred / (P2_pred + R2);

    // Update
    x2 = x2_pred + K2 * (z2 - x2_pred);
    P2 = (1 - K2) * P2_pred;

    return x2;
}


//========================HÀM IN DỮ LIỆU RA SERIAL MONITOR==========================
void printmonitor() {
        Serial.print("Front(cm): "); Serial.print(distanceFront_cm);
        Serial.print("\tBack(cm): "); Serial.print(distanceBack_cm);
        Serial.print("\tPhi(rad): "); Serial.print(phi);
        Serial.print("\tKhoang cach real(cm): "); Serial.println(distance_real_cm);
  // In xung PWM (0-255)
        Serial.print("\tPWM_L: "); Serial.print(pwm_trai);
        Serial.print("\tPWM_R: "); Serial.println(pwm_phai);
       // Serial.print("\tTin hieu dieu khien: "); Serial.println(uk_omega);
        Serial.print("\tdenta_omegaL: "); Serial.print(denta_omegaL);
        Serial.print("\tdenta_omegaR: "); Serial.println(denta_omegaR);
        Serial.print("\tvL: "); Serial.print(vL);
        Serial.print("\tvR: "); Serial.println(vR);
}
// ====================================================================================
// ================================== HÀM MỜ===========================================
// ====================================================================================
// (Hàm membershipTrap, membershipTri, tinhToanMoSugeno
float membershipTrap(float x, float a, float b, float c, float d) {
  if (x <= a || x >= d) {
    return 0.0;
  } else if (x >= b && x <= c) {
    return 1.0;
  } else if (x > a && x < b) {
    return (x - a) / (b - a);
  } else if (x > c && x < d) {
    return (d - x) / (d - c);
  }
  return 0.0;
}
float membershipTri(float x, float a, float b, float c) {
  if (x <= a || x >= c) {
    return 0.0;
  } else if (x == b) {
    return 1.0;
  } else if (x > a && x < b) {
    return (x - a) / (b - a);
  } else if (x > b && x < c) {
    return (c - x) / (c - b);
  }
  return 0.0;
}
//
float tinhtoanMoSugeno_omegaR(float e_bar_distance, float e_bar_angle)
{
    float e_distance_vals[3];
    e_distance_vals[0] = membershipTri(e_bar_distance, -1.5, -1.0, 0.0); // N
    e_distance_vals[1] = membershipTri(e_bar_distance, -1.0, 0.0, 1.0);  // Z
    e_distance_vals[2] = membershipTri(e_bar_distance, 0.0, 1.0, 1.5);   // P
    float e_angle_vals[3];
    e_angle_vals[0] = membershipTri(e_bar_angle, -1.5, -1.0, 0.0); // N
    e_angle_vals[1] = membershipTri(e_bar_angle, -1.0, 0.0, 1.0);  // Z
    e_angle_vals[2] = membershipTri(e_bar_angle, 0.0, 1.0, 1.5);   // P
    float w[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            w[i][j] = min(e_distance_vals[i], e_angle_vals[j]);
        }
    }
    const float OUT_S = -1.0, OUT_M = 0.0, OUT_F = 1.0;
    const float rules[3][3] = {
      //et = N       Z        P
        {OUT_S, OUT_M, OUT_M}, // ex = N
        {OUT_S, OUT_M, OUT_M}, // ex = Z
        {OUT_M, OUT_F, OUT_F}  // ex = P
    };
    float numerator = 0, denominator = 0;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float wi = w[i][j];
            float zi = rules[i][j];
            numerator += wi * zi;
            denominator += wi;
        }
    }
    if (denominator == 0) return 0.0;
    return numerator / denominator;
}
//
float tinhtoanMoSugeno_omegaL(float e_bar_distance, float e_bar_angle)
{
    float e_distance_vals[3];
    e_distance_vals[0] = membershipTri(e_bar_distance, -1.5, -1.0, 0.0); // N
    e_distance_vals[1] = membershipTri(e_bar_distance, -1.0, 0.0, 1.0);  // Z
    e_distance_vals[2] = membershipTri(e_bar_distance, 0.0, 1.0, 1.5);   // P
    float e_angle_vals[3];
    e_angle_vals[0] = membershipTri(e_bar_angle, -1.5, -1.0, 0.0); // N
    e_angle_vals[1] = membershipTri(e_bar_angle, -1.0, 0.0, 1.0);  // Z
    e_angle_vals[2] = membershipTri(e_bar_angle, 0.0, 1.0, 1.5);   // P
    float w[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            w[i][j] = min(e_distance_vals[i], e_angle_vals[j]);
        }
    }
    const float OUT_S = -1.0, OUT_M = 0.0, OUT_F = 1.0;
    const float rules[3][3] = {
      //et = N       Z        P
        {OUT_F, OUT_F, OUT_M}, // ex = N
        {OUT_M, OUT_M, OUT_S}, // ex = Z
        {OUT_M, OUT_M, OUT_S}  // ex = P
    };
    float numerator = 0, denominator = 0;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float wi = w[i][j];
            float zi = rules[i][j];
            numerator += wi * zi;
            denominator += wi;
        }
    }
    if (denominator == 0) return 0.0;
    return numerator / denominator;
}
//=====================================================================================
// =================================== HÀM CHẠY TIẾN ==================================
void chayTien(int speedL, int speedR) {
  digitalWrite(IN1L_PIN, HIGH);
  digitalWrite(IN2L_PIN, LOW);
  analogWrite(PWM_L_PIN, speedL);
//
  digitalWrite(IN1R_PIN, LOW);
  digitalWrite(IN2R_PIN, HIGH);
  analogWrite(PWM_R_PIN, speedR);
}

//====================================================================================
unsigned int timer1_counter = 0;
//===================================HÀM NGẮT TIMER1 5ms=============================
ISR(TIMER1_COMPA_vect) {
  timer1_counter++;
  if (timer1_counter >= 2) { // 5ms
      timer1_counter = 0;
  //================================ĐỌC VẬN TỐC BÁNH XE===============================
        // ĐỌC VẬN TỐC BÁNH XE VỚI BỘ LỌC THẤP
        const float dt_speed = 0.0108;   //10 ms + pid (0,4ms) mỗi CK
        //float vL_temp = readWheelSpeed(encoderCountL, dt_speed);
        //float vR_temp = readWheelSpeed(encoderCountR, dt_speed);
        //float alpha_v = 0.4; // Hệ số lọc
        //vL = alpha_v * vL + (1 - alpha_v) * vL_temp;
        //vR = alpha_v * vR + (1 - alpha_v) * vR_temp;
        vL = readWheelSpeed(encoderCountL, dt_speed);
        vR = readWheelSpeed(encoderCountR, dt_speed);
  }
  //================================FUZZY CONTROL 5ms================================
  //==============================PID CONTROL VẬN TỐC BÁNH XE===========================
  //========================== A. ĐIỀU KHIỂN VẬN TỐC BÁNH PHẢI==========================
  ek_vR = vR_SetPoint- vR; // Giả sử vR
  float de_vR = (ek_vR - ek_1_vR)/dt;
  interval_e_vR += (ek_vR * dt);
  float uk_vR_raw = Kp_vR * ek_vR + Ki_vR * interval_e_vR + Kd_vR * de_vR; //PID
  float uk_vR_sat = constrain(uk_vR_raw, 0.0, 12.0);
  //Anti-windup
  if (uk_vR_raw != uk_vR_sat) {
    interval_e_vR -= (ek_vR * dt); // Loại bỏ tích phân
  }
  uk_vR = uk_vR_sat;
  ek_1_vR = ek_vR; // Cập nhật lỗi trước
  //Chuyển uk_vR sang xung PWM
  pwm_phai = (int)((uk_vR/12.0) * 255.0);
  //=========================== B. ĐIỀU KHIỂN VẬN TỐC BÁNH TRÁI==========================
  ek_vL = vL_SetPoint - vL; 
  float de_vL = (ek_vL - ek_1_vL)/dt;
  interval_e_vL += (ek_vL * dt);
  float uk_vL_raw = Kp_vL * ek_vL + Ki_vL * interval_e_vL + Kd_vL * de_vL; //PID
  float uk_vL_sat = constrain(uk_vL_raw, 0.0, 12.0);
  //Anti-windup
  if (uk_vL_raw != uk_vL_sat) {
    interval_e_vL -= (ek_vL * dt); // Loại bỏ tích phân
  }
  uk_vL = uk_vL_sat;
  ek_1_vL = ek_vL; // Cập nhật lỗi trước
  //Chuyển uk_vL sang xung PWM
  pwm_trai = (int)((uk_vL/12.0) * 255.0);
  //Xuat xung PWM
  pwm_trai = constrain(pwm_trai, 0, 255);
  pwm_phai = constrain(pwm_phai, 0, 255);
  chayTien(pwm_trai, pwm_phai);  // Chạy tiến với xung PWM tính được
  //chayTien(255,150); //test
  }
  
// =======================================================================
// ==================== CÁC HÀM CALLBACK NEWPING =========================
// =======================================================================

// Hàm Callback cho cảm biến phía trước (Chạy trong Ngắt Timer2)
void echoCheckFront() {
  if (sonarFront.check_timer()) {
    unsigned int duration = sonarFront.ping_result;
    if (duration > 0) {
      temp_distanceFront_cm = (float)sonarFront.convert_cm(duration);
    }
    isPingingFront = false; // Đánh dấu ping Front đã hoàn thành
  }
}

// Hàm Callback cho cảm biến phía sau (Chạy trong Ngắt Timer2)
void echoCheckBack() {
  if (sonarBack.check_timer()) {
    unsigned int duration = sonarBack.ping_result;
    if (duration > 0) {
      temp_distanceBack_cm = (float)sonarBack.convert_cm(duration);
    }
    isPingingFront = true; // Đánh dấu ping Back đã hoàn thành
  }
}
//===================================HÀM SETUP() CHUONG TRINH==========================
void setup() {
  Serial.begin(9600);
  // Cài đặt PinMode cho ĐỘNG CƠ
  pinMode(PWM_L_PIN, OUTPUT);
  pinMode(IN1L_PIN, OUTPUT);
  pinMode(IN2L_PIN, OUTPUT);
 //
  pinMode(PWM_R_PIN, OUTPUT);
  pinMode(IN1R_PIN, OUTPUT);
  pinMode(IN2R_PIN, OUTPUT);
 
  // KHÔNG CẦN pinMode cho cảm biến, thư viện NewPing tự quản lý
  Serial.println("Khoi dong bo dieu khien Dong Hoc PD Mo (NewPing)...");
  Serial.print("r (banh xe) = "); Serial.print(BAN_KINH_BANH_XE); Serial.println(" m");
  Serial.print("L (khoang cach) = "); Serial.print(KHOANG_CACH_BANH_XE); Serial.println(" m");
  Serial.print("v_robot = "); Serial.print(V_ROBOT_BASE); Serial.println(" m/s");

  // ========================= ENCODER SETUP ===================================
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  //
  //===================== ENABLE PIN CHANGE INTERRUPTS =========================
  // Bánh trái — A4 (PCINT12), A5 (PCINT13)
  PCICR |= (1 << PCIE1);     // enable PCINT1 group
  PCMSK1 |= (1 << PCINT12);  // A4
  PCMSK1 |= (1 << PCINT13);  // A5

  // Bánh phải — D8 (PCINT0), D10 (PCINT2)
  PCICR |= (1 << PCIE0);     // enable PCINT0 group
  PCMSK0 |= (1 << PCINT0);   // pin 8
  PCMSK0 |= (1 << PCINT2);   // pin 10

  Serial.println("ENCODER READY!");
  
// ======================= TIMER1 SETUP 5MS CHO ĐIỀU KHIỂN VẬN TỐC =========================
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;      // Xóa các thanh ghi điều khiển Timer1
  TCCR1B = 0;
  // Chế độ CTC (Clear Timer on Compare Match)
  TCCR1B |= (1 << WGM12);
  // Set prescaler 64
  TCCR1B |= (1 << CS11) | (1 << CS10);
  // Tính OCR1A cho 5ms:
  // Clock = 16 MHz / 64 = 250 kHz (chu kỳ 4 us)
  // 5 ms = 5000 us / 4 us = 1250 counts
  OCR1A = 1250; //5ms
  // Bật ngắt so sánh OCR1A
  TIMSK1 |= (1 << OCIE1A);
  //interrupts();

  // ========================= NEWPING TIMER SETUP =========================
  // 30000 microseconds (30ms ) là thời gian chờ tối đa cho một lần ping (MAX_DISTANCE=400cm).
  NewPing::timer_us(30000, NULL); 
  
  // Bắt đầu ping cảm biến đầu tiên ngay lập tức
  sonarFront.ping_timer(echoCheckFront);
  
  interrupts();
  Serial.println("Robot Wall Follower Ready!");
} // --- KẾT THÚC SETUP() ---


// ================== VÒNG LẶP ĐỌC TÍN HIỆU CẢM BIẾN VÀ ĐIỀU KHIỂN MỜ =====================
void loop() {
  unsigned long now = millis();

  //=========================== XỬ LÝ NEWPING ===========================
  // =======================================================================
  // === LOGIC ĐỌC, LỌC VÀ ĐẶT LỊCH PING (50ms) ===
  // =======================================================================
  if (now - lastPingTime >= PING_INTERVAL_MS) {
    lastPingTime = now;
    
    // A. ÁP DỤNG LỌC & TÍNH KHOẢNG CÁCH THỰC TẾ
    // Sử dụng giá trị temp_distance...cm (đã được cập nhật bởi Ngắt Timer2)
    // Chỉ cập nhật Kalman nếu dữ liệu tạm thời > 0 (tránh nhiễu nếu cảm biến quá xa/gần)
     if (temp_distanceFront_cm > 0) {
         distanceFront_cm = kalman_update_d1(temp_distanceFront_cm);
        
     }
     if (temp_distanceBack_cm > 0) {
        distanceBack_cm = kalman_update_d2(temp_distanceBack_cm);      
    } 
    distance_real_cm = calculateDistance_1(distanceFront_cm, distanceBack_cm); 
    distance_real_m = distance_real_cm / 100.0; // (mét)

    // B. ĐẶT LỊCH PING TIẾP THEO (KHÔNG CHẶN)
    if (isPingingFront) {
        // Đặt lịch ping cho cảm biến TRƯỚC
        sonarFront.ping_timer(echoCheckFront);
    } else {
        // Đặt lịch ping cho cảm biến SAU
        sonarBack.ping_timer(echoCheckBack);
    }
  float e_distance = D_SETPOINT - distance_real_m; // (m)
  float denta_omegaR_bar = tinhtoanMoSugeno_omegaR(e_distance * K_e_distance, phi * K_e_angle);
  denta_omegaR = denta_omegaR_bar * KU_ROBOT_OMEGA; // (rad/s)
  float denta_omegaL_bar = tinhtoanMoSugeno_omegaL(e_distance * K_e_distance, phi * K_e_angle);
  denta_omegaL = denta_omegaL_bar * KU_ROBOT_OMEGA; // (rad/s)
  // TÍNH TOÁN ĐỘNG HỌC NGƯỢC (IK)
  vL_SetPoint= V_ROBOT_BASE + denta_omegaL * BAN_KINH_BANH_XE;
  vL_SetPoint= max(0.0, vL_SetPoint); // Giới hạn vL>=0
  vR_SetPoint = V_ROBOT_BASE + denta_omegaR * BAN_KINH_BANH_XE;
  vR_SetPoint= max(0.0, vR_SetPoint); // Giới hạn vR>=0
  }
  //============================= IN DỮ LIỆU RA SERIAL MONITOR ========================== 
  static unsigned long lastPrint = 0; //In mỗi 1 giây

    if (now - lastPrint >= 1000) {
        lastPrint = now;
       //Serial.print("d:"+String(distance_real_m));      
       //printmonitor();
       Serial.println(String(distance_real_m, 2)); // tối đa 2 số sau dấu phẩy
       //Serial.println("time delay(ms): " + String(timedelay));
    }
  // Điều khiển chu kỳ loop 5ms
//======================================================================================
// ================================ NHẬN SERIAL (SETPOINT) =============================
while (Serial.available() > 0) {
    char cc = Serial.read();   // <<< đổi từ c -> cc

    if (cc == '\n') {
        // kết thúc 1 lệnh
        String msg = uartBuffer;
        uartBuffer = "";
        msg.trim();

        bool isValid = true;
        bool dotOnce = false;

        if (msg.length() > 0) {
            for (size_t i = 0; i < msg.length(); i++) {
                char ch = msg[i];

                if (ch == '.') {
                    if (dotOnce) { isValid = false; break; }
                    dotOnce = true;
                }
                else if (!isDigit(ch)) {
                    isValid = false;
                    break;
                }
            }

            if (isValid) {
                float sp = msg.toFloat();

                if (sp >= 0.0 && sp <= 3.0) {
                    D_SETPOINT = sp;   // LƯU GIÁ TRỊ
                }
            }
        }
    }
    else {
        uartBuffer += cc;   // 
    }
}
// ========================================================================
} // --- KẾT THÚC LOOP() ---


