#include <PinChangeInterrupt.h>  // ใช้สำหรับสร้าง Interrupt บนขาดิจิทัลที่ไม่ใช่ขา Interrupt เพื่อใช้กับ encoder 4 ตัว
#include <ros.h>
#include <geometry_msgs/Twist.h>

// Encoder Pin Definitions
#define ENCODER1_PIN_A 11  // PORTB4
#define ENCODER1_PIN_B 12  // PORTB2
#define ENCODER2_PIN_A 7   // PORTD7
#define ENCODER2_PIN_B 10  // PORTB3
#define ENCODER3_PIN_A 5   // PORTD5
#define ENCODER3_PIN_B 6   // PORTD6
#define ENCODER4_PIN_A 3   // PORTD3
#define ENCODER4_PIN_B 4   // PORTD4
#define LOOP_INTERVAL 20  // 20 ms = 50 Hz


const int FILTER_SIZE = 3;
bool DEBUG_MODE = 0;


/*
   M1(FL)-----M3(FR)
     |          |
     |          |
     |          | 
     |          |     
   M4(RL)-----M4(RR)
*/


// Threshold and Limits
#define MAX_POSITION 1000000  
#define BAUD_RATE 57000

// Robot variable
  float r = 0.04;  // รัศมีล้อ (เมตร)
  float l = 0.085;   // ระยะจากศูนย์กลางถึงล้อหน้า/หลัง (เมตร)
  float w = 0.105;  // ระยะจากศูนย์กลางถึงล้อซ้าย/ขวา (เมตร)
  float dt_sec = 0.02;  // รอบเวลา (วินาที)

// Encoder Variables
  volatile long counterM1 = 0, counterM2 = 0, counterM3 = 0, counterM4 = 0;
  long prevM1 = 0, prevM2 = 0, prevM3 = 0, prevM4 = 0;
  const float PPR_M1 PROGMEM = 660;
  const float PPR_M2 PROGMEM = 660;
  const float PPR_M3 PROGMEM = 660;
  const float PPR_M4 PROGMEM = 660;
  volatile bool prevA1 = 0;
  volatile bool prevA2 = 0;
  volatile bool prevA3 = 0;
  volatile bool prevA4 = 0;


// Moving Average Filter Array
  long filterArrayM1[FILTER_SIZE] = {0};  // Array เก็บค่า Encoder ล่าสุด
  long filterArrayM2[FILTER_SIZE] = {0};
  long filterArrayM3[FILTER_SIZE] = {0};
  long filterArrayM4[FILTER_SIZE] = {0};
  // แยก index สำหรับแต่ละ filterArray
  int indexM1 = 0, indexM2 = 0, indexM3 = 0, indexM4 = 0;

  // ตัวแปรเช็คว่ามีค่าเพียงพอจะคำนวณค่าเฉลี่ยหรือยัง
  int countM1 = 0, countM2 = 0, countM3 = 0, countM4 = 0;

// mecanum variable
  float prev_vx = 0, prev_vy = 0, prev_omega = 0;
  float vx = 0, vy = 0, omega = 0;


ros::NodeHandle nh;
geometry_msgs::Twist vel_msg;
ros::Publisher vel_pub("/robot_velocity", &vel_msg);

//interrupt vaiable
volatile byte a1, b1, a2, b2, a3, b3, a4, b4;

unsigned long last_time = 0;

void setup() {
    Serial.begin(BAUD_RATE);

    nh.initNode();
    nh.advertise(vel_pub);

    pinMode(ENCODER1_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER1_PIN_B, INPUT_PULLUP);
    pinMode(ENCODER2_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER2_PIN_B, INPUT_PULLUP);
    pinMode(ENCODER3_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER3_PIN_B, INPUT_PULLUP);
    pinMode(ENCODER4_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER4_PIN_B, INPUT_PULLUP);

    attachPCINT(digitalPinToPCINT(ENCODER1_PIN_A), counterM1_ISR, CHANGE);
    attachPCINT(digitalPinToPCINT(ENCODER1_PIN_B), counterM1_ISR, CHANGE);
    attachPCINT(digitalPinToPCINT(ENCODER2_PIN_A), counterM2_ISR, CHANGE);
    attachPCINT(digitalPinToPCINT(ENCODER2_PIN_B), counterM2_ISR, CHANGE);
    attachPCINT(digitalPinToPCINT(ENCODER3_PIN_A), counterM3_ISR, CHANGE);
    attachPCINT(digitalPinToPCINT(ENCODER3_PIN_B), counterM3_ISR, CHANGE);
    attachPCINT(digitalPinToPCINT(ENCODER4_PIN_A), counterM4_ISR, CHANGE);
    attachPCINT(digitalPinToPCINT(ENCODER4_PIN_B), counterM4_ISR, CHANGE);

}

void loop() {
    unsigned long current_time = millis();

    if (current_time - last_time >= LOOP_INTERVAL) {  // ตรวจสอบช่วงเวลา 20 ms
        long deltaM1 = movingAverage(counterM1 - prevM1, filterArrayM1, indexM1, countM1);
        long deltaM2 = movingAverage(counterM2 - prevM2, filterArrayM2, indexM2, countM2);
        long deltaM3 = movingAverage(counterM3 - prevM3, filterArrayM3, indexM3, countM3);
        long deltaM4 = movingAverage(counterM4 - prevM4, filterArrayM4, indexM4, countM4);

        calculateOdometry(deltaM1, deltaM4, deltaM3, deltaM2, vx, vy, omega, dt_sec);

        vx = filterVelocity(vx, prev_vx, 0.7);
        vy = filterVelocity(vy, prev_vy, 0.7);
        omega = filterVelocity(omega, prev_omega, 0.7);

        prev_vx = vx;
        prev_vy = vy;
        prev_omega = omega;

        prevM1 = counterM1;
        prevM2 = counterM2;
        prevM3 = counterM3;
        prevM4 = counterM4;

        vel_msg.linear.x = vx;
        vel_msg.linear.y = vy;
        vel_msg.angular.z = omega;
        vel_pub.publish(&vel_msg);


        last_time = current_time;  // อัปเดตเวลา
    }

    checkAndResetEncoder(counterM1);
    checkAndResetEncoder(counterM2);
    checkAndResetEncoder(counterM3);
    checkAndResetEncoder(counterM4);

    // อัปเดต ROS Node และรอรับคำสั่งใหม่
    static unsigned long lastSpinTime = 0;
    if (current_time - lastSpinTime >= 5) {  // 200Hz
      nh.spinOnce();
      lastSpinTime = current_time;
    }
}


/**
 * @brief คำนวณ Odometry ของหุ่นยนต์ที่ใช้ล้อ Mecanum โดยใช้ข้อมูล Encoder
 * @param dL1 จำนวนพัลส์ที่อ่านได้จาก Encoder ของล้อหน้า-ซ้าย (Front Left - FL)
 * @param dR1 จำนวนพัลส์ที่อ่านได้จาก Encoder ของล้อหน้า-ขวา (Front Right - FR)
 * @param dR2 จำนวนพัลส์ที่อ่านได้จาก Encoder ของล้อหลัง-ขวา (Rear Right - RR)
 * @param dL2 จำนวนพัลส์ที่อ่านได้จาก Encoder ของล้อหลัง-ซ้าย (Rear Left - RL)
 * @param vx (output) ความเร็วเชิงเส้นตามแกน x (m/s)
 * @param vy (output) ความเร็วเชิงเส้นตามแกน y (m/s)
 * @param omega (output) ความเร็วเชิงมุมของหุ่นยนต์ (rad/s)
 * @param dt_sec เวลาที่ผ่านไปในวินาที (ใช้คำนวณความเร็วจาก Encoder)
 */
void calculateOdometry(long dL1, long dR1, long dR2, long dL2, float &vx, float &vy, float &omega, float dt_sec) {
    // คำนวณอัตราการหมุนของล้อแต่ละตัว (rad/s)
    float w_FL = (dL1 / PPR_M1) * 2 * PI / dt_sec;  // Front Left (FL)
    float w_FR = (dR1 / PPR_M3) * 2 * PI / dt_sec;  // Front Right (FR)
    float w_RR = (dR2 / PPR_M2) * 2 * PI / dt_sec;  // Rear Right (RR)
    float w_RL = (dL2 / PPR_M4) * 2 * PI / dt_sec;  // Rear Left (RL)

    // คำนวณความเร็วเชิงเส้น vx, vy และความเร็วเชิงมุม omega
    vx = (r / 4) * (w_FL + w_FR + w_RL + w_RR);           // ความเร็วแกน X
    vy = (r / 4) * (-w_FL + w_FR + w_RL - w_RR);          // ความเร็วแกน Y (Strafing)
    omega = (r / (4 * (l + w))) * (-w_FL + w_FR - w_RL + w_RR); // ความเร็วเชิงมุม (Yaw rate)
}


/**
 * @brief ตรวจสอบค่าของ Encoder และรีเซ็ตเป็น 0 หากถึง MAX_POSITION
 * @param encoderCount (volatile long&) ตัวแปรที่ใช้เก็บค่าจำนวนพัลส์ของ Encoder
 * 
 * ฟังก์ชันนี้ช่วยป้องกันค่า Encoder ไม่ให้เกินขีดจำกัดที่กำหนด (`MAX_POSITION`)
 * โดยจะรีเซ็ตค่า `encoderCount` กลับเป็น 0 เมื่อถึงขีดจำกัด
 */
void checkAndResetEncoder(volatile long &encoderCount) {
    // ตรวจสอบว่าค่า Encoder เกินขีดจำกัดที่กำหนดหรือไม่
    if (abs(encoderCount) >= MAX_POSITION) {
        // ปิดการขัดจังหวะชั่วคราว เพื่อป้องกันการเปลี่ยนแปลงค่า encoderCount โดย Interrupt
        noInterrupts();
        encoderCount = 0; // รีเซ็ตค่า Encoder กลับเป็น 0
        interrupts(); // เปิดใช้งาน Interrupt อีกครั้ง
        // แสดงข้อความแจ้งเตือนผ่าน Serial Monitor (สำหรับ Debug)
    }
}

/**
 * @brief ISR สำหรับมอเตอร์ M1 (Front Left, FL)
 * ใช้ขา D12 (PB4) และ D10 (PB2) สำหรับอ่านสัญญาณจาก Encoder
 */
void counterM1_ISR() { 
    a1 = (PINB & (1 << PB4)); // อ่านค่าจากขา D12 (PB4)
    b1 = (PINB & (1 << PB2)); // อ่านค่าจากขา D10 (PB2)

    if (a1 != prevA1) {  // ตรวจจับการเปลี่ยนแปลงของสัญญาณ A1
        if (a1 ^ b1) { counterM1++; } // หมุนตามเข็มนาฬิกา (CW)
        else { counterM1--; } // หมุนทวนเข็มนาฬิกา (CCW)
    }
    prevA1 = a1; // อัปเดตสถานะก่อนหน้า
}

/**
 * @brief ISR สำหรับมอเตอร์ M2 (Rear Right, RR)
 * ใช้ขา D7 (PD7) และ D10 (PB2) สำหรับอ่านสัญญาณจาก Encoder
 */
void counterM2_ISR() { 
    a2 = (PIND & (1 << PD7)); // อ่านค่าจากขา D7 (PD7)
    b2 = (PINB & (1 << PB2)); // **ตรวจสอบให้แน่ใจว่าใช้ขาที่ถูกต้อง**

    if (a2 != prevA2) {  // ตรวจจับการเปลี่ยนแปลงของสัญญาณ A2
        if (a2 ^ b2) { counterM2++; }  
        else { counterM2--; }  
    }
    prevA2 = a2; // อัปเดตสถานะก่อนหน้า
}

/**
 * @brief ISR สำหรับมอเตอร์ M3 (Front Right, FR)
 * ใช้ขา D5 (PD5) และ D6 (PD6) สำหรับอ่านสัญญาณจาก Encoder
 */
void counterM3_ISR() { 
    a3 = (PIND & (1 << PD5)); // อ่านค่าจากขา D5 (PD5)
    b3 = (PIND & (1 << PD6)); // อ่านค่าจากขา D6 (PD6)

    if (a3 != prevA3) {  // ตรวจจับการเปลี่ยนแปลงของสัญญาณ A3
        if (a3 ^ b3) { counterM3++; }  
        else { counterM3--; }  
    }
    prevA3 = a3; // อัปเดตสถานะก่อนหน้า
}

/**
 * @brief ISR สำหรับมอเตอร์ M4 (Rear Left, RL)
 * ใช้ขา D3 (PD3) และ D4 (PD4) สำหรับอ่านสัญญาณจาก Encoder
 */
void counterM4_ISR() { 
    a4 = (PIND & (1 << PD3)); // อ่านค่าจากขา D3 (PD3)
    b4 = (PIND & (1 << PD4)); // อ่านค่าจากขา D4 (PD4)

    if (a4 != prevA4) {  // ตรวจจับการเปลี่ยนแปลงของสัญญาณ A4
        if (a4 ^ b4) { counterM4++; }  
        else { counterM4--; }  
    }
    prevA4 = a4; // อัปเดตสถานะก่อนหน้า
}


/**
 * @brief ฟังก์ชันกรองค่าความเร็วโดยใช้ Exponential Moving Average (EMA)
 * @param newValue ค่าใหม่ที่ได้รับ (เช่น ค่าความเร็วล่าสุดจากเซ็นเซอร์)
 * @param oldValue ค่าความเร็วก่อนหน้า
 * @param alpha ค่าพารามิเตอร์ของฟิลเตอร์ (0-1) ควบคุมการตอบสนองของฟิลเตอร์
 *        - ค่าต่ำ (เช่น 0.1) จะให้การตอบสนองช้า (ค่าเปลี่ยนแปลงน้อย)
 *        - ค่าสูง (เช่น 0.9) จะให้การตอบสนองเร็ว (ค่าเปลี่ยนแปลงเร็ว)
 * @return ค่าใหม่ที่ผ่านการกรองเพื่อให้เปลี่ยนแปลงราบรื่นขึ้น
 */
float filterVelocity(float newValue, float oldValue, float alpha) {
    return (alpha * newValue) + ((1 - alpha) * oldValue);
}

/**
 * @brief ฟังก์ชันสำหรับคำนวณค่าเฉลี่ยเคลื่อนที่ (Moving Average)
 * @param newValue ค่าใหม่ที่รับเข้ามาเพื่ออัปเดตค่าเฉลี่ย
 * @param filterArray[] อาร์เรย์เก็บค่าล่าสุดตามขนาดของฟิลเตอร์
 * @param index ตำแหน่งปัจจุบันที่ค่าใหม่จะถูกบันทึก
 * @param count จำนวนค่าปัจจุบันที่มีในอาร์เรย์ (ใช้ป้องกันข้อผิดพลาดในช่วงเริ่มต้น)
 * @return ค่าเฉลี่ยของข้อมูลทั้งหมดที่มีในอาร์เรย์ ณ ปัจจุบัน
 */
long movingAverage(long newValue, long filterArray[], int &index, int &count) {
    long sum = 0;

    // เก็บค่าใหม่เข้าไปในอาร์เรย์ที่ใช้เป็นบัฟเฟอร์
    filterArray[index] = newValue;
    
    // ปรับ index ให้หมุนเวียนกลับไปที่ 0 หากถึงค่ามากที่สุดของ FILTER_SIZE
    index = (index + 1) % FILTER_SIZE;

    // ตรวจสอบว่าจำนวนค่าที่มีอยู่ยังไม่ถึงขนาดสูงสุดของฟิลเตอร์
    if (count < FILTER_SIZE) {
        count++;
    }

    // คำนวณผลรวมของค่าทั้งหมดที่มีอยู่ในอาร์เรย์
    for (int i = 0; i < count; i++) {
        sum += filterArray[i];
    }

    // คืนค่าเฉลี่ย โดยใช้ count เป็นตัวหารเพื่อป้องกันข้อผิดพลาด
    return sum / count;
}

void debugPrint() {
    if (DEBUG_MODE) {
    
    }
}

