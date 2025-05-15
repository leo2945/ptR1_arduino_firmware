#include <PinChangeInterrupt.h>  // ใช้สำหรับสร้าง Interrupt บนขาดิจิทัลที่ไม่ใช่ขา Interrupt เพื่อใช้กับ encoder 4 ตัว
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>


// Encoder Pin Definitions
// FL == M1
#define ENCODER1_PIN_A 10  // PORTB4   (D11)
#define ENCODER1_PIN_B 11  // PORTB3   (D10)

// FR == M2
#define ENCODER2_PIN_A 5   // PORTD5   (D5)
#define ENCODER2_PIN_B 6   // PORTD6   (D6)

// RL == M3
#define ENCODER3_PIN_A 3   // PORTD3   (D3)
#define ENCODER3_PIN_B 4   // PORTD4   (D4)

// RR == M4
#define ENCODER4_PIN_A 7   // PORTD7   (D7)
#define ENCODER4_PIN_B 12  // PORTB2   (D12)




#define LOOP_INTERVAL 20  // 20 ms = 50 Hz


const int FILTER_SIZE = 5;
bool DEBUG_MODE = 0;


/*
   M1(FL)-----M2(FR)
     |          |
     |          |
     |          | 
     |          |     
   M3(RL)-----M4(RR)
*/


// Threshold and Limits
#define MAX_POSITION 1000  
#define BAUD_RATE 57600

// Robot variable
  float r = 0.04;  // รัศมีล้อ (เมตร)
  float l2 = 0.0825;   // ระยะจากศูนย์กลางถึงล้อหน้า/หลัง (เมตร)
  float l1 = 0.1075;  // ระยะจากศูนย์กลางถึงล้อซ้าย/ขวา (เมตร)
  float dt_sec = 0.02;  // รอบเวลา (วินาที)

// Encoder Variables
  volatile long counterM1 = 0, counterM2 = 0, counterM3 = 0, counterM4 = 0;
  long prevM1 = 0, prevM2 = 0, prevM3 = 0, prevM4 = 0;
  const float PPR_M1  = 660;
  const float PPR_M2  = 660;
  const float PPR_M3  = 660;
  const float PPR_M4  = 660;
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

//still
    unsigned long last_pub_still = 0;
    const unsigned long PUB_INTERVAL_STILL = 250; // ms (1Hz)
    bool last_isStill = true;

ros::NodeHandle nh;
geometry_msgs::Twist vel_msg;
ros::Publisher vel_pub("/robot_velocity", &vel_msg);

std_msgs::Bool is_still_msg;
ros::Publisher is_still_pub("is_still", &is_still_msg);



//interrupt vaiable
volatile byte a1, b1, a2, b2, a3, b3, a4, b4;

unsigned long last_time = 0;

void setup() {
    Serial.begin(BAUD_RATE);
    delay(100); 
    nh.initNode();
    nh.advertise(vel_pub);
    nh.advertise(is_still_pub);


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
        debugPrint();
        long deltaM1 = movingAverage(counterM1 - prevM1, filterArrayM1, indexM1, countM1);
        long deltaM2 = movingAverage(counterM2 - prevM2, filterArrayM2, indexM2, countM2);
        long deltaM3 = movingAverage(counterM3 - prevM3, filterArrayM3, indexM3, countM3);
        long deltaM4 = movingAverage(counterM4 - prevM4, filterArrayM4, indexM4, countM4);

        calculateOdometry(deltaM1, deltaM2, deltaM4, deltaM3, vx, vy, omega, dt_sec);

        bool isStill = abs(deltaM1) < 2 &&
               abs(deltaM2) < 2 &&
               abs(deltaM3) < 2 &&
               abs(deltaM4) < 2;
        if (isStill != last_isStill || (current_time - last_pub_still > PUB_INTERVAL_STILL)) {
            is_still_msg.data = isStill;
            is_still_pub.publish(&is_still_msg);
            last_isStill = isStill;
            last_pub_still = current_time;
        }

        if (isStill) {
            vx = 0;
            vy = 0;
            omega = 0;
            resetFilter(filterArrayM1, indexM1, countM1);
            resetFilter(filterArrayM2, indexM2, countM2);
            resetFilter(filterArrayM3, indexM3, countM3);
            resetFilter(filterArrayM4, indexM4, countM4);
        } else {
            vx = filterVelocity(vx, prev_vx, 0.7);
            vy = filterVelocity(vy, prev_vy, 0.7);
            omega = filterVelocity(omega, prev_omega, 0.7);
        }

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

void resetFilter(long* buffer, int& index, int& count) {
  for (int i = 0; i < FILTER_SIZE; i++) buffer[i] = 0;
  index = 0;
  count = 0;
}


/**
 * @brief คำนวณ Odometry ของหุ่นยนต์ที่ใช้ล้อ Mecanum โดยใช้ข้อมูล Encoder
 * 
 * @param dFL จำนวนพัลส์จากล้อหน้า-ซ้าย (Front Left)
 * @param dFR จำนวนพัลส์จากล้อหน้า-ขวา (Front Right)
 * @param dRR จำนวนพัลส์จากล้อหลัง-ขวา (Rear Right)
 * @param dRL จำนวนพัลส์จากล้อหลัง-ซ้าย (Rear Left)
 * @param vx ความเร็วในแกน X (เชิงเส้น)
 * @param vy ความเร็วในแกน Y (Strafing)
 * @param omega ความเร็วเชิงมุม (Yaw)
 * @param dt_sec ระยะเวลา (วินาที)
 */
void calculateOdometry(long dFL, long dFR, long dRR, long dRL, float &vx, float &vy, float &omega, float dt_sec) {
    // คำนวณอัตราการหมุนของล้อแต่ละตัว (rad/s)
    float w_FL = (dFL / PPR_M1) * 2 * PI / dt_sec;  // M1 = FL
    float w_FR = (dFR / PPR_M2) * 2 * PI / dt_sec;  // M2 = FR
    float w_RR = (dRR / PPR_M4) * 2 * PI / dt_sec;  // M4 = RR
    float w_RL = (dRL / PPR_M3) * 2 * PI / dt_sec;  // M3 = RL

    // คำนวณความเร็วเชิงเส้น vx, vy และความเร็วเชิงมุม omega
    vx    = (r / 4.0) * (w_FL + w_FR + w_RL + w_RR);                            // ความเร็วแกน X
    vy    = (r / 4.0) * (-w_FL + w_FR + w_RL - w_RR);                           // ความเร็วแกน Y
    omega = (r / (4.0 * (l1 + l2))) * (-w_FL + w_FR - w_RL + w_RR);             // ความเร็วเชิงมุม
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

// M1 = Front Left = D11 (PB4), D12 (PB2)
    void counterM1_ISR() {
    a1 = (PINB >> PB4) & 1;
    b1 = (PINB >> PB3) & 1;

    if (a1 != prevA1) {
        if (a1 ^ b1) counterM1--;
        else         counterM1++;
    }
    prevA1 = a1;
    }

// M2 = Front Right = D5 (PD5), D6 (PD6)
    void counterM2_ISR() {
    a2 = (PIND >> PD5) & 1;
    b2 = (PIND >> PD6) & 1;

    if (a2 != prevA2) {
        if (a2 ^ b2) counterM2++;
        else         counterM2--;
    }
    prevA2 = a2;
    }

// M3 = Rear Left = D3 (PD3), D4 (PD4)
    void counterM3_ISR() {
    a3 = (PIND >> PD3) & 1;
    b3 = (PIND >> PD4) & 1;

    if (a3 != prevA3) {
        if (a3 ^ b3) counterM3++;
        else         counterM3--;
    }
    prevA3 = a3;
    }

// M4 = Rear Right = D7 (PD7), D10 (PB3)
    void counterM4_ISR() {
    a4 = (PIND >> PD7) & 1;
    b4 = (PINB >> PB2) & 1;

    if (a4 != prevA4) {
        if (a4 ^ b4) counterM4++;
        else         counterM4--;
    }
    prevA4 = a4;
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
        if (DEBUG_MODE) {
        Serial.print("M1: "); Serial.print(counterM1);
        Serial.print("  M2: "); Serial.print(counterM2);
        Serial.print("  M3: "); Serial.print(counterM3);
        Serial.print("  M4: "); Serial.println(counterM4);
    }
    
    }
}

