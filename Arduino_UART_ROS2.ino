  #include "MotorController.h"
  #include "EncoderReader.h"
  // Định nghĩa chân cho Module 1 (Động cơ 1)
  #define M1_IN_R 6   // PWM chiều thuận
  #define M1_IN_L 7   // PWM chiều ngược
  #define M1_EN_R 4   // Enable chiều thuận
  #define M1_EN_L 5   // Enable chiều ngược

  // Định nghĩa chân cho Module 2 (Động cơ 2)
  #define M2_IN_R 9   // PWM chiều thuận
  #define M2_IN_L 10   // PWM chiều ngược
  #define M2_EN_R 11   // Enable chiều thuận
  #define M2_EN_L 12   // Enable chiều ngược

  // Định nghĩa chân encoder
  #define ENC1_A 2  // Encoder 1 chân A (ngắt 5)
  #define ENC1_B 14  // Encoder 1 chân B
  #define ENC2_A 3  // Encoder 2 chân A (ngắt 3)
  #define ENC2_B 16  // Encoder 2 chân B

  // Khai báo động cơ & encoder
  MotorController motorLeft(6, 7, 4, 5);
  MotorController motorRight(9, 10, 11, 12);
  #define DISABLE LOW
  #define ENABLE HIGH

  // Khởi tạo encoder
  //Encoder enc1(ENC1_A, ENC1_B);
  //Encoder enc2(ENC2_A, ENC2_B);
  EncoderReader encoderLeft(2, 14);   // enc1
  EncoderReader encoderRight(3, 16); // enc2
  void isrLeft() {
    encoderLeft.update();
  }
  void isrRight() {
    encoderRight.update();
  }

  // Biến dữ liệu truyền
  uint8_t ros[8] = {0x02, 0, 0, 0, 0, 0, 0, 0x03};
  uint8_t rx_buffer[6];
  int16_t en1 = 0, en2 = 0;
  unsigned long lastSendTime = 0;

  void read_encoders();
  void send_ros();
  void receive_ros();

  void setup() {
    Serial.begin(115200);
    motorLeft.setup();
    motorRight.setup();
    encoderLeft.begin(isrLeft);
    encoderRight.begin(isrRight);
    encoderLeft.reset();
    encoderRight.reset();
    lastSendTime = millis();
    
  }

  void loop() {
    receive_ros();
    if (millis() - lastSendTime >= 10) {
      send_ros();
      lastSendTime = millis();
    }
  }
  // ========== HÀM ĐIỀU KHIỂN ========== //
  void read_encoders() {
    en1 = encoderLeft.read();
    en2 = -encoderRight.read();  // dao chieu encoder
    //Serial.println(en1);
    //Serial.println(en2);
  }

  void send_ros() {
    read_encoders();
    //read_imu();
    //ros[1] = angle >> 8;
    //ros[2] = angle & 0xFF;
    ros[1] = 0;
    ros[2] = 0;
    ros[3] = en1 >> 8;
    ros[4] = en1 & 0xFF;
    ros[5] = en2 >> 8;
    ros[6] = en2 & 0xFF;

    Serial.write(ros, 8);
  }
  void receive_ros() {
    if (Serial.available() >= 6) {
      Serial.readBytes(rx_buffer, 6);
      uint8_t reset = rx_buffer[0];
      uint8_t dir1 = rx_buffer[1];
      uint8_t speed1 = rx_buffer[2]; //uint8_t và unsigned char bản chất là 1 kiểu dữ liệu nên truyền biến speed vào được
      uint8_t dir2 = rx_buffer[3];
      uint8_t speed2 = rx_buffer[4];
      bool dir1_sign = (dir1 & 0x80) >> 7;  // Lay bit 7 de lay huong cua dong co
      bool dir2_sign = (dir2 & 0x80) >> 7;

      // Nếu cả hai tốc độ bằng 0 → dừng tất cả
      if (speed1 == 0 && speed2 == 0) {
        motorLeft.stop();
        motorRight.stop();
        return; 
      }
      motorLeft.drive(speed1, dir1_sign);  
      motorRight.drive(speed2, dir2_sign);
      // Reset encoder nếu cần
      if (reset == 1) {
        encoderLeft.reset();
        encoderRight.reset();
      }
    }
  }

