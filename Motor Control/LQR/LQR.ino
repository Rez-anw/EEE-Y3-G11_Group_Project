#include <Servo.h>
#include <Wire.h>
#include <MatrixMath.h>

// ========== Board Constants ==========
const float BOARD_WIDTH = 23.0; // cm
const float BOARD_HEIGHT = 17.0; 
const float HOLE_DIAMETER = 1.0;
const float SERVO_TAU = 0.2;
const float DT = 0.01;
const float ALPHA = DT / (SERVO_TAU + DT);  // ≈ 0.0476

// ========== Servo Setup ==========
Servo servoX, servoY;
const int PIN_SERVO_X = 6;
const int PIN_SERVO_Y = 3;

// ========== LQR Parameters ==========
//mtx_type K_pos[2] = {-0.2244, -0.3427};
mtx_type K_pos[2] = {-0.0036, -0.00033};
//mtx_type K_vel = -0.4662;
mtx_type K_vel = -0.0056;
const float d_max = 3.0; // cm

// ========== I2C Handling ==========
volatile int16_t currentX, currentY, targetX, targetY;

void receiveEvent(int bytes) {
  uint8_t buffer[8];
  for(int i=0; i<8 && Wire.available(); i++){
    buffer[i] = Wire.read();
  }
  currentX = (buffer[0] << 8) | buffer[1];
  currentY = (buffer[2] << 8) | buffer[3];
  targetX = (buffer[4] << 8) | buffer[5];
  targetY = (buffer[6] << 8) | buffer[7];
}

// ========== Control System ==========
float cm_to_angle(float cm, bool is_x_axis) {
  const float max_cm = is_x_axis ? 1 : 1;         // Max board deflection in cm
  const float max_deg = is_x_axis ? 7.0 : 8.0;        // Max allowed angle
  return constrain(cm * (max_deg / max_cm), -max_deg, max_deg);
}


void lqr_control() {
  // Convert pixels to cm
  float x_cm = currentX * (BOARD_WIDTH / 1280.0);
  float y_cm = currentY * (BOARD_HEIGHT / 720.0);
  float tx_cm = targetX * (BOARD_WIDTH / 1280.0);
  float ty_cm = targetY * (BOARD_HEIGHT / 720.0);
 
  // Position error
  float err_x = tx_cm - x_cm;
  if (fabs(err_x) < 0.1) err_x = 0;
  float err_y = ty_cm - y_cm;
  
  // Velocity estimation (cm/s)
  static float prev_x = 0, prev_y = 0;
  float vel_x = (x_cm - prev_x) * 100; // Assuming 100Hz update
  float vel_y = (y_cm - prev_y) * 100;
  prev_x = x_cm;
  prev_y = y_cm;



  // Gain scheduling
  float weight_x = constrain(fabs(err_x)/d_max, 0, 1);
  float weight_y = constrain(fabs(err_y)/d_max, 0, 1);
  
  // Control law
  float ux = weight_x*(K_pos[0]*err_x + K_pos[1]*vel_x) + 
            (1-weight_x)*K_vel*vel_x;
  float uy = weight_y*(K_pos[0]*err_y + K_pos[1]*vel_y) + 
            (1-weight_y)*K_vel*vel_y;

  // Limit control output to match angle constraints
  float max_ux_cm = cm_to_angle(6, true);   // Max ±6°
  float max_uy_cm = cm_to_angle(8, false);  // Max ±8°
  ux = constrain(ux, -max_ux_cm, max_ux_cm);
  uy = constrain(uy, -max_uy_cm, max_uy_cm);

// After velocity calculation in lqr_control()
  float vel_mag = sqrt(vel_x*vel_x + vel_y*vel_y);
  float theta = atan2(vel_y, vel_x);
  float fr = (vel_mag < 0.1) ? 0.013 : 0.006; // Thesis values
  ux += fr * cos(theta) * (9.82/1.2);
  uy += fr * sin(theta) * (9.82/1.2);


  static float prev_ux = 0, prev_uy = 0;

  float filtered_ux = ALPHA * ux + (1 - ALPHA) * prev_ux;
  float filtered_uy = ALPHA * uy + (1 - ALPHA) * prev_uy;

  prev_ux = filtered_ux;
  prev_uy = filtered_uy;

  servoX.write(45 + cm_to_angle(filtered_ux, true));
  servoY.write(52 + cm_to_angle(filtered_uy, false));

}

// ========== Main Functions ==========
void setup() {
  servoX.attach(PIN_SERVO_X);
  servoY.attach(PIN_SERVO_Y);
  Wire.begin(0x08);
  Wire.onReceive(receiveEvent);
  Serial.begin(115200); 
}

void loop() {
  lqr_control();
  delay(10); // Matches Raspberry Pi's ~100Hz rate
}
