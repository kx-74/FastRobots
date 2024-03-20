#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

// IMU
#define SERIAL_PORT Serial
#define SPI_PORT SPI    // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2        // Which pin you connect CS to. Used only when "USE_SPI" is defined
#define WIRE_PORT Wire  // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 0       // The value of the last bit of the I2C address
#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif

// ToF
#define XSHUT A0
SFEVL53L1X distanceSensor0;
SFEVL53L1X distanceSensor1;

// Arrays for sensor data
const int arr_len = 2000;
int arr_counter = 0;
int timestamp_arr[arr_len];

// IMU data
// float roll_comp_arr[arr_len];
// float pitch_comp_arr[arr_len];
float yaw_comp_arr[arr_len];
int ref_angle_arr[arr_len];
//int IMU_timestamp_arr[arr_len];

// ToF data
int motor_ctrl_PWM_arr[arr_len];
// int ToF0_arr[arr_len];
int ToF1_reading_arr[arr_len];
int ToF1_extrapolate_arr[arr_len];
// int ToF0_timestamp_arr[arr_len];
// int ToF1_timestamp_arr[arr_len];

// PID data
float p_ctrl_arr[arr_len];
float i_ctrl_arr[arr_len];
float d_ctrl_arr[arr_len];
int pid_ctrl_arr[arr_len];
int pid_error_arr[arr_len];

// Angle info
float acc_roll;
float acc_pitch;
float acc_roll_lp;     // 
float acc_pitch_lp;    // Low-pass filtered acc values
float comp_roll;
float comp_pitch;
float gyr_yaw = 0.;
float IMU_bias = 0.;               // Variable to record and eliminate IMU bias

int current_time_imu;
int previous_time_imu;

// main loop timing
int current_time;

// Variable for manually controlling the motors
int motor_ctrl_PWM;             

// ToF data extrapolation
int ToF1_first_time;
int ToF1_second_time;
int ToF1_first;
int ToF1_second;
int ToF1_temp;
int ToF1_extrapolate;
bool ToF1_extra_init_flag = false;


// IMU Angle Calculation
void angle_calc() {      // Calculate the angle
  int dt;       // dt for gyro

  // Get the sensor data
  myICM.getAGMT();

  // Decide the initial status of the robot purely depending on acc readings
  //if (IMU_arr_counter==0) {
  if (arr_counter==0) {
    comp_roll = atan2(myICM.agmt.acc.axes.y, myICM.agmt.acc.axes.z)*180/M_PI;
    comp_pitch = atan2(myICM.agmt.acc.axes.x, myICM.agmt.acc.axes.z)*180/M_PI;
    acc_roll_lp = acc_roll;
    acc_pitch_lp = acc_pitch;

    // Get the current time in milliseconds
    current_time_imu = (int) millis();
    previous_time_imu = current_time_imu;
  }
  else {
    // Calculate the angles from the measurements
    // Accelerometer
    acc_roll = atan2(myICM.agmt.acc.axes.y, myICM.agmt.acc.axes.z)*180/M_PI;
    acc_pitch = atan2(myICM.agmt.acc.axes.x, myICM.agmt.acc.axes.z)*180/M_PI;

    // Low-pass filtering the accelerometer - alpha:
    acc_roll_lp = 0.16*acc_roll + 0.84*acc_roll_lp;
    acc_pitch_lp = 0.16*acc_pitch + 0.84*acc_pitch_lp;

    // Complementary filtered - alpha: 0.3
    current_time_imu = (int) millis();
    dt = current_time_imu - previous_time_imu;
    previous_time_imu = current_time_imu;

    comp_roll = (comp_roll + myICM.gyrY() * dt/1000)*0.7 + acc_roll_lp*0.3;
    comp_pitch = (comp_pitch + myICM.gyrX() * dt/1000)*0.7 + acc_pitch_lp*0.3;
    gyr_yaw = gyr_yaw + (myICM.gyrZ()-IMU_bias) * dt/1000;
  }
}


// ToF_extrapolation
int extrapolation(int x1, int x2, int y1, int y2, int x) {
  int y;
  y = y1 + (x-x1) * (y2-y1) / (x2-x1);

  return (int)y;
}



// Data Storage
void data_save() {   // save the data in arrays
  if (arr_counter < arr_len) {
    // Store the IMU data
    yaw_comp_arr[arr_counter] = gyr_yaw;
    ref_angle_arr[arr_counter] = ref_angle;

    // Store the ToF data
    motor_ctrl_PWM_arr[arr_counter] = motor_ctrl_PWM;
    ToF1_reading_arr[arr_counter] = ToF1_second;
    ToF1_extrapolate_arr[arr_counter] = ToF1_extrapolate;

    // Store the PID data
    p_ctrl_arr[arr_counter] = p_ctrl;
    i_ctrl_arr[arr_counter] = i_ctrl;
    d_ctrl_arr[arr_counter] = d_ctrl;
    pid_ctrl_arr[arr_counter] = pid_ctrl;         // in the range of 0 to 255
    pid_error_arr[arr_counter] = pid_error;       // in millimeter

    // Store the timestamp
    timestamp_arr[arr_counter] = current_time;    

    arr_counter += 1;
  }
}

// Clear the arrays to store new data
void data_arrays_reset() {   
  for (int i=0; i<arr_len; i++){
    yaw_comp_arr[i] = 0;
    ref_angle_arr[i] = 0;
    motor_ctrl_PWM_arr[i] = 0;
    ToF1_reading_arr[i] = 0;
    ToF1_extrapolate_arr[i] = 0;
    p_ctrl_arr[i] = 0;
    i_ctrl_arr[i] = 0;
    d_ctrl_arr[i] = 0;
    pid_ctrl_arr[i] = 0;
    pid_error_arr[i] = 0;
    timestamp_arr[i] = 0;    
  }
  arr_counter = 0;
}

#endif // SENSOR_DATA_H