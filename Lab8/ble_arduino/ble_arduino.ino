#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include "ICM_20948.h"
#include <math.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h"
#include "PIDControl.h"
#include "MotorControl.h"
#include "SensorData.h"
#include "CommandTypes.h"


/////////////////////// BLE UUIDs ///////////////////////
#define BLE_UUID_TEST_SERVICE "7452d2dc-9a63-4f8a-9b94-7c60eb0d78bb"
#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"
#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"


/////////////////// Global Variables ///////////////////
BLEService testService(BLE_UUID_TEST_SERVICE);
BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);
BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");
// TX
EString tx_estring_value;
float tx_float_value = 0.0;

long interval = 1;
static long previousMillis = 0;
unsigned long currentMillis = 0;

//////////////
char message[MAX_MSG_SIZE];      // Storing the user input command string
bool start_ToF_pid;              // Flag to start the robot
bool start_IMU_pid;              // Flag to start the robot
bool start_manual_motor_ctrl;    // Flag to start the robot
int start_time;                  // The time when the robot is started
//int current_time;//Declared in SensorData.h
int IMU_init_counter;            // Counter for noise detection at IMU initialization 

// PID control
int ref_pos = 304;  // desired distance from the obstacle in millimeter
// int ref_angle = 0;  // desired angle difference from the initial position in degree//Declared in SensorData.h
float Kp = 0.3;
float Ki = 0.001;
float Kd = 100;

/////////////////// Global Variables ///////////////////



void handle_command() {   
  // Set the command string from the characteristic value
  robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                            rx_characteristic_string.valueLength());

  bool success;
  int cmd_type = -1;

  // Get robot command type (an integer)
  /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
    * since it uses strtok internally (refer RobotCommand.h and 
    * https://www.cplusplus.com/reference/cstring/strtok/)
    */
  success = robot_cmd.get_command_type(cmd_type);

  // Check if the last tokenization was successful and return if failed
  if (!success) {
      return;
  }

  // Handle the command type accordingly
  switch (cmd_type) {        
    /*
      * Start the position PID controller
      */
    case START_TOF_PID:
      data_arrays_reset();

      Serial.println("Start position PID controller...");
      start_ToF_pid = true;

      start_time = millis();
      integral_error = 0;         //
      previous_pid_error = 0;     //  reset the history error value
      pid_ctrl = 0;

      break;

    /*
      * Stop the position PID controller
      */
    case STOP_TOF_PID:
        Serial.println("Stop position PID controller...");
        start_ToF_pid = false;
        break;

    /*
      * Start the orientation PID controller
      */
    case START_IMU_PID:
      data_arrays_reset();

      // Get a rough estimate of the noise bias of the IMU sensor
      Serial.println("Detecting noise bias...");
      IMU_init_counter = 0;
      while (IMU_init_counter < 10) {
        if (myICM.dataReady()) {
          IMU_bias += myICM.gyrZ();
          IMU_init_counter++;
        }
      }
      IMU_bias = IMU_bias / 10;

      Serial.println("Start orientation PID controller...");
      start_IMU_pid = true;

      start_time = millis();
      gyr_yaw = 0;    // reset the yaw angle for initial position
      integral_error = 0;         //
      previous_pid_error = 0;     //  reset the history error value
      ref_angle = 0;
      pid_ctrl = 0;

      break;

    /*
      * Stop the position PID controller
      */
    case STOP_IMU_PID:
        Serial.println("Stop orientation PID controller...");
        start_IMU_pid = false;
        break;

    /*
      * Start the manual motor control
      */
    case START_MOTOR_CTRL:
      data_arrays_reset();

      Serial.println("Start manual motor control...");
      start_manual_motor_ctrl = true;

      start_time = millis();
      break;

    /*
      * Stop the manual motor control
      */
    case STOP_MOTOR_CTRL:
        Serial.println("Stop manual motor control...");
        motor_ctrl_PWM = 0;
        start_manual_motor_ctrl = false;
        break;

    /*
      * Send the recorded distance readings along with timestamps via Bluetooth
      */
    case GET_TOF_PID_INFO:
      for (int i=0; i<arr_counter; i++) {
        tx_estring_value.clear();
        tx_estring_value.append(ToF1_reading_arr[i]);
        tx_estring_value.append(", ");
        tx_estring_value.append(ToF1_extrapolate_arr[i]);
        tx_estring_value.append(", ");
        tx_estring_value.append(pid_ctrl_arr[i]);
        tx_estring_value.append(", ");
        tx_estring_value.append(p_ctrl_arr[i]);
        tx_estring_value.append(", ");
        tx_estring_value.append(i_ctrl_arr[i]);
        tx_estring_value.append(", ");
        tx_estring_value.append(d_ctrl_arr[i]);
        tx_estring_value.append(", ");
        tx_estring_value.append(pid_error_arr[i]);
        tx_estring_value.append(", ");
        tx_estring_value.append(timestamp_arr[i]);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());

        Serial.print("Sent back: ");
        Serial.println(tx_estring_value.c_str());
      }

      Serial.println("Data transmission completed.");
      break;
    
    /*
      * Send the recorded angle readings along with timestamps via Bluetooth
      */
    case GET_IMU_PID_INFO:
      for (int i=0; i<arr_counter; i++) {
        tx_estring_value.clear();
        tx_estring_value.append(yaw_comp_arr[i]);
        tx_estring_value.append(", ");
        tx_estring_value.append(pid_ctrl_arr[i]);
        tx_estring_value.append(", ");
        tx_estring_value.append(p_ctrl_arr[i]);
        tx_estring_value.append(", ");
        tx_estring_value.append(i_ctrl_arr[i]);
        tx_estring_value.append(", ");
        tx_estring_value.append(d_ctrl_arr[i]);
        tx_estring_value.append(", ");
        tx_estring_value.append(pid_error_arr[i]);
        tx_estring_value.append(", ");
        tx_estring_value.append(ref_angle_arr[i]);
        tx_estring_value.append(", ");
        tx_estring_value.append(timestamp_arr[i]);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());

        Serial.print("Sent back: ");
        Serial.println(tx_estring_value.c_str());
      }

      Serial.println("Data transmission completed.");
      break;

    /*
      * Tweak the gain values
      */
    case KP_SET:
      // Collect the PID parameter values assigned by the user
      success = robot_cmd.get_next_value(message);
      if (!success) {
        Serial.println("Invalid Command Message: Kp value expected");
        return;
      }
      Kp = atof(message);
      break;
    
    case KI_SET:
      // Collect the PID parameter values assigned by the user
      success = robot_cmd.get_next_value(message);
      if (!success) {
        Serial.println("Invalid Command Message: Kp value expected");
        return;
      }
      Ki = atof(message);
      break;

    case KD_SET:
      // Collect the PID parameter values assigned by the user
      success = robot_cmd.get_next_value(message);
      if (!success) {
        Serial.println("Invalid Command Message: Kp value expected");
        return;
      }
      Kd = atof(message);
      break;

    /*
      * Change the reference position/angle for PID control
      */
    case REF_POS_SET:
      // Collect the value assigned by the user
      success = robot_cmd.get_next_value(message);
      if (!success) {
        Serial.println("Invalid Command Message: Kp value expected");
        return;
      }
      ref_pos = atoi(message);
      break;
    
    case REF_ANGLE_SET:
      // Collect the value assigned by the user
      success = robot_cmd.get_next_value(message);
      if (!success) {
        Serial.println("Invalid Command Message: Kp value expected");
        return;
      }
      ref_angle = atoi(message);
      break;

    case MOTOR_PMW_SET:
      // Collect the value assigned by the user
      success = robot_cmd.get_next_value(message);
      if (!success) {
        Serial.println("Invalid Command Message: PMW value expected");
        return;
      }
      motor_ctrl_PWM = atoi(message);
      motor_left_PWM = 0;
      motor_right_PWM = 0;
      break;

    case GET_MOTOR_TOF_INFO:
      for (int i=0; i<arr_counter; i++) {
          tx_estring_value.clear();
          tx_estring_value.append(ToF1_reading_arr[i]);
          tx_estring_value.append(", ");
          tx_estring_value.append(motor_ctrl_PWM_arr[i]);
          tx_estring_value.append(", ");
          tx_estring_value.append(timestamp_arr[i]);
          tx_characteristic_string.writeValue(tx_estring_value.c_str());

          Serial.print("Sent back: ");
          Serial.println(tx_estring_value.c_str());
        }

        Serial.println("Data transmission completed.");
        break;

    case GET_DATA_STUNT:
      for (int i=0; i<arr_counter; i++) {
        tx_estring_value.clear();
        tx_estring_value.append(yaw_comp_arr[i]);
        tx_estring_value.append(", ");
        tx_estring_value.append(ref_angle_arr[i]);
        tx_estring_value.append(", ");
        tx_estring_value.append(motor_ctrl_PWM_arr[i]);
        tx_estring_value.append(", ");
        tx_estring_value.append(motor_left_PWM_arr[i]);
        tx_estring_value.append(", ");
        tx_estring_value.append(motor_right_PWM_arr[i]);
        tx_estring_value.append(", ");
        tx_estring_value.append(ToF1_reading_arr[i]);
        tx_estring_value.append(", ");
        tx_estring_value.append(pid_ctrl_arr[i]);
        tx_estring_value.append(", ");
        tx_estring_value.append(timestamp_arr[i]);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());

        Serial.print("Sent back: ");
        Serial.println(tx_estring_value.c_str());
        }

        Serial.println("Data transmission completed.");
        break;

    /* 
      * The default case may not capture all types of invalid commands.
      * It is safer to validate the command string on the central device (in python)
      * before writing to the characteristic.
      */
    default:
        Serial.print("Invalid Command Type: ");
        Serial.println(cmd_type);
        break;
  }
}


void
setup()
{
    Serial.begin(115200);

    /////////////////////// Bluetooth Setup //////////////////////
    BLE.begin();

    // Set advertised local name and service
    BLE.setDeviceName("Artemis BLE");
    BLE.setLocalName("Artemis BLE");
    BLE.setAdvertisedService(testService);

    // Add BLE characteristics
    testService.addCharacteristic(tx_characteristic_float);
    testService.addCharacteristic(tx_characteristic_string);
    testService.addCharacteristic(rx_characteristic_string);

    // Add BLE service
    BLE.addService(testService);

    // Initial values for characteristics
    // Set initial values to prevent errors when reading for the first time on central devices
    tx_characteristic_float.writeValue(0.0);

    // Output MAC Address
    Serial.print("Advertising BLE with MAC: ");
    Serial.println(BLE.address());

    BLE.advertise();


    ////////////////////////// IMU Setup /////////////////////////
    SERIAL_PORT.begin(115200);
    while (!SERIAL_PORT) {};

    #ifdef USE_SPI
      SPI_PORT.begin();
    #else
      WIRE_PORT.begin();
      WIRE_PORT.setClock(400000);
    #endif

    bool initialized = false;
    while (!initialized) {
      #ifdef USE_SPI
          myICM.begin(CS_PIN, SPI_PORT);
      #else
          myICM.begin(WIRE_PORT, AD0_VAL);
      #endif
      
      // Set full scale ranges for gyr
      ICM_20948_fss_t myFSS;
      myFSS.a = gpm2;
      myFSS.g = dps1000;
      myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
      if (myICM.status != ICM_20948_Stat_Ok)
      {
        SERIAL_PORT.print(F("setFullScale returned: "));
        SERIAL_PORT.println(myICM.statusString());
      }

      SERIAL_PORT.print(F("Initialization of the IMU sensor returned: "));
      SERIAL_PORT.println(myICM.statusString());
      if (myICM.status != ICM_20948_Stat_Ok) {
        SERIAL_PORT.println("Trying again...");
        delay(500);
      }
      else {
        initialized = true;
      }
    }
    Serial.println("IMU setup finished.");


    ////////////////////////// ToF Setup /////////////////////////
    Wire.begin();

    // Change the address of one of the ToFs for them to function simultaneously
    pinMode(XSHUT, OUTPUT);
    digitalWrite(XSHUT, LOW);             // Put sensor0 under hardware standby mode
    distanceSensor1.setI2CAddress(0x30);  // Change the address of the other sensor
    digitalWrite(XSHUT, HIGH);

    if (distanceSensor0.begin() != 0) {   //Begin returns 0 on a good init
      Serial.println("Sensor 0 failed to begin. Please check wiring. Freezing...");
      while (1);
    }
    if (distanceSensor1.begin() != 0) {
      Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
      while (1);
    }
    distanceSensor0.setDistanceModeLong();
    distanceSensor1.setDistanceModeLong();
    distanceSensor0.startRanging(); //Write configuration bytes to initiate measurement
    distanceSensor1.startRanging(); //Write configuration bytes to initiate measurement

    Serial.println("ToF setup finished.");


    ////////////////////////// Motor Driver Setup /////////////////////////
    pinMode(ABIN1_L, OUTPUT);
    pinMode(ABIN2_L, OUTPUT);
    pinMode(ABIN1_R, OUTPUT);
    pinMode(ABIN2_R, OUTPUT);
    Serial.println("Motor driver setup finished.");


    // Blink the LED as a visual indication that the board is running
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
}


void
write_data() {
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {
        tx_float_value = tx_float_value + 0.5;
        tx_characteristic_float.writeValue(tx_float_value);

        if (tx_float_value > 10000) {
            tx_float_value = 0;
        }
        previousMillis = currentMillis;
    }
}


void
read_data() {
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command();
    }
}


void loop() {
  // Listen for connections
  BLEDevice central = BLE.central();

  // If a central is connected to the peripheral
  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());

    // While central is connected
    while (central.connected()) {
      // Position PID controller
      if (start_ToF_pid && (millis()-start_time)<20000) {  // Run for at most 20 seconds
        current_time = millis();

        if (distanceSensor1.checkForDataReady()) {
          // Set the two data points as the same at initialization
          if (ToF1_extra_init_flag == false) {
            ToF1_second_time = current_time;
            ToF1_first_time = ToF1_second_time-1; // Offset to avoid zero on the dinominator
                                                // The offset doesn't matter because the term will be zero
            
            ToF1_second = distanceSensor1.getDistance();
            ToF1_first = ToF1_second;

            ToF1_extra_init_flag = true;
          }
          // Shift the history ToF readings
          else {
            ToF1_temp = distanceSensor1.getDistance();

            // Only shuffle the history when a new value of the reading comes
            if (ToF1_second != ToF1_temp) { 
              ToF1_first = ToF1_second;
              ToF1_first_time = ToF1_second_time;    // record the first sensor reading time
              ToF1_second = ToF1_temp;
              ToF1_second_time = current_time;       // record the second sensor reading time
            }
          }
        }
        
        // Extrapolated ToF data, based on the past two readings
        ToF1_extrapolate = extrapolation(ToF1_first_time, ToF1_second_time, ToF1_first, ToF1_second, current_time);

        // Calculate the PID control value, saved to the global variable pid_ctrl
        PID_control(Kp, Ki, Kd, ref_pos, ToF1_extrapolate);

        // Feed in the PWM values based on the PID control result
        if (pid_ctrl < 0)                 backward(abs(pid_ctrl));
        else if (pid_ctrl > 0)            forward(pid_ctrl);
        else                              stop();

        // Save the sensor and debugging data in arrays until full
        data_save();
      }

      // Orientation PID controller
      else if (start_IMU_pid && (millis()-start_time)<20000) { // Run for at most 20 seconds
        current_time = millis();

        if (myICM.dataReady()) {
          angle_calc();
        }

        // Calculate the PID control value, saved to the global variable pid_ctrl
        PID_control(Kp, Ki, Kd, ref_angle, gyr_yaw);

        // If the motor is not manually set, run the orientation control
        // Otherwise, the pid_ctrl will be integrated to motor_ctrl_PWM later 
        if (!start_manual_motor_ctrl) {
          // Feed in the PWM values based on the PID control result
          if (pid_ctrl < 0)                 cturn(abs(pid_ctrl));
          else if (pid_ctrl > 0)            ccturn(pid_ctrl);
          else                              stop();
        }

        // Save the sensor and debugging data in arrays until full
        data_save();
      }
      
      if (start_manual_motor_ctrl && (millis()-start_time)<20000) {
        // Manually control the motor
        current_time = millis();

        motor_left_PWM = motor_ctrl_PWM - pid_ctrl;
        motor_right_PWM = motor_ctrl_PWM + pid_ctrl;
        if (motor_left_PWM < 0)          motor_left_PWM = 0;
        else if (motor_left_PWM >255)    motor_left_PWM = 255;
        if (motor_right_PWM < 0)         motor_right_PWM = 0;
        else if (motor_right_PWM >255)   motor_right_PWM = 255;

        turn(motor_left_PWM, motor_right_PWM);

        if (distanceSensor1.checkForDataReady()) {
          ToF1_second = distanceSensor1.getDistance();

          if (!start_IMU_pid) { // if the IMU PID is running, data is saved in that statement
            // Only store the ToF data when the reading changes
            if (ToF1_second != ToF1_first){
              data_save();
              ToF1_first = ToF1_second;
            }
          }
        }
      }
      
      if ((!start_manual_motor_ctrl) && (!start_IMU_pid) && (!start_ToF_pid) || ((millis()-start_time)>=20000)) {
        // Stop if no control is running or the car has been running over 20 seconds
        stop();
      }

      // Send data
      write_data();

      // Read data
      read_data();
    }

    Serial.println("Disconnected");
  }
}
