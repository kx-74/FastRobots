
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include "ICM_20948.h"
#include <math.h>

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "7452d2dc-9a63-4f8a-9b94-7c60eb0d78bb"
#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"
#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

//////////// Global Variables ////////////
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

const int arr_len = 2000;
char message[MAX_MSG_SIZE];     // Storing the user input command string
int timestamp_arr[arr_len];
float roll_comp_arr[arr_len];
float pitch_comp_arr[arr_len];
float yaw_comp_arr[arr_len];
bool start_record;
int arr_counter = 0;
int missed_counter = 0;   // To measure if main loop run fasters than the IMU producing values
int dt;               // For calculating dt for gyro

// IMU
#define SERIAL_PORT Serial
#define SPI_PORT SPI    // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2        // Which pin you connect CS to. Used only when "USE_SPI" is defined
#define WIRE_PORT Wire  // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 0
#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif

// Angle Calculations
double acc_roll;
double acc_pitch;
double acc_roll_lp;     // 
double acc_pitch_lp;    // Low-pass filtered acc values
double gyr_roll;
double gyr_pitch;
double gyr_yaw;
double comp_roll;
double comp_pitch;
//////////// Global Variables ////////////

enum CommandTypes
{
    GET_ANGLES,
    START_RECORD,
    STOP_RECORD,
    GET_RECORDED_DATA
};

void
handle_command()
{   
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
         * Send the roll, pitch and yaw computed from the IMU and corresponding time stamps
         * Repeat n times where n is decided by the command string input by the user
         */
        case GET_ANGLES:
            int loop_time;    // the number of sets of data to grab
            int start_time;       //
            int current_time;     //

            // Extract the next value from the command string as a character array
            success = robot_cmd.get_next_value(message);
            if (!success)
                return;
              
            loop_time = atoi(message);

            // Assume the robot starts up on a flat surface 
            gyr_roll = 0;
            gyr_pitch = 0;
            gyr_yaw = 0;
            comp_roll = 0;
            comp_pitch = 0;

            // Get the current time in milliseconds
            start_time = (int) millis();

            for (int i=0; i<loop_time; i++){
                // Grab data from the IMU
                if (myICM.dataReady())
                {
                    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
                                            //    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
                    
                    // Calculate the angles from the measurements
                    // Accelerometer
                    acc_roll = atan2(myICM.agmt.acc.axes.y, myICM.agmt.acc.axes.z)*180/M_PI;
                    acc_pitch = atan2(myICM.agmt.acc.axes.x, myICM.agmt.acc.axes.z)*180/M_PI;
                    
                    // Low-pass filtering the accelerometer - alpha: 0.16
                    acc_roll_lp = 0.16*acc_roll + 0.84*acc_roll_lp;
                    acc_pitch_lp = 0.16*acc_pitch + 0.84*acc_pitch_lp;

                    // Gyro
                    current_time = (int) millis();
                    dt = current_time - start_time;
                    gyr_roll = gyr_roll + myICM.gyrY() * dt/1000;
                    gyr_pitch = gyr_pitch + myICM.gyrX() * dt/1000;
                    gyr_yaw = gyr_yaw + myICM.gyrZ() * dt/1000;
                    start_time = current_time;
                
                    // Complementary filtered
                    comp_roll = (comp_roll + myICM.gyrY() * dt/1000)*0.7 + acc_roll_lp*0.3;
                    comp_pitch = (comp_pitch + myICM.gyrX() * dt/1000)*0.7 + acc_pitch_lp*0.3;
                }

                tx_estring_value.clear();
                tx_estring_value.append(acc_roll);
                tx_estring_value.append(", ");
                tx_estring_value.append(acc_pitch);
                tx_estring_value.append(", ");
                tx_estring_value.append(gyr_roll);
                tx_estring_value.append(", ");
                tx_estring_value.append(gyr_pitch);
                tx_estring_value.append(", ");
                tx_estring_value.append(gyr_yaw);
                tx_estring_value.append(", ");
                tx_estring_value.append(comp_roll);
                tx_estring_value.append(", ");
                tx_estring_value.append(comp_pitch);
                tx_estring_value.append(", ");
                tx_estring_value.append((int) millis());
                tx_characteristic_string.writeValue(tx_estring_value.c_str());

                // Serial.print("Sent back: ");
                // Serial.println(tx_estring_value.c_str());

                // Formatted for Better Serial Plotter to recognize the data
                Serial.print(acc_roll);
                Serial.print("\t");
                Serial.print(acc_pitch);
                Serial.print("\t");
                Serial.print(gyr_roll);
                Serial.print("\t");
                Serial.print(gyr_pitch);
                Serial.print("\t");
                Serial.print(gyr_yaw);
                Serial.print("\t");
                Serial.print(comp_roll);
                Serial.print("\t");
                Serial.println(comp_pitch);
            }
            break;
        
        /*
         * Start recording the angle values
         */
        case START_RECORD:
            // Clear the arrays to store new data
            for (int i=0; i<arr_len; i++){
                roll_comp_arr[i] = 0;
                pitch_comp_arr[i] = 0;
                yaw_comp_arr[i] = 0;
                timestamp_arr[i] = 0;
            }
            arr_counter = 0;
            missed_counter = 0;

            Serial.println("Start recording...");
            start_record = true;
            break;

        /*
         * Stop recording the angle values
         */
        case STOP_RECORD:
            Serial.println("Stop recording...");
            start_record = false;
            break;

        /*
         * Send the recorded angle readings alongwith timestamps via Bluetooth
         */
        case GET_RECORDED_DATA:
            for (int i=0; i<arr_counter; i++) {
                tx_estring_value.clear();
                tx_estring_value.append(pitch_comp_arr[i]);
                tx_estring_value.append(", ");
                tx_estring_value.append(roll_comp_arr[i]);
                tx_estring_value.append(", ");
                tx_estring_value.append(yaw_comp_arr[i]);
                tx_estring_value.append(", ");
                tx_estring_value.append(timestamp_arr[i]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());

                Serial.print("Sent back: ");
                Serial.println(tx_estring_value.c_str());
            }

            Serial.print("Number of missed loops: ");
            Serial.println(missed_counter);
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

    //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

    bool initialized = false;
    while (!initialized) {
        #ifdef USE_SPI
            myICM.begin(CS_PIN, SPI_PORT);
        #else
            myICM.begin(WIRE_PORT, AD0_VAL);
        #endif

        SERIAL_PORT.print(F("Initialization of the sensor returned: "));
        SERIAL_PORT.println(myICM.statusString());
        if (myICM.status != ICM_20948_Stat_Ok)
        {
          SERIAL_PORT.println("Trying again...");
          delay(500);
        }
        else
        {
          initialized = true;
        }
    }

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
write_data()
{
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
read_data()
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command();
    }
}


void loop()
{
    // Listen for connections
    BLEDevice central = BLE.central();

    // If a central is connected to the peripheral
    if (central) {
        Serial.print("Connected to: ");
        Serial.println(central.address());

        // While central is connected
        while (central.connected()) {
            // Record the sensor readings when the flag is raised and the array is not full
            if (start_record && arr_counter<arr_len) {
                if (myICM.dataReady()) {
                    myICM.getAGMT();

                    // Decide the initial status of the robot purely depending on acc readings
                    if (arr_counter==0) {
                        roll_comp_arr[0] = atan2(myICM.agmt.acc.axes.y, myICM.agmt.acc.axes.z)*180/M_PI;
                        pitch_comp_arr[0] = atan2(myICM.agmt.acc.axes.x, myICM.agmt.acc.axes.z)*180/M_PI;
                        yaw_comp_arr[0] = 0;
                        acc_roll_lp = roll_comp_arr[0];
                        acc_pitch_lp = pitch_comp_arr[0];

                        // Get the current time in milliseconds
                        timestamp_arr[0] = (int) millis();
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
                        timestamp_arr[arr_counter] = (int) millis();
                        dt = timestamp_arr[arr_counter] - timestamp_arr[arr_counter-1];

                        roll_comp_arr[arr_counter] = (roll_comp_arr[arr_counter-1] + myICM.gyrY() * dt/1000)*0.7 + acc_roll_lp*0.3;
                        pitch_comp_arr[arr_counter] = (pitch_comp_arr[arr_counter-1] + myICM.gyrX() * dt/1000)*0.7 + acc_pitch_lp*0.3;
                        yaw_comp_arr[arr_counter] = yaw_comp_arr[arr_counter-1] + myICM.gyrZ() * dt/1000;
                    }

                    arr_counter += 1;
                    if (arr_counter == arr_len)
                        Serial.println("Recording finished.");
                }
                else {
                   // Record that the IMU data missed a main loop
                    missed_counter += 1;
                }
            }

            // Send data
            write_data();

            // Read data
            read_data();
        }

        Serial.println("Disconnected");
    }
}
