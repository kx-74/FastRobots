
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

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

const int array_len = 1000;
char time_buf[MAX_MSG_SIZE];
int time_array[array_len];
float temp_array[array_len];
//////////// Global Variables ////////////

enum CommandTypes
{
    PING,
    SEND_TWO_INTS,
    SEND_THREE_FLOATS,
    ECHO,
    DANCE,
    SET_VEL,
    GET_TIME_MILLIS,
    GET_TIME_MILLIS_LOOP,
    SEND_TIME_DATA,
    GET_TEMP_READINGS,
    SEND_MESSAGES,
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
         * Write "PONG" on the GATT characteristic BLE_UUID_TX_STRING
         */
        case PING:
            tx_estring_value.clear();
            tx_estring_value.append("PONG");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
        /*
         * Extract two integers from the command string
         */
        case SEND_TWO_INTS:
            int int_a, int_b;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_a);
            if (!success)
                return;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_b);
            if (!success)
                return;

            Serial.print("Two Integers: ");
            Serial.print(int_a);
            Serial.print(", ");
            Serial.println(int_b);
            
            break;
        /*
         * Extract three floats from the command string
         */
        case SEND_THREE_FLOATS:
            /*
             * Your code goes here.
             */

            break;
        /*
         * Add a prefix and postfix to the string value extracted from the command string
         */
        case ECHO:

            char char_arr[MAX_MSG_SIZE];

            // Extract the next value from the command string as a character array
            success = robot_cmd.get_next_value(char_arr);
            if (!success)
                return;
            
            tx_estring_value.clear();
            tx_estring_value.append("Robot says -> ");
            tx_estring_value.append(char_arr);
            tx_estring_value.append(" :)");

            Serial.println(tx_estring_value.get_length());
            // Check if augmented string length exceeds MAX_MSG_SIZE
            if (tx_estring_value.get_length() > MAX_MSG_SIZE) {
                Serial.println("Augmented string length exceeds the max write length.");
                return;
            }

            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());
            
            break;
        /*
         * DANCE
         */
        case DANCE:
            Serial.println("Look Ma, I'm Dancin'!");

            break;
        
        /*
         * SET_VEL
         */
        case SET_VEL:

            break;

        /*
         * GET_TIME_MILLIS
         */
        case GET_TIME_MILLIS:
            itoa(millis(), time_buf, 10);

            tx_estring_value.clear();
            tx_estring_value.append("T: ");
            tx_estring_value.append(time_buf);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
        /*
         * Collect time stamps for a few seconds
         */
        case GET_TIME_MILLIS_LOOP:
            for(int i=0; i<array_len; i++){
                // // Task 4: send each time stamp
                // tx_estring_value.clear();
                // tx_estring_value.append("T: ");
                // tx_estring_value.append((int) millis());
                // tx_characteristic_string.writeValue(tx_estring_value.c_str());

                // Serial.print("Sent back: ");
                // Serial.println(tx_estring_value.c_str());  

                // Task 5: store time stamps
                time_array[i] = (int) millis();

                // // Task 6: store time stamps with temperature readings
                // time_array[i] = (int) millis();
                // temp_array[i] = getTempDegF();
            }

            break;
        /*
         * Send each collected data point in time_array
         */
        case SEND_TIME_DATA:
            for (int i=0; i<array_len; i++){
                tx_estring_value.clear();
                tx_estring_value.append("T: ");
                tx_estring_value.append(time_array[i]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());

                Serial.print("Sent back: ");
                Serial.println(tx_estring_value.c_str());                
            }
        
                tx_estring_value.clear();
                tx_estring_value.append("End time: ");
                tx_estring_value.append((int) millis());
                tx_characteristic_string.writeValue(tx_estring_value.c_str());

            break;
        /*
         * Send each temperature reading with a time stamp
         */
        case GET_TEMP_READINGS:
            for (int i=0; i<array_len; i++){
                tx_estring_value.clear();
                tx_estring_value.append("Time: ");
                tx_estring_value.append(time_array[i]);
                tx_estring_value.append(", Temp: ");
                tx_estring_value.append(temp_array[i]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());

                Serial.print("Sent back: ");
                Serial.println(tx_estring_value.c_str());                
            }

            break;
        /*
         * Send fixed-length messages to the computer
         */
        case SEND_MESSAGES:
            char message[MAX_MSG_SIZE];
            char message_part[11];

            // Extract the next value from the command string as a character array
            success = robot_cmd.get_next_value(message);
            if (!success)
                return;
            
            // Copy the first 10 characters of the command string for command identifing
            strncpy(message_part, message, 10);
            message_part[10] = '\0';
            Serial.println(message);
            Serial.println(message_part);
            Serial.println((int)message[10]);

            if (strcmp(message, "5-bytes")==0){
                tx_estring_value.clear();
                tx_estring_value.append("Start time: ");
                tx_estring_value.append((int) millis());
                tx_characteristic_string.writeValue(tx_estring_value.c_str());

                for (int i=0; i<1000; i++){
                    tx_estring_value.clear();
                    tx_estring_value.append("Heyyy");
                    tx_characteristic_string.writeValue(tx_estring_value.c_str());
                }

                tx_estring_value.clear();
                tx_estring_value.append("End time: ");
                tx_estring_value.append((int) millis());
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            else if (strcmp(message, "120-bytes")==0){
                char long_message[120] = {};
                for (int i=0;i<8; i++){
                    strcat(long_message, "DogsAreTheBest!");
                }
                Serial.print("Size of the message sent: ");
                Serial.println(sizeof(long_message));
                Serial.println(long_message);

                tx_estring_value.clear();
                tx_estring_value.append("Start time: ");
                tx_estring_value.append((int) millis());
                tx_characteristic_string.writeValue(tx_estring_value.c_str());

                for (int i=0; i<400; i++){
                    tx_estring_value.clear();
                    tx_estring_value.append(long_message);
                    tx_characteristic_string.writeValue(tx_estring_value.c_str());
                }

                tx_estring_value.clear();
                tx_estring_value.append("End time: ");
                tx_estring_value.append((int) millis());
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            else if (strcmp(message_part, "customized")==0){
                char long_message[MAX_MSG_SIZE] = {};
                for (int i=0;i<(int)message[10]; i++){
                    strcat(long_message, "Dogs Are The Best!!!");
                }
                Serial.print("Size of the message sent: ");
                Serial.println(sizeof(long_message));
                Serial.println(long_message);

                tx_estring_value.clear();
                tx_estring_value.append("Start time: ");
                tx_estring_value.append((int) millis());
                tx_characteristic_string.writeValue(tx_estring_value.c_str());

                for (int i=0; i<200; i++){
                    tx_estring_value.clear();
                    tx_estring_value.append(long_message);
                    tx_characteristic_string.writeValue(tx_estring_value.c_str());
                }

                tx_estring_value.clear();
                tx_estring_value.append("End time: ");
                tx_estring_value.append((int) millis());
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            else {
                tx_estring_value.clear();
                tx_estring_value.append("Invalid Command Parameter");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            
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

    /*
     * An example using the EString
     */
    // Clear the contents of the EString before using it
    tx_estring_value.clear();

    // Append the string literal "[->"
    tx_estring_value.append("[->");

    // Append the float value
    tx_estring_value.append(9.0);

    // Append the string literal "<-]"
    tx_estring_value.append("<-]");

    // Write the value to the characteristic
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    // Output MAC Address
    Serial.print("Advertising BLE with MAC: ");
    Serial.println(BLE.address());

    BLE.advertise();
}

void
write_data()
{
    currentMillis = millis();
    Serial.println("Hi");
    if (currentMillis - previousMillis > interval) {
        Serial.println("Hey");
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

void
loop()
{
    // Listen for connections
    BLEDevice central = BLE.central();

    // If a central is connected to the peripheral
    if (central) {
        Serial.print("Connected to: ");
        Serial.println(central.address());

        // While central is connected
        while (central.connected()) {
            // Send data
            write_data();

            // Read data
            read_data();
        }

        Serial.println("Disconnected");
    }
}
