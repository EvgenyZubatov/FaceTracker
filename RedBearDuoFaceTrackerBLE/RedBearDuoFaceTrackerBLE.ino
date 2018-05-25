#include "ble_config.h"

/*
 * Provides skeleton code to interact with the Android FaceTrackerBLE app 
 * 
 * Created by Jon Froehlich, May 7, 2018
 * 
 * Based on previous code by Liang He, Bjoern Hartmann, 
 * Chris Dziemborowicz and the RedBear Team. See: 
 * https://github.com/jonfroehlich/CSE590Sp2018/tree/master/A03-BLEAdvanced
 */

#if defined(ARDUINO) 
SYSTEM_MODE(SEMI_AUTOMATIC); 
#endif

#define RECEIVE_MAX_LEN  5 // TODO: change this based on how much data you are sending from Android 
#define SEND_MAX_LEN    3

// Must be an integer between 1 and 9 and and must also be set to len(BLE_SHORT_NAME) + 1
#define BLE_SHORT_NAME_LEN 5 

// The number of chars should be BLE_SHORT_NAME_LEN - 1. So, for example, if your BLE_SHORT_NAME was 'J', 'o', 'n'
// then BLE_SHORT_NAME_LEN should be 4. If 'M','a','k','e','L','a','b' then BLE_SHORT_NAME_LEN should be 8
// TODO: you must change this name. Otherwise, you will not be able to differentiate your RedBear Duo BLE
// device from everyone else's device in class.
#define BLE_SHORT_NAME 'L','a','b','4'

/* Define the pins on the Duo board
 * TODO: change and add/subtract the pins here for your applications (as necessary)
 */

#define MAX_SERVO_ANGLE  180
#define MIN_SERVO_ANGLE  0

#define BLE_DEVICE_CONNECTED_DIGITAL_OUT_PIN D7

#define ALARM_LED_PIN D10

// Servo PIN
const int SERVO_OUTPUT_PIN = D0;
const int DELAY_MS = 50;

// Ultra Sonic sensor PINs
const int TRIG_PIN = D8;
const int ECHO_PIN = D9;

// Anything over 400 cm (23200 us pulse) is "out of range"
const unsigned int MAX_DIST = 23200;

int _stepAmount = 1; // the amount to change the angle of servo on each pass
int _minAngle = 0;
int _maxAngle = 180;
int _curAngle = 0;
Servo _servo;  

// Device connected and disconnected callbacks
void deviceConnectedCallback(BLEStatus_t status, uint16_t handle);
void deviceDisconnectedCallback(uint16_t handle);

// UUID is used to find the device by other BLE-abled devices
static uint8_t service1_uuid[16]    = { 0x71,0x3d,0x00,0x00,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e };
static uint8_t service1_tx_uuid[16] = { 0x71,0x3d,0x00,0x03,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e };
static uint8_t service1_rx_uuid[16] = { 0x71,0x3d,0x00,0x02,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e };

// Define the receive and send handlers
static uint16_t receive_handle = 0x0000; // recieve
static uint16_t send_handle = 0x0000; // send

static uint8_t receive_data[RECEIVE_MAX_LEN] = { 0x01 };
int bleReceiveDataCallback(uint16_t value_handle, uint8_t *buffer, uint16_t size); // function declaration for receiving data callback
static uint8_t send_data[SEND_MAX_LEN] = { 0x00 };

// Define the configuration data
static uint8_t adv_data[] = {
  0x02,
  BLE_GAP_AD_TYPE_FLAGS,
  BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE, 
  
  BLE_SHORT_NAME_LEN,
  BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME,
  BLE_SHORT_NAME, 
  
  0x11,
  BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE,
  0x1e,0x94,0x8d,0xf1,0x48,0x31,0x94,0xba,0x75,0x4c,0x3e,0x50,0x00,0x00,0x3d,0x71 
};

static btstack_timer_source_t send_characteristic;
static void bleSendDataTimerCallback(btstack_timer_source_t *ts); // function declaration for sending data callback
int _sendDataFrequency = 200; // 200ms (how often to read the pins and transmit the data to Android)

// distance to the object
float cm = 0;
float inches;

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("Face Tracker BLE Demo.");

  // Initialize ble_stack.
  ble.init();
  
  // Register BLE callback functions
  ble.onConnectedCallback(bleConnectedCallback);
  ble.onDisconnectedCallback(bleDisconnectedCallback);

  //lots of standard initialization hidden in here - see ble_config.cpp
  configureBLE(); 
  
  // Set BLE advertising data
  ble.setAdvertisementData(sizeof(adv_data), adv_data);
  
  // Register BLE callback functions
  ble.onDataWriteCallback(bleReceiveDataCallback);

  // Add user defined service and characteristics
  ble.addService(service1_uuid);
  receive_handle = ble.addCharacteristicDynamic(service1_tx_uuid, ATT_PROPERTY_NOTIFY|ATT_PROPERTY_WRITE|ATT_PROPERTY_WRITE_WITHOUT_RESPONSE, receive_data, RECEIVE_MAX_LEN);
  send_handle = ble.addCharacteristicDynamic(service1_rx_uuid, ATT_PROPERTY_NOTIFY, send_data, SEND_MAX_LEN);

  // BLE peripheral starts advertising now.
  ble.startAdvertising();
  Serial.println("BLE start advertising.");
 
  _servo.attach(SERVO_OUTPUT_PIN);
  _curAngle = (int)((MAX_SERVO_ANGLE - MIN_SERVO_ANGLE) / 2.0);
  _servo.write( _curAngle );
  Serial.println("Attached servo");

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  pinMode(ALARM_LED_PIN, OUTPUT);

  // Start a task to check status of the pins on your RedBear Duo
  // Works by polling every X milliseconds where X is _sendDataFrequency
  send_characteristic.process = &bleSendDataTimerCallback;
  ble.setTimer(&send_characteristic, _sendDataFrequency); 
  ble.addTimer(&send_characteristic);
}

void measureDistance() {
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(12);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  int counter = 0;
  while ( digitalRead(ECHO_PIN) == 0 ) {
    Serial.println("Waiting while ECHO_PIN is 0");
    if (counter++ > 100) break;
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min
  t1 = micros();
  counter = 0;
  while ( digitalRead(ECHO_PIN) == 1) {
    Serial.println("Waiting while ECHO_PIN is 1");
    if (counter++ > 100) break;
  }
  t2 = micros();
  pulse_width = t2 - t1;

  Serial.println(pulse_width);

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed 
  // of sound in air at sea level (~340 m/s).
  // Datasheet: https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if ( pulse_width > MAX_DIST ) {
    Serial.println("Out of range");
  } else {
    Serial.print(cm);
    Serial.print(" cm \t");
    Serial.print(inches);
    Serial.println(" in");
  }

  if (cm <= 100) {
    Serial.print("Object is in close proximity, firing an alarm");
    digitalWrite(ALARM_LED_PIN, HIGH);
  } else {
    Serial.print("Object is far away");
    digitalWrite(ALARM_LED_PIN, LOW);
  }
  
  // The HC-SR04 datasheet recommends waiting at least 60ms before next measurement
  // in order to prevent accidentally noise between trigger and echo
  // See: https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
  delay(70);  
}

void loop() 
{
  /*
  // Not currently used. The "meat" of the program is in the callback bleWriteCallback and send_notify
  // set the current angle
  _servo.write(_curAngle);
  Serial.println(_curAngle);

  // update the angle for next time the loop
  _curAngle += _stepAmount;

  // reverse the direction of the angle (as necessary)
  if (_curAngle <= _minAngle || _curAngle >= _maxAngle) {
    _stepAmount = -_stepAmount;
  }

  // wait for 30 milliseconds to see the dimming effect
  //delay(DELAY_MS);

  //measureDistance();
  /**/
}

/**
 * @brief Connect handle.
 *
 * @param[in]  status   BLE_STATUS_CONNECTION_ERROR or BLE_STATUS_OK.
 * @param[in]  handle   Connect handle.
 *
 * @retval None
 */
void bleConnectedCallback(BLEStatus_t status, uint16_t handle) {
  switch (status) {
    case BLE_STATUS_OK:
      Serial.println("BLE device connected!");
      digitalWrite(BLE_DEVICE_CONNECTED_DIGITAL_OUT_PIN, HIGH);
      break;
    default: break;
  }
}

/**
 * @brief Disconnect handle.
 *
 * @param[in]  handle   Connect handle.
 *
 * @retval None
 */
void bleDisconnectedCallback(uint16_t handle) {
  Serial.println("BLE device disconnected.");
  digitalWrite(BLE_DEVICE_CONNECTED_DIGITAL_OUT_PIN, LOW);
}

/**
 * @brief Callback for receiving data from Android (or whatever device you're connected to).
 *
 * @param[in]  value_handle  
 * @param[in]  *buffer       The buffer pointer of writting data.
 * @param[in]  size          The length of writting data.   
 *
 * @retval 
 */
int bleReceiveDataCallback(uint16_t value_handle, uint8_t *buffer, uint16_t size) {

  if (receive_handle == value_handle) {
    memcpy(receive_data, buffer, RECEIVE_MAX_LEN);
    Serial.print("Received data: ");
    for (uint8_t index = 0; index < RECEIVE_MAX_LEN; index++) {
      Serial.print(receive_data[index]);
      Serial.print(" ");
    }
    Serial.println(" ");
    
    // process the data. 
    if (receive_data[0] == 0x01) { //receive the face data 
      // CSE590 Student TODO
      // Write code here that processes the FaceTrackerBLE data from Android
      // and properly angles the servo + ultrasonic sensor towards the face
      // Example servo code here: https://github.com/jonfroehlich/CSE590Sp2018/tree/master/L06-Arduino/RedBearDuoServoSweep   

      // TODO: that was for blinking and smiling
    } else if (receive_data[0] == 0x02) { // face position
      uint8_t positionServo = receive_data[1];
      _servo.write( positionServo );
      _curAngle = positionServo;

      measureDistance();
    }
  }
  return 0;
}

/**
 * @brief Timer task for sending status change to client.
 * @param[in]  *ts   
 * @retval None
 * 
 * Send the data from either analog read or digital read back to 
 * the connected BLE device (e.g., Android)
 */
static void bleSendDataTimerCallback(btstack_timer_source_t *ts) {
  // CSE590 Student TODO
  // Write code that uses the ultrasonic sensor and transmits this to Android
  // Example ultrasonic code here: https://github.com/jonfroehlich/CSE590Sp2018/tree/master/L06-Arduino/RedBearDuoUltrasonicRangeFinder
  // Also need to check if distance measurement < threshold and sound alarm
  
    send_data[0] = (0x02);
    send_data[1] = (byte)(cm/2.0);
    send_data[2] = (byte)(inches/2.0);
    if (ble.attServerCanSendPacket()) {
      ble.sendNotify(send_handle, send_data, SEND_MAX_LEN);
      Serial.println("Sent distance data");
    }
  
  // Restart timer.
  ble.setTimer(ts, _sendDataFrequency);
  ble.addTimer(ts);
}
