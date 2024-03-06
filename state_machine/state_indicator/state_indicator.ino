/******************************** PREPROCESSOR DIRECTIVES ********************************/

#define USE_USBCON  // Ensures the serial communication is correctly initialized with rosserial over USB
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <Adafruit_NeoPixel.h>


/******************************** GLOBAL VARIABLES ********************************/

// Constants
constexpr uint8_t LED_COUNT = 12;                 // Number of LEDs on the ring
constexpr uint8_t LED_PIN = 13;                   // Digital pin on board connected to the ring
constexpr uint8_t MAIN_LOOP_DELAY = 90;           // [ms] Delay introduced in the main loop. Neccesary for the OVERTAKE animation
constexpr uint16_t TIMEOUT_THRESHOLD = 10000;     // [ms] Time since last message to trigger a timeout
constexpr uint8_t SPEED_STEERING_SCALING = 100;   // Factor to scale speed and steering angle values.
                                                  // This is done to convert them from a float to an int. Adapt the scaling to adjust the accuracy of the value

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Enums for state and mode management
enum OperationMode : uint8_t { STATE_MODE = 1, SPEED_STEERING_MODE = 2 };
enum CarState : uint8_t { GB_TRACK, TRAILING, OVERTAKE, FTGONLY };

OperationMode currentMode = STATE_MODE;
CarState currentState = GB_TRACK, previousState = GB_TRACK;

// Function declarations
void stateCallback (const std_msgs::String& stateMsg);
void modeCallback(const std_msgs::Int32& modeMsg);
void trailingAngleLedIndexCallback(const std_msgs::Int8& trailingAngleLedIndexMsg);
void speedCallback(const std_msgs::Float32& speedMsg);
void steeringAngleCallback(const std_msgs::Float32& steeringAngleMsg);

void updateLedSpeed();
void updateLedSteering();
void updateLedTrailingAngle();
void updateLedState();
void overtakeAnimation();

// Variables for tracking timeout
unsigned long lastMessageTimeState = 0;
unsigned long lastMessageTimeSpeedSteering = 0;
bool timeoutModeState = false;
bool timeoutModeSpeedSteering = false;

// Variables for OVERTAKE animation
unsigned long previousMillis = 0;
uint8_t currentBlink = 0;
uint8_t currentShift = 0;
bool overtakeAnimationActive = false;
bool overtakeAnimationComplete = false;

// Variables for storing the last received messages from the callbacks
int16_t lastSteeringAngleMsg = 0;
uint16_t lastSpeedMsg = 0;
uint8_t lastTrailingAngleLedIndexMsg = 0;

// ROS subscribers
ros::NodeHandle nh;
ros::Subscriber<std_msgs::String> stateSubscriber("/state_indicator/state", stateCallback);
ros::Subscriber<std_msgs::Int32> modeSubscriber("/state_indicator/mode", modeCallback);
ros::Subscriber<std_msgs::Int8> trailingAngleLedIndexSubscriber("/state_indicator/trailing_angle_led_index", trailingAngleLedIndexCallback);
ros::Subscriber<std_msgs::Float32> speedSubscriber("/state_indicator/speed", speedCallback);
ros::Subscriber<std_msgs::Float32> steeringAngleSubscriber("/state_indicator/steering_angle", steeringAngleCallback);


/******************************** CALLBACK FUNCTIONS ********************************/

void modeCallback(const std_msgs::Int32& modeMsg){
/*
 *  Callback function for the mode message.
 *
 *  Args:
 *  - modeMsg: message containing the mode.
 */

  if (currentMode != modeMsg.data) {
    strip.clear();
    strip.show();
  }
  currentMode = modeMsg.data;
}

void stateCallback(const std_msgs::String& stateMsg){
/*
 *  Callback function for the state message.
 *
 *  Args:
 *  - stateMsg: message containing the state of the racecar.
 */

  if (currentMode != STATE_MODE) return;  // Only process the message in STATE_MODE

  if (strcmp(stateMsg.data, "GB_TRACK") == 0) {
    currentState = GB_TRACK;
  } else if (strcmp(stateMsg.data, "TRAILING") == 0) {
    currentState = TRAILING;
  } else if (strcmp(stateMsg.data, "OVERTAKE") == 0) {
    currentState = OVERTAKE;
  } else if (strcmp(stateMsg.data, "FTGONLY") == 0) {
    currentState = FTGONLY;
  }
  lastMessageTimeState = millis();
  timeoutModeState = false;
}

void trailingAngleLedIndexCallback(const std_msgs::Int8& trailingAngleLedIndexMsg){
/*
 *  Callback function for the trailing angle LED index message.
 *
 *  Args:
 *  - trailingAngleLedIndexMsg: message containing the LED index, representing the position
 *    of the opponent racecar relative to the racecar on its LED ring.
 */

  if(currentMode != STATE_MODE) return;  // Only process the message in STATE_MODE

  lastTrailingAngleLedIndexMsg = trailingAngleLedIndexMsg.data;
}

void speedCallback(const std_msgs::Float32& speedMsg){
/*
 *  Callback function for the speed message.
 *
 *  Args:
 *  - speedMsg: message containing the speed of the racecar.
 */

  if(currentMode != SPEED_STEERING_MODE) return; // Only process the message in SPEED_STEERING_MODE

  lastSpeedMsg = static_cast<uint16_t>(speedMsg.data * SPEED_STEERING_SCALING);  // Scale Float32 to convert it to an unsinged integer
  lastMessageTimeSpeedSteering = millis();
  timeoutModeSpeedSteering = false;
}

void steeringAngleCallback(const std_msgs::Float32& steeringAngleMsg){
/*
 *  Callback function for the steering angle message.
 *
 *  Args:
 *  - steeringAngleMsg: message containing the steering angle of the racecar.
 */

  if(currentMode != SPEED_STEERING_MODE) return; // Only process the message in SPEED_STEERING_MODE

  lastSteeringAngleMsg = static_cast<int16_t>(steeringAngleMsg.data * SPEED_STEERING_SCALING); // Scale Float32 to convert it to a signed integer
  lastMessageTimeSpeedSteering = millis();
  timeoutModeSpeedSteering = false;
}


/******************************** LED VISUALIZATION FUNCTIONS ********************************/

void updateLedState() {
/*
 *  Function to update the LED ring color uniformly based on the current state of the racecar.
 *  Blue for TRAILING, Green for GB_TRACK, Red for FTGONLY and a RGB colored animation for OVERTAKE.
 */

  bool stateChanged = (currentState != previousState); // Check if the state has changed since the last update

  // Only update the LEDs if the overtake animation is not active or has just completed
  if (!overtakeAnimationActive || overtakeAnimationComplete) {
    strip.clear();

    switch (currentState) {
      case GB_TRACK:
        if (!overtakeAnimationActive) {
          strip.fill(strip.Color(0, 255, 0), 0, LED_COUNT); // Update LED ring to green
        }
        break;

      case TRAILING:
        if (!overtakeAnimationActive) {
          strip.fill(strip.Color(0, 0, 255), 0, LED_COUNT); // Update LED ring to blue
        }
        break;

      case OVERTAKE:
        // Update flags and variables for OVERTAKE animation if the state has just changed
        if (stateChanged) {
          overtakeAnimationActive = true;
          overtakeAnimationComplete = false;
          currentShift = 0;
          currentBlink = 0;
          previousMillis = millis();
        }
        break;

      case FTGONLY:
        if (!overtakeAnimationActive) {
          strip.fill(strip.Color(255, 0, 0), 0, LED_COUNT); // Update LED ring to red
        }
        break;

      default:
        strip.clear(); // Clear LEDs if the state is undefined
    }
    strip.show();
  }
    if (stateChanged) {
      previousState = currentState;
    }
}

void overtakeAnimation(){
/*
 *  Function to conduct an animation of alternately blinking RGB colors on the LED ring.
 *  Blue for TRAILING, Green for GB_TRACK, Red for FTGONLY and a RGB colored animation for OVERTAKE.
 */

  uint32_t colors[] = {strip.Color(255, 0, 0), strip.Color(0, 255, 0), strip.Color(0, 0, 255)}; // Red, Green, Blue

  // Check if enough time has passed to update the animation
  if (millis() - previousMillis >= 30) {
    previousMillis = millis();

    // Ensure the animation runs only if the currentState is OVERTAKE and the animation is active
    if (currentState == OVERTAKE && overtakeAnimationActive) {
      for (uint8_t i = 0; i < LED_COUNT; i++) {
        strip.setPixelColor(i, colors[(i + currentShift) % 3]);
      }
      strip.show();

      // Change the color of every LED to the next one in the colors array for three times 
      currentShift++;
      if (currentShift >= 3) {
        currentShift = 0;
        currentBlink++;
      }

      // Conduct five iterations of the currentShift pattern
      if (currentBlink >= 5) {
        overtakeAnimationActive = false;
        overtakeAnimationComplete = true;
        currentBlink = 0;
      }
    } else {
      overtakeAnimationActive = false;
    }
  }
}

void updateLedTrailingAngle(){
/*
 *  Function to set one LED, which resembles the relative direction to the opponent racecar, to purple.
 *  Is only called only, if the current state is TRAILING.
 */

  strip.setPixelColor(lastTrailingAngleLedIndexMsg, 255, 0, 255); // Purple color
}

void updateLedSpeed(){
/*
 *  Function to set the front five LEDs to an uniformly color, which resembles the speed of the racecar.
 *  Depending on how the LED ring is mounted relative to the sombrero hat, other indices of the LEDs that are controled need to be set.
 */

  uint16_t speed = map(lastSpeedMsg, 0, 700, 0, 255); // Map speed value (0m/s - 7m/s) to RGB range (0 - 255)
  for (uint8_t i = 4; i < 9; i++) {
    strip.setPixelColor(i, speed, 255 - speed, 0);  // Fast driving -> red ; slow driving -> green
  }
}

void updateLedSteering(){
/*
 *  Function to set three LEDs on the side to an uniformly blue color, which resembles the steering angle of the racecar.
 *  Depending on how the LED ring is mounted relative to the sombrero, other indices of the LEDs that are controled need to be set.
 */

  uint16_t steerRight = 0;
  uint16_t steerLeft = 0;

  if(lastSteeringAngleMsg > 0){
    steerLeft = map(abs(lastSteeringAngleMsg), 0, 30, 0, 255); // Map steering value (0rad - 0.3rad) to RGB range (0 - 255)
  } else if(lastSteeringAngleMsg < 0){
    steerRight = map(abs(lastSteeringAngleMsg), 0, 30, 0, 255); // Map steering value (-0.3rad - 0rad) to RGB range (0 - 255)
  }

  for (uint8_t i = 1; i < 4; i++) {
    strip.setPixelColor(i, 0, 0, steerLeft); // Harder steering -> more intensive blue
  }

  for (uint8_t i = 9; i < 12; i++) {
    strip.setPixelColor(i, 0, 0, steerRight); // Harder steering -> more intensive blue
  }
}


/******************************** LOOP AND SETUP FUNCTIONS ********************************/

void setup(){
/*
 * Initializes the Arduino setup: sets up ROS node subscriptions, initializes the LED strip and ensures all systems are ready for operation.
 */

  nh.initNode();
  nh.subscribe(stateSubscriber);
  nh.subscribe(modeSubscriber);
  nh.subscribe(trailingAngleLedIndexSubscriber);
  nh.subscribe(speedSubscriber);
  nh.subscribe(steeringAngleSubscriber);

  strip.begin();
  strip.show();

  // Initizalize the lastMessageTime for the first call of the timeout clause
  lastMessageTimeState = millis();
  lastMessageTimeSpeedSteering = millis();
}


void loop(){
/*
 * Main loop where the main logic takes place. Depeding on the mode and the time, at which the last message was received, 
 * either a timeout is triggered or the updateLED functions are called.
 */

  nh.spinOnce(); // Process ROS messages
  unsigned long currentTime = millis();

  // Turn off LEDs if the connection to the ROS node is lost
  if (!nh.connected()) {
    strip.clear();
    strip.show();
  }
  
  if(currentMode == STATE_MODE){
    if (currentTime - lastMessageTimeState > TIMEOUT_THRESHOLD) {
      strip.clear();

      // Log when no messages to the callback have arrived in the last TIMEOUT_THRESHOLD milliseconds
      char log_msg[47];
      snprintf(log_msg, sizeof(log_msg), "STATE: No message arrived in the last %hu ms", TIMEOUT_THRESHOLD);
      nh.loginfo(log_msg);
      timeoutModeState = true;
    }
    else{
      strip.clear();
      updateLedState();
      if(currentState == TRAILING){
        updateLedTrailingAngle();
      }
      if(overtakeAnimationActive){
        overtakeAnimation();
      }
    }
  }
  else if(currentMode == SPEED_STEERING_MODE){
    if (currentTime - lastMessageTimeSpeedSteering > TIMEOUT_THRESHOLD) {
      strip.clear();

      // Log when no messages to the callback have arrived in the last TIMEOUT_THRESHOLD milliseconds
      char log_msg[56];
      snprintf(log_msg, sizeof(log_msg), "SPEED_STEERING: No message arrived in the last %hu ms", TIMEOUT_THRESHOLD);
      nh.loginfo(log_msg);
      timeoutModeSpeedSteering = true;
    }
    else{
      strip.clear();
      updateLedSpeed();
      updateLedSteering();
    }
  }
  
  strip.show();  
  delay(MAIN_LOOP_DELAY); // Delay the main loop to allow the OVERTAKE animation to finish before the next update
}
