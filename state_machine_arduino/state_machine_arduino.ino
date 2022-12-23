#include <ros.h>
#include <std_msgs/Int8.h>
#include <keys/LRM.h>

#define ROLLING_AVG_LEN 35
#define NUM_MEASURES_CALIBRATION 1000

int analogPin = A0;  // we'll read in the values from here
int dark_offset = 0;
int bright_offset = 0; // higher bound
int measurements[ROLLING_AVG_LEN] = {};

ros::NodeHandle nh;
keys::LRM light_reading_msg;

enum States {
  IDLE_ST           = 1,
  CALIBRATE_MIN_ST  = 2,
  CALIBRATE_MAX_ST  = 3,
  ACTIVE_ST         = 4,
} current_st;

struct helperData { 
  int replacement_index;
  int value_replaced;
};

helperData store_measurement(int measurement) {
  
  // for help with calculating the average without using too much dynamic memory
  static int replacement_index = 0;

  // our photoresistor setup measures "dark", but we want to measure light, so reverse it's mapping.
  measurement = map(measurement, 0, 1023, 1023, 0);

  // which index is next in line to replace: replace it and since you're a helper function tell the function that you return to which index you replaced
  // and the value you removed.
  replacement_index = replacement_index % ROLLING_AVG_LEN;
  helperData to_return;
  to_return.replacement_index = replacement_index;
  to_return.value_replaced = measurements[replacement_index];
  measurements[replacement_index] = measurement;
  
  replacement_index++;
  return to_return;
}

float calc_rolling_avg(helperData data = {}) {
  static long sum = 0;
  sum -= data.value_replaced;
  sum += measurements[data.replacement_index];
  return (sum / ROLLING_AVG_LEN);
}

void calibrate(int& offset) {
  for (int i = 0; i < NUM_MEASURES_CALIBRATION; ++i) {
    offset = calc_rolling_avg(store_measurement(analogRead(analogPin)));
  }
}

void statesCallback(const std_msgs::Int8& input_st) {
  // transitions
  switch(current_st) {
    case IDLE_ST:
    if (input_st.data == CALIBRATE_MIN_ST) {
      current_st = CALIBRATE_MIN_ST;
    } else if (input_st.data == CALIBRATE_MAX_ST) {
      current_st = CALIBRATE_MAX_ST; 
    } else if (input_st.data == ACTIVE_ST) {
      current_st = ACTIVE_ST;
    } else {
      current_st = IDLE_ST;
    }
    break;
    case CALIBRATE_MIN_ST:
    current_st = CALIBRATE_MIN_ST;
    break;
    case CALIBRATE_MAX_ST:
    current_st = CALIBRATE_MAX_ST;
    case ACTIVE_ST:
    if (input_st.data == IDLE_ST) {
      current_st = IDLE_ST;
    } else {
      current_st = ACTIVE_ST;
    }
    break;
    default:
    break;
  }
}

ros::Publisher light_reading_publisher("light_readings", &light_reading_msg);
ros::Subscriber<std_msgs::Int8> sm_tran_cmds("change_st", &statesCallback);

void setup() {
  current_st = IDLE_ST;
  nh.initNode();
  nh.advertise(light_reading_publisher);
  nh.subscribe(sm_tran_cmds);
  nh.negotiateTopics();
  light_reading_msg.light_reading = 0;

  delay(1000); // if we don't delay here, we start to run into a problem when we start the arduino node. that problem is "topic id 125: Tried to publish before configured"
}

void loop() {
  static float reading = 0;
  // actions
  switch(current_st) {
    case IDLE_ST:
    // publish the message we promise to over the "light_readings" topic and tell it we're in state 1
    light_reading_msg.current_st = 1;
    break;
    case CALIBRATE_MIN_ST:
    // call calibration code and publish the message over "light_readings" topic again, telling the user we're in state 2
    calibrate(dark_offset);
    light_reading_msg.current_st = 2;
    reading = dark_offset;
    current_st = IDLE_ST;
    break;
    case CALIBRATE_MAX_ST:
    calibrate(bright_offset);
    light_reading_msg.current_st = 3;
    reading = bright_offset;
    current_st = IDLE_ST;
    break;
    case ACTIVE_ST:
    light_reading_msg.current_st = 4;
    reading = (calc_rolling_avg(store_measurement(analogRead(analogPin))) - dark_offset)/((float)(bright_offset-dark_offset))*1000;
    reading = constrain(reading, 0, 1000);
    break;
  }

  light_reading_msg.light_reading = reading;
  
  nh.spinOnce();
  light_reading_publisher.publish(&light_reading_msg);
}
