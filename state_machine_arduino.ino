#include <ros.h>
#include <std_msgs/Int8.h>
#include <keys/LRM.h>

#define ROLLING_AVG_LEN 40
#define NUM_MEASURES_CALIBRATION 1000
#define MULTIPLIER 2

int analogPin = A0;  // we'll read in the values from here
int room_offset = 0; // the ambient light level in the room we will subtract from measurements
int measurements[ROLLING_AVG_LEN] = {};

ros::NodeHandle nh;
keys::LRM light_reading_msg;

enum States {
  IDLE_ST      = 1,
  CALIBRATE_ST = 2,
  ACTIVE_ST    = 3,
} current_st;

struct helperData { 
  int replacement_index;
  int value_replaced;
};

helperData store_measurement(int measurement) {
  static int replacement_index = 0;
  replacement_index = replacement_index % ROLLING_AVG_LEN;

  helperData to_return;
  to_return.replacement_index = replacement_index;
  to_return.value_replaced = measurements[replacement_index];
  measurements[replacement_index] = measurement;
  
  replacement_index++;
  return to_return;
}

int calc_rolling_avg(helperData data = {}) {
  static long sum = 0;
//  if (recalculate_sum) {
//    sum = 0;
//    for (int i = 0; i < ROLLING_AVG_LEN; ++i)
//    {
//      sum += measurements[i];
//    }
//  } else {
  sum -= data.value_replaced;
  sum += measurements[data.replacement_index];
//  }
  
  return -((sum / ROLLING_AVG_LEN) - room_offset);
}

// gets the lowest of 30 measurements and uses that as the offset
void calibrate() {
  static int latest;
  static int prev_value;

  prev_value = analogRead(analogPin);
  
  // get the lowest of NUM_MEASURES_CALIBRATION measurements and assume that's the average
  for (int i = 0; i < NUM_MEASURES_CALIBRATION; ++i) {
    latest = analogRead(analogPin);
    if (latest < prev_value) prev_value = latest;
  }

  int average = prev_value;
  for (int i = 1; i < ROLLING_AVG_LEN; ++i) {
    latest = analogRead(analogPin);
    average = average * float((i))/float(i+1) + latest / float(i+1);
  }

  room_offset = average*MULTIPLIER; 
}

void statesCallback(const std_msgs::Int8& input_st) {
  // transitions
  switch(current_st) {
    case IDLE_ST:
    if (input_st.data == CALIBRATE_ST) {
      current_st = CALIBRATE_ST; 
    } else if (input_st.data == ACTIVE_ST) {
      current_st = ACTIVE_ST;
    } else {
      current_st = IDLE_ST;
    }
    break;
    case CALIBRATE_ST:
    current_st = CALIBRATE_ST;
    break;
    case ACTIVE_ST:
    if (input_st.data == IDLE_ST) {
      current_st = IDLE_ST;
    } else if (input_st.data == CALIBRATE_ST) {
      current_st = CALIBRATE_ST;
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

  delay(1000); // if we don't delay here, we start to run into a problem when we start the arduino node. that problem is "topic id 125: Tried to publish before configured"
}

void loop() {
  // actions
  switch(current_st) {
    case IDLE_ST:
    // publish the message we promise to over the "light_readings" topic and tell it we're in state 1
    light_reading_msg.current_st = 1;
    light_reading_msg.light_reading = room_offset;
    break;
    case CALIBRATE_ST:
    // call calibration code and publish the message over "light_readings" topic again, telling the user we're in state 2
    calibrate();
    light_reading_msg.current_st = 2;
    light_reading_msg.light_reading = room_offset;
    current_st = IDLE_ST;
    break;
    case ACTIVE_ST:
    // calculate the next average of the moving average and publish it, telling the user we're in state 3...
    light_reading_msg.current_st = 3;
    light_reading_msg.light_reading = calc_rolling_avg(store_measurement(analogRead(analogPin)*MULTIPLIER));
    break;
  }

  nh.spinOnce();
  light_reading_publisher.publish(&light_reading_msg);
}
