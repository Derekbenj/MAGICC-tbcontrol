#define ROLLING_AVG_LEN 50

struct helperData { 
  int replacement_index;
  int value_replaced;
};

int analogPin = A0;
int room_offset = 0; // used to determine the average reading for a given room
int measurements[ROLLING_AVG_LEN] = {};

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

int calc_rolling_average(bool recalculate_sum, helperData data ={}) {
  static long sum = 0;
  if (recalculate_sum) {
    sum = 0;
    for (int i = 0; i < ROLLING_AVG_LEN; ++i)
    {
      sum += measurements[i];
    }
  } else {
    sum -= data.value_replaced;
    sum += measurements[data.replacement_index];
  }
  
  return sum / ROLLING_AVG_LEN;
}

int calcRoomOffset() {

  static int latest;
  static int prev_value;
  static int jump_timeout;
  
  // get the lowest of 5 measurements as a starting value
  prev_value = analogRead(analogPin);
  for (int i = 0; i < 5; ++i) {
    latest = analogRead(analogPin);
    if (latest < prev_value) prev_value = latest;
  }

  // set the average to that lowest value
  int average = latest;
  for(int i = 1; i < 1000; ++i) {
    latest = analogRead(analogPin);
    if (latest >= 100*prev_value) {
      jump_timeout++;
      if (jump_timeout >= 3) {
        average = average * float((i))/float(i+1) + latest / float(i+1);
        prev_value = latest;
      } else {
        // the last value is clearly too big... so redo this calculation
        i--;
        continue;
      }
    }
    // otherwise, we're good to add this value to a simple calculated average
    average = average * float((i))/float(i+1) + latest / float(i+1);
    prev_value = latest;
  }

  return average;
}

void setup() {
  Serial.begin(9600);           //  setup serial  
  delay(1000);
  room_offset = calcRoomOffset();
  calc_rolling_average(true);
  for (int i = 0; i < 50; ++i) {
    calc_rolling_average(false, store_measurement(analogRead(analogPin)));
  }
}

void loop()
{
  static int prev_val = calc_rolling_average(false, store_measurement(analogRead(analogPin)));
  static int new_val;
  static int avg;
  new_val = analogRead(analogPin);
  if (new_val > 1.5*prev_val) {
    // do nothing... let this continue into the next loop
    Serial.println(0);
  } else {
    avg = -calc_rolling_average(false, store_measurement(new_val));
    avg -= -room_offset;
    if (avg < 0) avg = 0;
    String withScale = "0 ";
    withScale += avg;
    withScale += " 700";
    Serial.println(withScale);
    prev_val = new_val;
  }
}
