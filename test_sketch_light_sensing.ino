

int smoothMeasure();
int analogPin = A0; // potentiometer wiper (middle terminal) connected to analog pin 3
                    // outside leads to ground and +5V
int val = 0;  // variable to store the value read
int room_offset = 0; // used to determine the average reading for a given room

void setup() {
  Serial.begin(9600);           //  setup serial
}

void loop() {
  val = smoothMeasure();   // read the input pin
  Serial.print("Room offset: ");
  Serial.println(val);
}

int smoothMeasure()
{
  int i;
  int val = 0;
  int numReadings = 20;

  for (i = 0; i < numReadings; ++i)
  {
    val = val + analogRead(analogPin);
    delay(1); 
  }
  val = val / numReadings;
  return val;
}
