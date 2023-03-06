//Adding interrupts to ultrasonic sensor connected to Piezo buzzer
const int TRIG1 = 6; // Arduino pin connected to Sensor's TRIG pin
const int ECHO1 = 3;   // Arduino pin connected to Sensor's ECHO pin
const int TRIG2 = 7;
const int ECHO2 = 2;
const int PIR = 4;
const int BUZZER = 13;  // Arduino pin connected to Piezo Buzzer's pin
const int LED = 12;
const int DISTANCE_THRESHOLD = 50; //centimeters
static unsigned long startTime = 0;
static unsigned long endTime = 0;

bool PIR_triggered = false;
bool LED_ON = false;

void setup() {
  Serial.begin(9600);  // initializing serial port
  pinMode(TRIG1, OUTPUT); // sets arduino pin to output mode
  pinMode(ECHO1, INPUT);  // sets arduino pin to input mode
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  pinMode(BUZZER, OUTPUT);   // sets arduino pin to output mode
  pinMode(PIR, INPUT);
  pinMode(LED, LOW);
  attachInterrupt(digitalPinToInterrupt(ECHO1), ISR_Ultrasonic1, CHANGE);  //adding interrupt via echo pin
  attachInterrupt(digitalPinToInterrupt(ECHO2), ISR_Ultrasonic2, CHANGE);
    
  PCICR |= 0b00000100;
  PCMSK2 |= (1 << PD4);
  
  noInterrupts();
	// Clear registers
	TCCR1A = 0;
	TCCR1B = 0;
	TCNT1 = 0;
	// we want turn led 13 to blink every 4 seconds = 0.25 Hz
	// 0.25Hz = (16000000/((62499+1)*1024))
	//Set timer compare
	OCR1A = 62499;
	// Prescaler 1024
	TCCR1B |= (1 << CS12) | (1 << CS10);
	// Output Compare Match A Interrupt Enable
	TIMSK1 |= (1 << OCIE1A);
	// CTC
	TCCR1B |= (1 << WGM12);

	interrupts();
}
 
int measure(int TRIG_PIN, int ECHO_PIN)
{
   digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  // generating a 10-microsecond pulse to TRIG pin
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long pulseDuration = pulseIn(ECHO_PIN, HIGH);
  
  int dist = pulseDuration / 58;   // calculates the distance
  return dist;
}
void loop() {
  
  int dist1 = measure(TRIG1, ECHO1);
  Serial.print("dist1: ");
  Serial.print(dist1);
  Serial.println(" cm");
  
  int dist2 = measure(TRIG2, ECHO2);
  Serial.print("dist2: ");
  Serial.print(dist2);
  Serial.println(" cm");
  delay(1000);
  
}

// Function to generate interrupts
void ISR_Ultrasonic1() {
  int echo = digitalRead(ECHO1);
  if(echo == HIGH){
  	startTime = micros();
  }else{
    endTime = micros();
  	long duration = endTime - startTime;
    int dist = duration/58;
    if(dist < DISTANCE_THRESHOLD)
    {
      Serial.println("Ultrasonic Interrupt genrated due to 1st sensor!");
   	  digitalWrite(BUZZER,HIGH);  // turn on buzzer
      delay(500);
      digitalWrite(BUZZER,LOW);   // turn off buzzer
      delay(500);
    }
  }
}
  void ISR_Ultrasonic2() {
  int echo = digitalRead(ECHO2);
  if(echo == HIGH){
  	startTime = micros();
  }else{
    endTime = micros();
  	long duration = endTime - startTime;
    int dist = duration/58;
    if(dist < DISTANCE_THRESHOLD)
    {
      Serial.println("Ultrasonic Interrupt genrated due to 2nd sensor!");
   	  digitalWrite(BUZZER,HIGH);  // turn on buzzer
      delay(500);
      digitalWrite(BUZZER,LOW);   // turn off buzzer
      delay(500);
    }
  }
 }
  
    ISR(TIMER1_COMPA_vect) {
	 LED_ON = !LED_ON;
     digitalWrite(LED, LED_ON? LOW:HIGH);
     Serial.println("Interrupt genrated due to timer");
  }

  ISR(PCINT2_vect) {
   digitalWrite(BUZZER,HIGH);  // turn on buzzer
   delay(500);
   digitalWrite(BUZZER,LOW);   // turn off buzzer
   delay(500);
   Serial.println("Interrupt genrated due to PIR motion sensor");
}