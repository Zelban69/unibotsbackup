
const int freq = 5000;
const int resolution = 8;

// Motor A (Left Side) Control Pins
// Motor A (Left Side) Control Pins
const int motorLeftFront_Fwd = 16; //IN1
const int motorLeftFront_Bwd = 17; //IN2
const int motorLeftBack_Fwd = 18; //IN3
const int motorLeftBack_Bwd = 19;  //in4
//right side
const int motorRightFront_Fwd = 27; //IN1 
const int motorRightFront_Bwd = 26; //in2
const int motorRightBack_Fwd = 32;  //IN3
const int motorRightBack_Bwd = 33; //IN4
//const int enablePin1 = 15; 
//const int enablePin2 = 21;
String current_command = ""; // Store the current command
String last_command = ""; // Store the last command

void setup() {
  // Initialize motor control pins as outputs
  pinMode(motorLeftFront_Fwd, OUTPUT);
  pinMode(motorLeftBack_Fwd, OUTPUT);
  pinMode(motorRightFront_Fwd, OUTPUT);
  pinMode(motorRightBack_Fwd, OUTPUT);

  pinMode(motorLeftFront_Bwd, OUTPUT);
  pinMode(motorLeftBack_Bwd, OUTPUT);
  pinMode(motorRightFront_Bwd, OUTPUT);
  pinMode(motorRightBack_Bwd, OUTPUT);
  //pinMode(enablePin1,OUTPUT);
  //pinMode(enablePin2, OUTPUT);
  //LeScooper.attach(LeScooperPin);
  Serial.begin(9600);
}

void forward_move(int throttle){
  analogWrite(motorLeftFront_Fwd, throttle);
  analogWrite(motorLeftBack_Fwd, throttle);
  analogWrite(motorRightFront_Fwd, throttle);
  analogWrite(motorRightBack_Fwd, throttle);
  // Pause for the movement to take effect
  //delay(15); // Adjust this delay to your needs

  //  Reset pins after the movement
  resetMotorPins();
}

void right_move(int throttle){
  analogWrite(motorLeftFront_Fwd, throttle);
  analogWrite(motorLeftBack_Bwd, throttle);
  analogWrite(motorRightFront_Bwd, throttle);
  analogWrite(motorRightBack_Fwd, throttle);
  // Pause for the movement to take effect
  delay(100); // Adjust this delay to your needs

  // Reset pins after the movement
  resetMotorPins();
}
//THIS ONE AINT WORKING PROPERLY
void left_move(int throttle){
  analogWrite(motorLeftFront_Bwd, throttle);
  analogWrite(motorLeftBack_Fwd, throttle);
  analogWrite(motorRightFront_Fwd, throttle);
  analogWrite(motorRightBack_Bwd, throttle);
  // Pause for the movement to take effect
  delay(100); // Adjust this delay to your needs

  // Reset pins after the movement
  resetMotorPins();
}

void stop_move() {
  // Stop motors by writing a low signal (0) to both forward and backward pins for each motor
  analogWrite(motorLeftFront_Fwd, 0);
  analogWrite(motorLeftBack_Fwd, 0);
  analogWrite(motorRightFront_Fwd, 0);
  analogWrite(motorRightBack_Fwd, 0);

  analogWrite(motorLeftFront_Bwd, 0);
  analogWrite(motorLeftBack_Bwd, 0);
  analogWrite(motorRightFront_Bwd, 0);
  analogWrite(motorRightBack_Bwd, 0);
  delay(10);
  forward_move(200);
  delay(1000);
  resetMotorPins();
}

void resetMotorPins() {
  digitalWrite(motorLeftFront_Fwd, LOW);
  digitalWrite(motorLeftFront_Bwd, LOW);
  digitalWrite(motorRightFront_Fwd, LOW);
  digitalWrite(motorRightFront_Bwd, LOW);
  digitalWrite(motorLeftBack_Fwd, LOW);
  digitalWrite(motorLeftBack_Bwd, LOW);
  digitalWrite(motorRightBack_Fwd, LOW);
  digitalWrite(motorRightBack_Bwd, LOW);
}

void clockwise_move(int throttle){
  analogWrite(motorLeftFront_Fwd, throttle);
  analogWrite(motorLeftBack_Fwd, throttle);
  analogWrite(motorRightFront_Bwd, throttle);
  analogWrite(motorRightBack_Bwd, throttle);

}

void loop() {
  
  //digitalWrite(enablePin1,HIGH);
  //digitalWrite(enablePin2,HIGH);
  //digitalWrite(enablePin3,HIGH);
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    delay(2); 
    command.trim(); // Removes any whitespace

    // Update the commands
    last_command = current_command; // Store last command
    current_command = command; // Update current command

    // Reset motor pins if the new command is different from the last
    if (current_command != last_command) {
      stop_move();
    }
    if (command.equals("forward_move")) {
      forward_move(120);
    } else if (command.equals("left_move")) {
      const int throttle = 150;
      analogWrite(motorLeftFront_Bwd, throttle);
      analogWrite(motorLeftBack_Fwd, throttle);
      analogWrite(motorRightFront_Fwd, throttle);
      analogWrite(motorRightBack_Bwd, throttle);
      resetMotorPins();

    } else if (command.equals("right_move")) {
      right_move(150);
    } else if (command.equals("stop")) {
      stop_move();
      
    } else if (command.equals("searching")) {
      clockwise_move(40);
    }
  }
}

