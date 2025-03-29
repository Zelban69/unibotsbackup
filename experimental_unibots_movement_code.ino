// --- Pin Definitions ---
// Button
const int stopButtonPin = 2; // GPIO 2 for the stop button

// Motor A (Left Side) Control Pins
const int motorLeftFront_Fwd = 16; // IN1
const int motorLeftFront_Bwd = 17; // IN2
const int motorLeftBack_Fwd = 19;  // IN3
const int motorLeftBack_Bwd = 18;  // IN4
// Motor B (Right Side) Control Pins
const int motorRightFront_Fwd = 26; // IN1
const int motorRightFront_Bwd = 27; // IN2
const int motorRightBack_Fwd = 33;  // IN3
const int motorRightBack_Bwd = 32; // IN4

// Enable Pins (If your motor driver uses them, uncomment and set pins)
// const int enablePinLeft = 15; // Example for Left Motors
// const int enablePinRight = 21; // Example for Right Motors

// --- State Variables ---
bool emergencyStopped = false; // Track emergency stop state

// --- Setup ---
void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB on some boards
  }
  Serial.println("ESP32 Robot Initializing...");

  // Initialize motor control pins as outputs
  pinMode(motorLeftFront_Fwd, OUTPUT);
  pinMode(motorLeftFront_Bwd, OUTPUT);
  pinMode(motorLeftBack_Fwd, OUTPUT);
  pinMode(motorLeftBack_Bwd, OUTPUT);
  pinMode(motorRightFront_Fwd, OUTPUT);
  pinMode(motorRightFront_Bwd, OUTPUT);
  pinMode(motorRightBack_Fwd, OUTPUT);
  pinMode(motorRightBack_Bwd, OUTPUT);

  // Initialize enable pins if used
  // pinMode(enablePinLeft, OUTPUT);
  // pinMode(enablePinRight, OUTPUT);
  // digitalWrite(enablePinLeft, HIGH); // Enable motors by default if needed
  // digitalWrite(enablePinRight, HIGH); // Enable motors by default if needed

  // Initialize Button Pin
  pinMode(stopButtonPin, INPUT_PULLUP); // Use internal pull-up resistor

  // Ensure motors are stopped initially
  stop_move();
  Serial.println("Robot Ready.");
}

// --- Stop Function ---
void stop_move() {
  // Set all motor control pins to LOW
  digitalWrite(motorLeftFront_Fwd, LOW);
  digitalWrite(motorLeftFront_Bwd, LOW);
  digitalWrite(motorLeftBack_Fwd, LOW);
  digitalWrite(motorLeftBack_Bwd, LOW);
  digitalWrite(motorRightFront_Fwd, LOW);
  digitalWrite(motorRightFront_Bwd, LOW);
  digitalWrite(motorRightBack_Fwd, LOW);
  digitalWrite(motorRightBack_Bwd, LOW);
}

// --- Basic Movement Functions ---
// Move both left and right tracks forward
void forward_move(int throttle) {
  stop_move();
  // Left Forward
  analogWrite(motorLeftFront_Fwd, throttle);
  analogWrite(motorLeftBack_Fwd, throttle);
  digitalWrite(motorLeftFront_Bwd, LOW);
  digitalWrite(motorLeftBack_Bwd, LOW);
  // Right Forward
  analogWrite(motorRightFront_Fwd, throttle);
  analogWrite(motorRightBack_Fwd, throttle);
  digitalWrite(motorRightFront_Bwd, LOW);
  digitalWrite(motorRightBack_Bwd, LOW);
}

// Move both left and right tracks backward
void backward_move(int throttle) {
  stop_move();
  // Left Backward
  digitalWrite(motorLeftFront_Fwd, LOW);
  digitalWrite(motorLeftBack_Fwd, LOW);
  analogWrite(motorLeftFront_Bwd, throttle);
  analogWrite(motorLeftBack_Bwd, throttle);
  // Right Backward
  digitalWrite(motorRightFront_Fwd, LOW);
  digitalWrite(motorRightBack_Fwd, LOW);
  analogWrite(motorRightFront_Bwd, throttle);
  analogWrite(motorRightBack_Bwd, throttle);
}


// --- Tank Steer Turning Functions (Causes curved path with Mecanum) ---
// Turn Left (Tank steer: Left backward, Right forward)
void turn_left(int throttle) {
  stop_move();
  // Left Backward
  digitalWrite(motorLeftFront_Fwd, LOW);
  digitalWrite(motorLeftBack_Fwd, LOW);
  analogWrite(motorLeftFront_Bwd, throttle);
  analogWrite(motorLeftBack_Bwd, throttle);
  // Right Forward
  digitalWrite(motorRightFront_Bwd, LOW);
  digitalWrite(motorRightBack_Bwd, LOW);
  analogWrite(motorRightFront_Fwd, throttle);
  analogWrite(motorRightBack_Fwd, throttle);
}

// Turn Right (Tank steer: Left forward, Right backward)
void turn_right(int throttle) {
  stop_move();
  // Left Forward
  digitalWrite(motorLeftFront_Bwd, LOW);
  digitalWrite(motorLeftBack_Bwd, LOW);
  analogWrite(motorLeftFront_Fwd, throttle);
  analogWrite(motorLeftBack_Fwd, throttle);
  // Right Backward
  digitalWrite(motorRightFront_Fwd, LOW);
  digitalWrite(motorRightBack_Fwd, LOW);
  analogWrite(motorRightFront_Bwd, throttle);
  analogWrite(motorRightBack_Bwd, throttle);
}

// --- Mecanum Wheel Rotation Functions (Spinning in place) ---
void rotate_clockwise(int throttle) {
  stop_move();
  // Left Front: Forward
  analogWrite(motorLeftFront_Fwd, throttle);
  digitalWrite(motorLeftFront_Bwd, LOW);
  // Left Rear: Backward
  digitalWrite(motorLeftBack_Fwd, LOW);
  analogWrite(motorLeftBack_Bwd, throttle);
  // Right Front: Backward
  digitalWrite(motorRightFront_Fwd, LOW);
  analogWrite(motorRightFront_Bwd, throttle);
  // Right Rear: Forward
  analogWrite(motorRightBack_Fwd, throttle);
  digitalWrite(motorRightBack_Bwd, LOW);
}

void rotate_counter_clockwise(int throttle) {
  stop_move();
  // Left Front: Backward
  digitalWrite(motorLeftFront_Fwd, LOW);
  analogWrite(motorLeftFront_Bwd, throttle);
  // Left Rear: Forward
  analogWrite(motorLeftBack_Fwd, throttle);
  digitalWrite(motorLeftBack_Bwd, LOW);
  // Right Front: Forward
  analogWrite(motorRightFront_Fwd, throttle);
  digitalWrite(motorRightFront_Bwd, LOW);
  // Right Rear: Backward
  digitalWrite(motorRightBack_Fwd, LOW);
  analogWrite(motorRightBack_Bwd, throttle);
}

// --- Mecanum Wheel Strafing Functions (Sideways movement) ---
void strafe_left(int throttle) {
    stop_move();
    // Left Front: Backward
    digitalWrite(motorLeftFront_Fwd, LOW);
    analogWrite(motorLeftFront_Bwd, throttle);
    // Left Rear: Forward
    analogWrite(motorLeftBack_Fwd, throttle);
    digitalWrite(motorLeftBack_Bwd, LOW);
    // Right Front: Backward
    digitalWrite(motorRightFront_Fwd, LOW);
    analogWrite(motorRightFront_Bwd, throttle);
    // Right Rear: Forward
    analogWrite(motorRightBack_Fwd, throttle);
    digitalWrite(motorRightBack_Bwd, LOW);
}

void strafe_right(int throttle) {
    stop_move();
    // Left Front: Forward
    analogWrite(motorLeftFront_Fwd, throttle);
    digitalWrite(motorLeftFront_Bwd, LOW);
    // Left Rear: Backward
    digitalWrite(motorLeftBack_Fwd, LOW);
    analogWrite(motorLeftBack_Bwd, throttle);
    // Right Front: Forward
    analogWrite(motorRightFront_Fwd, throttle);
    digitalWrite(motorRightFront_Bwd, LOW);
    // Right Rear: Backward
    digitalWrite(motorRightBack_Fwd, LOW);
    analogWrite(motorRightBack_Bwd, throttle);
}


// --- Main Loop ---
void loop() {
  // --- Emergency Stop Check ---
  if (digitalRead(stopButtonPin) == LOW) {
    if (!emergencyStopped) {
      stop_move();
      Serial.println("EMERGENCY_STOP");
      emergencyStopped = true;
      while (Serial.available() > 0) { Serial.read(); }
    }
    delay(50);
    return;
  } else {
    if (emergencyStopped) {
      Serial.println("EMERGENCY_STOP_RELEASED");
      emergencyStopped = false;
    }
  }

  // --- Normal Operation ---
  if (!emergencyStopped) {
    // Enable motors if using enable pins
    // digitalWrite(enablePinLeft, HIGH);
    // digitalWrite(enablePinRight, HIGH);

    if (Serial.available()) {
      String command = Serial.readStringUntil('\n');
      command.trim();

      // Process the command
      if (command.equals("forward_move")) {
        forward_move(70);
      } else if (command.equals("stop")) {
        stop_move(); // First, ensure any current motion stops
        delay(20);   // Small pause (optional)
        forward_move(80); // Move forward briefly
        delay(100);  // <<< IMPORTANT: How long to move forward (e.g., 200ms)
        stop_move(); // Final stop
      } else if (command.equals("left_move")) { // Added strafing
          strafe_left(70);
      } else if (command.equals("right_move")) { // Added strafing
          strafe_right(70);
      }
      // Add other commands here if needed

    } else {
      //stop if no command received 
       stop_move();
    }
  } // end if (!emergencyStopped)
} // end loop
