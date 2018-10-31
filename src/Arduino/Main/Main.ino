
// Pins for the motor driver
int PW1 = 8;
int DR1 = 9;
int PW2 = 10;
int DR2 = 11;

void setup() {
  Serial.begin(9600);
  //Serial.println("Set motor speed (0-255)");
  pinMode(PW1, OUTPUT);
  pinMode(DR1, OUTPUT);
  pinMode(PW2, OUTPUT);
  pinMode(DR2, OUTPUT);
}

// 255 = full speed
void forward_left(int speed) {
  digitalWrite(DR1, LOW);
  analogWrite(PW1, speed);
}

// 255 = full speed
void forward_right(int speed) {
  digitalWrite(DR2, LOW);
  analogWrite(PW2, speed);
}

void backward_left(int speed) {
  digitalWrite(PW1, LOW);
  analogWrite(DR1, speed);
}

void backward_right(int speed) {
  digitalWrite(PW2, LOW);
  analogWrite(DR2, speed);
}

char rx_byte = 0;
String rx_str = "";
boolean not_number = false;
int speed = 0;

void loop() {

  if (Serial.available() > 0) {
    rx_byte = Serial.read();  
    if ((rx_byte >= '0') && (rx_byte <= '9')) {
      rx_str += rx_byte;
    }
    else if (rx_byte == '\n') {
      // end of string
      if (not_number) {
        Serial.println("Not a number");
      }
      else {
        speed = rx_str.toInt();
        Serial.println("Speed: ");
        Serial.print(speed);
        forward_right(speed);
        forward_left(speed);
      }
      not_number = false;         // reset flag
      rx_str = "";                // clear the string for reuse
    }
    else {
      not_number = true;    // flag a non-number
    }
  } 
 
}
  
  
