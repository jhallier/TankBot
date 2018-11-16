
// Pins for the motor driver
int PW1 = 9;
int DR1 = 8;
int PW2 = 11;
int DR2 = 10;

// Length of the serial message
const int msg_len = 8;
char receivedData[msg_len];
boolean newData = false;
char mode = 'M';
int signal1 = 127;
int signal2 = 127;
int min_motor_speed = 80; // Minimum PWM value to start the DC motors (0-255)

void setup() {
  // Set baudrate. Make sure to select one with low error probability
  Serial.begin(38400);
  pinMode(PW1, OUTPUT);
  pinMode(DR1, OUTPUT);
  pinMode(PW2, OUTPUT);
  pinMode(DR2, OUTPUT);
}

/* Move robot with separate speeds for the left and right motor
*/
void M_move(int left, int right) {
  if (left > 127) {
    left = (left - 127) * 2;
    int left_fwd_spd = speed_mapping(left);
    forward_left(left_fwd_spd);
    Serial.print("Left motor forward");
    Serial.print(left_fwd_spd);
    Serial.print("\n");
  }
  else if (left < 127) {
    left = (127 - left) * 2;
    int left_bwd_spd = speed_mapping(left);
    backward_left(left_bwd_spd);
    Serial.print("Left motor backward");
    Serial.print(left_bwd_spd);
    Serial.print("\n");
  }
  else {
    forward_left(0);
  }
  if (right > 127) {
    right = (right - 127) * 2;
    int right_fwd_spd = speed_mapping(right);
    forward_right(right_fwd_spd);
    Serial.print("Right motor forward");
    Serial.print(right_fwd_spd);
    Serial.print("\n");
  }
  else if (right < 127) {
    right = (127 - right) * 2;
    int right_bwd_spd = speed_mapping(right);
    backward_right(right_bwd_spd);
    Serial.print("Right motor backward ");
    Serial.print(right_bwd_spd);
    Serial.print("\n");
  }
  else {
    forward_right(0);
  }
  newData = false;
}
    
/* Move robot with speed and steering signal.
The signal will be separated to the left and right motor to yield
a similar behavior like the steering command */
void S_move(int speed, int steering) {
   float steer_float = (steering - 127) / 127;

   // forward
   if (speed > 127) {
     int forward_speed = int((speed - 127) * 2);
     Serial.println("Forward at speed: ");
     Serial.print(forward_speed);
     Serial.println("Steering value: ");
     Serial.print(steer_float); 
     if (steer_float > 0) {
       forward_left(forward_speed);
       forward_right(int(forward_speed * steer_float));
     }
     else {
       forward_left(int((-1) * forward_speed * steer_float));
       forward_right(forward_speed);
     }
   }
   // backward
   else if (speed < 127) {
     int backward_speed = int((127 - speed) * 2);
     if (steer_float > 0) {
       backward_left(backward_speed);
       backward_right(int(backward_speed * steer_float));
     }
     else {
       backward_left(int((-1) * backward_speed * steer_float));
       backward_right(backward_speed);
     }
   }
  newData = false;   
}

// Reads serial data and stores in in char array
void receiveSerialData() {
  char endSerialChar = '\n'; // marks end of serial message
  char rx_byte = '0';
  int index = 1;
  
  if (Serial.available() > 0) {
    rx_byte = Serial.read();

    if ((rx_byte == 'M') || (rx_byte == 'S')) {

      receivedData[0] = rx_byte;
      
      while (rx_byte != endSerialChar) {

        if (Serial.available() > 0) {
          
          rx_byte = Serial.read();
          receivedData[index] = rx_byte;
          Serial.print("Index/Character: ");
          Serial.print(index);
          Serial.print(rx_byte);
          Serial.print("\n");

          ++index;
          if (index >= msg_len) {
            Serial.println("Message length exceeded!");
            index = msg_len - 1; // overwrites last character
          }
        }
      }
    }
    
    if (rx_byte == endSerialChar) {
      Serial.println("End character received, finishing serial message");
      receivedData[index] = '\0';
      newData = true;
    }
  }
}

int speed_mapping(int speed) {
  return int((float(speed) / 255.0) * (255 - min_motor_speed) + min_motor_speed);
}

void printReceivedData() {
  for (int i=0; i<msg_len; i++) {
    Serial.print(receivedData[i]);
  }
  Serial.print("\n");
}

boolean checkSignal() {
  boolean check = true;
  
  if ((receivedData[0] == 'M') || (receivedData[0] == 'S')) {
    int i = 1;
    while (i < (msg_len-1)) {
      if ((receivedData[i] < '0') || (receivedData[i] > '9')) {
        check = false;
        Serial.println("Data bit is wrong");
      }
      i++;
    }
    //Serial.println("Signal is correct");
  }
  else {
    //Serial.println("Check signal failed");
    check = false;
  }
  
  return check;
}

void convertSignal() {
  char str_signal1[4];
  char str_signal2[4];
  mode = receivedData[0];
    
  for (int i = 0; i < 3; i++) {
    str_signal1[i] = receivedData[i+1];
    str_signal2[i] = receivedData[i+4];
  }
  str_signal1[3] = '\0';
  str_signal2[3] = '\0';
  Serial.println("Signals for conversion:");
  Serial.println(str_signal1);
  Serial.println(str_signal2);
  
  signal1 = atoi(str_signal1);
  signal2 = atoi(str_signal2);
  Serial.print("Signals converted: ");
  Serial.print(signal1);
  Serial.print(signal2);
  Serial.print("\n");
}

void setSignalDefault() {
  int signal1 = 127;
  int signal2 = 127;
  int mode = 'M';
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

void loop() {
  
  // if new Data has been read in, send data to motors
  if (newData == true) {
    if (mode == 'S') {
      S_move(signal1, signal2);
    }
    else if (mode == 'M') {
      M_move(signal1, signal2);
    }
  }
  // otherwise, read new serial data. If successful, print the data
  // and if the data is correct, convert it to motor signals.
  else {
    receiveSerialData();
    if (newData == true) {
      printReceivedData();
      if (checkSignal() == true) {
        convertSignal();
      }
    }
  } 
}
  
  
