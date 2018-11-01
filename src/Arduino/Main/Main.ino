
// Pins for the motor driver
int PW1 = 8;
int DR1 = 9;
int PW2 = 10;
int DR2 = 11;

// Length of the serial message
int msg_len = 6;

void setup() {
  Serial.begin(9600);
  //Serial.println("Set motor speed (0-255)");
  pinMode(PW1, OUTPUT);
  pinMode(DR1, OUTPUT);
  pinMode(PW2, OUTPUT);
  pinMode(DR2, OUTPUT);
}

void move(int speed, int steering) {
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

int speed = 127;
int steering = 127;

void loop() {

  if (Serial.available() > 0) {
    rx_str = "";
    rx_byte = Serial.read();

    while ((rx_byte >= '0') && (rx_byte <= '9') && Serial.available()) {
      rx_str += rx_byte;
      rx_byte = Serial.read();
    }
    
    Serial.println("Serial string length: ");
    Serial.print(rx_str.length());
    
    if ((rx_byte == '\n') && (rx_str.length() == msg_len)) {
        
      String speed_str = "";
      String steering_str = "";
      
      for (int i=0; i<=2; i++) {
        speed_str += rx_str[i];
      }
      speed = speed_str.toInt();
      
      for (int i=3; i<=5; i++) {
        steering_str += rx_str[i];
      }
      steering = steering_str.toInt();
    }
    else {
      speed = 127;
      steering = 127;
    }
    
    //Serial.println("Speed and steering values: ");
    //Serial.print(speed);
    //Serial.print(steering);
    
    move(speed, steering);
  } 
}
  
  
