#include <Servo.h>

Servo esc1, esc2, esc3, esc4;

//test
int val = 0;
bool first = true;

void setup() {
  // put your setup code here, to run once:
  esc1.attach(3);
  esc2.attach(5);
  esc3.attach(6);
  esc4.attach(9);
  
//  Serial.begin(9600);
//  pinMode(13, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(first){
    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc4.writeMicroseconds(1000);
    delay(5000);
    esc1.writeMicroseconds(1100);
    esc2.writeMicroseconds(1100);
    esc3.writeMicroseconds(1100);
    esc4.writeMicroseconds(1100);
    delay(1000);
    first = false;
  }
  delay(100);
//  esc.writeMicroseconds(2000);
//  val = analogRead(A0);
  esc1.writeMicroseconds(1200);
  esc2.writeMicroseconds(1200);
  esc3.writeMicroseconds(1200);
  esc4.writeMicroseconds(1200);
//  Serial.println(val);
//  digitalWrite(13, HIGH);
//  delay(val);   
//  digitalWrite(13, LOW);
//  delay(val);
}
