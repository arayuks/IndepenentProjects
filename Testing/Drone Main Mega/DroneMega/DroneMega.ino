#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

#define CSN_PIN 8
#define CE_PIN 7

Servo esc1, esc2, esc3, esc4;
RF24 radio(CE_PIN, CSN_PIN); // CE, CSN

const byte addr[6] = "00001";

int val = 0;
int countNoRes = 0;
bool first = true;

void setup() {
  esc1.attach(3);
  esc2.attach(5);
  esc3.attach(6);
  esc4.attach(9);
  
  radio.begin();
  radio.openReadingPipe(0, addr); //openReadingPipe(number 0-5 iindicating pipe#  , addr)
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  if(radio.available(0)){
    int inputValues[2]; // Vertical val, Horizontal val. 
    // up: pos, right : pos
    radio.read((void*)inputValues, sizeof(int) * 2);
    
    //temp
    val = inputValues[0];
    
    countNoRes = 0;
  } else {
    countNoRes++;
    if(countNoRes == 5){
      val = 512;
    }
  }
  delay(200);
  
  if(first){
    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc4.writeMicroseconds(1000);
    delay(5000);
    first = false;
  }
  
  esc1.writeMicroseconds(1000 + val - 512);
  esc2.writeMicroseconds(1000 + val - 512);
  esc3.writeMicroseconds(1000 + val - 512);
  esc4.writeMicroseconds(1000 + val - 512);
}
