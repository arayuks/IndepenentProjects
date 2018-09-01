#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CSN_PIN 8
#define CE_PIN 7

#define JOYSTICK_VERT_PIN A1
#define JOYSTICK_HORI_PIN A0

RF24 radio(CE_PIN, CSN_PIN); // CE, CSN

const byte address[6] = "00001";

int inputValues[2] = {512, 512}; // Vertical val, Horizontal val.

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}
void loop() {
  int inputValues[2]; // Vertical val, Horizontal val.
  inputValues[0] = 1024 - analogRead(JOYSTICK_VERT_PIN);
  inputValues[1] = analogRead(JOYSTICK_HORI_PIN);
  
  radio.write((void*)inputValues, sizeof(int) * 2);

    Serial.print("Vertical: ");  
    Serial.print(inputValues[0]);  
    Serial.print(" Horizaontal: "); 
    Serial.println(inputValues[1]); 
    Serial.println("---------------------------------");
  
  delay(200);
}
