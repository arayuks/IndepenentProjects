// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
#include<Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
long GyX_cal,GyY_cal,GyZ_cal; 

const float rad_to_deg = 180/PI;

float Acceleration_angle[2];
float Acceleration_angle_cal[2];
float Gyro_angle[2];
float Total_angle[2];

unsigned long time, timePrev;
float elapsedTime;

void setup(){
  
  Wire.begin();
  Serial.begin(9600);
  initMPU();
  calibrateGyroAndAccel();
 

  time = millis();
}

// https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
void initMPU(){
  Serial.println("PMU 6050 initializing...");
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission();
  Wire.beginTransmission(MPU_addr); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4)
  Wire.write(0); //Setting the gyro to full scale +/- 250deg./s
  Wire.endTransmission();
  Wire.beginTransmission(MPU_addr); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
  Wire.write(0); //Setting the accel to +/- 2g
  Wire.endTransmission();  
}

void calibrateGyroAndAccel(){
  Serial.println("Gyro and Accel Calibrating ...");
  for(int i = 0; i < 2000; i++){
      readNonCalibratedGyroData();
      GyX_cal += GyX;
      GyY_cal += GyY;
      GyZ_cal += GyZ;

      Acceleration_angle_cal[0] += atan((AcY/16384.0)/sqrt(pow((AcX/16384.0),2) + pow((AcZ/16384.0),2)))*rad_to_deg;
      Acceleration_angle_cal[1] += atan(-1*(AcX/16384.0)/sqrt(pow((AcY/16384.0),2) + pow((AcZ/16384.0),2)))*rad_to_deg;
      
      delay(3);
  }
  GyX_cal /= 2000;
  GyY_cal /= 2000;
  GyZ_cal /= 2000;

  Acceleration_angle_cal[0] /= 2000;
  Acceleration_angle_cal[1] /= 2000;
  
  Serial.print("Gyro offsets-- X: ");
  Serial.print(GyX_cal /131.0);
  Serial.print(" Gyro offsets-- Y: ");
  Serial.print(GyY_cal /131.0);
  Serial.print(" Gyro offsets-- Z: ");
  Serial.println(GyZ_cal /131.0);
  Serial.println("Gyro Calibration finished.");
  Serial.print("Accel X angle offset: ");
  Serial.print(Acceleration_angle_cal[0]);
  Serial.print(" Accel Y angle offset: ");
  Serial.println(Acceleration_angle_cal[1]);  
  Serial.println("Accel Calibration finished.");
}

void readGyroData(){
  readNonCalibratedGyroData();
  GyX -= GyX_cal;
  GyY -= GyY_cal;
  GyZ -= GyZ_cal;
}

void readNonCalibratedGyroData(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void processGyroData(){
  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  elapsedTime = (time - timePrev) / 1000.0; 

  readGyroData();
 
//  Serial.print("AcX = "); Serial.print(AcX / 16384.0);
//  Serial.print(" | AcY = "); Serial.print(AcY / 16384.0);
//  Serial.print(" | AcZ = "); Serial.print(AcZ / 16384.0);
//  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
//  Serial.print(" | GyX = "); Serial.print(GyX/131.0);
//  Serial.print(" | GyY = "); Serial.print(GyY/131.0);
//  Serial.print(" | GyZ = "); Serial.println(GyZ/131.0);


  // X
  Acceleration_angle[0] = atan((AcY/16384.0)/sqrt(pow((AcX/16384.0),2) + pow((AcZ/16384.0),2)))*rad_to_deg;
  // Y
  Acceleration_angle[1] = atan(-1*(AcX/16384.0)/sqrt(pow((AcY/16384.0),2) + pow((AcZ/16384.0),2)))*rad_to_deg;

  /*--- account for angle offset ---*/
  Acceleration_angle[0] -= Acceleration_angle_cal[0];
  Acceleration_angle[1] -= Acceleration_angle_cal[1];

   /*--------------------------------------------------------------*/
   Gyro_angle[0] = GyX/131.0; 
   /*---Y---*/
   Gyro_angle[1] = GyY/131.0;

   /*Now in order to obtain degrees we have to multiply the degree/seconds
   *value by the elapsedTime.*/
   /*Finnaly we can apply the final filter where we add the acceleration
   *part that afects the angles and ofcourse multiply by 0.98 */

   /*---X axis angle---*/
   Total_angle[0] = 0.9 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.1 *Acceleration_angle[0];
   /*---Y axis angle---*/
   Total_angle[1] = 0.9 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.1  *Acceleration_angle[1];


   /*--------------------------------------------------------------*/

//  if(abs(GyX/131.0) > 10 || abs(GyY/131.0) > 10 || abs(GyZ/131.0) > 10){
//    Total_angle[0] = Acceleration_angle[0];
//    Total_angle[1] = Acceleration_angle[1]; 
//  } else {
//     Total_angle[0] = 0.7 * Total_angle[0] + 0.4 *  Acceleration_angle[0];
//     Total_angle[1] = 0.7 * Total_angle[1] + 0.4 *  Acceleration_angle[1];
//  }

}

void loop(){
  processGyroData();
  Serial.print("X angle : ");
  Serial.print(Total_angle[0]);
  Serial.print("  Y angle : ");
  Serial.println(Total_angle[1]);

/*--- TEST print outs ---*/

//  Serial.print("X angle : ");
//  Serial.print(Acceleration_angle[0]);
//  Serial.print("  Y angle : ");
//  Serial.println(Acceleration_angle[1]);

/*--- END TEST print outs ---*/

    
  delay(300);
}
