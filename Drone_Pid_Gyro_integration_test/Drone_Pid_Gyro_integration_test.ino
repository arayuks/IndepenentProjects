/*** radio ***/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

/*** ESC ***/
#include <Servo.h>

/*** Gyro ***/
#include<Wire.h>


/*** ESC pwm pins  ***/
#define FRONT_RIGHT 10
#define FRONT_LEFT 5
#define BACK_RIGHT 6
#define BACK_LEFT 9


/*** Gyro pins ***/
#define CSN_PIN 8
#define CE_PIN 7

/*** PID ***/
#define CALCURATION_INTERVAL 0.3

#define STATUS_LED 12


/*** Radio ***/
RF24 radio(CE_PIN, CSN_PIN); // CE, CSN
const byte addr[6] = "00001";
int countNoRes = 0;

/*** ESC ***/
Servo escFR, escFL, escBR, escBL;
int val = 0;

/*** Gyro ***/
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
long GyX_cal,GyY_cal,GyZ_cal; 

const float rad_to_deg = 180/PI;

float Acceleration_angle[2];  // [x, y]
float Acceleration_angle_cal[2];
float Gyro_angle[2];
float Total_angle[2];

unsigned long gyroTime, gyroTimePrev;
float gyroElapsedTime;



/*** PID ***/
const float Kp = 0.0;
const float Ki = 0.0;
const float Kd = 0.0;

unsigned long pidTime, pidPrevTime;
float dt;  //in seconds

float goalPosX, currentPosX;
float goalPosY, currentPosY;
float errorX, errorSumX, dErrorX, prevErrorX;
float errorY, errorSumY, dErrorY, prevErrorY;

float pidOutputX;
float pidOutputY;

bool runPID;

/*
 * single 500ms flash - post radio
 * 
 * fast flashes x 20 - gyro config
 * 1 sec flash - gyro done
 * 
 * 7 sec continuous - esc init
 */


void setup() {
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  /*** ESC ***/
  escFR.attach(FRONT_RIGHT);
  escFL.attach(FRONT_LEFT);
  escBR.attach(BACK_RIGHT);
  escBL.attach(BACK_LEFT);
  initESC();

  
  /*** Radio ***/
  radio.begin();
  radio.openReadingPipe(0, addr); //openReadingPipe(number 0-5 iindicating pipe#  , addr)
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  
  flashStatusLED(500);

  /*** Gyro ***/
  Wire.begin();
  initMPU();
  calibrateGyroAndAccel();

  /*** PID ***/
  prevErrorX = 0;
  prevErrorY = 0;

  /*** timers ***/
  gyroTime = millis();
  pidPrevTime = gyroTime;  


  /********** TEMP *********/
  goalPosX = 0;
  goalPosY = 0;
}

void loop() {
  processGyroData();
  radioFetch();
  pidPrep();

  /*-- x --*/
  pid(goalPosX, Total_angle[0], &errorX, &errorSumX, &dErrorX, &prevErrorX, &pidOutputX);
  /*-- y --*/
  pid(goalPosY, Total_angle[1], &errorY, &errorSumY, &dErrorY, &prevErrorY, &pidOutputY);

  commandESC(&escFR, (val < 512 ? 0:val-512), -pidOutputX, pidOutputY);
  commandESC(&escFL, (val < 512 ? 0:val-512), -pidOutputX, -pidOutputY);
  commandESC(&escBR, (val < 512 ? 0:val-512), pidOutputX, pidOutputY);
  commandESC(&escBL, (val < 512 ? 0:val-512), pidOutputX, -pidOutputY);

  delay(100);
}


/*--------------------------------------------------------------------------------------*/
/*-------------------------------          RADIO         -------------------------------*/
/*--------------------------------------------------------------------------------------*/
/*
 * void radioFetch()
 */
 
// read user inputs from radio and process.
void radioFetch(){
  // check whether if radio is available or not.
  if(radio.available(0)){
    digitalWrite(STATUS_LED, HIGH);
    int inputValues[2]; // Vertical val, Horizontal val. 
    // up: pos, right : pos
    radio.read((void*)inputValues, sizeof(int) * 2);
    
    //temp, as throttle. raw imput value.
    val = inputValues[0];

    // reset count of no responses.
    countNoRes = 0;
    delay(25);
    digitalWrite(STATUS_LED, LOW);
  } else {  // if radio is unavailable for few times in a row, reset values.
    countNoRes++;
    if(countNoRes == 10){
      // TODO: change this value to desired behavior of drone when disconnected.
      val = 0;
    }
  }
}

/*--------------------------------------------------------------------------------------*/
/*--------------------------------          GYRO         -------------------------------*/
/*--------------------------------------------------------------------------------------*/
/*
 * void initMPU()
 * void calibrateGyroAndAccel()
 * void readGyroData()
 * void readNonCalibratedGyroData()
 * void processGyroData()
 * 
 */


// https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
void initMPU(){
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
  for(int i = 0; i < 2000; i++){
    if(i%100 == 0){
      flashStatusLED(100);
    }
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
  
  flashStatusLED(1000);
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
  gyroTimePrev = gyroTime;  // the previous time is stored before the actual time read
  gyroTime = millis();  // actual time read
  gyroElapsedTime = (gyroTime - gyroTimePrev) / 1000.0; 

  readGyroData();
  
  /*---X---*/
  Acceleration_angle[0] = atan((AcY/16384.0)/sqrt(pow((AcX/16384.0),2) + pow((AcZ/16384.0),2)))*rad_to_deg;
  /*---Y---*/
  Acceleration_angle[1] = atan(-1*(AcX/16384.0)/sqrt(pow((AcY/16384.0),2) + pow((AcZ/16384.0),2)))*rad_to_deg;

  /*--- account for angle offset ---*/
  Acceleration_angle[0] -= Acceleration_angle_cal[0];
  Acceleration_angle[1] -= Acceleration_angle_cal[1];

  /*---X---*/
  Gyro_angle[0] = GyX/131.0; 
  /*---Y---*/
  Gyro_angle[1] = GyY/131.0;
  
  /*---X axis angle---*/
  Total_angle[0] = 0.9 *(Total_angle[0] + Gyro_angle[0]*gyroElapsedTime) + 0.1 *Acceleration_angle[0];
  /*---Y axis angle---*/
  Total_angle[1] = 0.9 *(Total_angle[1] + Gyro_angle[1]*gyroElapsedTime) + 0.1  *Acceleration_angle[1];
}



/*------------------------------------------------------------------------------------*/
/*-------------------------------          ESC         -------------------------------*/
/*------------------------------------------------------------------------------------*/
/*
 * void initESC()
 * void commandESC(Servo *esc, float throttle, float pidx, float pidy)
 * 
 */

  // initialied ESC by sending base signal of 1000us for 7 seconds.
  void initESC(){
    digitalWrite(STATUS_LED, HIGH);
    escFR.writeMicroseconds(1000);
    escFL.writeMicroseconds(1000);
    escBR.writeMicroseconds(1000);
    escBL.writeMicroseconds(1000);
    delay(7000);
    
    digitalWrite(STATUS_LED, LOW);
  }

  /**   throttle: wanted power
   *    pidx: pid output offset for x
   *    pidy: pid output offset for y
   */
  void commandESC(Servo *esc, float throttle, float pidx, float pidy){
    int output = (int) (1000 + throttle + pidx + pidy);
    if(output < 1000){
      output = 1000;  
    } else if (output > 1600){                                                           // NERFED FOR TESTING
      output = 1600;
    }
    (*esc).writeMicroseconds(output);
  }




/*------------------------------------------------------------------------------------*/
/*-------------------------------          PID         -------------------------------*/
/*------------------------------------------------------------------------------------*/
/*
 * void pidPrep()
 * void pid(float goalPos, float currentPos,
 *          float *error, float *errorSum, float *dError, float *prevError,
 *          float *pidOutput)
 * 
 */



void pidPrep(){              
  pidTime = millis();
  dt = (pidPrevTime - pidTime) / 1000;
  if(dt >= CALCURATION_INTERVAL){
    pidPrevTime = pidTime;
    runPID = true;
  } else {
    runPID = false;
  }
}

void pid(float goalPos, float currentPos,
          float *error, float *errorSum, float *dError, float *prevError,
          float *pidOutput){

  if(runPID){
    *error = goalPos - currentPos;
    *errorSum += *error * dt;
    *dError = (*error - *prevError) / dt;

    // when we use this output, we need to make sure that total throttle is at least 1000 and at most 2000.
    *pidOutput = Kp * *error + Ki * *errorSum + Kd * *dError;
    
    *prevError = *error;
  }
}




/*------------------------------------------------------------------------------------*/
/*---------------------------          Utility         -------------------------------*/
/*------------------------------------------------------------------------------------*/
/*
 * void flashStatusLED(int t)
 * 
 */

void flashStatusLED(int t){
  digitalWrite(STATUS_LED, HIGH);
  delay(t);
  digitalWrite(STATUS_LED, LOW);
  delay(t/2);
}

 



  

