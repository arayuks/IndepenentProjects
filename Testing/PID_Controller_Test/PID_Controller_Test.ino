// test pid sketch.

#define CALCURATION_INTERVAL 0.3

const float Kp;
const float Ki;
const float Kd;

unsigned long time, prevTime;
float dt;  //in seconds
float goalPos, currentPos;
float error, errorSum, dError, prevError;

float pidOutput;



void setup() {
  // put your setup code here, to run once:
  prevError = 0;
  prevTime = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  time = millis();
  dt = (prevTime - time) / 1000;

  if(dt >= CALCURATION_INTERVAL){
    error = goalPos - currentPos;
    errorSum += error * dt;
    dError = (error - prevError) / dt;

    // when we use this output, we need to make sure that total throttle is at least 1000 and at most 2000.
    pidOutput = Kp * error + Ki * errorSum + Kd * dError;
    
    prevError = error;
    prevTime = time;
  }

}
