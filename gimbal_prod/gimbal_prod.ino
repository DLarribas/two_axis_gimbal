/*
  Production Gimbal code.

  This is the most up to date code. Changes will be added to this over time.

  sources:
    * jeff rowberg's I2Cdev.h and MPU6050.h header files borrowed from his github
    * that one dude who rules (arduino robotics book author)
    * http://www.geekmomprojects.com/mpu-6050-dmp-data-from-i2cdevlib/
    * MIT filters.pdf
    * MPU6050 PS and PM manuals/datasheets

  NOTE: this file requires the I2Cdev and MPU6050 libraries either installed to
  the arduino libraries directory, or in the path of the gimbal_prod.ino file.

  TODO: come up with better Gimbal name.
*/
 
#include "Wire.h"                
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Servo.h"

#define LED_PIN 13
#define LEAST_SENSITIVE_GYRO MPU6050_GYRO_FS_2000
#define LEAST_SENSITIVE_ACCEL MPU6050_ACCEL_FS_16
#define M_PI 3.14159265359

Servo roll_servo; // axis
Servo pitch_servo; // axis
MPU6050 mpu; 

bool blinkState=false;
bool debug = false; // set to true for testing/debugging.


// these variables hold sensor data
int16_t ax, ay, az;
int16_t gx, gy, gz;

//these are angles written to servos
float xAngle = 0.00;
float yAngle = 0.00;

//calibrator values (set by initial function)
int xGyro_calibrate, yGyro_calibrate;
int xAccel_calibrate, yAccel_calibrate;

//gyro variables
float xGyro_raw, yGyro_raw;
float xGyro_rate = 0.00;
float yGyro_rate = 0.00;
float xGyro_angle, yGyro_angle;
float xGyro_tst;
float yGyro_tst;

float xGyro_scale = 0.13;
float yGyro_scale = 0.13;

float gyro_time = 0.05; // possibly modify to increase gyro accuracy

//accelerometer variables
float xAccel_raw, yAccel_raw;
float xAccel_math, yAccel_math;
float xAccel_angle, yAccel_angle; 

int xAccel_scale, yAccel_scale;

//complementary filter scaling
double gyro_weight = 0.90;
double accel_weight = 0.10;

// timing parameters
int last_update;
int cycle_time;
long last_cycle = 0;
int timerVal = 50;


/**************************************************************
FUNCTION: setup()
	initializes i2c bus
	attaches servos to pins
		writes 90 to servo (initialization)
	set sensitivities and offsets
	calls calibration function for more precise baselines
**************************************************************/
void setup()
{
  Wire.begin();      // join I2C bus  
  Serial.begin(9600);    //  initialize serial communication

  if(debug == true){
    Serial.println("Initializing I2C devices...");
   // verify connection
    Serial.println("Testing device connections...");
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    
    Serial.println("\n\n\nUpdating internal sensor offsets...\n");
  }
  pinMode(LED_PIN, OUTPUT);  // configure LED pin
  offsetter();
  calibrate();
  if(debug==true){
    timerVal=120;
  }

  //attach servos to outputs 6 and 5. 
  //and write the nominal value(90) to set the operating point
  roll_servo.attach(6); //yellow
  roll_servo.write(90);
  pitch_servo.attach(5); //blue
  pitch_servo.write(90);

//  Serial.println("Initialization complete.....");

}


void offsetter(){
  mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
  mpu.setFullScaleGyroRange(LEAST_SENSITIVE_GYRO);
  mpu.setFullScaleAccelRange(LEAST_SENSITIVE_ACCEL);
  mpu.setXGyroOffset(40);
  mpu.setYGyroOffset(20);
  mpu.setSleepEnabled(false);
  //mpu.setZGyroOffset(-30);
  
  mpu.setXAccelOffset(-1250);
  mpu.setYAccelOffset(275);
//  mpu.setZAccelOffset(-850);
}

/***************************************************************
FUNCTION: calibrate
	gets 100 samples of gyro and accel data to further decrease 
  offset sensitivity

	//TODO: MAKE THIS FUNCTION AMAZING
***************************************************************/
void calibrate()
{
  int xGyro_avg, yGyro_avg;
  int xAccel_avg, yAccel_avg;
  long sample_size = 1000;
  // setup loop to read gyro 10 times
  for (int i = 0; i < sample_size; i++)
  {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // read gyro and each time, add the value to the running total
    xGyro_avg = xGyro_avg + gx; 
    xAccel_avg = xAccel_avg + ay;

    yGyro_avg = yGyro_avg + gy;
    yAccel_avg = yAccel_avg + ax; 
  }
  // with a sum of 10 readings, divide by 10 to get the average
  xGyro_calibrate = xGyro_avg / sample_size;
  xAccel_calibrate = xAccel_avg / sample_size;
  yGyro_calibrate = yGyro_avg / sample_size;
  yAccel_calibrate = yAccel_avg / sample_size;
//  Serial.print("xGyro_calibrate:");
//  Serial.print(xGyro_calibrate);
//  Serial.print("\tyGyro_calibrate:");
//  Serial.print(yGyro_calibrate);
//  Serial.print("\txAccel_calibrate:");
//  Serial.print(xAccel_calibrate);
//  Serial.print("\tyAccel_calibrate:");
//  Serial.println(yAccel_calibrate);
}


/***************************************************************
FUNCTION: sampleGyro()
  samples gyro data, storing data in global variables
***************************************************************/
void sampleGyro()
{
  mpu.getRotation(&gx, &gy, &gz);
  xGyro_raw = gx - xGyro_calibrate;
  xGyro_rate = (xGyro_raw * xGyro_scale) * gyro_time;
  xGyro_tst = xGyro_tst + xGyro_rate;
  xGyro_angle = xAngle + xGyro_rate;


  yGyro_raw = (gy - yGyro_calibrate) * -1;
  yGyro_rate = (yGyro_raw * yGyro_scale) * gyro_time;
  yGyro_tst = yGyro_tst + yGyro_rate;
  yGyro_angle = yAngle + yGyro_rate;
  //TODO: GET PRECISE LOOPTIME known as gyro_time


  // Serial.print("xGyro_raw: ");
  // Serial.print((int)xGyro_raw);
  // Serial.print("\tyGyro_raw: ");
  // Serial.print((int)yGyro_raw);
  // Serial.print("\txGyro_rate: ");
  // Serial.print((int)xGyro_rate);
  // Serial.print("\tyGyro_rate:");
  // Serial.print((int)yGyro_rate);
  // Serial.print("\txGyro_angle:");
  // Serial.print((int)xGyro_tst);
  // Serial.print("\tyGyro_angle:");
  // Serial.println((int)yGyro_tst);
}


/***************************************************************
FUNCTION sampleAccel()
  samples accelerometer data, storing data in global variables
  TODO: update to use arctan2f instead of lame simple sin()
***************************************************************/
void sampleAccel()
{
	//x portion (note, ay corresponds to gx)
	mpu.getAcceleration(&ax, &ay, &az);
  // three methods of approximating accel angle:
  // method one: arctan!
  	//xAccel_raw = ay - xAccel_calibrate;
  	//xAccel_math = atan2f(ay, (sqrt((ax^2+az^2))));
  	//xAccel_angle = (float)(xAccel_math * xAccel_scale);
  // method two: maping
    //xAccel_raw = (float)(map((ay), -2000, 2000, -90, 90)*1 -xAccel_calibrate);
    //xAccel_angle = xAccel_raw;
  // method three: small angle approximation
    //xAccel_raw = ay - xAccel_calibrate;
    //xAccel_angle = (float) xAccel_raw;

  //method4 experimenting
    xAccel_raw = ay/30; // - xAccel_calibrate;
    xAccel_angle = (float) xAccel_raw;


  // method one: arctan!
    //yAccel_raw = ax - yAccel_calibrate;
    //yAccel_math = atan2f(ax, (sqrt((ay^2+az^2))));
    //yAccel_angle = (float)(yAccel_math * yAccel_scale);
  //method two:
    //yAccel_raw = (float)(map((ax), -2000, 2000, -90, 90)*1 -yAccel_calibrate);
    //yAccel_angle = yAccel_raw;
    //method three
      //yAccel_raw = ax - yAccel_calibrate;
      //yAccel_angle = (float) yAccel_raw;


    //method4 experimenting
      yAccel_raw = ax/30;// - yAccel_calibrate;
      yAccel_angle = (float) yAccel_raw;
}

/***************************************************************
FUNCTION compute()
  computes the highpass/lowpass complementary filter
***************************************************************/
void compute()
{
  xAngle = (int)((gyro_weight * xGyro_angle) + (accel_weight * xAccel_angle));
  yAngle = (int)((gyro_weight * yGyro_angle) + (accel_weight * yAccel_angle));
}



/***************************************************************
FUNCTION execute()
  writes appropriate data to servo
***************************************************************/
void execute()
{
  //NOTE THESE MIGHT BE REVERSED!!!! also they might need to be mapped
  //if one is rolling/tilting wrong direction, multiply angle by -1

  pitch_servo.write(map(xAngle, -90, 90, 0, 180));
  roll_servo.write(map(yAngle, -90, 90, 0, 180));
}



/***************************************************************
FUNCTION timing()
  borrowed this function from arduino robotics textbook.
  pure genius.
***************************************************************/
void timing()
{
  // check to make sure it has been exactly timerVal milliseconds since the last recorded time-stamp
  while((millis() - last_cycle) < timerVal){
    delay(1);
  }
  // once the loop cycle reaches 50 mS, reset timer value and proceed
  cycle_time = millis() - last_cycle;
  last_cycle = millis();
}


/**************************************************************
FUNCTION printDEBUG()
  borrowed this function from arduino robotics textbook. 
  consider rewriting a bit.
**************************************************************/
void printDEBUG()
{
  // Debug with the Serial monitor
  if (debug == true)
  {
    Serial.print("xAccel: ");
    Serial.print(xAccel_angle);

    Serial.print("\t");
    Serial.print("xGyro:\t");
    if (xGyro_tst < 0)
      Serial.print(xGyro_tst);
    else 
    {
      Serial.print(" ");
      Serial.print(xGyro_tst);
    }

    Serial.print("\t");
        Serial.print("yAccel: ");
    Serial.print(yAccel_angle);

    Serial.print("\t");
    Serial.print("yGyro:\t");
    if (yGyro_tst < 0)
      Serial.print(yGyro_tst);
    else 
    {
      Serial.print(" ");
      Serial.print(yGyro_tst);
    }
    Serial.print("\t");
  }  

  Serial.print("Filtered xAngle: ");
  if (xAngle < 0)
    Serial.print(xAngle);
  else {
    Serial.print(" ");
    Serial.print(xAngle);
  }
  Serial.print("\t");
  Serial.print("Filtered yAngle: ");
  if (yAngle < 0)
    Serial.print(yAngle);
  else {
    Serial.print(" ");
    Serial.print(yAngle);
  }
  Serial.print(" ");
  Serial.print(" time: ");
  Serial.println(cycle_time); // print the loop cycle time
  //Serial.print ("\n");
}


/**************************************************************
FUNCTION: printCSV
  function prints data to serial monitor in hopes to save to
  csv file for use in plotting. note the xGyro_tst and yGyro_tst
  are NOT the "gyro angle values". they are there so that drift 
  can be seen

  
description: gyro x   ; accel x   ; filtered angle x     ; gyro y     ; accel y; filtered y;
**************************************************************/
void printCSV()
{
    Serial.print(xGyro_tst);
    Serial.print(", ");
    Serial.print(xAccel_angle);
    Serial.print(", ");
    Serial.print(xAngle);
    Serial.print(", ");
    Serial.print(yGyro_tst);
    Serial.print(", ");
    Serial.print(yAccel_angle);
    Serial.print(", ");
    Serial.print(yAngle);
    Serial.println("; ");
}

/**************************************************************
FUNCTION: graphME

inputs 
**************************************************************/
void graphME(char(axis))
{
  if(axis == 'x')
  {
    //reminder: xangle is filtered 
    Serial.print(xGyro_tst);
    Serial.print(" ");
    Serial.print(xAccel_angle);
    Serial.print(" ");
    Serial.println(xAngle);
  }
  
  if(axis == 'y')
  {
    //reminder: yAngle is filtered
    Serial.print(yGyro_tst);
    Serial.print(" ");
    Serial.print(yAccel_angle);
    Serial.print(" ");
    Serial.println(yAngle);
  }
  if(axis == 'z')
  {
    //reminder.... z is fking both axes, not the z fking axis
    Serial.print(xGyro_tst);
    Serial.print(" ");
    Serial.print(xAccel_angle);
    Serial.print(" ");
    Serial.print(xAngle);
    Serial.print(" ");
    Serial.print(yGyro_tst);
    Serial.print(" ");
    Serial.print(yAccel_angle);
    Serial.print(" ");
    Serial.println(yAngle);
  }
}

/***************************************************************
Function: MAIN LOOP
  performs data gathering, computing, and executing. uncomment
  printDEBUG or printCSV for debugging/extracting data.
***************************************************************/
void loop()
{
  sampleGyro();
  sampleAccel();
  compute();
  execute();
  timing();
  //printDEBUG();
  //printCSV();
  graphME('x');
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}