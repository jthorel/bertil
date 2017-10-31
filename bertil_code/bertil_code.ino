#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <Hash.h>
#include "quaternionFilters.h"
#include "MPU9250.h"


// DEBUGGING
#define AHRS true         // Set to false for basic data read
#define SerialDebug false  // Set to true to get Serial output for debugging
#define drinkDebug false
#define wifi true
#define forceDebug false
#define waterDebug false
#define analogReadDebug false


// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling


//ESP8266WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(80);
const char* ssid     = "iPhone2";
const char* password = "sealer6040";
const int bluePin = 0;
const int redPin = 14;
const int greenPin = 15;
int value = 0;
 

MPU9250 myIMU;

// our vars
int drinks = 0;
float prevValue;
const int R_FORCE = 800;
const float VDC = 3.3;
const int EMPTY_VALUE = 200;
const int FILL_VALUE = 900;
const int DEF_WATER_GOAL = 2000;
unsigned long drinkTimer = 0;
unsigned long delta = 0;
unsigned long t = 0;
unsigned long blinkPrev;
unsigned long blinkMillis;
bool ledState;
bool drinking = false;
bool liftingState = false;
bool lastLiftingState = false;
bool hasBeenTilted = false;
int waterAmount = 0;
int drankAmount = 0;
int totalWaterIntake = 0;
int totalWaterIntakeWarning = 0;
unsigned long decayTimer = 0;
int waterGoal = DEF_WATER_GOAL; // in milliliters


const int waterDecayTime = 500; // when to decay water (in milliseconds, 60000 equals 1 minute)
const int waterDecayAmount = waterGoal / (24*60); // decay this much (ml) per waterDecayTime


// smoothing average resolution
int SAMPLES = 100;
int TOLERANCE = 5;


// drink warning levels
// in milliliters
int warningThreshold1 = 15;
int warningThreshold2 = 10;
int warningThreshold3 = 5;


// RESET
void resetValues(){
  drinks = 0;
  drinkTimer = 0;
  delta = 0;
  t = 0;
  waterAmount = 0;
  drankAmount = 0;
  totalWaterIntake = 0;
  totalWaterIntakeWarning = 0;
  decayTimer = 0;
  waterGoal = DEF_WATER_GOAL; // in milliliters
}


// function
// check if weight is "stable" and read
int analogSmoothRead(int pin) {
  bool timeout = false;
  int s_val[SAMPLES];
  int timer = 0;
  
  while(!timeout){
    // Gather sample data 
    float sampleSum = 0.0;
    for(int i = 0; i < SAMPLES; i++) {
      s_val[i] = analogRead(pin);
      sampleSum += s_val[i];
      delay(1); // set this to whatever you want
    }
    float meanSample = float(sampleSum)/float(SAMPLES);

    if(analogReadDebug){
      Serial.println("meansample: " + String(int(meanSample)));
    }
  
    // HOW TO FIND STANDARD DEVIATION
    // STEP 1, FIND THE MEAN. (We Just did.)
    // STEP 2, sum the squares of the differences from the mean
    float sqDevSum = 0.0;
    for(int i = 0; i < SAMPLES; i++) {
      // pow(x, 2) is x squared.
      sqDevSum += pow((meanSample - float(s_val[i])), 2);
    }
  
    // STEP 3, FIND THE MEAN OF THAT
    // STEP 4, TAKE THE SQUARE ROOT OF THAT
    float stDev = sqrt(sqDevSum/float(SAMPLES));
    
    // TADA, STANDARD DEVIATION.
    // this is in units of sensor ticks (0-1023)
    if(stDev < TOLERANCE) {
      // reading is stable enough
      return int(meanSample);
    }  
    timer += timer;
    if(timer > 500){
      Serial.println("Timed out");
      timeout = true;
    }
  }
  // if timed out
  return waterAmount;
}


// CHECK IF LIFTED AND TILTED OVER 90
void checkTilted() {
  // CHECK TILT WHEN TILTING > 90 degrees
  float tilt = myIMU.pitch;
  if(prevValue < -20 ){
    //setColor(0,255,0);
    if(tilt > -20){
      if(liftingState){
        hasBeenTilted = true; // tilted while lifting?
        if(forceDebug){
          Serial.println("tilted");
        }
      }
      //setColor(255,0,255);
      
    }
  }
  prevValue = tilt;
}

// SEnd variables to arduino
void messageToApp(){
  int cssFill = map(totalWaterIntake, 0, waterGoal, 300, 0);
  String s = String(cssFill) + " " + String(waterAmount) + " " + String(waterGoal);
  webSocket.broadcastTXT(s);
}


// DECAY total amount of water drank for warnings
void decayWater(){
  decayTimer = decayTimer + delta;
  if(decayTimer >= waterDecayTime){ 
    decayTimer = 0;
    if(totalWaterIntakeWarning > 0){
      totalWaterIntakeWarning = totalWaterIntakeWarning - waterDecayAmount;
    }
      
    if(waterDebug){
      Serial.println("Waterdecay:" + String(totalWaterIntakeWarning));
    }
  }
}

// Check the lifted state and state changes
// Measures the water amount when put down and has been tilted.
// Adds it to the total drink amount
void checkLifted(){
  if(analogSmoothRead(A0) < 20 ) {
    liftingState = true;
    if(forceDebug){
      Serial.println("lifted");
    }
  } else {
    liftingState = false;
    if(forceDebug){
      Serial.println(" not lifted");
    }
  }
  
  if(liftingState != lastLiftingState) { // state change
    if(!liftingState) {  // state changed from lifted to not lifted (set down bottle)
      int newWaterAmount = getML(analogSmoothRead(A0));
      if(hasBeenTilted){ // has it been tilted? (not "is it currently tilted")
        // read water amount in bottle
        
        Serial.println(" drank water! ");
        drinkTimer = 0;
        drinks++;

        // check difference
        if(newWaterAmount < waterAmount){ // filled it up?
           // water removed, drank water
          drankAmount = waterAmount - newWaterAmount;
          if(waterDebug){
            Serial.println("drank: " + String(drankAmount));
          }

          // add to total sum of water 
          totalWaterIntake = totalWaterIntake + drankAmount;    
          // reset the water sum timer
          totalWaterIntakeWarning = totalWaterIntake;
        }
        // update new water amount in bottle
        waterAmount = newWaterAmount;

        // reset has been tilted
        hasBeenTilted = false;
        
        if(forceDebug){
          Serial.println("total water: "+String(waterAmount));
          Serial.println("total consumed: "+String(totalWaterIntake));
        }        
      } else { // set down without drinking, filled up?
        waterAmount = newWaterAmount;
      }
      messageToApp();

    } 
  } else if(!liftingState && hasBeenTilted){  // it's not lifted and has not been "set down" (its grounded)
        hasBeenTilted = false;
        //waterAmount = analogRead(A0);
  }

  lastLiftingState = liftingState;
}


// Convert sensor reading to water amount
// 
int getML(int s){
  //float map_factor = 750/(FILL_VALUE-EMPTY_VALUE);
  s = constrain(s, EMPTY_VALUE, FILL_VALUE);
  int ml = map(s, EMPTY_VALUE, FILL_VALUE, 0, 750);
  if(analogReadDebug){
    Serial.println("getml:" + String(ml));
  }
  return ml;
}


// Change color on the rgb
void setColor(int red, int green, int blue)
{
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}


void blinkRGB(int r, int g, int b, int interval) {
  blinkMillis = millis();
  if (blinkMillis - blinkPrev >= interval) {
    // save the last time you blinked the LED
    blinkPrev = blinkMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == true) {
      ledState = false;
      setColor(0,0,0);
    } else {
      ledState = true;
      setColor(r,g,b);
    }
  }
}

void drinkWarning(int t){
  if(t == warningThreshold1){
    setColor(0,0,255);
  } else if(t == warningThreshold2){
    blinkRGB(0,0,255,200);
  } else if(t == warningThreshold3){
    blinkRGB(0,0,255,100);
  }
}


void checkDrinkWarning(){
  int ml = totalWaterIntakeWarning;
  //if(!hasBeenTilted){ // add lifted state
    if(ml < warningThreshold1){
      blinkRGB(0,0,255,100);
      //drinkWarning(warningThreshold3);
    } else if(ml < warningThreshold2){
      blinkRGB(0,0,255,200);
      //drinkWarning(warningThreshold2);
    } else if(ml < warningThreshold3){
      setColor(0,0,255);
      //drinkWarning(warningThreshold1);
    } else {
      setColor(0,0,0);    
    }
  //}   
}


// WEBSOCKET EVENT, RECEIVE MESSAGES
// prepend with different characters for different messages
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght) {
    switch(type) {
        case WStype_DISCONNECTED:
          break;
        case WStype_CONNECTED:
        {
          IPAddress ip = webSocket.remoteIP(num);
          Serial.print("IP");
          Serial.print(ip);
          Serial.println();  
          break;
        } 
        case WStype_TEXT:
        {
          String text = String((char *) &payload[0]);
          if(text=="LED"){
           
            digitalWrite(13,HIGH);
            delay(500);
            digitalWrite(13,LOW);
            Serial.println("led just lit");
            webSocket.sendTXT(num, "led just lit", lenght);
            webSocket.broadcastTXT("nåntong annat", lenght);
          }
            
          if(text.startsWith("x")){  
            String xVal=(text.substring(text.indexOf("x")+1,text.length())); 
            int xInt = xVal.toInt();
            analogWrite(redPin,xInt); 
            Serial.println(xVal);
            webSocket.sendTXT(num, "red changed", lenght);
          }


          if(text.startsWith("y")){
            
            String yVal=(text.substring(text.indexOf("y")+1,text.length())); 
            int yInt = yVal.toInt();
            analogWrite(greenPin,yInt); 
            Serial.println(yVal);
            webSocket.sendTXT(num, "green changed", lenght);
          }

          if(text.startsWith("z")){
            
            String zVal=(text.substring(text.indexOf("z")+1,text.length())); 
            int zInt = zVal.toInt();
            analogWrite(bluePin,zInt); 
            Serial.println(zVal);
            webSocket.sendTXT(num, "blue changed", lenght);
          }
          
          if(text=="reset"){
            resetValues();
            Serial.println("reset");
          }                         
          
          if(text=="fetch"){
            messageToApp();
          }
          //webSocket.sendTXT(num, payload, lenght);
          //webSocket.broadcastTXT(payload, lenght);
          break;
        }
        case WStype_BIN:
        {
          hexdump(payload, lenght);
          // echo data back to browser
          webSocket.sendBIN(num, payload, lenght);
          break;
        }
    }
}





void setup() {
    pinMode(bluePin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(redPin, OUTPUT);
    Serial.begin(115200);

    waterAmount = getML(analogSmoothRead(A0));
    if(wifi){
      WiFi.begin(ssid, password);
      setColor(255,0,0);
      
      while(WiFi.status() != WL_CONNECTED) {
          
          delay(100);
          Serial.print(".");
      }
  
      setColor(0,255,0);
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      webSocket.begin();
      webSocket.onEvent(webSocketEvent);      
    }

    Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);


  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x71, HEX);

    
  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    Serial.println(F("MPU9250 is online..."));

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[0],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[1],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[2],1); Serial.println("% of factory value");
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[3],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[4],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    //myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 ");
    Serial.print("I AM 0x");
    Serial.print(d, HEX);
    Serial.print(" I should be 0x");
    Serial.println(0x48, HEX);


    /*if (d != 0x48)
    {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }*/

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");

    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();



    // CALIBRATION CODE -----------------
    /* 
    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
    */
    Serial.println("AK8963 mag biases (mG)");
    Serial.println(myIMU.magBias[0]);
    Serial.println(myIMU.magBias[1]);
    Serial.println(myIMU.magBias[2]);

    Serial.println("AK8963 mag scale (mG)");
    Serial.println(myIMU.magScale[0]);
    Serial.println(myIMU.magScale[1]);
    Serial.println(myIMU.magScale[2]);
    delay(2000); // Add delay to see results before serial spew of data

    if(SerialDebug)
    {
      Serial.println("Magnetometer:");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }

  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }
}



//////////////////////////////////////////////////////
void loop() {

  unsigned long t2 = millis();
  delta = t2-t;
  t = t2;
  drinkTimer += delta;
  webSocket.loop();
  
  

  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {

    // ACCELDATA, USE FOR TAPS AND SHIT
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    // GYRO DATA
    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;


    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

//  if (!AHRS)
//  {
//    myIMU.delt_t = millis() - myIMU.count;
//    if (myIMU.delt_t > 500)
//    {
//      if(SerialDebug)
//      {
//        // Print acceleration values in milligs!
//        
//        Serial.print("X-acceleration: "); Serial.print(1000 * myIMU.ax);
//        Serial.print(" mg ");
//        Serial.print("Y-acceleration: "); Serial.print(1000 * myIMU.ay);
//        Serial.print(" mg ");
//        Serial.print("Z-acceleration: "); Serial.print(1000 * myIMU.az);
//        Serial.println(" mg ");
//        
//  
//        // Print gyro values in degree/sec
//        /*
//        Serial.print("X-gyro rate: "); Serial.print(myIMU.gx, 3);
//        Serial.print(" degrees/sec ");
//        Serial.print("Y-gyro rate: "); Serial.print(myIMU.gy, 3);
//        Serial.print(" degrees/sec ");
//        Serial.print("Z-gyro rate: "); Serial.print(myIMU.gz, 3);
//        Serial.println(" degrees/sec");*/
//
//        // Print mag values in degree/sec
//        /*
//        Serial.print("X-mag field: "); Serial.print(myIMU.mx);
//        Serial.print(" mG ");
//        Serial.print("Y-mag field: "); Serial.print(myIMU.my);
//        Serial.print(" mG ");
//        Serial.print("Z-mag field: "); Serial.print(myIMU.mz);
//        Serial.println(" mG");*/
//
//        /*
//        myIMU.tempCount = myIMU.readTempData();  // Read the adc values
//        // Temperature in degrees Centigrade
//        myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
//        // Print temperature in degrees Centigrade
//        Serial.print("Temperature is ");  Serial.print(myIMU.temperature, 1);
//        Serial.println(" degrees C");
//        */
//      }
//
//
//      myIMU.count = millis();
//      digitalWrite(myLed, !digitalRead(myLed));  // toggle led
//    } // if (myIMU.delt_t > 500)
//  } // if (!AHRS)
//  else
  //{
    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;
    
    // update LCD once per half-second independent of read rate
    if (myIMU.delt_t > 500)
    {
      if(SerialDebug)
      {
        /*
        Serial.print("ax = ");  Serial.print((int)1000 * myIMU.ax);
        Serial.print(" ay = "); Serial.print((int)1000 * myIMU.ay);
        Serial.print(" az = "); Serial.print((int)1000 * myIMU.az);
        Serial.println(" mg");
        

        Serial.print("gx = ");  Serial.print(myIMU.gx, 2);
        Serial.print(" gy = "); Serial.print(myIMU.gy, 2);
        Serial.print(" gz = "); Serial.print(myIMU.gz, 2);
        Serial.println(" deg/s");

        Serial.print("mx = ");  Serial.print((int)myIMU.mx);
        Serial.print(" my = "); Serial.print((int)myIMU.my);
        Serial.print(" mz = "); Serial.print((int)myIMU.mz);
        Serial.println(" mG");

        Serial.print("q0 = ");  Serial.print(*getQ());
        Serial.print(" qx = "); Serial.print(*(getQ() + 1));
        Serial.print(" qy = "); Serial.print(*(getQ() + 2));
        Serial.print(" qz = "); Serial.println(*(getQ() + 3));
        */
      }

      // Define output variables from updated quaternion---these are Tait-Bryan
      // angles, commonly used in aircraft orientation. In this coordinate system,
      // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
      // x-axis and Earth magnetic North (or true North if corrected for local
      // declination, looking down on the sensor positive yaw is counterclockwise.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the
      // Earth is positive, up toward the sky is negative. Roll is angle between
      // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
      // arise from the definition of the homogeneous rotation matrix constructed
      // from quaternions. Tait-Bryan angles as well as Euler angles are
      // non-commutative; that is, the get the correct orientation the rotations
      // must be applied in the correct order which for this configuration is yaw,
      // pitch, and then roll.
      // For more see
      // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
      // which has additional links.
      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                    * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                    * *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                    * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;

      // Declination of KTH
      //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      myIMU.yaw  -= 6.03;
      myIMU.roll *= RAD_TO_DEG;

      if(SerialDebug)
      {
        Serial.print("Yaw, Pitch, Roll: ");
        Serial.print(myIMU.yaw, 2);
        Serial.print(", ");
        Serial.print(myIMU.pitch, 2);
        Serial.print(", ");
        Serial.println(myIMU.roll, 2);

        /*
        Serial.print("rate = ");
        Serial.print((float)myIMU.sumCount / myIMU.sum, 2);
        Serial.println(" Hz");*/
      }


      // check if lifted and tilted 
      checkTilted();
      checkLifted();
      


      // BROADCAST MESSAGE
      
      //String v = "Drinks: " + String(drinks) + "<br/>Time since last drink: " + String(drinkTimer/60000) + " minutes<br/>Water in bottle: " + String((waterAmount)) + " ml<br/>You have consumed " + String((totalWaterIntake)) + " ml of water today";
      //webSocket.broadcastTXT(v);
      


      // PRINT VALUES
      if(drinkDebug){
        Serial.print("Water: ");Serial.println(String(waterAmount));
        //Serial.println("Drinks: " + String(drinks));
        //Serial.println(ml);
        //Serial.println(drinkTimer);
      }


      // for timing the IMU readings
      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
    } // if (myIMU.delt_t > 500)
  //} // if (AHRS)
  

  // State of the led
  checkDrinkWarning();

  // decay water
  decayWater();
  
}





