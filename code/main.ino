#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL



#define INTERRUPT_PIN 12  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define MoterA_speed 11
#define MoterB_speed 10
#define MoterA_1 9
#define MoterA_2 8
#define MoterB_1 7
#define MoterB_2 6

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float pitch_180;

unsigned long Time_prev, Time_now, dt;
unsigned long ratio=0;

char mode = 5;
int motor_speed = 120;

float Error;
float Error_prev = 0;
float Error_sum = 0;
float target = 0;
float Output;
float Kp, Kd, Ki;


uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void forward()
{
      digitalWrite(MoterA_1, HIGH);
      digitalWrite(MoterA_2, LOW);
      digitalWrite(MoterB_1, HIGH);
      digitalWrite(MoterB_2, LOW);
}

void back()
{
      digitalWrite(MoterA_2, HIGH);
      digitalWrite(MoterA_1, LOW);
      digitalWrite(MoterB_2, HIGH);
      digitalWrite(MoterB_1, LOW);
}

void left()
{
      digitalWrite(MoterA_2, LOW);
      digitalWrite(MoterA_1, HIGH);
      digitalWrite(MoterB_2, HIGH);
      digitalWrite(MoterB_1, LOW);
}

void right()
{
      digitalWrite(MoterA_2, HIGH);
      digitalWrite(MoterA_1, LOW);
      digitalWrite(MoterB_2, LOW);
      digitalWrite(MoterB_1, HIGH);
}

void speed_motor(float data)
{
    if(data < 0 )
    {
       data = -data;
    }

    data = data + 49;
    
    analogWrite(MoterA_speed, data);
    analogWrite(MoterB_speed, data);  
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif


    Serial.begin(115200);
   
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    pinMode(MoterA_speed, OUTPUT);
    pinMode(MoterA_1, OUTPUT);
    pinMode(MoterA_2, OUTPUT);
    pinMode(MoterB_speed, OUTPUT);
    pinMode(MoterB_2, OUTPUT);
    pinMode(MoterB_2, OUTPUT);
    pinMode(12, OUTPUT);
    digitalWrite(12, HIGH);

     

    Time_prev = micros();
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    /*
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }
    */
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } 
    else if (mpuIntStatus & 0x02) 
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize)
          fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        #endif

    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
    pitch_180 = ypr[1] * 180 / M_PI;
   

//-------------------PID-----------------------------------------------
/*
    if(pitch_180 > 0)
    {
      digitalWrite(MoterA_1, HIGH);
      digitalWrite(MoterA_2, LOW);
      digitalWrite(MoterB_1, HIGH);
      digitalWrite(MoterB_2, LOW);
      
    }

    else if(pitch_180 < 0)
    {
      digitalWrite(MoterA_2, HIGH);
      digitalWrite(MoterA_1, LOW);
      digitalWrite(MoterB_2, HIGH);
      digitalWrite(MoterB_1, LOW);
      pitch_180 = pitch_180 * -1;
    }
    */
    Kp = 3;
    Ki = 0.001;
    
     mode = Serial.read();
     Time_now = micros();
     dt = ( Time_now - Time_prev ) / 1000;
     Time_prev = Time_now;

     Error = pitch_180 - target;
     Error_sum = Error_sum + Error * dt;
     Output = Kp * Error + Ki * Error_sum;

     ratio = Time_now % 10000;

     //if(ratio < 2)
      //{     
        if(Output > 0)
        {
          forward();
        }

        else if (Output < 0)
        {
          back();
        }

        speed_motor(Output); 
    /* }

     else
     {
      
        if(mode == '0')
        {
          forward();
          speed_motor(motor_speed);
        }
        
       if(mode == '1')
        {   
          back();
          speed_motor(motor_speed);
        }
        
       if(mode == '2')
        {   
          //left();
          //speed_motor(motor_speed);
        }

        if(mode == '3')
        {
          right();
          speed_motor(motor_speed);
        }

        if(mode == '4');

        if(mode == '5')
        {
          if((Time_now % 10000) == 0)
            motor_speed++;
        }

        if(mode == '6')
        {
          if((Time_now % 10000) == 0)
            motor_speed--;
        }
      }
     

      
     
/*
    if( digitalRead(2) == HIGH)
    {
      forward();
      speed_motor(100);
    }

    else if( digitalRead(13) == HIGH)
    {
      back();
      speed_motor(100);
    }
 */  
 
     Serial.print("pitch : ");
     Serial.print(pitch_180);
     Serial.print("\t Output : ");  
     Serial.println(Output);
  
}
