#include <EEPROM.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include <PID_v1.h>

//
// External libraries used:
// - PID 1.2 Brett Beauregard

// class default I2C address is 0x68 
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 _mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

// ******************************************** Arduino pins ***********************************
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define M1 3
#define M2 4
#define SPD_PIN 5
#define MANUAL_OVERRIDE_PIN 6
#define CALIBRATE_PIN 7
#define LED_PIN 13
#define ACTUATOR_FEEDBACK_PIN A0
#define MANUAL_SETPOINT_PIN A1

uint16_t BASE_ADDRESS = 0;
uint16_t _packetSize;    // expected DMP packet size (default is 42 bytes)
bool _mpuInitialized = false;
bool _manualOperation = false;

double _output = 0;
double _setpoint = 0;
double _position = 0;
double _adjustedPosition = 0;
//========================================== USER ADJUSTMENTS ====================================
// External libraries used:
// - PID 1.2 Brett Beauregard
// 
// MPU6050 
//
// Connect pin 6 to ground for Potentiometer mode, leave open for MPU6050 mode
// In potentiometer mode, the code needs setpoint potentiometer connected to A1 and actuator feedback pot to A0
// both pots outer terminals connected to 5V & GND.
//
// MPU mode has MPU6050 connected to I2C pins
//
//************************************************ PID ADJUSTMENTS
double _kP = 25.0;
double _kI = 10.0;
double _kD = 0.0;

PID _control(&_adjustedPosition, &_output, &_setpoint, _kP, _kI, _kD, DIRECT);

//======================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

// indicates whether MPU interrupt pin has gone high
volatile bool _mpuInterrupt = false;     
void dmpDataReady() 
{ 
	  _mpuInterrupt = true;
}


void setup() 
{
    pinMode(MANUAL_OVERRIDE_PIN, INPUT_PULLUP);
    
    // Motor setup
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(SPD_PIN, OUTPUT);
    pinMode(CALIBRATE_PIN, INPUT_PULLUP);
    
    digitalWrite(M1, LOW);
    digitalWrite(M2, LOW);
    analogWrite(SPD_PIN, 0);

    // Pin 5 base PWM frequency is 62500. Set it to 15kHz
    setPwmFrequency(SPD_PIN, 1);
    
    Serial.begin(115200);

    pinMode(INTERRUPT_PIN, INPUT_PULLUP);
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    
    _manualOperation = (digitalRead(MANUAL_OVERRIDE_PIN) == LOW);
    if (_manualOperation)
    {
        _kP = 10.0;
        _kI = 5.0;
        _kD = 0;
        _control.SetTunings(_kP, _kI, _kD);
        Serial.println("Operating from feedback pot");
    }
  	else 
  	{
  	    Serial.println("Operating from IMU");
        if (!initMpu())
      	{
        		for (int i=0; i < 5; i++)
        		{
        			digitalWrite(LED_PIN, HIGH);
        			delay(200);
        			digitalWrite(LED_PIN, LOW);
        			delay(200);
        		}
        }
  	}
   
	_control.SetMode(AUTOMATIC);
	_control.SetOutputLimits(-255, 255);
  _control.SetSampleTime(5);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() 
{
    static int interval = 0;
	  double sensitivity = 1.0;
    float y, p, r;
    const int FILTER_SIZE = 16;
    static int filterBuf[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    static int index = 0;
 
    if (_manualOperation)
    {
        int manual = map(analogRead(MANUAL_SETPOINT_PIN), 0, 1024, -180, 180);
        _setpoint = manual;
        filterBuf[index++]= analogRead(ACTUATOR_FEEDBACK_PIN);
        if (index == FILTER_SIZE)
        {
            index = 0;
        }
       
         _adjustedPosition = map(medianAverage(filterBuf, FILTER_SIZE), 0, 1024, -180, 180);
    }
    else
    {
        if (getPositionData(&y, &p, &r))
        {
            _position = r * 180.0/M_PI;
            _adjustedPosition = _position;
        }     
    }
    // Calculate control value
    _control.Compute();
    int pwm = abs(_output);

	  bool dir = _output > 0 ? true : false;
	  digitalWrite(M1, dir);
	  digitalWrite(M2, !dir);
	  analogWrite(SPD_PIN, pwm);    

    interval++;
    if (interval == 100)
    {
        interval = 0;
        char buffer[100];
        sprintf(buffer, "Set=%d current=%d output=%d PWM=%d",(int)_setpoint, (int)_adjustedPosition, (int)_output, pwm);
        Serial.println(buffer);
    }
}

// Averaging median filter
double medianAverage(int* buffer, int length)
{
    double average = 0;
    int hiPos, lowPos, hi=-32767, low=32767;

    // Find high, low points
    for (int i =0; i < length; i++)
    {
        if (hi < buffer[i])
        {
            hiPos = i;
            hi = buffer[i];
        }
        if (low > buffer[i])
        {
            lowPos = i;
            low = buffer[i];
        }
    }
 
    int *newBuf = new int[length];
    int j = 0;
    for (int i = 0; i < length; i++)
    {
        if (i != hiPos && i != lowPos)
        {
            newBuf[j] = buffer[i];
            j++;
        }
        else
        {
            continue;
        }
    }

    int sum = 0;
    for (int i =0; i < length-2; i++)
    {
        sum += newBuf[i];
    }
    delete[] newBuf;
    average = sum/(length-2);
    return average;
}

bool getPositionData(float *yaw, float* pitch, float* roll)
{
    // MPU control/status vars
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    
    // orientation/motion vars
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    // if programming failed, don't try to do anything
    if (!_mpuInitialized) 
    {
        return false;
    }

    // wait for MPU interrupt or extra packet(s) available
    while (false)//LW_TODO !_mpuInterrupt && (fifoCount < _packetSize)) 
    {
        if (_mpuInterrupt && (fifoCount < _packetSize)) 
        {
          // try to get out of the infinite loop 
          fifoCount = _mpu.getFIFOCount();
        }  
    }
	
    // reset interrupt flag and get INT_STATUS byte
    _mpuInterrupt = false;
    uint8_t mpuIntStatus = _mpu.getIntStatus();

    // get current FIFO count
    fifoCount = _mpu.getFIFOCount();
    if(fifoCount < _packetSize)
    {
      //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
      // This is blocking so don't do it   while (fifoCount < _packetSize) fifoCount = _mpu.getFIFOCount();
    }
    // check for overflow (this should never happen unless our code is too inefficient)
    else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) 
    {
        // reset so we can continue cleanly
        _mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } 
    else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) 
    {
        // read a packet from FIFO
        while(fifoCount >= _packetSize)
        { 
          // Lets catch up to NOW, someone is using the dreaded delay()!
          _mpu.getFIFOBytes(fifoBuffer, _packetSize);
          // track FIFO count here in case there is > 1 packet available
          // (this lets us immediately read more without waiting for an interrupt)
          fifoCount -= _packetSize;
        }
        // display Euler angles in degrees
        _mpu.dmpGetQuaternion(&q, fifoBuffer);
        _mpu.dmpGetGravity(&gravity, &q);
        _mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        *yaw = ypr[0];
        *pitch = ypr[1];
        *roll = ypr[2];
    }
	return true;
}


bool initMpu()
{
    uint16_t address = BASE_ADDRESS;
    bool calibrated;
    EEPROM.get(address, calibrated);
    // Defaults to 0xFF so we have to invert it
    calibrated = !(calibrated & 0x01);
	  Serial.print("Calibrated status "); Serial.println(calibrated);	
	  // initialize device
	  Serial.println(F("Initializing I2C devices..."));
	
	  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    _mpu.initialize();

    // verify connection
    Serial.println(_mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    devStatus = _mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) 
    {
        int16_t Gx = 220;
        int16_t Gy = 76;
        int16_t Gz = -85;
        int16_t Az = 1788;

        // To calibrate, hold CALIBRATE_PIN low and reset
        if (!digitalRead(CALIBRATE_PIN))
        {
          // Calibration Time: generate offsets and calibrate our MPU6050
          _mpu.CalibrateAccel(6);
          _mpu.CalibrateGyro(6);

          Gx = _mpu.getXGyroOffset();
          Gy = _mpu.getYGyroOffset();
          Gz = _mpu.getZGyroOffset();
          Az = _mpu.getZAccelOffset();

          // Store readings
          // Indicate calibrated
          EEPROM.put(address+= sizeof(bool), 0);
          EEPROM.put(address+= sizeof(int16_t), Gx);
          EEPROM.put(address+= sizeof(int16_t), Gy);
          EEPROM.put(address+= sizeof(int16_t), Gz);
          EEPROM.put(address+= sizeof(int16_t), Az);
          calibrated = true;
          // Show calibration done
          for (int i = 0; i < 5; i++)
          {
            digitalWrite(LED_PIN, HIGH);
            delay(200);
            digitalWrite(LED_PIN, LOW);
            delay(400);           
          }
        }
        
        // Recall calibration
        if (calibrated)
        {
          address = BASE_ADDRESS + sizeof(bool);
          EEPROM.get(address+= sizeof(int16_t), Gx);
          EEPROM.get(address+= sizeof(int16_t), Gy);
          EEPROM.get(address+= sizeof(int16_t), Gz);
          EEPROM.get(address+= sizeof(int16_t), Az);
        }
        else
        {
          Serial.println(F("Needs calibration"));
        }
        
        _mpu.setXGyroOffset(Gx);
        _mpu.setYGyroOffset(Gy);
        _mpu.setZGyroOffset(Gz);
        _mpu.setZAccelOffset(Az);
        
        _mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        _mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt "));

        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));

        // get expected DMP packet size for later comparison
		    _packetSize = 42;//LW_TODO _mpu.dmpGetFIFOPacketSize();
		    _mpuInitialized = true;
       _mpu.resetFIFO();
    } 
    else 
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    return _mpuInitialized;
}

void setPwmFrequency(int pin, int divisor) 
{
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & (0b11111000 | mode);
    } else {
      TCCR1B = TCCR1B & (0b11111000 | mode);
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & (0b11111000 | mode);
  }
}
