
#define _PWM_LOGLEVEL_        0

#if ( defined(ARDUINO_NANO_RP2040_CONNECT) || defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ADAFRUIT_FEATHER_RP2040) || \
      defined(ARDUINO_GENERIC_RP2040) ) && defined(ARDUINO_ARCH_MBED)

#if(_PWM_LOGLEVEL_>3)
  #warning USING_MBED_RP2040_PWM
#endif

#elif ( defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ADAFRUIT_FEATHER_RP2040) || \
        defined(ARDUINO_GENERIC_RP2040) ) && !defined(ARDUINO_ARCH_MBED)

#if(_PWM_LOGLEVEL_>3)
  #warning USING_RP2040_PWM
#endif
#else
#error This code is intended to run on the RP2040 mbed_nano, mbed_rp2040 or arduino-pico platform! Please check your Tools->Board setting.
#endif



enum {
    OPCODE_read,
    OPCODE_write
};

enum {
    VAR_setpoint = 0b0100,

    VAR_kP = 0b0101,
    VAR_kI = 0b0110,
    VAR_kD = 0b0111,

    VAR_err_I = 0b1000,

    VAR_err_P_limits = 0b1001,
    VAR_err_I_limits = 0b1010,

    // special
    CMD_stream_start = 0b0001,
    CMD_stream_stop = 0b0000,
    CMD_lock_start = 0b0010,
    CMD_lock_stop = 0b0011,
    CMD_save_to_eeprom = 0b1011
};

enum {
    RESULT_ok,
    RESULT_error
};

typedef struct request {
    unsigned char _reserved: 3;
    unsigned char var_cmd : 4;
    unsigned char opcode : 1;
} request_t;

typedef struct response {
    unsigned char _reserved: 2;
    unsigned char result : 1;
    unsigned char var_cmd : 4;
    unsigned char opcode : 1;
} response_t;

int process_request(unsigned char *request_buf);

#include "RP2040_PWM.h"

#define SDA_PIN 8
#define SCL_PIN 9

#define pinToUse      2

RP2040_PWM* PWM_Instance;

float frequency;
float dutyCycle;

char dashLine[] = "=============================================================";

#include "SHT85.h"

#define SHT85_ADDRESS         0x44

uint32_t start;
uint32_t stop;

SHT85 sht(SHT85_ADDRESS);

#include <PID_v2.h>
// #include <PID_v2.h>
//Define Variables we'll be connecting to
float Setpoint, Input;
int i;
float Input_acc;

//Specify the links and initial tuning parameters
// PID myPID(&Input, &Output, &Setpoint, 2, 4, 1, P_ON_M, DIRECT); //P_ON_M specifies that Proportional on Measurement be used

static volatile bool stream_run = false;

static float setpoint = 27.0f;
static float kP = 2.0f;
static float kI = 3.0f;
static float kD = 1.0f;
static float err_I = 0.0f;
static float err_P_limits[2] = {-3500.0f, 3500.0f};
static float err_I_limits[2] = {-6500.0f, 6500.0f};

#define REQUEST_RESPONSE_BUF_SIZE (sizeof(char)+2*(sizeof(float)))  // same size for both requests and responses
unsigned char buf[REQUEST_RESPONSE_BUF_SIZE];  // message buffer (both for receiving and sending)

#define STREAM_BUF_SIZE (sizeof(char)+2*sizeof(float))
#define STREAM_THREAD_SLEEP_TIME_MS 20
static float stream_values[2];

#define STREAM_PREFIX 0b00000001
unsigned char stream_buf[STREAM_BUF_SIZE];

PID_v2 myPID(kP, kI, kD, PID::Direct, PID::P_On::Measurement);

// float analogscale=30.22*3.3/4095.0;
// float vcurrent = 2.5;        //current piezo voltage
float output = 0;

void printPWMInfo(RP2040_PWM* PWM_Instance)
{
  uint32_t div = PWM_Instance->get_DIV();
  uint32_t top = PWM_Instance->get_TOP();

  Serial.print("Actual PWM Frequency = ");
  Serial.println(PWM_Instance->getActualFreq());

  PWM_LOGDEBUG5("TOP =", top, ", DIV =", div, ", CPU_freq =", PWM_Instance->get_freq_CPU());
}

void stream_start(void) {
    if (!stream_run)
        stream_run = true;
}

void stream_stop(void) {
    if (stream_run) {
        stream_run = false;
    }
}

int process_request(unsigned char *request_response_buf) {
    int result = 0;

    /*
     *  Currently we use the same one buffer for both parsing the request and constructing the response. As
     *  corresponding bit fields are match each other we can map the real request byte to the response structure. Also,
     *  the first byte of the response buffer is the same as the first one of the request except the result field.
     *
     *  Such approach looks more messy but, guess, should be faster to execute in hardware
     */
    response_t request;
    // request_t request;
    memcpy(&request, &request_response_buf[0], sizeof(char));

    if (request.opcode == OPCODE_read) {
        //printf("read: ");
        // 'read' request from the client - we do not need cells allocated for values (doesn't care whether they were
        // supplied or not). Instead, we will use them to return values
        memset(&request_response_buf[1], 0, 2*sizeof(float));
        
        switch (request.var_cmd) {
            case CMD_stream_stop:
                //printf("CMD_stream_stop\n");
                stream_stop();
                result = RESULT_ok;
                break;
            case CMD_stream_start:
                //printf("CMD_stream_start\n");
                stream_start();
                result = RESULT_ok;
                break;

            case CMD_lock_stop:
                //printf("CMD_stream_stop\n");
                myPID.SetMode(PID::Manual);
                result = RESULT_ok;
                break;
            case CMD_lock_start:
                //printf("CMD_stream_start\n");
                output = 0.0f;
                myPID.SetMode(PID::Automatic);
                result = RESULT_ok;
                break;

            case VAR_setpoint:
                //printf("VAR_setpoint\n");
                memcpy(&request_response_buf[1], &setpoint, sizeof(float));
                result = RESULT_ok;
                break;
            case VAR_kP:
                //printf("VAR_kP\n");
                memcpy(&request_response_buf[1], &kP, sizeof(float));
                result = RESULT_ok;
                break;
            case VAR_kI:
                //printf("VAR_kI\n");
                memcpy(&request_response_buf[1], &kI, sizeof(float));
                result = RESULT_ok;
                break;
            case VAR_kD:
                //printf("VAR_kD\n");
                memcpy(&request_response_buf[1], &kD, sizeof(float));
                result = RESULT_ok;
                break;
            case VAR_err_I:
                //printf("VAR_err_I\n");
                memcpy(&request_response_buf[1], &err_I, sizeof(float));
                result = RESULT_ok;
                break;
            case VAR_err_P_limits:
                //printf("VAR_err_P_limits\n");
                memcpy(&request_response_buf[1], err_P_limits, 2*sizeof(float));
                result = RESULT_ok;
                break;
            case VAR_err_I_limits:
                //printf("VAR_err_I_limits\n");
                memcpy(&request_response_buf[1], err_I_limits, 2*sizeof(float));
                result = RESULT_ok;
                break;

            case CMD_save_to_eeprom:
                //printf("CMD_save_to_eeprom\n");
                result = RESULT_ok;
                break;

            default:
                //printf("Unknown request\n");
                result = RESULT_error;
                break;
        }
    }
    
    else {
        //printf("write: ");
        
        switch (request.var_cmd) {
            case VAR_setpoint:
                //printf("VAR_setpoint\n");
                memcpy(&setpoint, &request_response_buf[1], sizeof(float));
                myPID.Setpoint(setpoint);
                result = RESULT_ok;
                break;
            case VAR_kP:
                //printf("VAR_kP\n");
                memcpy(&kP, &request_response_buf[1], sizeof(float));
                myPID.SetTunings(kP, kI, kD);
                result = RESULT_ok;
                break;
            case VAR_kI:
                //printf("VAR_kI\n");
                memcpy(&kI, &request_response_buf[1], sizeof(float));
                myPID.SetTunings(kP, kI, kD);
                result = RESULT_ok;
                break;
            case VAR_kD:
                //printf("VAR_kD\n");
                memcpy(&kD, &request_response_buf[1], sizeof(float));
                myPID.SetTunings(kP, kI, kD);
                result = RESULT_ok;
                break;
            case VAR_err_I:
                //printf("VAR_err_I\n");
                if (*(float *)&request_response_buf[1] == 0.0f) {
                    err_I = 0.0f;
                    result = RESULT_ok;
                }
                else {
                    result = RESULT_error;
                }
                break;
            case VAR_err_P_limits:
                //printf("VAR_err_P_limits\n");
                memcpy(err_P_limits, &request_response_buf[1], 2*sizeof(float));
                result = RESULT_ok;
                break;
            case VAR_err_I_limits:
                //printf("VAR_err_I_limits\n");
                memcpy(err_I_limits, &request_response_buf[1], 2*sizeof(float));
                result = RESULT_ok;
                break;

            default:
                //printf("Unknown request\n");
                result = RESULT_error;
                break;
        }
        
        memset(&request_response_buf[1], 0, 2*sizeof(float));
    }

    request.result = result;
    memcpy(request_response_buf, &request, sizeof(char));
    //    response.result = result;
    //    memcpy(response_buf, &response, sizeof(char));

    return result;
}


void setup() {
  memset(buf, 0, REQUEST_RESPONSE_BUF_SIZE);  // explicitly reset the buffer
  // put your setup code here, to run once:
  // analogWriteResolution(12);
  
  // // initialize digital pin 13 as an output.
  // pinMode(7, INPUT);
  // pinMode(13, OUTPUT);
  // pinMode(A0, INPUT);
  // pinMode(DAC0, OUTPUT);
  // pinMode(52, INPUT);
  
  // Serial.begin(115200);
  // analogReadResolution(12);//XXX
  // Serial.setTimeout(30);

  // vcurrent = analogRead(A0);
  // vcurrent = analogscale*vcurrent; 

  pinMode(pinToUse, OUTPUT_12MA);
  // Serial.begin(115200);
  Serial.begin(19200);

  while (!Serial);

  delay(100);

  Serial.print(F("\nStarting PWM_DynamicDutyCycle on "));
  // Serial.println(BOARD_NAME);
  // Serial.println(RP2040_PWM_VERSION);

  frequency = 20000;
  PWM_Instance = new RP2040_PWM(pinToUse, frequency, 0);

  if (PWM_Instance)
  {
    PWM_Instance->setPWM();
  }

  // Serial.println(dashLine);

  // Serial.print("SHT_LIB_VERSION: \t");
  // Serial.println(SHT_LIB_VERSION);
  // Serial.println();
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(100000);
  sht.begin();

  uint16_t stat = sht.readStatus();
  // Serial.print(stat, HEX);
  // Serial.println();

  uint32_t ser;
  bool b = sht.getSerialNumber(ser, true);
  if (b)
  {
    // Serial.print(ser, HEX);
    // Serial.println();
  }
  else
  {
    Serial.println("Error: could not get serial number.");
  }

  sht.read(false);
  Input = sht.getTemperature();
  // Serial.println("Initial temperature:");
  // Serial.println(Input, 1);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);


  myPID.SetOutputLimits(0, 2047); 
  myPID.SetSampleTime(300);
  myPID.Start(Input,  // input
              output,                      // current output
              setpoint);                   // setpoint
  myPID.SetMode(PID::Manual);

}

int incomingBytes = 0; // for incoming serial data
void loop() {
  // put your main code here, to run repeatedly:
  //incomingBytes = Serial.available();
  //if (incomingBytes>0){
//    Serial.readBytes(buf, incomingBytes);
    incomingBytes = Serial.readBytes(buf, REQUEST_RESPONSE_BUF_SIZE);
    if (incomingBytes>0){
      process_request(buf);
      Serial.write(buf, REQUEST_RESPONSE_BUF_SIZE);
      memset(buf, 0, REQUEST_RESPONSE_BUF_SIZE);  // explicitly reset the buffer
    }


    Input_acc = 0.0f;
    for(i=0;i<2;i++) {
      sht.read(false);
      Input_acc += sht.getTemperature();

    }

    Input = Input_acc/2.0f;
    if (Input < 0)
    {
      // sensor failure! Do not heat anything!
      Serial.println("SENSOR FAILURE!!! Current temperature:");
      Serial.println(Input, 3);
      PWM_Instance->setPWM(pinToUse, frequency, 0.0f);
    }
    else
    {
    
      // Serial.println("Current temperature:");
      // Serial.println(Input, 3);

      // myPID.Compute();
      // analogWrite(3,Output);
      output = myPID.Run(Input);
      // Serial.println("Current PID Output:");
      // Serial.println(output, 2);
      dutyCycle = output/2048*30.0f;
      PWM_Instance->setPWM(pinToUse, frequency, dutyCycle);
    }
    
    i++;
    if (i>2) {
      i = 0;
      if (stream_run) {
        stream_buf[0] = 0b00000001;
        stream_values[0] = Input;  // Process Variable
        stream_values[1] = output;  // Controller Output
        memcpy(&stream_buf[1], stream_values, 2*sizeof(float));
        Serial.write(stream_buf, STREAM_BUF_SIZE);
      }

    }
    

  
}

