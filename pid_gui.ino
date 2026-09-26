#include <PID_v2.h>

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

    VAR_d_window = 0b1100,
    VAR_mode = 0b1101,

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

static volatile bool stream_run = false;

static float setpoint = 40.0f;
static float kP = 0.01f;
static float kI = 0.0f;
static float kD = 0.0f;
static float err_I = 2055.0f;
static float err_P_limits[2] = {-3500.0f, 3500.0f};
static float err_I_limits[2] = {-6500.0f, 6500.0f};

// [s] window of the least-squares process variable slope used by the D term (see smoothed_slope())
static float d_window = 10.0f;
#define D_WINDOW_MIN 2.0f
#define D_WINDOW_MAX 120.0f

// controller mode (VAR_mode, sent as a float): MODE_pid holds the setpoint, MODE_damp only acts on dV/dt, starting
// from the output it had when engaged: kD * dV/dt plus kI * integral of dV/dt, which removes a steady drift (setpoint and
// kP are ignored)
enum {
    MODE_pid = 0,
    MODE_damp = 1
};
static int mode = MODE_pid;
static bool locked = false;
static float damp_bias = 0.0f;  // MODE_damp output without the D term, moved by the integral of dV/dt
static float slope = 0.0f;  // latest smoothed dV/dt [V/s]
static uint32_t last_loop_ms = 0;

#define REQUEST_RESPONSE_BUF_SIZE (sizeof(char)+2*(sizeof(float)))  // same size for both requests and responses
unsigned char buf[REQUEST_RESPONSE_BUF_SIZE];  // message buffer (both for receiving and sending)

#define STREAM_BUF_SIZE (sizeof(char)+2*sizeof(float))
#define STREAM_THREAD_SLEEP_TIME_MS 20
static float stream_values[2];

#define STREAM_PREFIX 0b00000001
#define STREAM_PREFIX_DERIVATIVE 0b00000010  // sent right before every STREAM_PREFIX message: dV/dt, 0
unsigned char stream_buf[STREAM_BUF_SIZE];

// the library does P and I only - its D term (difference of two consecutive noisy readings) is replaced by kD times
// the smoothed slope, see loop()
PID_v2 myPID(kP, kI, 0.0, PID::Direct, PID::P_On::Measurement);
float analogscale=30.22*3.3/4095.0;
float vcurrent = 2.5;        //current piezo voltage
float output = 0;
int i = 0;


void stream_start(void) {
    if (!stream_run)
        stream_run = true;
}

void stream_stop(void) {
    if (stream_run) {
        stream_run = false;
    }
}

// (re)start the lock in the current mode from the present output, so neither locking nor switching modes makes a jump
void engage(void) {
    myPID.SetMode(PID::Manual);
    if (mode == MODE_pid)
        myPID.Start(vcurrent, output, setpoint);  // back to Automatic, the integral starts from 'output'
    else
        damp_bias = output;
}

/*
 *  Process variable history for a smooth derivative: the D term uses the slope of a least-squares straight line through the
 *  samples of the last d_window seconds instead of the difference of two consecutive (noisy) readings
 */
#define DBUF_N 1024
#define DBUF_MIN_SPACING_MS 200  // keeps D_WINDOW_MAX within the buffer (1024 * 200 ms > 120 s)
static uint32_t dbuf_ms[DBUF_N];
static float dbuf_temp[DBUF_N];
static int dbuf_head = 0;  // next slot to write
static int dbuf_count = 0;

void dbuf_push(uint32_t now, float temp) {
    if (dbuf_count > 0 && now - dbuf_ms[(dbuf_head + DBUF_N - 1) % DBUF_N] < DBUF_MIN_SPACING_MS)
        return;

    dbuf_ms[dbuf_head] = now;
    dbuf_temp[dbuf_head] = temp;
    dbuf_head = (dbuf_head + 1) % DBUF_N;
    if (dbuf_count < DBUF_N)
        dbuf_count++;
}

// the loop runs much faster than DBUF_MIN_SPACING_MS - average the readings in between instead of dropping them
static float dbuf_acc = 0.0f;
static int dbuf_acc_n = 0;

void dbuf_sample(uint32_t now, float temp) {
    dbuf_acc += temp;
    dbuf_acc_n++;
    if (dbuf_count > 0 && now - dbuf_ms[(dbuf_head + DBUF_N - 1) % DBUF_N] < DBUF_MIN_SPACING_MS)
        return;

    dbuf_push(now, dbuf_acc / dbuf_acc_n);
    dbuf_acc = 0.0f;
    dbuf_acc_n = 0;
}

// slope [V/s] of the least-squares line through the samples of the last d_window seconds, 0 if there is not enough data
float smoothed_slope(uint32_t now) {
    uint32_t window_ms = (uint32_t)(d_window * 1000.0f);
    float t_sum = 0.0f, temp_sum = 0.0f;
    uint32_t newest_age = 0, oldest_age = 0;
    int n = 0;

    // walk from the newest sample to the oldest one; times are relative to 'now' (seconds, <= 0) to keep the precision
    for (int k = 0; k < dbuf_count; k++) {
        int idx = (dbuf_head - 1 - k + DBUF_N) % DBUF_N;
        uint32_t age = now - dbuf_ms[idx];
        if (age > window_ms)
            break;
        if (n == 0)
            newest_age = age;
        oldest_age = age;
        t_sum += -(float)age / 1000.0f;
        temp_sum += dbuf_temp[idx];
        n++;
    }
    if (n < 3 || oldest_age - newest_age < 1000)
        return 0.0f;

    float t_mean = t_sum / n;
    float temp_mean = temp_sum / n;
    float s_tt = 0.0f, s_ttemp = 0.0f;
    for (int k = 0; k < n; k++) {
        int idx = (dbuf_head - 1 - k + DBUF_N) % DBUF_N;
        float dt = -(float)(now - dbuf_ms[idx]) / 1000.0f - t_mean;
        s_tt += dt * dt;
        s_ttemp += dt * (dbuf_temp[idx] - temp_mean);
    }

    return s_ttemp / s_tt;
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
                locked = false;
                myPID.SetMode(PID::Manual);
                result = RESULT_ok;
                break;
            case CMD_lock_start:
                //printf("CMD_stream_start\n");
                locked = true;
                engage();
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
            case VAR_d_window:
                memcpy(&request_response_buf[1], &d_window, sizeof(float));
                result = RESULT_ok;
                break;
            case VAR_mode: {
                float mode_f = (float)mode;
                memcpy(&request_response_buf[1], &mode_f, sizeof(float));
                result = RESULT_ok;
                break;
            }

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
                myPID.SetTunings(kP, kI, 0.0);
                result = RESULT_ok;
                break;
            case VAR_kI:
                //printf("VAR_kI\n");
                memcpy(&kI, &request_response_buf[1], sizeof(float));
                myPID.SetTunings(kP, kI, 0.0);
                result = RESULT_ok;
                break;
            case VAR_kD:
                //printf("VAR_kD\n");
                memcpy(&kD, &request_response_buf[1], sizeof(float));
                myPID.SetTunings(kP, kI, 0.0);
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
            case VAR_d_window:
                // clamp instead of replying with an error (the GUI raises an exception on error replies)
                memcpy(&d_window, &request_response_buf[1], sizeof(float));
                if (!(d_window >= D_WINDOW_MIN))  // also catches NaN
                    d_window = D_WINDOW_MIN;
                else if (d_window > D_WINDOW_MAX)
                    d_window = D_WINDOW_MAX;
                result = RESULT_ok;
                break;
            case VAR_mode: {
                float mode_f;
                memcpy(&mode_f, &request_response_buf[1], sizeof(float));
                if (mode_f == MODE_pid || mode_f == MODE_damp) {
                    if ((int)mode_f != mode) {
                        mode = (int)mode_f;
                        if (locked)
                            engage();
                    }
                    result = RESULT_ok;
                }
                else {
                    result = RESULT_error;
                }
                break;
            }

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
  analogWriteResolution(12);
  
  // initialize digital pin 13 as an output.
  pinMode(7, INPUT);
  pinMode(13, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(DAC0, OUTPUT);
  pinMode(52, INPUT);
  
  Serial.begin(115200);
  analogReadResolution(12);//XXX
  Serial.setTimeout(30);

  vcurrent = analogRead(A0);
  vcurrent = analogscale*vcurrent; 
  myPID.SetOutputLimits(-2047, 2047); 
  myPID.SetSampleTime(30);
  myPID.Start(vcurrent,  // input
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

    vcurrent = analogRead(A0);
    vcurrent = analogscale*vcurrent; 
    uint32_t now = millis();
    dbuf_sample(now, vcurrent);
    slope = smoothed_slope(now);
    float dt = (now - last_loop_ms) / 1000.0f;
    last_loop_ms = now;
    if (locked) {
      if (mode == MODE_pid) {
        output = myPID.Run(vcurrent) - kD * slope;  // P and I from the library, D on the smoothed slope
      }
      else {
        // derivative damp: the integral of dV/dt accumulates into the held output (clamped - no windup) so a change of
        // kI doesn't make a jump, then D opposes dV/dt
        damp_bias = constrain(damp_bias - kI * slope * dt, -2047.0f, 2047.0f);
        output = damp_bias - kD * slope;
      }
      output = constrain(output, -2047.0f, 2047.0f);
    }
    // unlocked: hold the last output
    analogWrite(DAC0, (int)(output + 2047));

    i++;
    if (i>19) {
      i = 0;
      if (stream_run) {
        stream_buf[0] = STREAM_PREFIX_DERIVATIVE;
        stream_values[0] = slope;  // dV/dt the D term acts on
        stream_values[1] = 0.0f;  // unused
        memcpy(&stream_buf[1], stream_values, 2*sizeof(float));
        Serial.write(stream_buf, STREAM_BUF_SIZE);

        stream_buf[0] = STREAM_PREFIX;
        stream_values[0] = vcurrent;  // Process Variable
        stream_values[1] = output;  // Controller Output
        memcpy(&stream_buf[1], stream_values, 2*sizeof(float));
        Serial.write(stream_buf, STREAM_BUF_SIZE);
      }
    }
    

  
}
