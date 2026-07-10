#include "pid.h"
#include "pwm.h"

QueueHandle_t pid_queue;

#define PID_OUT_MAX  400.0f
#define PID_OUT_MIN -300.0f

float clamp_float(float value, float min, float max){
	return (value > max) ? max : ((value < min) ? min : value);
}

void pid_init(pid_t* p, float KP, float KI, float KD){
    if (p == NULL) return;

    p->kp = KP;
    p->ki = KI;
    p->kd = KD;

    p->i_error = 0.0f;
    p->last_error = 0.0f;

    p->MAX = PID_OUT_MAX;
    p->MIN = PID_OUT_MIN;
}

float pid_control(pid_t* p, float set_point, float current_meas, float Ts){
    if (p == NULL) return 0.0f;

    if (Ts <= 0.0f || Ts > 0.1f)
        return 0.0f;

    float error = set_point - current_meas;

    float P = error * p->kp;

    p->i_error += error * Ts;

    float I = p->ki * p->i_error;

    float I_MAX = p->MAX * 0.1f;
    float I_MIN = p->MIN * 0.1f;

    if (I > I_MAX){
        I = I_MAX;
        if (p->ki != 0.0f){
            p->i_error = I_MAX / p->ki;
        }
    }

    if (I < I_MIN){
        I = I_MIN;
        if (p->ki != 0.0f){
            p->i_error = I_MIN / p->ki;
        }
    }

    float D = p->kd * ((error - p->last_error) / Ts);

    p->last_error = error;

    float out = P + I + D;

    out = clamp_float(out, p->MIN, p->MAX);

    return out;
}

void pid_task(void* pvParameters){
    QueueHandle_t mpu_queue_handle = (QueueHandle_t)pvParameters;

    pid_t pitch_pid;
    //zerando o fator derivativo melhorou a resposta do motor
    pid_init(&pitch_pid, 15.0f, 0.02f, 0.0f);

    pid_t roll_pid;
    pid_init(&roll_pid, 15.0f, 0.2f, 0.0f);

    float pid_pitch_out;
    float pid_roll_out;

    mpu6050_t mpu_received;

    int64_t last_time = esp_timer_get_time();

    control_t pid;

    pid.throttle = 200.0f; // dc base
    float out_raw[N_MOTORs];

    while (1) {
        if (xQueueReceive(mpu_queue_handle, &mpu_received, portMAX_DELAY) ==
																		pdPASS){
            int64_t current_time = esp_timer_get_time();
            float Ts = (current_time - last_time) / 1e6f;
            last_time = current_time;

            if (Ts <= 0.0f || Ts > 0.1f)
                Ts = 0.002f;

            pid_pitch_out = pid_control(&pitch_pid, 0.0f,
											 mpu_received.pitch, Ts);
            pid_roll_out  = pid_control(&roll_pid,  0.0f,
									         mpu_received.roll,  Ts);

            out_raw[MOTOR_FL] = pid.throttle + pid_pitch_out - pid_roll_out;
            out_raw[MOTOR_FR] = pid.throttle - pid_pitch_out + pid_roll_out;

            out_raw[MOTOR_FL] = clamp_float(out_raw[MOTOR_FL], MOTOR_CMD_MIN,
															MOTOR_CMD_MAX);
            out_raw[MOTOR_FR] = clamp_float(out_raw[MOTOR_FR], MOTOR_CMD_MIN,
																	MOTOR_CMD_MAX);

            pid.out[MOTOR_FL] = (out_raw[MOTOR_FL] < 30) ? 0 : (uint16_t)out_raw[MOTOR_FL];
            pid.out[MOTOR_FR] = (out_raw[MOTOR_FR] < 30) ? 0 : (uint16_t)out_raw[MOTOR_FR];

			xQueueSend(pid_queue, &pid, 0);
        }
    }
}
