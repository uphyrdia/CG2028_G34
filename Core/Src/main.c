/******************************************************************************
 * @file           : main.c
 * @brief          : CG2028 Assignment - ElderCare Wearable Safety Companion
 * @author         : Hou Linxin
 * (c) CG2028 Teaching Team
 ******************************************************************************/

/*--------------------------- Includes ---------------------------------------*/
#include "main.h"
#include "thingsboard.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <stdbool.h>
#include <math.h>

/*--------------------------- Configuration ----------------------------------*/
/* Parameters apply to the assembly-filtered readings. The sensors run at 52 Hz.
 * Alpha 75% helps retain the short low-g interval and landing peak of a drop. */
#define EWMA_ALPHA_ACCEL_PERCENT    85
#define EWMA_ALPHA_GYRO_PERCENT     35
#define SAMPLE_DELAY_MS             20U
#define REPORT_DISABLE				 0
#define UART_REPORT_EVERY_SAMPLES    5U  /* Change to 5 for more frequent reports. */
#define NORMAL_LED_DELAY_MS       1000U
#define FALL_LED_DELAY_MS          150U
#define STARTUP_SETTLE_MS          500U  /* Ignore initial zero EWMA history. */
#define LOW_G_THRESHOLD_G          0.60f
#define ROTATION_THRESHOLD_DPS    100.0f
#define TRIGGER_HOLD_MS             70U  /* Either condition must last this long. */
#define IMPACT_THRESHOLD_G         1.65f
#define CONFIRM_TIMEOUT_MS        1000U  /* No impact in this window: return normal. */
#define ACK_HOLD_MS               2000U

static void UART1_Init(void);
static void UART_Send(const char *text);

extern int ewma_filter(int new_data, int old_output, int alpha_percent);
int ewma_filter_C(int new_data, int old_output, int alpha_percent);

UART_HandleTypeDef huart1;

typedef enum {
    NORMAL,
    CONFIRMING,
    FALL_CONFIRMED
} WearableState;

static const char *const state_names[] = {
    "NORMAL", "CONFIRMING", "FALL_CONFIRMED"
};

float norm(float *v) {
	return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

int main(void)
{
    HAL_Init();
    UART1_Init();
    BSP_LED_Init(LED2);
    BSP_ACCELERO_Init();
    BSP_GYRO_Init();
    BSP_LED_Off(LED2);
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO); /* Active-low acknowledgement. */

    /* Joining the hotspot can take seconds. Do it before starting the sampling
     * timers; a missing network must not stop the local fall detector. */
    UART_Send("Wi-Fi: initialising ThingsBoard connection...\r\n");
    if (ThingsBoard_Init()) {
        UART_Send("Wi-Fi: hotspot connected, ThingsBoard address resolved.\r\n");
    } else {
        UART_Send("Wi-Fi: unavailable or unconfigured; local detection active.\r\n");
    }

    /* Previous EWMA outputs. The first test/application sample starts from 0. */
    int accel_ewma_asm[3] = {0, 0, 0};
    int gyro_ewma_asm[3]  = {0, 0, 0};

    /* Reference C states are kept separately for assembly verification. */
    int accel_ewma_c[3] = {0, 0, 0};
    int gyro_ewma_c[3]  = {0, 0, 0};

    unsigned long sample_number = 0;
    WearableState state = NORMAL;
    uint32_t startup_at = HAL_GetTick();
    uint32_t led_toggled_at = startup_at;
    uint32_t low_g_since = 0, rotation_since = 0, confirming_since = 0, ack_since = 0;
    bool low_g_active = false, rotation_active = false, ack_active = false;

    while (1)
    {
        sample_number++;
        bool report_due = (sample_number % UART_REPORT_EVERY_SAMPLES) == 0U;

        int16_t accel_raw_i16[3] = {0, 0, 0};
        float gyro_raw_float[3] = {0.0f, 0.0f, 0.0f};
        int gyro_raw_int[3] = {0, 0, 0};

        BSP_ACCELERO_AccGetXYZ(accel_raw_i16);
        BSP_GYRO_GetXYZ(gyro_raw_float);

        /* The supplied BSP reports gyroscope readings as floating-point raw
         * values. Convert them to signed integers before passing them to the
         * integer assembly routine. */
        for (int axis = 0; axis < 3; axis++)
        {
            gyro_raw_int[axis] = (int)gyro_raw_float[axis];

            accel_ewma_asm[axis] = ewma_filter(
                (int)accel_raw_i16[axis],
                accel_ewma_asm[axis],
                EWMA_ALPHA_ACCEL_PERCENT);

            gyro_ewma_asm[axis] = ewma_filter(
                gyro_raw_int[axis],
                gyro_ewma_asm[axis],
                EWMA_ALPHA_GYRO_PERCENT);

            accel_ewma_c[axis] = ewma_filter_C(
                (int)accel_raw_i16[axis],
                accel_ewma_c[axis],
                EWMA_ALPHA_ACCEL_PERCENT);

            gyro_ewma_c[axis] = ewma_filter_C(
                gyro_raw_int[axis],
                gyro_ewma_c[axis],
                EWMA_ALPHA_GYRO_PERCENT);
        }

        /* Accelerometer filtered readings are in meters per second squared. */
        float accel_axes_g[3] = {
            accel_ewma_asm[0] / 1000.0f,
            accel_ewma_asm[1] / 1000.0f,
            accel_ewma_asm[2] / 1000.0f
        };

        /* Gyroscope filtered readings are in degrees per second. */
        float gyro_dps[3] = {
            gyro_ewma_asm[0] / 1000.0f,
            gyro_ewma_asm[1] / 1000.0f,
            gyro_ewma_asm[2] / 1000.0f
        };

        /* Optional debugging check. This confirms that the assembly routine
         * matches the reference C routine for the current samples. */
        if ((accel_ewma_asm[0] != accel_ewma_c[0]) ||
            (accel_ewma_asm[1] != accel_ewma_c[1]) ||
            (accel_ewma_asm[2] != accel_ewma_c[2]) ||
            (gyro_ewma_asm[0] != gyro_ewma_c[0]) ||
            (gyro_ewma_asm[1] != gyro_ewma_c[1]) ||
            (gyro_ewma_asm[2] != gyro_ewma_c[2]))
        {
        	UART_Send("WARNING: Assembly and C EWMA outputs do not match.\r\n");
        }

        /**************** Elderly wearable state logic starts here *************
         * NORMAL -> sustained low-g OR rapid rotation -> CONFIRMING
         * CONFIRMING -> impact peak -> FALL_CONFIRMED
         * No impact before timeout -> NORMAL. Hold USER to clear a latched fall.
         *********************************************************************/
        uint32_t now = HAL_GetTick();
        WearableState previous_state = state;
        float accel_g = norm(accel_axes_g);
        float angular_dps = norm(gyro_dps);

        if (state == NORMAL && now - startup_at >= STARTUP_SETTLE_MS) {
            /* Separate timers: alternating brief low-g and rotation readings
             * must not combine into one sustained trigger. */
            if (accel_g < LOW_G_THRESHOLD_G) {
                if (!low_g_active) low_g_since = now;
                low_g_active = true;
            } else {
                low_g_active = false;
            }
            if (angular_dps > ROTATION_THRESHOLD_DPS) {
                if (!rotation_active) rotation_since = now;
                rotation_active = true;
            } else {
                rotation_active = false;
            }

            if ((low_g_active && now - low_g_since >= TRIGGER_HOLD_MS) ||
                (rotation_active && now - rotation_since >= TRIGGER_HOLD_MS)) {
                state = CONFIRMING;
                confirming_since = now;
                low_g_active = false;
                rotation_active = false;
            }
        }

        if (state == CONFIRMING) {
            if (now - confirming_since >= CONFIRM_TIMEOUT_MS) {
                state = NORMAL;
            } else if (accel_g >= IMPACT_THRESHOLD_G) {
                /* Confirm immediately on a sampled impact. No rotation is
                 * required if sustained low-g caused the initial trigger. */
                state = FALL_CONFIRMED;
            }
        }

        if (state == FALL_CONFIRMED) {
        	// If BUTTON_USER is pressed down for more than ACK_HOLD_MS, reset to NORMAL state
            if (BSP_PB_GetState(BUTTON_USER) == GPIO_PIN_RESET) {
                if (!ack_active) ack_since = now;
                ack_active = true;
                if (now - ack_since >= ACK_HOLD_MS) {
                    state = NORMAL;
                    startup_at = now;
                    ack_active = false;
                }
            } else {
                ack_active = false;
            }
        }

        /* LED timing never sets the sensor delay. A confirmed fall stays ON. */
        uint32_t blink_ms = (state == CONFIRMING)
                            ? FALL_LED_DELAY_MS : NORMAL_LED_DELAY_MS;
        if (state == FALL_CONFIRMED || state != previous_state) {
            BSP_LED_On(LED2);
            led_toggled_at = now;
        } else if (now - led_toggled_at >= blink_ms) {
            BSP_LED_Toggle(LED2);
            led_toggled_at = now;
        }

        /* LED has already been latched ON above. Upload once on this transition,
         * not on every sample while the fall remains latched. Networking runs
         * only on fall/acknowledgement transitions, so button polling and
         * sampling pause during the attempt, but the LED stays ON throughout. */
        if (state == FALL_CONFIRMED && previous_state != FALL_CONFIRMED) {
            UART_Send("Wi-Fi: sending confirmed fall...\r\n");
            if (ThingsBoard_SendFall(accel_g, angular_dps, now)) {
                UART_Send("ThingsBoard: fall accepted (HTTP 200).\r\n");
            } else {
                UART_Send("ThingsBoard: fall delivery not confirmed; LED stays ON.\r\n");
            }
            /* A button press during a blocking upload was not continuously
             * sampled. Start its hold timer afresh rather than counting that gap. */
            ack_active = false;
        } else if (state == NORMAL && previous_state == FALL_CONFIRMED) {
            /* Only an accepted USER-button hold takes this path. A confirming
             * timeout must not send a recovery update. Give immediate local
             * feedback before the blocking request, even if the network fails. */
            BSP_LED_Off(LED2);
            UART_Send("Wi-Fi: sending normal state...\r\n");
            if (ThingsBoard_SendNormal()) {
                UART_Send("ThingsBoard: normal state accepted (HTTP 200).\r\n");
            } else {
                UART_Send("ThingsBoard: normal delivery not confirmed; local state is NORMAL.\r\n");
            }
            /* Resume slow blinking and allow fresh samples to settle after the
             * network pause, rather than counting that pause as sampled time. */
            startup_at = HAL_GetTick();
            led_toggled_at = startup_at;
        }

        if (!REPORT_DISABLE) {
			char buffer[320];
			if (state != previous_state) {
				snprintf(buffer, sizeof(buffer), "State: %s\r\n", state_names[state]);
				UART_Send(buffer);
			}
			if (report_due) {
				snprintf(buffer, sizeof(buffer),
						 "Sample %lu [%s] |A|=%.2fg |W|=%.1fdps\r\n"
						 "Accel [g]  : X=%8.3f Y=%8.3f Z=%8.3f\r\n"
						 "Gyro  [dps]: X=%8.3f Y=%8.3f Z=%8.3f\r\n",
						 sample_number, state_names[state], accel_g, angular_dps,
						 accel_axes_g[0], accel_axes_g[1], accel_axes_g[2],
						 gyro_dps[0], gyro_dps[1], gyro_dps[2]);
				UART_Send(buffer);
			}
        }

        /* Simple pacing: processing and occasional UART output add to this
         * delay, so the sampling interval is approximate, not exactly 20 ms. */
        HAL_Delay(SAMPLE_DELAY_MS);
    }
}

int ewma_filter_C(int new_data, int old_output, int alpha_percent)
{
    /* Reference implementation for verification only. The assembly routine
     * must be used in the actual sensor-processing and detection pipeline. */
    int numerator = alpha_percent * new_data
                  + (100 - alpha_percent) * old_output;
    return numerator / 100;
}

static void UART_Send(const char *text)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)text, strlen(text), HAL_MAX_DELAY);
}

static void UART1_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        while (1) { }
    }
}

/* Do not modify these lines. They suppress UART-related warnings. */
int _write(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    return len;
}
int _read(int file, char *ptr, int len) { (void)file; (void)ptr; (void)len; return 0; }
int _fstat(int file, struct stat *st) { (void)file; (void)st; return 0; }
int _lseek(int file, int ptr, int dir) { (void)file; (void)ptr; (void)dir; return 0; }
int _isatty(int file) { (void)file; return 1; }
int _close(int file) { (void)file; return -1; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { (void)pid; (void)sig; return -1; }
