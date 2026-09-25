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
/* Both sensors produce measurements at 208 Hz (one update every 4.81 ms).
 * The paced application loop is slower; sensor ODR is not the loop rate.
 * The accelerometer range is +/-8 g per axis. Application readings are in g
 * and degrees per second. */
#define EWMA_ALPHA_ACCEL_PERCENT    50
#define EWMA_ALPHA_GYRO_PERCENT     15
/* About 4 ms of acquisition/processing plus HAL's tick-rounded 1 ms delay
 * gives roughly 5-6 ms per loop, longer than the 4.81 ms sensor period.
 * Recheck this pacing if CPU speed, I2C timing or processing changes. */
#define SAMPLE_DELAY_MS              4U
#define REPORT_DISABLE				 1  /* Disable routine sensor reports while measuring. */
/* Roughly preserve the old reporting interval as the loop rate doubles. */
#define UART_REPORT_EVERY_SAMPLES   20U
/* Time whole sampling loops, then print one summary outside the batch.
 * Set SAMPLE_TIMING_ENABLE to 0 for a demonstration without timing output. */
#define SAMPLE_TIMING_ENABLE         0
#define SAMPLE_TIMING_BATCH_SIZE  1000U
#define NORMAL_LED_DELAY_MS       1000U  /* Toggle interval before confirmation. */
#define FALL_LED_DELAY_MS          150U  /* Toggle interval in both alarm states. */
#define STARTUP_SETTLE_MS          500U  /* Ignore initial zero EWMA history. */
#define LOW_G_THRESHOLD_G          0.60f
#define ROTATION_THRESHOLD_DPS    100.0f
#define TRIGGER_HOLD_MS             70U  /* Either condition must last this long. */
#define IMPACT_THRESHOLD_G         2.40f
#define CONFIRM_TIMEOUT_MS        1000U  /* No impact in this window: return normal. */
#define ACK_HOLD_MS               2000U
/* Long lie requires continuously observed stillness after a confirmed fall.
 * Acceleration near 1 g allows any resting orientation; the gyro limit rejects
 * rotation. Any sample outside these limits restarts the 30-second hold. */
#define LONG_LIE_ACCEL_MIN_G       0.80f
#define LONG_LIE_ACCEL_MAX_G       1.20f
#define LONG_LIE_GYRO_MAX_DPS      50.0f
#define LONG_LIE_HOLD_MS          30000U

static void UART1_Init(void);
static void UART_Send(const char *text);

extern int ewma_filter(int new_data, int old_output, int alpha_percent);
int ewma_filter_C(int new_data, int old_output, int alpha_percent);

UART_HandleTypeDef huart1;

//Main selects the blink period.
static volatile uint32_t led_blink_period_ms = 0;

//This is called by SysTick_Handler(), which interrupts main() every 1ms,
//such that LED can keep blinking when the main loop is blocked.
void Wearable_LED_Tick(void)
{
    static uint32_t previous_period_ms = 0;
    static uint32_t toggled_at = 0;
    uint32_t period_ms = led_blink_period_ms;

    //Zero return prevents GPIO access before LED initialization is complete.
    if (period_ms == 0U) return;

    uint32_t now = HAL_GetTick();
    if (period_ms != previous_period_ms) {
        previous_period_ms = period_ms;
        toggled_at = now;
        BSP_LED_On(LED2);
    } else if (now - toggled_at >= period_ms) {
        BSP_LED_Toggle(LED2);
        toggled_at = now;
    }
}

typedef enum {
    NORMAL,
    CONFIRMING,
    FALL_CONFIRMED,
    LONG_LIE
} WearableState;

static const char *const state_names[] = {
    "NORMAL", "CONFIRMING", "FALL_CONFIRMED", "LONG_LIE"
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
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);

    /* Joining the hotspot can take seconds. Do it before starting the sampling
     * timers; a missing network must not stop the local fall detector. */
    UART_Send("Wi-Fi: initialising ThingsBoard connection...\r\n");
    if (ThingsBoard_Init())
        UART_Send("Wi-Fi: connected, ThingsBoard address resolved.\r\n");
    else
        UART_Send("Wi-Fi: unavailable or unconfigured; local detection active.\r\n");

    /* Raw sensor readings. */
    int16_t accel_raw_i16[3] = {0, 0, 0};
    float gyro_raw_float[3] = {0.0f, 0.0f, 0.0f};

    /* EWMA outputs. */
    int accel_ewma_asm[3] = {0, 0, 0};
    int gyro_ewma_asm[3]  = {0, 0, 0};

    /* Reference C states are kept separately for assembly verification. */
    int accel_ewma_c[3] = {0, 0, 0};
    int gyro_ewma_c[3]  = {0, 0, 0};

    unsigned long sample_number = 0;
    WearableState state = NORMAL;
    uint32_t startup_at = HAL_GetTick();
    led_blink_period_ms = NORMAL_LED_DELAY_MS;
    uint32_t low_g_since = 0, rotation_since = 0, confirming_since = 0, ack_since = 0;
    bool low_g_active = false, rotation_active = false, ack_active = false;
    uint32_t still_since = 0;
    bool still_active = false;
#if SAMPLE_TIMING_ENABLE
    uint32_t batch_started_at = 0;
    uint32_t batch_samples = 0;
#endif

    while (1)
    {
#if SAMPLE_TIMING_ENABLE
        /* Begin after the preceding summary was fully transmitted. */
        if (batch_samples == 0U) {
            batch_started_at = HAL_GetTick();
        }
#endif
        sample_number++;
        bool report_due = (sample_number % UART_REPORT_EVERY_SAMPLES) == 0U;

        int gyro_raw_int[3] = {0, 0, 0};

        BSP_ACCELERO_AccGetXYZ(accel_raw_i16);
        BSP_GYRO_GetXYZ(gyro_raw_float);

        /* The BSP returns acceleration in integer mg (1 mg = 0.001 g) and
         * angular velocity in floating-point mdps (0.001 degrees/second).
         * Convert mdps to signed integers for the assembly filter. */
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

        /* Convert the assembly-filtered acceleration from mg to g. */
        float accel_axes_g[3] = {
            accel_ewma_asm[0] / 1000.0f,
            accel_ewma_asm[1] / 1000.0f,
            accel_ewma_asm[2] / 1000.0f
        };

        /* Convert the assembly-filtered angular velocity from mdps to dps. */
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
         * FALL_CONFIRMED -> 30 seconds of continuous stillness -> LONG_LIE
         * No impact before timeout -> NORMAL. Hold USER for over 2 seconds to
         * return to NORMAL from either latched alarm state.
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

        /* Acknowledgement takes priority if its hold completes on the same
         * sample as the long-lie timer. Releasing USER restarts its hold. */
        if (state == FALL_CONFIRMED || state == LONG_LIE) {
            if (BSP_PB_GetState(BUTTON_USER) == GPIO_PIN_RESET) {
                if (!ack_active) ack_since = now;
                ack_active = true;
                if (now - ack_since > ACK_HOLD_MS) {
                    state = NORMAL;
                    startup_at = now;
                    ack_active = false;
                    still_active = false;
                }
            } else {
                ack_active = false;
            }
        }

        if (state == FALL_CONFIRMED) {
            /* Use magnitudes from the assembly-filtered axes. Stillness is
             * continuous, not accumulated across separate quiet periods. */
            bool still = accel_g >= LONG_LIE_ACCEL_MIN_G &&
                         accel_g <= LONG_LIE_ACCEL_MAX_G &&
                         angular_dps < LONG_LIE_GYRO_MAX_DPS;
            if (still) {
                if (!still_active) still_since = now;
                still_active = true;
                if (now - still_since >= LONG_LIE_HOLD_MS) {
                    state = LONG_LIE;
                    still_active = false;
                }
            } else {
                still_active = false;
            }
        }

        /* Rapid blinking begins only after confirmation and continues through
         * LONG_LIE until acknowledgement. SysTick keeps it running even when
         * the main loop is blocked in UART or Wi-Fi calls. */
        led_blink_period_ms = (state == FALL_CONFIRMED || state == LONG_LIE)
                             ? FALL_LED_DELAY_MS : NORMAL_LED_DELAY_MS;

        /* Upload once per fall/long-lie/acknowledgement transition. Sampling
         * and button polling pause during the attempt; LED blinking continues. */
        if (state == FALL_CONFIRMED && previous_state != FALL_CONFIRMED) {
            UART_Send("Wi-Fi: sending confirmed fall...\r\n");
            if (ThingsBoard_SendFall(accel_g, angular_dps, now)) {
                UART_Send("ThingsBoard: fall accepted (HTTP 200).\r\n");
            } else {
                UART_Send("ThingsBoard: fall delivery not confirmed; alarm keeps blinking.\r\n");
            }
            /* A button press during a blocking upload was not continuously
             * sampled. Start its hold timer afresh rather than counting that gap. */
            ack_active = false;
            /* Start observing stillness with fresh samples after the upload.
             * Time spent blocked on Wi-Fi is not evidence of a long lie. */
            still_active = false;
        } else if (state == LONG_LIE && previous_state != LONG_LIE) {
            UART_Send("Wi-Fi: sending long lie...\r\n");
            if (ThingsBoard_SendLongLie()) {
                UART_Send("ThingsBoard: long lie accepted (HTTP 200).\r\n");
            } else {
                UART_Send("ThingsBoard: long lie delivery not confirmed; alarm keeps blinking.\r\n");
            }
            /* As for a fall upload, do not count an unobserved button hold. */
            ack_active = false;
        } else if (state == NORMAL &&
                   (previous_state == FALL_CONFIRMED || previous_state == LONG_LIE)) {
            /* Only an accepted USER-button hold takes this path. A confirming
             * timeout must not send a recovery update. Give immediate local
             * feedback before the blocking request, even if the network fails. */
            UART_Send("Wi-Fi: sending normal state...\r\n");
            if (ThingsBoard_SendNormal()) {
                UART_Send("ThingsBoard: normal state accepted (HTTP 200).\r\n");
            } else {
                UART_Send("ThingsBoard: normal delivery not confirmed; local state is NORMAL.\r\n");
            }
            /* Allow fresh samples to settle after the network pause, rather
             * than counting that pause as sampled time. */
            startup_at = HAL_GetTick();
        }

        if (!REPORT_DISABLE) {
			char buffer[320];
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

        /* Pace reads below the 208 Hz sensor ODR with the measured processing
         * time included. This simple delay is not data-ready synchronization. */
        HAL_Delay(SAMPLE_DELAY_MS);
#if SAMPLE_TIMING_ENABLE
        /* Measure normal operation only. Discard a batch interrupted by a fall
         * or acknowledgement, since those transitions may block on Wi-Fi. */
        if (state != NORMAL || previous_state != NORMAL) {
            batch_samples = 0;
        } else {
            batch_samples++;
        }
        if (batch_samples >= SAMPLE_TIMING_BATCH_SIZE) {
            uint32_t elapsed_ms = HAL_GetTick() - batch_started_at;
            if (elapsed_ms > 0U) {
                char timing_buffer[160];
                snprintf(timing_buffer, sizeof(timing_buffer),
                         "Sampling: %lu samples in %lu ms; avg=%.3f ms/sample; rate=%.2f Hz\r\n",
                         (unsigned long)batch_samples, (unsigned long)elapsed_ms,
                         (double)elapsed_ms / batch_samples,
                         1000.0 * batch_samples / elapsed_ms);
                UART_Send(timing_buffer);
            }
            /* Formatting and transmitting this summary are excluded from
             * both batches. Whole-loop time also includes loop/timing overhead
             * and the pacing delay (plus routine reports if enabled).
             * Counts are loops, not guaranteed fresh sensor samples. */
            batch_samples = 0;
        }
#endif
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
