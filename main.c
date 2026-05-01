/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body — corrected TDPA passivity observer
  ******************************************************************************
  * Changes vs prior version (the dissertation-grade fix):
  *
  *  1. SIGNED force estimate.  F_est now carries the sign of (theta_cmd -
  *     theta_actual), so F_est * omega can be NEGATIVE when the operator/
  *     channel pushes back against the rendered force.  The previous version
  *     used |delta_theta| which made E_tdpa monotonically non-decreasing, so
  *     the "E_tdpa < 0" guard was unreachable by construction.
  *
  *  2. FILTERED actual angle.  theta_actual is EMA-smoothed with a separate
  *     coefficient (THETA_FB_ALPHA) before differencing, suppressing LSB
  *     quantisation that previously produced phantom omega peaks of >10000
  *     deg/s on a hobby servo with ~360 deg/s mechanical bandwidth.
  *
  *  3. OMEGA dead-band.  Below OMEGA_DEADBAND_DPS the velocity is treated as
  *     zero, which prevents the integrator from accumulating sign-cancelling
  *     noise products at rest.
  *
  *  4. TWO-SIDED energy ledger.  E_in (positive power, channel sourcing
  *     energy to operator) and E_out (positive power, operator returning
  *     energy) are tracked separately.  The passivity observer is
  *     E_obs = E_in - E_out + E_dissipated.  The Passivity Controller (PC)
  *     injects damping when E_obs < 0.  CSV exposes both halves so the
  *     write-up can plot the ledger.
  *
  *  5. RECTIFIED PC.  When E_obs < 0 we add a damping term proportional to
  *     omega (resistive, dissipative) rather than nudging the position
  *     command directly.  The damping work is added back to E_dissipated
  *     so the PC budget is auditable.
  *
  *  6. CSV unchanged width-wise (18 fields) for backward compatibility with
  *     the bridge logger; semantics of fields 12..18 are now the corrected
  *     quantities.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* -------------------------------------------------------------------------- */
/* Types                                                                       */
/* -------------------------------------------------------------------------- */
typedef struct {
  uint32_t ms;
  uint16_t raw;
  uint16_t ema;
  uint16_t milli;
  uint16_t servo;
  uint16_t vmax_milli;
  uint16_t flags;
  uint16_t spike_mag;

  /* TDPA telemetry (appended after existing 11 CSV fields) */
  uint16_t servo_feedback;   /* ADC2 raw 0..4095 */
  float    theta_cmd;        /* deg */
  float    theta_actual;     /* deg, EMA-filtered */
  float    omega;            /* deg/s, signed, dead-banded */
  float    F_est;            /* N, SIGNED */
  float    E_tdpa;           /* J, observer state E_obs = E_in - E_out + E_diss */
  float    error;            /* deg, signed (theta_cmd - theta_actual_filt) */
} sample_t;

/* -------------------------------------------------------------------------- */
/* Defines                                                                     */
/* -------------------------------------------------------------------------- */
#define SERVO_MIN_PULSE         600u
#define SERVO_MAX_PULSE        2400u
#define FLAT_DEADZONE_COUNTS    30u
#define DROP_SPIKE_GUARD      1500u
#define SERVO_SLEW_MAX          3u    /* max pulse change per 1ms ISR tick */

#define CONTROL_HZ             1000u
#define LOG_HZ                  100u
#define LOG_DECIM       (CONTROL_HZ / LOG_HZ)

#define RB_SIZE                256u
#define RX_LINE_MAX             64u

#define DBG_GPIO_PORT GPIOA
#define DBG_GPIO_PIN  GPIO_PIN_5

/* TDPA constants */
#define K_SERVO_NM_PER_DEG   0.09712f      /* servo torque coefficient */
#define R_HORN_M             0.018f        /* horn radius (m) */
#define DAMP_GAIN            0.0008f       /* PC damping (Nm s/rad-equivalent), TUNE */
#define DT_ISR_S             0.001f        /* 1 kHz */
#define DEG2RAD              0.01745329251994329577f
#define THETA_FB_ALPHA       0.20f         /* feedback EMA, distinct from sensor EMA */
#define OMEGA_DEADBAND_DPS   30.0f         /* below this, omega := 0 */

/* -------------------------------------------------------------------------- */
/* HAL handles                                                                 */
/* -------------------------------------------------------------------------- */
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* -------------------------------------------------------------------------- */
/* Private variables                                                           */
/* -------------------------------------------------------------------------- */
static volatile float SENSOR_FLAT     = 0.0f;
static volatile float SENSOR_BENT     = 4095.0f;
static volatile uint8_t OUTPUT_INVERT = 0u;
static volatile float EMA_ALPHA       = 0.50f;

static volatile uint32_t raw_adc  = 0;
static volatile uint32_t last_raw = 0;
static volatile float    ema_f    = 0.0f;
static volatile uint16_t ema_u16  = 0;
static volatile uint8_t  ema_init = 0;
static volatile uint32_t decim    = 0;

static volatile uint16_t vmax_milli          = 1000u;
static volatile uint32_t spike_guard_count   = 0;
static volatile uint16_t spike_guard_max_mag = 0;
static volatile uint32_t isr_cycles_last     = 0;
static volatile uint32_t isr_cycles_max      = 0;

static volatile sample_t rb[RB_SIZE];
static volatile uint16_t rb_head = 0, rb_tail = 0;
static volatile uint8_t  tx_busy = 0;

static uint8_t  rx_byte = 0;
static char     rx_line[RX_LINE_MAX];
static uint16_t rx_idx  = 0;
static uint8_t  tx_buf[320];

/* TDPA runtime state */
static volatile uint16_t servo_fb_raw   = 0u;
static volatile float    theta_act_filt = 0.0f;
static volatile uint8_t  theta_init     = 0u;
static volatile float    theta_act_prev = 0.0f;

/* Two-sided energy ledger */
static volatile float    E_in           = 0.0f;   /* sum of max(P_est,0) dt */
static volatile float    E_out          = 0.0f;   /* sum of max(-P_est,0) dt */
static volatile float    E_dissipated   = 0.0f;   /* PC damping work, always >= 0 */

/* Latched-for-log */
static volatile float    theta_cmd_last = 0.0f;
static volatile float    theta_act_last = 0.0f;
static volatile float    omega_deg_last = 0.0f;
static volatile float    F_est_last     = 0.0f;
static volatile float    err_signed_last= 0.0f;
static volatile float    E_obs_last     = 0.0f;

/* -------------------------------------------------------------------------- */
/* Function prototypes                                                         */
/* -------------------------------------------------------------------------- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);

static inline void     DWT_Init(void);
static inline uint32_t DWT_GetCycles(void);
static float    clamp01(float x);
static float    compute_bend01(uint16_t adc);
static uint16_t bend_to_servo(float x);
static inline void rb_push(sample_t s);
static uint8_t     rb_pop(sample_t *out);
static void process_rx_line(const char *s);
static void start_uart_rx_it(void);

/* -------------------------------------------------------------------------- */
/* DWT                                                                         */
/* -------------------------------------------------------------------------- */
static inline void DWT_Init(void){
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}
static inline uint32_t DWT_GetCycles(void){ return DWT->CYCCNT; }

/* -------------------------------------------------------------------------- */
/* Math helpers                                                                */
/* -------------------------------------------------------------------------- */
static float clamp01(float x){
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

static float compute_bend01(uint16_t adc){
  float a    = (float)adc;
  float flat = SENSOR_FLAT + (float)FLAT_DEADZONE_COUNTS;
  float bent = SENSOR_BENT;
  if (bent <= flat) return 0.0f;
  if (a < flat) a = flat;
  return clamp01((a - flat) / (bent - flat));
}

static uint16_t bend_to_servo(float x){
  float p = (float)SERVO_MIN_PULSE + x * (float)(SERVO_MAX_PULSE - SERVO_MIN_PULSE);
  uint32_t u = (uint32_t)(p + 0.5f);
  if (u < SERVO_MIN_PULSE) u = SERVO_MIN_PULSE;
  if (u > SERVO_MAX_PULSE) u = SERVO_MAX_PULSE;
  return (uint16_t)u;
}

/* -------------------------------------------------------------------------- */
/* Ring buffer                                                                 */
/* -------------------------------------------------------------------------- */
static inline void rb_push(sample_t s){
  uint16_t n = (uint16_t)((rb_head + 1u) % RB_SIZE);
  if (n == rb_tail) return;
  rb[rb_head] = s;
  rb_head = n;
}

static uint8_t rb_pop(sample_t *out){
  if (rb_tail == rb_head) return 0;
  *out = rb[rb_tail];
  rb_tail = (uint16_t)((rb_tail + 1u) % RB_SIZE);
  return 1;
}

/* -------------------------------------------------------------------------- */
/* UART callbacks                                                              */
/* -------------------------------------------------------------------------- */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  if (huart->Instance == USART2) tx_busy = 0;
}

static void process_rx_line(const char *s){
  if (!s || !s[0]) return;
  if (s[0]=='A'){
    float a = (float)atof(&s[1]);
    if (a>=0.05f && a<=0.95f) EMA_ALPHA = a;
  } else if (s[0]=='V'){
    int v = atoi(&s[1]);
    if (v<0) v=0; if (v>1000) v=1000;
    vmax_milli = (uint16_t)v;
  } else if (s[0]=='I'){
    OUTPUT_INVERT = atoi(&s[1]) ? 1u : 0u;
  } else if (s[0]=='R'){
    spike_guard_count=0; spike_guard_max_mag=0; isr_cycles_max=0;
    E_in = 0.0f; E_out = 0.0f; E_dissipated = 0.0f;
  } else if (s[0]=='F'){
    float v = (float)atof(&s[1]);
    if (v>=0.0f && v<4000.0f) SENSOR_FLAT = v;
  } else if (s[0]=='B'){
    float v = (float)atof(&s[1]);
    if (v>100.0f && v<=4095.0f) SENSOR_BENT = v;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if (huart->Instance == USART2){
    char c = (char)rx_byte;
    if (c=='\r'||c=='\n'){
      rx_line[rx_idx]='\0';
      process_rx_line(rx_line);
      rx_idx=0;
    } else {
      if (rx_idx<(RX_LINE_MAX-1u)) rx_line[rx_idx++]=c;
      else rx_idx=0;
    }
    start_uart_rx_it();
  }
}

static void start_uart_rx_it(void){
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

/* -------------------------------------------------------------------------- */
/* TIM6 ISR — 1 kHz control loop with corrected TDPA                           */
/* -------------------------------------------------------------------------- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if (htim->Instance != TIM6) return;

  uint32_t c0 = DWT_GetCycles();
  HAL_GPIO_WritePin(DBG_GPIO_PORT, DBG_GPIO_PIN, GPIO_PIN_SET);

  /* ADC1 flex read */
  if (HAL_ADC_PollForConversion(&hadc1, 0) == HAL_OK){
    raw_adc = HAL_ADC_GetValue(&hadc1);
  }

  /* ADC2 servo feedback read */
  if (HAL_ADC_PollForConversion(&hadc2, 0) == HAL_OK){
    servo_fb_raw = (uint16_t)HAL_ADC_GetValue(&hadc2);
  }

  /* Spike guard */
  uint8_t  spike_guard_fired = 0u;
  uint16_t spike_mag         = 0u;
  if (last_raw && raw_adc + DROP_SPIKE_GUARD < last_raw){
    spike_guard_fired = 1u;
    spike_mag = (uint16_t)(last_raw - raw_adc);
    raw_adc   = last_raw;
    spike_guard_count++;
    if (spike_mag > spike_guard_max_mag) spike_guard_max_mag = spike_mag;
  }
  last_raw = raw_adc;

  /* EMA filter on flex sensor */
  if (!ema_init){
    ema_f    = (float)raw_adc;
    ema_init = 1u;
  } else {
    ema_f = EMA_ALPHA*(float)raw_adc + (1.0f-EMA_ALPHA)*ema_f;
  }
  ema_u16 = (uint16_t)(ema_f + 0.5f);

  /* Normalise */
  float bend01 = compute_bend01(ema_u16);
  float out01  = OUTPUT_INVERT ? (1.0f - bend01) : bend01;

  /* Wall clamp */
  uint8_t wall_clamp = 0u;
  float vmax01 = (float)vmax_milli / 1000.0f;
  if (out01 > vmax01){ out01 = vmax01; wall_clamp = 1u; }

  /* ----- TDPA: corrected observer ----- */

  /* (a) raw actual angle from servo feedback ADC */
  float theta_actual_raw = ((float)servo_fb_raw * 180.0f) / 4095.0f;

  /* (b) EMA-smooth the feedback channel separately from the flex-sensor EMA.
        This is the key fix for phantom 17000 deg/s peaks: a 12-bit ADC
        sampled at 1 kHz with 1 LSB jitter gives 44 deg/s of synthetic
        velocity per LSB.  With alpha=0.20 the LPF cutoff is ~35 Hz, well
        above the servo's mechanical bandwidth. */
  if (!theta_init){
    theta_act_filt = theta_actual_raw;
    theta_act_prev = theta_actual_raw;
    theta_init     = 1u;
  } else {
    theta_act_filt = THETA_FB_ALPHA * theta_actual_raw
                   + (1.0f - THETA_FB_ALPHA) * theta_act_filt;
  }

  float theta_cmd = out01 * 180.0f;

  /* (c) signed velocity, with dead-band */
  float omega_deg_s = (theta_act_filt - theta_act_prev) / DT_ISR_S;
  theta_act_prev    = theta_act_filt;
  if (fabsf(omega_deg_s) < OMEGA_DEADBAND_DPS) omega_deg_s = 0.0f;

  /* (d) SIGNED force estimate: positive when command leads actual */
  float err_signed = theta_cmd - theta_act_filt;            /* deg, SIGNED */
  float F_est      = (K_SERVO_NM_PER_DEG * err_signed) / R_HORN_M; /* N, SIGNED */

  /* (e) instantaneous power and two-sided energy ledger.
        P > 0  : channel delivers energy to operator (E_in)
        P < 0  : operator returns energy to channel  (E_out)            */
  float omega_rad_s = omega_deg_s * DEG2RAD;
  float P_est       = F_est * omega_rad_s;                  /* W, SIGNED */

  if (P_est >= 0.0f) E_in  += P_est * DT_ISR_S;
  else               E_out += (-P_est) * DT_ISR_S;

  /* (f) Passivity Observer.  E_obs >= 0 == passive.
        E_obs = E_in - E_out + E_dissipated
        i.e. all energy that has entered must be accounted for as either
        returned, dissipated by the PC, or currently stored.            */
  float E_obs = E_in - E_out + E_dissipated;

  /* (g) Passivity Controller: when E_obs < 0, inject resistive damping
        and credit the work to E_dissipated.  Damping torque is applied
        as a corrective trim on out01, opposite the velocity direction. */
  if (E_obs < 0.0f){
    /* damping force opposing motion */
    float F_damp = -DAMP_GAIN * omega_rad_s;                /* N, opposes omega */
    /* dissipated work this tick (always >= 0) */
    float dE     = fabsf(F_damp * omega_rad_s) * DT_ISR_S;
    E_dissipated += dE;

    /* translate F_damp to a small position trim (small-angle, Nm linear approx).
       The trim only opposes the current motion, never adds energy. */
    float trim_deg = (F_damp * R_HORN_M) / K_SERVO_NM_PER_DEG;
    float trim01   = trim_deg / 180.0f;
    out01 += trim01;
    if (out01 < 0.0f) out01 = 0.0f;
    if (out01 > vmax01) out01 = vmax01;

    /* recompute theta_cmd / err / F_est for logging consistency */
    theta_cmd  = out01 * 180.0f;
    err_signed = theta_cmd - theta_act_filt;
    F_est      = (K_SERVO_NM_PER_DEG * err_signed) / R_HORN_M;

    /* refresh observer with the credited dissipation */
    E_obs = E_in - E_out + E_dissipated;
  }

  /* ----- end TDPA ----- */

  /* Servo with slew limiter (after PC trim) */
  static uint16_t servo_last = 1500u;
  uint16_t servo = bend_to_servo(out01);
  if (servo > (uint16_t)(servo_last + SERVO_SLEW_MAX)) servo = (uint16_t)(servo_last + SERVO_SLEW_MAX);
  if (servo + SERVO_SLEW_MAX < servo_last)              servo = (uint16_t)(servo_last - SERVO_SLEW_MAX);
  servo_last = servo;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, servo);

  /* Latch for log */
  theta_cmd_last  = theta_cmd;
  theta_act_last  = theta_act_filt;
  omega_deg_last  = omega_deg_s;
  F_est_last      = F_est;
  err_signed_last = err_signed;
  E_obs_last      = E_obs;

  /* Log at 100 Hz */
  decim++;
  if (decim >= LOG_DECIM){
    decim = 0;
    sample_t s;
    s.ms             = HAL_GetTick();
    s.raw            = (uint16_t)raw_adc;
    s.ema            = ema_u16;
    s.milli          = (uint16_t)(out01*1000.0f+0.5f);
    s.servo          = servo;
    s.vmax_milli     = vmax_milli;
    s.flags          = (uint16_t)((spike_guard_fired?1u:0u)|(wall_clamp?2u:0u));
    s.spike_mag      = spike_mag;

    s.servo_feedback = servo_fb_raw;
    s.theta_cmd      = theta_cmd_last;
    s.theta_actual   = theta_act_last;
    s.omega          = omega_deg_last;
    s.F_est          = F_est_last;
    s.E_tdpa         = E_obs_last;     /* now holds signed observer state */
    s.error          = err_signed_last;/* now signed */

    rb_push(s);
  }

  HAL_GPIO_WritePin(DBG_GPIO_PORT, DBG_GPIO_PIN, GPIO_PIN_RESET);
  uint32_t c1 = DWT_GetCycles();
  isr_cycles_last = c1 - c0;
  if (isr_cycles_last > isr_cycles_max) isr_cycles_max = isr_cycles_last;
}

/* -------------------------------------------------------------------------- */
/* main                                                                        */
/* -------------------------------------------------------------------------- */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();

  DWT_Init();

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1500);

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 10);

  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADC_Start(&hadc2);
  HAL_ADC_PollForConversion(&hadc2, 10);

  HAL_TIM_Base_Start_IT(&htim6);

  start_uart_rx_it();

  HAL_UART_Transmit(&huart2, (uint8_t*)"BOOT\r\n", 6, 100);

  uint32_t last_wcet_ms = 0;

  while (1)
  {
    sample_t s;
    if (!tx_busy && rb_pop(&s)){
      int len = snprintf((char*)tx_buf, sizeof(tx_buf),
        "%lu,%u,%u,%u,%u,%u,%u,%u,%lu,%u,%.2f,%u,%.3f,%.3f,%.3f,%.3f,%.4f,%.3f\r\n",
        (unsigned long)s.ms,
        (unsigned)s.raw,
        (unsigned)s.ema,
        (unsigned)s.milli,
        (unsigned)s.servo,
        (unsigned)s.vmax_milli,
        (unsigned)s.flags,
        (unsigned)s.spike_mag,
        (unsigned long)spike_guard_count,
        (unsigned)spike_guard_max_mag,
        (double)EMA_ALPHA,
        (unsigned)s.servo_feedback,
        (double)s.theta_cmd,
        (double)s.theta_actual,
        (double)s.omega,
        (double)s.F_est,
        (double)s.E_tdpa,        /* now signed E_obs */
        (double)s.error);        /* now signed */

      if (len > 0){
        if (HAL_UART_Transmit_DMA(&huart2, tx_buf, (uint16_t)len)==HAL_OK)
          tx_busy = 1u;
      }
    }

    uint32_t now = HAL_GetTick();
    if (!tx_busy && (now - last_wcet_ms >= 1000u)){
      last_wcet_ms = now;
      float last_us = (float)isr_cycles_last / 170.0f;
      float max_us  = (float)isr_cycles_max  / 170.0f;
      int len = snprintf((char*)tx_buf, sizeof(tx_buf),
        "#WCET,last_cyc=%lu,max_cyc=%lu,last_us=%.3f,max_us=%.3f,Ein=%.4f,Eout=%.4f,Ediss=%.4f\r\n",
        (unsigned long)isr_cycles_last,
        (unsigned long)isr_cycles_max,
        (double)last_us,
        (double)max_us,
        (double)E_in,
        (double)E_out,
        (double)E_dissipated);
      if (len > 0){
        if (HAL_UART_Transmit_DMA(&huart2, tx_buf, (uint16_t)len)==HAL_OK)
          tx_busy = 1u;
      }
    }
  }
}

/* -------------------------------------------------------------------------- */
/* Clock config                                                                */
/* -------------------------------------------------------------------------- */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM            = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN            = 85;
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                   |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

/* -------------------------------------------------------------------------- */
/* ADC1                                                                        */
/* -------------------------------------------------------------------------- */
static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef   multimode = {0};
  ADC_ChannelConfTypeDef sConfig   = {0};

  hadc1.Instance                   = ADC1;
  hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation      = 0;
  hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait      = DISABLE;
  hadc1.Init.ContinuousConvMode    = ENABLE;
  hadc1.Init.NbrOfConversion       = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode      = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) Error_Handler();

  sConfig.Channel      = ADC_CHANNEL_1; /* PA0 */
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

/* -------------------------------------------------------------------------- */
/* ADC2 (servo feedback PA1/ADC2_IN2)                                         */
/* -------------------------------------------------------------------------- */
static void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc2.Instance                   = ADC2;
  hadc2.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation      = 0;
  hadc2.Init.ScanConvMode          = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait      = DISABLE;
  hadc2.Init.ContinuousConvMode    = ENABLE;
  hadc2.Init.NbrOfConversion       = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
  hadc2.Init.OversamplingMode      = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK) Error_Handler();

  sConfig.Channel      = ADC_CHANNEL_2; /* PA1 */
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) Error_Handler();
}

/* -------------------------------------------------------------------------- */
/* TIM2 — servo PWM 50 Hz                                                     */
/* -------------------------------------------------------------------------- */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig      = {0};
  TIM_OC_InitTypeDef      sConfigOC          = {0};

  htim2.Instance               = TIM2;
  htim2.Init.Prescaler         = 169;
  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim2.Init.Period            = 19999;
  htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();

  sConfigOC.OCMode     = TIM_OCMODE_PWM1;
  sConfigOC.Pulse      = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();

  HAL_TIM_MspPostInit(&htim2);
}

/* -------------------------------------------------------------------------- */
/* TIM6 — 1 kHz                                                               */
/* -------------------------------------------------------------------------- */
static void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance               = TIM6;
  htim6.Init.Prescaler         = 169;
  htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim6.Init.Period            = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) Error_Handler();
}

/* -------------------------------------------------------------------------- */
/* USART2                                                                      */
/* -------------------------------------------------------------------------- */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance            = USART2;
  huart2.Init.BaudRate       = 115200;
  huart2.Init.WordLength     = UART_WORDLENGTH_8B;
  huart2.Init.StopBits       = UART_STOPBITS_1;
  huart2.Init.Parity         = UART_PARITY_NONE;
  huart2.Init.Mode           = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling   = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) Error_Handler();
}

/* -------------------------------------------------------------------------- */
/* DMA                                                                         */
/* -------------------------------------------------------------------------- */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/* -------------------------------------------------------------------------- */
/* GPIO                                                                        */
/* -------------------------------------------------------------------------- */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin   = DBG_GPIO_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DBG_GPIO_PORT, &GPIO_InitStruct);

  HAL_GPIO_WritePin(DBG_GPIO_PORT, DBG_GPIO_PIN, GPIO_PIN_RESET);
}

/* -------------------------------------------------------------------------- */
/* Error handler                                                               */
/* -------------------------------------------------------------------------- */
void Error_Handler(void)
{
  __disable_irq();
  while (1){}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line){ (void)file; (void)line; }
#endif
