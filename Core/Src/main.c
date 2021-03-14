#include "init.h"
#include "CAN.h"
#include "rgb.h"
#include "sin-math.h"
#include "uart-disp-tools.h"


#define NUM_JOINTS 2

joint chain[NUM_JOINTS] = {
		{
				.id = 22,
				.frame = 1,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.q_offset = 0.f,
				.tau = {.v = 0.f},
				.qd = 0.f,
				.misc_cmd = LED_OFF
		},
		{
				.id = 21,
				.frame = 1,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.q_offset = 0.f,
				.tau = {.v = 0.f},
				.qd = 0.f,
				.misc_cmd = LED_OFF
		}
};

static inline float v_pctl(float targ, float ref, float k)
{
	float thr = fmod_2pi(ref + PI) - PI;
	float vr1 = cos_fast(thr);
	float vr2 = sin_fast(thr);

	float th_targ = fmod_2pi(targ + PI) - PI;
	float vt1 = cos_fast(th_targ);
	float vt2 = sin_fast(th_targ);

	float vd1 = vt1 - vr1;
	float vd2 = vt2 - vr2;

	return k*(vr1*vd2 - vr2*vd1);
}

static inline float sat_v(float in, float hth, float lth)
{
	if(in > hth)
		in = hth;
	if(in < lth)
		in = lth;
	return in;
}

int main(void)
{

	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_ADC1_Init();
	HAL_Delay(100);
	CAN_Init();
	//	MX_CAN1_Init();
	MX_USART2_UART_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	CAN_comm_misc(chain);
	HAL_Delay(100);
	for(int i = 0; i < NUM_JOINTS; i++)
		chain[i].misc_cmd = DIS_UART_ENC; //chain[i].misc_cmd = EN_UART_ENC;
	for(int i = 0; i < NUM_JOINTS; i++)
		CAN_comm_misc(&chain[i]);

	int led_idx = NUM_JOINTS;
	int prev_led_idx = NUM_JOINTS-1;
	uint32_t can_tx_ts = 0;
	chain[0].tau.v = 15.f;
	chain[1].tau.v = 0;
	CAN_comm_motor(chain, NUM_JOINTS);

	rgb_play((rgb_t){0,255,0});



	uint32_t disp_ts = HAL_GetTick()+15;
	uint32_t tstart = HAL_GetTick();
	while(1)
	{
		if(HAL_GetTick()>can_tx_ts)
		{
			if(led_idx == NUM_JOINTS)
			{
				rgb_play((rgb_t){0,255,0});
				for(int i = 0; i < NUM_JOINTS; i++)
					chain[i].misc_cmd = LED_OFF;
			}
			else
			{
				rgb_play((rgb_t){0,0,0});
				for(int i = 0; i < NUM_JOINTS; i++)
					chain[i].misc_cmd = LED_OFF;
			}

			led_idx = (led_idx + 1) % (NUM_JOINTS+1);
			if(led_idx == NUM_JOINTS)
				can_tx_ts = HAL_GetTick()+1000;
			else
				can_tx_ts = HAL_GetTick()+75;

			if(led_idx < NUM_JOINTS)
			{
				chain[led_idx].misc_cmd = LED_ON;
				CAN_comm_misc(&(chain[led_idx]));
			}
			if(prev_led_idx < NUM_JOINTS)
			{
				chain[prev_led_idx].misc_cmd = LED_OFF;
				CAN_comm_misc(&(chain[prev_led_idx]));
			}

			prev_led_idx = led_idx;
		}

		float t = ((float)(HAL_GetTick()-tstart))*.001f;
		if(t < 3.f)
			chain[1].tau.v = -0.2f;
		else if (t >= 3.f && t < 3.2f)
		{
			chain[1].tau.v = -0.0f;
			chain[1].q_offset = chain[1].q;

		}
		else
		{
			float qd = 2.5f*TWO_PI*(.5f*sin_fast( (3.f*t-3.2f) - HALF_PI )+.5f)+.5f;
			float tau = (qd-chain[1].q)*0.15f;
			chain[1].tau.v = sat_v(tau, .25f, -.25f);
		}
		chain[0].tau.v = .1f;
		CAN_comm_motor(chain, NUM_JOINTS);

		if(HAL_GetTick()>disp_ts)
		{
			for(int i = 0; i < NUM_JOINTS; i++)
			{
				int pv = (int)(chain[i].q*1000.f);
				char sc = '+';
				if(pv < 0)
				{
					sc = '-';
					pv = -pv;
				}
				int pv10 = pv/1000;

				sprintf(gl_print_str, "q[%d]=%c%d.%d, ", i, sc,pv10,pv);
				print_string(gl_print_str);
			}
			print_string("\r\n");

			disp_ts = HAL_GetTick()+15;
		}

	}
}
