#include "init.h"
#include "CAN.h"
#include "rgb.h"
#include "sin-math.h"
#include "uart-disp-tools.h"


#define NUM_JOINTS 9

joint chain[NUM_JOINTS] = {
		{
				.id = 23,
				.frame = 1,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.q_offset = -0.1f,
				.tau = {.v = 0.f},
				.qd = 0.f,
				.misc_cmd = LED_OFF
		},
		{
				.id = 24,
				.frame = 1,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.q_offset = -0.258f,
				.tau = {.v = 0.f},
				.qd = 0.f,
				.misc_cmd = LED_OFF
		},
		{
				.id = 25,
				.frame = 1,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.q_offset = 0.190f,
				.tau = {.v = 0.f},
				.qd = 0.f,
				.misc_cmd = LED_OFF
		},
		{
				.id = 26,
				.frame = 1,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.q_offset = -1.111f,
				.tau = {.v = 0.f},
				.qd = 0.f,
				.misc_cmd = LED_OFF
		},
		{
				.id = 27,
				.frame = 1,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.q_offset = -3.004f,
				.tau = {.v = 0.f},
				.qd = 0.f,
				.misc_cmd = LED_OFF
		},
		{
				.id = 28,
				.frame = 1,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.q_offset = -3.341f,
				.tau = {.v = 0.f},
				.qd = 0.,
				.misc_cmd = LED_OFF
		},
		{
				.id = 20,
				.frame = 2,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.q_offset = 0.598f,
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
				.q_offset = -0.308f,
				.tau = {.v = 0.f},
				.qd = 0.f,
				.misc_cmd = LED_OFF
		},
		{
				.id = 22,
				.frame = 3,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.q_offset = 1.1163f,
				.tau = {.v = 0.f},
				.qd = 0.f,
				.misc_cmd = LED_OFF
		}
};

static inline float wrap(float in)
{
	return fmod_2pi(in + PI) - PI;
}

static inline float sat_v(float in, float hth, float lth)
{
	if(in > hth)
		in = hth;
	if(in < lth)
		in = lth;
	return in;
}
void can_network_keyboard_discovery(void);


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
		chain[i].misc_cmd = EN_UART_ENC; //chain[i].misc_cmd = EN_UART_ENC;
	for(int i = 0; i < NUM_JOINTS; i++)
		CAN_comm_misc(&chain[i]);

	int led_idx = NUM_JOINTS;
	int prev_led_idx = NUM_JOINTS-1;
	uint32_t can_tx_ts = 0;
	chain[0].tau.v = 15.f;
	chain[1].tau.v = 0;
	CAN_comm_motor(chain, NUM_JOINTS);
	for(int m = 0; m < NUM_JOINTS; m++)
		chain[m].qd=chain[m].q;

	rgb_play((rgb_t){0,255,0});
	uint32_t disp_ts = HAL_GetTick()+15;
//	can_network_keyboard_discovery();
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

//		chain[0].tau.v = 0.f;
		float t = ((float)HAL_GetTick())*.001f;
		for(int m = 0; m < NUM_JOINTS; m++)
		{
			if(chain[m].id == 21 || chain[m].id == 26 || chain[m].id == 24)
				chain[m].qd = 0.33f*sin_fast(t*2.f);
			else
				chain[m].qd = 0.f;
		}
		for(int m = 0; m < NUM_JOINTS; m++)
		{
			float tau = -0.33f*wrap(chain[m].qd - chain[m].q);
			tau = sat_v(tau, 0.2f, -0.2f);
			chain[m].tau.v = tau;
		}
		CAN_comm_motor(chain, NUM_JOINTS);

//		if(HAL_GetTick()>disp_ts)
//		{
//			for(int i = 0; i < NUM_JOINTS; i++)
//			{
//				int pv = (int)(chain[i].q*1000.f);
//				char sc = '+';
//				if(pv < 0)
//				{
//					sc = '-';
//					pv = -pv;
//				}
//				int pv10 = pv/1000;
//
//				sprintf(gl_print_str, "q[%d]=%c%d.%d, ", i, sc,pv10,pv);
//				print_string(gl_print_str);
//			}
//			print_string("\r\n");
//
//			disp_ts = HAL_GetTick()+15;
//		}

	}
}

void can_network_keyboard_discovery(void)
{

	int can_node_discovery_idx = 0;
	int prev_discovery_idx = 0;
	uint32_t led_ts = 0;
	uint8_t led_state = 0;
	while(1)
	{
		uint32_t tick = HAL_GetTick();
		if(tick > led_ts)
		{
			led_state = (~led_state)&1;
			if(led_state)
			{
				rgb_play((rgb_t){50,100,10});
			}
			else
			{
				rgb_play((rgb_t){0,0,0});
			}
			led_ts = tick + 250;
		}

		uint8_t rxne = (huart2.Instance->ISR & (1 << 5)) != 0;
		if(rxne)
		{
			uint16_t rdr = (uint16_t)huart2.Instance->RDR;	//read RDR, thus clearing RXNE
			char uart_cmd = (char)rdr;

			if(uart_cmd == '>')
				can_node_discovery_idx++;
			else if(uart_cmd == '<')
				can_node_discovery_idx--;
			if(can_node_discovery_idx < 0)
				can_node_discovery_idx = 0;
			else if(can_node_discovery_idx > 128)
				can_node_discovery_idx = 128;//arbitary limit on node address which is lower than can-limited node address upper bound.

			if(can_node_discovery_idx != prev_discovery_idx)
			{
				chain[0].misc_cmd = LED_OFF;
				chain[0].id = prev_discovery_idx;
				CAN_comm_misc(&(chain[0]));


				chain[0].misc_cmd = EN_UART_ENC;
				chain[0].id = can_node_discovery_idx;
				CAN_comm_misc(&(chain[0]));

				chain[0].misc_cmd = LED_ON;
				chain[0].id = can_node_discovery_idx;
				CAN_comm_misc(&(chain[0]));
			}
			prev_discovery_idx = can_node_discovery_idx;

			sprintf(gl_print_str, "kbin = %c, id = %d, qenc = %d\r\n", uart_cmd, can_node_discovery_idx, (int)(chain[0].q*1000.f));
			print_string(gl_print_str);
		}
		CAN_comm_motor(chain, 1);

	}

}
