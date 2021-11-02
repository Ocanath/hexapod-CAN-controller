#include "init.h"
#include "joint.h"
#include "CAN.h"
#include "rgb.h"
#include "uart-disp-tools.h"
#include <string.h>

static int led_idx = NUM_JOINTS;
static int prev_led_idx = NUM_JOINTS-1;
static uint32_t can_tx_ts = 0;


static inline float sat_v(float in, float hth, float lth)
{
	if(in > hth)
		in = hth;
	if(in < lth)
		in = lth;
	return in;
}
void can_network_keyboard_discovery(void);

/*
 * x is state variable for integral delay
 */
float ctl_PI(float err, ctl_params_t * ctl)
{
	float u = ctl->x_pi + ctl->kp*err;
	ctl->x_pi += (err/(ctl->ki_div));
	u = sat_v(u, ctl->tau_sat, -ctl->tau_sat);
	ctl->x_pi = sat_v(ctl->x_pi, ctl->x_sat, -ctl->x_sat);
	return u;
}

joint * joint_with_id(int16_t id, joint * joint_table, int num_joints)
{
	for(int i = 0; i < num_joints; i++)
	{
		if(joint_table[i].id == id)
			return &(joint_table[i]);
	}
	return NULL;
}

void blink_motors_in_chain(void);

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

	uint32_t td = 0;
	for(uint32_t start_ts = HAL_GetTick(); td < 3000;)
	{
		td = HAL_GetTick() - start_ts;
		float t = ((float)td)*.001f;
		uint8_t amp = (uint8_t)(t*(255.f/3000.f));
		rgb_play((rgb_t){amp/4, amp, amp/2});
	}

	joint_comm_misc(chain);
	HAL_Delay(100);
	for(int i = 0; i < NUM_JOINTS; i++)
		chain[i].misc_cmd = EN_UART_ENC; //chain[i].misc_cmd = EN_UART_ENC;
	for(int i = 0; i < NUM_JOINTS; i++)
		joint_comm_misc(&chain[i]);

	chain[0].tau.f32[0] = 15.f;

	joint_comm_motor(chain, NUM_JOINTS);
//	for(int m = 0; m < NUM_JOINTS; m++)
//		chain[m].qd=chain[m].q;

	rgb_play((rgb_t){0,255,0});
	uint32_t disp_ts = HAL_GetTick()+15;
//	can_network_keyboard_discovery();

	joint * j32 = joint_with_id(32,chain,NUM_JOINTS);
	j32->ctl.kp = 0.3f;
	j32->ctl.ki_div = 777.f;
	j32->ctl.x_pi = 0.f;
	j32->ctl.x_sat = 0.5f;
	j32->ctl.tau_sat = 0.5f;
	while(1)
	{

		for(int m = 0; m < NUM_JOINTS; m++)
		{
//			float err = wrap(chain[m].qd - chain[m].q);
//			float tau = ctl_PI(err, 0.3f, 222.f, &x_PI[m]);
//			x_PI[m] = sat_v(x_PI[m], 0.5f, -0.5f);
//			tau = sat_v(-tau, 0.5f, -0.5f);
			chain[m].tau.f32[0] = 0;
		}

		float err = wrap(j32->qd - j32->q);
		j32->tau.f32[0] = -ctl_PI(err, &j32->ctl);
		joint_comm_motor(chain, NUM_JOINTS);

		blink_motors_in_chain();
		if(HAL_GetTick()>disp_ts)
		{
			for(int i = 0; i < NUM_JOINTS; i++)
			{
				int pv = (int)(chain[i].q*RAD_TO_DEG*10000.f);
				char sc = '+';
				if(pv < 0)
				{
					sc = '-';
					pv = -pv;
				}
				int pv10 = pv/10000;

				for(int i = 0; i < 64; i++)
					gl_print_str[i] = 0;
				sprintf(gl_print_str, "%d.q=%c%d.%.4d, ", chain[i].id, sc, pv10, pv);
				print_string(gl_print_str);
			}
			print_string("\r\n");

			disp_ts = HAL_GetTick()+15;
		}

//		if(HAL_GetTick()>disp_ts)
//		{
//			for(int j = 0; j < NUM_JOINTS; j++)
//				print_float(chain[j].q);
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
				joint_comm_misc(&(chain[0]));


				chain[0].misc_cmd = EN_UART_ENC;
				chain[0].id = can_node_discovery_idx;
				joint_comm_misc(&(chain[0]));

				chain[0].misc_cmd = LED_ON;
				chain[0].id = can_node_discovery_idx;
				joint_comm_misc(&(chain[0]));
			}
			prev_discovery_idx = can_node_discovery_idx;

			sprintf(gl_print_str, "kbin = %c, id = %d, qenc = %d\r\n", uart_cmd, can_node_discovery_idx, (int)(chain[0].q*1000.f));
			print_string(gl_print_str);
		}
		joint_comm_motor(chain, 1);

	}

}


void blink_motors_in_chain(void)
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
			joint_comm_misc(&(chain[led_idx]));
		}
		if(prev_led_idx < NUM_JOINTS)
		{
			chain[prev_led_idx].misc_cmd = LED_OFF;
			joint_comm_misc(&(chain[prev_led_idx]));
		}

		prev_led_idx = led_idx;
	}

}
