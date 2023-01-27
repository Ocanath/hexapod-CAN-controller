#include "init.h"
#include "joint.h"
#include "CAN.h"
#include "rgb.h"
#include "uart-disp-tools.h"
#include "trig_fixed.h"
#include "hexapod_params.h"
#include "m_uart.h"
#include "RS485-master.h"
#include <string.h>

static int led_idx = NUM_JOINTS;
static int prev_led_idx = NUM_JOINTS-1;
static uint32_t can_tx_ts = 0;


/*
Generic hex checksum calculation.
TODO: use this in the psyonic API
*/
uint32_t fletchers_checksum32(uint32_t* arr, int size)
{
	int32_t checksum = 0;
	int32_t fchk = 0;
	for (int i = 0; i < size; i++)
	{
		checksum += (int32_t)arr[i];
		fchk += checksum;
	}
	return fchk;
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

void buffer_data(uint8_t * d, int size_d, uint8_t * buf, int * bidx)
{
	for(int i = 0; i < size_d; i++)
	{
		buf[*bidx] = d[i];
		*bidx = *bidx + 1;
	}
}




void blink_motors_in_chain(void);

int main(void)
{

	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
//	MX_ADC1_Init();
	HAL_Delay(100);
	CAN_Init();
	//	MX_CAN1_Init();
	MX_USART2_UART_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	setup_dynamic_hex(&gl_hex);		//setup the structure to do forward kinematics
//	gl_hex.p_joint[0]->q16 = PI_12B/4;
//	gl_hex.p_joint[0]->child->q16 = PI_12B/4;
//	gl_hex.p_joint[0]->child->child->q16 = PI_12B/4;
//	joint * j = gl_hex.p_joint[0];
//	while(j != NULL)
//	{
//		int32_t sth = sin_lookup(j->q16,30);
//		int32_t cth = cos_lookup(j->q16,30);
//
//		j->sin_q_float = ((float)sth)/1073741824.f;
//		j->cos_q_float = ((float)cth)/1073741824.f;
//
//		j->sin_q = sth >> 1;
//		j->cos_q = cth >> 1;
//
//		j = j->child;
//	}
//	forward_kinematics(&gl_hex.hb_0[0], gl_hex.p_joint[0]);
	//forward_kinematics_64(&gl_hex.h32_b_0[0],gl_hex.p_joint[0],29);

	uint32_t start_ts = HAL_GetTick();
	for(uint32_t exp_ts = start_ts + 3000; HAL_GetTick() < exp_ts;)
	{
		uint32_t t = HAL_GetTick() - start_ts;
		uint8_t c = (t*255)/3000;
		rgb_t rgb = {c,c,c};
		rgb_play(rgb);
	}
//	joint_comm_misc(chain);
	HAL_Delay(100);

	for(int i = 0; i < NUM_JOINTS; i++)
		chain[i].misc_cmd = EN_UART_ENC; //chain[i].misc_cmd = EN_UART_ENC;
	for(int i = 0; i < NUM_JOINTS; i++)
		joint_comm_misc(&chain[i]);

	HAL_Delay(100);

	for(int i = 0; i < NUM_JOINTS; i++)
		chain[i].misc_cmd = SET_SINUSOIDAL_MODE;//SET_FOC_MODE; //chain[i].misc_cmd = EN_UART_ENC;
	for(int i = 0; i < NUM_JOINTS; i++)
		joint_comm_misc(&chain[i]);

	chain_comm(chain, NUM_JOINTS);

	rgb_play((rgb_t){0,255,0});
	//can_network_keyboard_discovery();

//	joint * j32 = joint_with_id(32,chain,NUM_JOINTS);
//	j32->ctl.kp = 9.0f;
//	j32->ctl.ki_div = 377.f;
//	j32->ctl.x_pi = 0.f;
//	j32->ctl.x_sat = 1.5f;
//	j32->ctl.kd = 0.20f/3.f;
//	j32->ctl.tau_sat = 0.85f;
//
//	j32->sin_q = sin_lookup(400,30);
//	j32->cos_q = (int32_t)(sin62b(400)>>32);
	uint32_t disp_ts = 0;
	while(1)
	{

		for(int m = 0; m < NUM_JOINTS; m++)
		{
			chain[m].mtn16.i16[0] = 0;
		}

		for(int m = 0; m < NUM_JOINTS; m++)
		{
			/*In the main/actual motion loop, only move joints that have responded properly*/
			if(chain[m].responsive)
			{
				joint_comm(&chain[m]);
			}
		}

		blink_motors_in_chain();

		if(HAL_GetTick() > disp_ts)
		{
			disp_ts = HAL_GetTick() + 10;

			u32_fmt_t payload[19] = {0};
			payload[0].i32 = (int32_t)chain[0].q32_rotor;
			payload[1].i32 = (int32_t)chain[1].q32_rotor;
			payload[18].u32 = fletchers_checksum32((uint32_t*)payload, 18);

			m_uart_tx_start(&m_huart2, (uint8_t*)payload, sizeof(u32_fmt_t)*19 );
		}
	}
}



/*Spoof a normal joint comm, without the overhead.
 * Send a command of 0 torque out, and wait for a
 * response from a matching ID
 *
 * CAN network analysis tool.
 * */
int check_joint(int id)
{
	can_tx_header.StdId = id;
	HAL_CAN_AddTxMessage(&hcan1, &can_tx_header, 0, &can_tx_mailbox);

	for(uint32_t exp_ts = HAL_GetTick()+1; HAL_GetTick() < exp_ts;)
	{
		if(HAL_CAN_IsTxMessagePending(&hcan1, can_tx_mailbox) == 0)
			break;
	}

	for(uint32_t exp_ts = HAL_GetTick()+10;  HAL_GetTick() < exp_ts;)
	{
		if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) >= 1)
		{
			if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can_rx_header, can_rx_data.d) == HAL_OK)
			{
				if(can_rx_header.StdId == id)
				{
					return 0;
				}
			}
		}
	}
	return -1;
}

static joint kb_j = {0};
void can_network_keyboard_discovery(void)
{

	print_string("STARTING NETWORK DISCOVERY...\r\n");
	print_string("Press any key to begin\r\n");
	gl_uart_rx_kb_activity_flag = 0;
	while(gl_uart_rx_kb_activity_flag == 0);
	gl_uart_rx_kb_activity_flag = 0;

	int num_responsive = 0;
	for(int i = 0; i < NUM_JOINTS; i++)
	{
		if(chain[i].responsive == 1)
		{
			sprintf(gl_print_str, "Joint %d (id = %d) responsive\r\n", i, chain[i].id);
			print_string(gl_print_str);
			num_responsive++;

			kb_j.misc_cmd = LED_ON;
			kb_j.id = chain[i].id;
			joint_comm_misc(&(kb_j));
		}
	}
	sprintf(gl_print_str, "Discovered %d out of %d expected nodes\r\n", num_responsive, NUM_JOINTS);
	print_string(gl_print_str);
	print_string("Do full network analysis? Y/N\r\n");
	while(gl_uart_rx_kb_activity_flag == 0);
	gl_uart_rx_kb_activity_flag = 0;
	char cmd = m_huart2.rx_buf[0];
	if(cmd != 'Y')
	{
		print_string("Skipping full network analysis.\r\n");
	}
	else
	{
		num_responsive = 0;
		print_string("Scanning for responsive motor joints...\r\n");
		int num_possible_ids = 1<<11;
		for(int id = 0; id < num_possible_ids; id++)
		{
			int rc = check_joint(id);
			if(rc == 0)
			{
				sprintf(gl_print_str, "Found node ID: %d\r\n", id);
				print_string(gl_print_str);
				num_responsive++;
			}
		}
		sprintf(gl_print_str, "Discovered %d nodes\r\n", num_responsive);
		print_string(gl_print_str);
	}

	print_string("Keyboard mode. Press > to go to next node, < to go back\r\n");

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

		if(gl_uart_rx_kb_activity_flag)
		{
			gl_uart_rx_kb_activity_flag = 0;
			char uart_cmd = (char)m_huart2.rx_buf[0];

			/*Keyboard in handle node selection*/
			if(uart_cmd == '>')
			{
				can_node_discovery_idx++;
			}
			else if(uart_cmd == '<')
			{
				can_node_discovery_idx--;
			}
			if(can_node_discovery_idx < 0)
				can_node_discovery_idx = 0;
			else if(can_node_discovery_idx > 128)
				can_node_discovery_idx = 128;//arbitary limit on node address which is lower than can-limited node address upper bound.

			/*Handle exit, motor velocity control commands*/
			if(uart_cmd == 'X')
			{
				print_string("Exiting keyboard interface...\r\n");
				break;
			}
			else if(uart_cmd == 'w')
			{
				kb_j.mtn16.i16[0] += 10;
				sprintf(gl_print_str, "n%d i/vq = %d, ", can_node_discovery_idx, (int)kb_j.mtn16.i16[0]);
				print_string(gl_print_str);
			}
			else if(uart_cmd == 's')
			{
				kb_j.mtn16.i16[0] -= 10;
				sprintf(gl_print_str, "n%d i/vq = %d, ", can_node_discovery_idx, (int)kb_j.mtn16.i16[0]);
				print_string(gl_print_str);
			}


			if(can_node_discovery_idx != prev_discovery_idx)
			{
				int stat_word = 0;
				int rc = 0;
				kb_j.misc_cmd = LED_OFF;
				kb_j.id = prev_discovery_idx;
				rc = joint_comm_misc(&(kb_j));
				stat_word |= rc;

				kb_j.misc_cmd = SET_SINUSOIDAL_MODE;
				kb_j.id = can_node_discovery_idx;
				rc = joint_comm_misc(&(kb_j));
				stat_word |= (rc << 3);

				kb_j.misc_cmd = EN_UART_ENC;
				kb_j.id = can_node_discovery_idx;
				rc = joint_comm_misc(&(kb_j));
				stat_word |= (rc << 1);

				kb_j.misc_cmd = LED_ON;
				kb_j.id = can_node_discovery_idx;
				rc = joint_comm_misc(&(kb_j));
				stat_word |= (rc << 2);


				sprintf(gl_print_str, "id %d setup code 0x%.2X ", can_node_discovery_idx, stat_word);
				print_string(gl_print_str);
			}
			prev_discovery_idx = can_node_discovery_idx;

			sprintf(gl_print_str, "kbin = %c, id = %d, qenc = %d, raw = 0x%.4X\r\n", uart_cmd, can_node_discovery_idx, (int)(kb_j.q*1000.f), (unsigned int)can_rx_data.ui32[0]);
			print_string(gl_print_str);
		}
		joint_comm(&kb_j);
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

		/*Search the leg list for the next valid joint*/
		for(int jidx = 0; jidx < NUM_JOINTS; jidx++)
		{
			led_idx = (led_idx + 1) % (NUM_JOINTS+1);
			if(led_idx == NUM_JOINTS)
				can_tx_ts = HAL_GetTick()+700;
			else
				can_tx_ts = HAL_GetTick()+500;
			if(chain[led_idx].responsive==1)
			{
				break;
			}
		}

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
