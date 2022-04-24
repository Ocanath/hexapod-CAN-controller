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


	chain[0].tau.f32[0] = 15.f;

	chain_comm(chain, NUM_JOINTS);
//	for(int m = 0; m < NUM_JOINTS; m++)
//		chain[m].qd=chain[m].q;

	rgb_play((rgb_t){0,255,0});
	can_network_keyboard_discovery();

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
	while(1)
	{
		handle_RS485_master(1);

		int16_t q1 = gl_rs485_nodes[0].theta.v;
		int16_t q2 = gl_rs485_nodes[1].theta.v;
		int16_t q3 = gl_rs485_nodes[2].theta.v;

		rgb_t rgb = {0};
		if(q1 > -2000 && q1 < 2000)
			rgb.r = 255;
		if(q2 > -2000 && q2 < 2000)
			rgb.g = 255;
		if(q3 > -2000 && q3 < 2000)
			rgb.b = 255;

		rgb_play(rgb);
	}
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

		float t = ((float)HAL_GetTick())*.001f;

//		j32->qd = 1.5f*sin_fast(t);
//		j32->qd = 2.0f;

//		float dqout_from_rotor = (j32->dq_rotor*-0.0625f);	//convert rotor speed to gearbox speed in rad/s. Sign flip from mechanism

//  		float err = wrap(j32->qd - j32->q);
//		float u = ctl_PI(err, &j32->ctl);
//		u -= j32->ctl.kd*dqout_from_rotor;//if we use rotor as damping instead of output velocity estimate, much lower noise (and higher damping as a result) is possible

//		j32->tau.i16[0] = (int16_t)(-4096.f*u);



		chain_comm(chain, NUM_JOINTS);

		/*
		 * HEY!!!!!!!!!!!!!!!!!!!!!
		 * HEY HEY HEY HEY!!!!!!!!!!!!!!!!!!!!!
		 * ALKSJALSDJALSDJALSDJALSDJALSDJALSDJALSDJ
		 *
		 *
		 * This does forward kinematics for properly initialized chain array,
		 * where you do triples with ascending frame association.
		 *
		 * It is floating point and appears to... disagree...
		 * with visual studio. issue likely arises from floating point numbers
		 * in the wrong scale, i.e. mixing 0-1 scale with
		 * 100 scale when computing distances.
		 *
		 *
		 * Solutions!!!
		 * 1. 32bit FIXED POINT.
		 * 		This option will be the absolute fastest option available.
		 * 		However, you can't just scale up mm to mm*4096 and do these
		 * 		calculations with impunity.
		 *
		 * 		For instance, (700mm * 4096)*4096 is a 33 bit number!!
		 *
		 * 		This is most likely the reason we're losing reso. It's
		 * 		also a reasonable culprit for simulation instability when
		 * 		doing numerical IK (by updating q+=tau/bignumber)
		 *
		 *		----------------FIXED POINT mm SCALING orders of magnitude maximum possible distances------------------
		 *
		 *		The max distance X you can have at any point in your 32bit fixed point kinematics calculation, given a scaling order n
		 *		is given by:
		 			X < (2^31-1)/(4096*2^n);
					X > (-2^31)/(4096*2^n);
		 *
		 *		That means:
		 *			order 8, draw a sphere centered at origin_baseframe of radius 2047mm. If any part
		 *				of your robot, or any imaginary point your robot foot might want to try and go to,
		 *				breaks out of that sphere, you'll overflow.
		 *
		 *		Reasonable orders are 4-8. You have to be careful about adding terms;
		 *		if you're order 8, but you add two sin*X terms, your real order is 9 wrt. overflow risk
		 *
		 *
		 * 2. 32bit floats, with normalized distance
		 * 		The idea here is that we would avoid
		 * 		repeated/accumulated floating point errors by
		 * 		normalizing our translations, doing FK, then
		 * 		re-scaling the translations back.
		 *
		 * 		This would be similar to, for instance,
		 * 		converting all the link distances from mm to m
		 * 		or inches, doing FK, then converting back to inches.
		 *
		 * 		This would potentially improve accuracy, but would not be
		 * 		much faster.
		 *
		 *
		 * 3. 64bit fixed. This allows for a large scaling factor for translations without risk of overflow.
		 * 		Same as option 1. but twice as slow.
		 *
		 *
		for(int leg = 0; leg < NUM_LEGS; leg++)
			forward_kinematics(gl_hex.hb_0[leg], gl_hex.p_joint[leg]);
		 */


		blink_motors_in_chain();

//		uint32_t tick = HAL_GetTick();
//		if(tick >= disp_ts)
//		{
//			disp_ts = tick+5;
//
//			floatsend_t fmt;
//			int bidx = 0;
//			uint8_t buf[NUM_JOINTS*sizeof(float)];
//
//			for(int i = 0; i < NUM_JOINTS; i++)
//			{
//				fmt.v = chain[i].q;
//				buffer_data(fmt.d, sizeof(float),buf,&bidx);
//			}
//			m_uart_tx_start(&m_huart2, buf, NUM_JOINTS*sizeof(float));
//		}
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
		chain_comm(chain, 1);

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
			can_tx_ts = HAL_GetTick()+700;
		else
			can_tx_ts = HAL_GetTick()+500;

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
