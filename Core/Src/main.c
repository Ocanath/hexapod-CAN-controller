#include "init.h"
#include "joint.h"
#include "CAN.h"
#include "rgb.h"
#include "uart-disp-tools.h"
#include "trig_fixed.h"
#include "hexapod_params.h"
#include "m_uart.h"
#include "RS485-master.h"
#include "hexapod_footpath.h"
#include "kinematics.h"
#include "dynahex.h"
#include "m_mcpy.h"
#include <string.h>


static int led_idx = NUM_MOTORS;
static int prev_led_idx = NUM_MOTORS-1;
static uint32_t can_tx_ts = 0;

int32_t get_qkinematic_from_qenc(motor_t * m)
{
	int32_t offset = (int32_t)(m->q_offset*4096.f);
	if(m->reverese_dir == 0)
		return (m->q16 - offset);
	else
		return -(m->q16 - offset);
}


int32_t get_qenc_from_qkinematic(int32_t qkinematic, motor_t * m)
{
	int32_t offset = (int32_t)(m->q_offset*4096.f);
	if(m->reverese_dir == 0)
		return (qkinematic + offset);
	else
		return  (offset - qkinematic);	//algembras
}

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

motor_t * motor_t_with_id(int16_t id, motor_t * motor_t_table, int num_motor_ts)
{
	for(int i = 0; i < num_motor_ts; i++)
	{
		if(motor_t_table[i].id == id)
			return &(motor_t_table[i]);
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


void send_i32_val(motor_t * m, uint8_t header, int32_t val)
{
	int bkp = can_tx_header.DLC;
	can_tx_header.DLC = 5;

	m->misc_cmd = header;

	m->mtn16.d[0] = header;	//redundant
	u32_fmt_t fmt;
	fmt.i32 = val;
	for(int i = 0; i < sizeof(int32_t); i++)
	{
		m->mtn16.d[i+1] = fmt.u8[i];
	}
	motor_t_comm_misc(m);

	can_tx_header.DLC = bkp;
}

void send_u8_val(motor_t * m, uint8_t header, uint8_t val)
{
	int bkp = can_tx_header.DLC;
	can_tx_header.DLC = 2;

	m->misc_cmd = header;

	m->mtn16.d[0] = header;	//redundant
	m->mtn16.d[1] = val;
	motor_t_comm_misc(m);

	can_tx_header.DLC = bkp;
}

void heartbeat_blinkall(void)
{
	for(int blink = 0; blink < 2; blink++)
	{
		for(int led_idx = 0; led_idx < NUM_MOTORS; led_idx++)
		{
			send_u8_val(&chain[led_idx], LED_ON, 0);
		}
		HAL_Delay(50);
		for(int led_idx = 0; led_idx < NUM_MOTORS; led_idx++)
		{
			send_u8_val(&chain[led_idx], LED_OFF, 0);
		}
		HAL_Delay(50);
	}
}

uint8_t do_midpoint_analysis(motor_t * j)
{
	send_u8_val(j, CALC_ENC_MIDPOINTS, 0);
	if(can_rx_data.ui32[0] == 0x143833ce)
	{
		for(uint32_t exp_ts = HAL_GetTick()+30000;  HAL_GetTick() < exp_ts;)
		{
			if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) >= 1)
			{
				if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can_rx_header, can_rx_data.d) == HAL_OK)
				{
					if(can_rx_header.StdId == 32)
					{
						if(can_rx_data.ui32[0] == 0xbeb8ac63)
						{
							return 1;
						}
					}
				}
			}
		}
	}
	return 0;
}




uint8_t do_align_offset_calibration(motor_t * j)
{
	send_u8_val(j, CALC_ALIGN_OFFSET, 0);
	if(can_rx_data.ui32[0] == 0xAD57E3B9)
	{
		int32_t new_align_offset = 0;
		int32_t prev_align_offset = 0;
		int match_count = 0;
		while(1)
		{
			while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0);	//blocking wait
			if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can_rx_header, can_rx_data.d) != HAL_OK)
			{
				if(can_rx_header.StdId == can_tx_header.StdId)
				{
					new_align_offset = can_rx_data.i32[0];
					break;
				}
			}

			if(prev_align_offset != new_align_offset)
				match_count = 0;
			else
				match_count++;

			prev_align_offset = new_align_offset;


			if(match_count >= 20)
			{
				can_payload_t ctx = {0};
				ctx.d[0] = 0xDE;
				HAL_CAN_AddTxMessage(&hcan1, &can_tx_header, ctx.d, &can_tx_mailbox);
			}
		}
	}
	return 0;
}


uint8_t save_flash_mem(motor_t * j)
{
	send_u8_val(j, CMD_WRITE_FLASH, 0);
	if(can_rx_data.ui32[0] == 0xDEADBEEF)
	{
		return 1;
	}
	return 0;
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


	dynahex_t hexapod;
	init_dynahex_kinematics(&hexapod);
	for(int leg = 0; leg < 6; leg++)
	{
		joint * j = hexapod.leg[leg].chain;
		j[1].q = 0;
		j[2].q = PI/2;
		j[3].q = 0;
	}
	forward_kinematics_dynahexleg(&hexapod);

	vect3_t targs[6] = {
			{{332.5646f, -233.385513f, 5.380200f}},
			{{ 368.400146, 171.316727, 5.380200}},
			{{35.835365, 404.702271, 5.380200}},
			{{-332.564697, 233.385468, 5.380200}},
			{{ -368.400085, -171.316803, 5.380200}},
			{{-35.835426,  -404.702271, 5.380200 }}
	};
	volatile uint32_t ik_start_ts = HAL_GetTick();
	vect3_t zero = { {0,0,0} };
	for (int leg = 0; leg < 6; leg++)
	{
		vect3_t targ_b;
		m_mcpy(&targ_b, &targs[leg], sizeof(vect3_t));

		mat4_t* hb_0 = &hexapod.leg[leg].chain[0].him1_i;
		joint* start = &hexapod.leg[leg].chain[1];
		joint* end = &hexapod.leg[leg].chain[3];
		vect3_t anchor_b;
		int cycles = gd_ik_single(hb_0, start, end, &zero, &targ_b, &anchor_b, 20000.f);
	}
	volatile uint32_t ik_end_ts = HAL_GetTick();

	uint32_t start_ts = HAL_GetTick();
	const uint32_t fade_up_time = 1500;
	for(uint32_t exp_ts = start_ts + fade_up_time; HAL_GetTick() < exp_ts;)
	{
		uint32_t t = HAL_GetTick() - start_ts;
		uint8_t c = (t*255)/fade_up_time;
		rgb_t rgb = {c,c,c};
		rgb_play(rgb);
	}
	//	motor_t_comm_misc(chain);

	heartbeat_blinkall();
	HAL_Delay(300);

	for(int i = 0; i < NUM_MOTORS; i++)
		chain[i].misc_cmd = EN_UART_ENC; //chain[i].misc_cmd = EN_UART_ENC;
	for(int i = 0; i < NUM_MOTORS; i++)
		motor_t_comm_misc(&chain[i]);

	HAL_Delay(100);

	for(int i = 0; i < NUM_MOTORS; i++)
		chain[i].misc_cmd = SET_SINUSOIDAL_MODE;//SET_FOC_MODE; //chain[i].misc_cmd = EN_UART_ENC;
	for(int i = 0; i < NUM_MOTORS; i++)
		motor_t_comm_misc(&chain[i]);

	rgb_play((rgb_t){0,255,0});
	//can_network_keyboard_discovery();

	ctl_params_t template_ctl =
	{
			.kp = 1000.f,
			.tau_sat = 1500.f,

			.ki_div = 3.f,
			.x_sat = 1500.f,

			.x_pi = 0
	};
	m_mcpy(&chain[0].ctl, &template_ctl,sizeof(ctl_params_t));
	m_mcpy(&chain[1].ctl, &template_ctl,sizeof(ctl_params_t));


	for(int i = 0; i < NUM_MOTORS; i++)
	{
		chain[i].control_mode = SET_PCTL_VQ_MODE;
		send_u8_val(&chain[i], SET_PCTL_VQ_MODE, 0);
		send_i32_val(&chain[i], CHANGE_PCTL_VQ_KP_VALUE, 7000);
		send_u8_val(&chain[i], CHANGE_PCTL_VQ_KP_RADIX, 5);
		send_i32_val(&chain[i], CHANGE_PCTL_VQ_KI_VALUE, 1000);
		send_u8_val(&chain[i], CHANGE_PCTL_VQ_KI_RADIX, 7);
		send_i32_val(&chain[i], CHANGE_PCTL_VQ_XSAT, 300);
		send_u8_val(&chain[i], CHANGE_PCTL_VQ_OUT_RSHIFT, 11);
		send_i32_val(&chain[i], CHANGE_PCTL_VQ_KD_VALUE, 10);
		send_u8_val(&chain[i], CHANGE_PCTL_VQ_KD_RADIX, 6);
		send_i32_val(&chain[i], CHANGE_PCTL_VQ_OUTSAT, 0);

		send_u8_val(&chain[i], EN_REVERSE_DIRECTION, 0);
	}
	send_i32_val(&chain[0], CHANGE_PCTL_VQ_OUTSAT, 500);
	send_i32_val(&chain[1], CHANGE_PCTL_VQ_OUTSAT, 500);
	send_i32_val(&chain[2], CHANGE_PCTL_VQ_OUTSAT, 500);

	//	send_u8_val(&chain[test_motor_idx], SET_SINUSOIDAL_MODE, 0);
	//	chain[test_motor_idx].control_mode = SET_SINUSOIDAL_MODE;

	int16_t qdes[NUM_MOTORS] = {0};
	qdes[0] = 0;
	qdes[1] = (int16_t)((-17.466686f*DEG_TO_RAD)*4096.f);
	qdes[2] = (int16_t)((-10.74216371f*DEG_TO_RAD)*4096.f);

	u32_fmt_t payload[19] = {0};

	heartbeat_blinkall();
	HAL_Delay(300);

	uint32_t disp_ts = 0;
	while(1)
	{
		float time = ((float)HAL_GetTick())*.001f;


		ik_start_ts = HAL_GetTick();

		{
			vect3_t foot_xy_1;
			float h = 40; float w = 100;
			float period = 2.f;
			foot_path(time, h, w, period, &foot_xy_1);

			vect3_t foot_xy_2;
			foot_path(time+period/2.f, h, w, period, &foot_xy_2);


			/*Rotate and translate*/
			float forward_direction_angle = 0.f;	//y axis!
			mat4_t xrot = Hx(HALF_PI);
			mat4_t zrot = Hz(forward_direction_angle - HALF_PI);
			vect3_t tmp;
			htmatrix_vect3_mult(&xrot, &foot_xy_1, &tmp);
			htmatrix_vect3_mult(&zrot, &tmp, &foot_xy_1);	//done 1
			htmatrix_vect3_mult(&xrot, &foot_xy_2, &tmp);
			htmatrix_vect3_mult(&zrot, &tmp, &foot_xy_2);	//done 2


			//			int leg = 0;
			for(int leg = 0; leg < NUM_LEGS; leg++)
			{
				mat4_t lrot = Hz((TWO_PI / 6.f) * (float)leg);
				vect3_t o_motion_b = {{ 270.f,0.f,-270.f }};
				htmatrix_vect3_mult(&lrot, &o_motion_b, &tmp);
				o_motion_b = tmp;

				vect3_t targ_b;
				for (int i = 0; i < 3; i++)
				{
					if (leg % 2 == 0)
					{
						targ_b.v[i] = foot_xy_1.v[i] + o_motion_b.v[i];
					}
					else
					{
						targ_b.v[i] = foot_xy_2.v[i] + o_motion_b.v[i];
					}
				}

				mat4_t* hb_0 = &hexapod.leg[leg].chain[0].him1_i;
				joint* start = &hexapod.leg[leg].chain[1];
				//joint* end = &hexapod.leg[leg].chain[3];
				//vect3_t zero = { {0,0,0} };
				//vect3_t anchor_b;
				//gd_ik_single(hb_0, start, end, &zero, &targ_b, &anchor_b, 20000.f);
				ik_closedform_hexapod(hb_0, start, &targ_b);
			}
		}
		ik_end_ts = HAL_GetTick();
		/*uncomment to map kinematics to qdes*/
		//		int pld_idx = 0;
		//		for(int leg = 0; leg < NUM_LEGS; leg++)
		//		{
		//			joint * j = &hexapod.leg[leg].chain[1];
		//			for(int i = 0; i < 3; i++)
		//			{
		//				qdes[pld_idx++] = (int16_t)(j->q*4096.f);
		//				j = j->child;
		//			}
		//		}

		for(int m = 0; m < NUM_MOTORS; m++)
		{
			motor_t * pmotor = &chain[m];
			pmotor->mtn16.i16[0] = get_qenc_from_qkinematic(qdes[m], pmotor);
			motor_t_comm(pmotor);
		}

		/*This interferes with i16 encoder reception on joint id 24 and possibly 25*/
		blink_motors_in_chain();

		if(HAL_GetTick() > disp_ts)
		{
			disp_ts = HAL_GetTick() + 15;

			for(int i = 0; i < NUM_MOTORS; i++)
			{
				motor_t * m = &chain[i];
				payload[i].i32 = get_qkinematic_from_qenc(m);
			}
			payload[18].u32 = fletchers_checksum32((uint32_t*)payload, 18);

			m_uart_tx_start(&m_huart2, (uint8_t*)payload, sizeof(u32_fmt_t)*19 );
		}

		//test function for centralized position control
		//		{
		//			float e = wrap_2pi(hexapod.leg[0].chain[1].q - chain[0].q);
		//			float vq = ctl_PI(e, &chain[0].ctl);
		//			chain[0].mtn16.i16[0] = (int16_t)vq;
		//		}
		//		{
		//			float e = wrap_2pi(hexapod.leg[0].chain[2].q - chain[1].q);
		//			float vq = ctl_PI(e, &chain[1].ctl);
		//			chain[1].mtn16.i16[0] = (int16_t)vq;
		//		}
	}
}



/*Spoof a normal motor_t comm, without the overhead.
 * Send a command of 0 torque out, and wait for a
 * response from a matching ID
 *
 * CAN network analysis tool.
 * */
int check_motor_t(int id)
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

static motor_t kb_j = {0};
void can_network_keyboard_discovery(void)
{

	print_string("STARTING NETWORK DISCOVERY...\r\n");
	print_string("Press any key to begin\r\n");
	gl_uart_rx_kb_activity_flag = 0;
	while(gl_uart_rx_kb_activity_flag == 0);
	gl_uart_rx_kb_activity_flag = 0;

	int num_responsive = 0;
	for(int i = 0; i < NUM_MOTORS; i++)
	{
		if(chain[i].responsive == 1)
		{
			sprintf(gl_print_str, "motor_t %d (id = %d) responsive\r\n", i, chain[i].id);
			print_string(gl_print_str);
			num_responsive++;

			kb_j.misc_cmd = LED_ON;
			kb_j.id = chain[i].id;
			motor_t_comm_misc(&(kb_j));
		}
	}
	sprintf(gl_print_str, "Discovered %d out of %d expected nodes\r\n", num_responsive, NUM_MOTORS);
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
		print_string("Scanning for responsive motor motor_ts...\r\n");
		int num_possible_ids = 1<<11;
		for(int id = 0; id < num_possible_ids; id++)
		{
			int rc = check_motor_t(id);
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
				rc = motor_t_comm_misc(&(kb_j));
				stat_word |= rc;

				kb_j.misc_cmd = SET_SINUSOIDAL_MODE;
				kb_j.id = can_node_discovery_idx;
				rc = motor_t_comm_misc(&(kb_j));
				stat_word |= (rc << 3);

				kb_j.misc_cmd = EN_UART_ENC;
				kb_j.id = can_node_discovery_idx;
				rc = motor_t_comm_misc(&(kb_j));
				stat_word |= (rc << 1);

				kb_j.misc_cmd = LED_ON;
				kb_j.id = can_node_discovery_idx;
				rc = motor_t_comm_misc(&(kb_j));
				stat_word |= (rc << 2);


				sprintf(gl_print_str, "id %d setup code 0x%.2X ", can_node_discovery_idx, stat_word);
				print_string(gl_print_str);
			}
			prev_discovery_idx = can_node_discovery_idx;

			sprintf(gl_print_str, "kbin = %c, id = %d, qenc = %d, raw = 0x%.4X\r\n", uart_cmd, can_node_discovery_idx, (int)(kb_j.q*1000.f), (unsigned int)can_rx_data.ui32[0]);
			print_string(gl_print_str);
		}
		motor_t_comm(&kb_j);
	}

}


void blink_motors_in_chain(void)
{
	if(HAL_GetTick()>can_tx_ts)
	{
		if(led_idx == NUM_MOTORS)
		{
			rgb_play((rgb_t){0,255,0});
			for(int i = 0; i < NUM_MOTORS; i++)
				chain[i].misc_cmd = LED_OFF;
		}
		else
		{
			rgb_play((rgb_t){0,0,0});
			for(int i = 0; i < NUM_MOTORS; i++)
				chain[i].misc_cmd = LED_OFF;
		}

		/*Search the leg list for the next valid motor_t*/
		for(int jidx = 0; jidx < NUM_MOTORS; jidx++)
		{
			led_idx = (led_idx + 1) % (NUM_MOTORS+1);
			if(led_idx == NUM_MOTORS)
				can_tx_ts = HAL_GetTick()+0;
			else
				can_tx_ts = HAL_GetTick()+50;
			//			if(chain[led_idx].responsive==1)
			//			{
			//				break;
			//			}
		}

		if(led_idx < NUM_MOTORS)
		{
			chain[led_idx].misc_cmd = LED_ON;
			motor_t_comm_misc(&(chain[led_idx]));
		}
		if(prev_led_idx < NUM_MOTORS)
		{
			chain[prev_led_idx].misc_cmd = LED_OFF;
			motor_t_comm_misc(&(chain[prev_led_idx]));
		}

		prev_led_idx = led_idx;
	}

}
