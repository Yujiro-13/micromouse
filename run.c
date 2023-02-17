/***********************************************************************/
/*                                                                     */
/*  FILE        :spi.c			                               */
/*  DATE        :Tue, Jun 08, 2017                                     */
/*  DESCRIPTION :SPI Program                                           */
/*  CPU TYPE    :                                                      */
/*                                                                     */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/*                                                                     */
/***********************************************************************/
#include "iodefine.h"
#include "mathf.h"
#include "sci.h"
#include "init.h"
#include "spi.h"
#include "parameters.h"
#include "glob_var.h"
#include "mytypedef.h"
#include "portdef.h"
#include "interface.h"

#define r_wall 3800
#define l_wall 3800
#define r_hosei 60
#define l_hosei 60
#define half_r_hosei 20
#define half_l_hosei 19
#define tar_min_len 40.06
#define tar_max_len 101.32
#define OFFSET_PRE 6
#define OFFSET_FOL 6

extern wait_ms(int wtime);

void straight(float len, float acc, float max_sp, float end_sp)
{
	char r_wall_check = 0, l_wall_check = 0, hosei_f = 0;
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	// tar_speed = 0;
	I_start_degree = 0;
	I_degree = 0;
	TH_R_len_mouse = 0;
	TH_L_len_mouse = 0;
	// ���s���[�h�𒼐��ɂ���
	run_mode = STRAIGHT_MODE;
	// �ǐ����L���ɂ���
	con_wall.enable = false;
	/*if (con_wall.r_flag == 1 && con_wall.l_flag == 1)
	{
		con_wall.enable = true;
	}
	else
	{
		con_wall.enable = false;
	}*/
	// �ڕW�������O���[�o���ϐ��ɑ������
	len_target = len;
	// �ڕW���x��ݒ�
	end_speed = end_sp;
	// �����x��ݒ�
	accel = acc;
	// �ō����x��ݒ�
	max_speed = max_sp;
	if (len_count == 0)
	{
		start_degree = degree;
	}

	// ���[�^�o�͂�ON
	MOT_POWER_ON;

	if ((end_speed != 0) && (len == SECTION))
	{
		r_wall_check = sen_r.is_wall;
		l_wall_check = sen_l.is_wall;
	}

	if (end_speed == 0)
	{ // �ŏI�I�ɒ�~����ꍇ

		// �����������n�߂�ׂ��ʒu�܂ŉ����A�葬��Ԃ𑱍s
		while (((len_target - 10) - len_mouse) < 1000.0 * ((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed)) / (float)(2.0 * accel))
		{
		}

		// ���������J�n
		accel = -acc; // �������邽�߂ɉ����x�𕉂̒l�ɂ���

		while (len_mouse < len_target)
		{ // ��~�����������̏�����O�܂Ōp��
			if ((sen_fr.value > r_wall) && (sen_fl.value > l_wall))
			{ // �ڕW�̈ʒu��5mm��O
				break;
			}

			// ��葬�x�܂Ō���������Œ�쓮�g���N�ő��s
			if (tar_speed <= MIN_SPEED)
			{ // �ڕW���x���Œᑬ�x�ɂȂ�����A�����x��0�ɂ���
				accel = 0;
				tar_speed = MIN_SPEED;
			}
		}
		accel = 0;
		tar_speed = 0;
		// ���x��0�ȉ��ɂȂ�܂ŋt�]����
		while (speed >= 0.0)
			;
	}
	else
	{
		// �����������n�߂�ׂ��ʒu�܂ŉ����A�葬��Ԃ𑱍s
		while (((len_target - 10) - len_mouse) > 1000.0 * ((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed)) / (float)(2.0 * accel))
		{

			if (len == SECTION)
			{
				if ((sen_r.is_wall == false) && (r_wall_check == true) && (hosei_f == 0))
				{
					len_mouse = (len_mouse + r_hosei) / 2;
					hosei_f = 1;
				}
				if ((sen_l.is_wall == false) && (l_wall_check == true) && (hosei_f == 0))
				{
					len_mouse = (len_mouse + l_hosei) / 2;
					hosei_f = 1;
				}
			}
		}

		// ���������J�n
		accel = -acc;
		// �������邽�߂ɉ����x�𕉂̒l�ɂ���
		while (len_mouse < len_target)
		{ // ��~�����������̏�����O�܂Ōp��

			// ��葬�x�܂Ō���������Œ�쓮�g���N�ő��s
			if (tar_speed <= end_speed)
			{ // �ڕW���x���Œᑬ�x�ɂȂ�����A�����x��0�ɂ���
				accel = 0;
				// tar_speed = end_speed;
			}
		}
	}
	// �����x��0�ɂ���
	accel = 0;
	// ���݋�����0�Ƀ��Z�b�g
	len_mouse = 0;

	if (len_target == 90)
	{
		len_count++;
	}
}

void back_straight(float len, float acc, float max_sp, float end_sp)
{ // �����p
	char r_wall_check = 0, l_wall_check = 0, hosei_f = 0;
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	// tar_speed = 0;
	I_start_degree = 0;
	I_degree = 0;
	TH_R_len_mouse = 0;
	TH_L_len_mouse = 0;
	len_mouse = 0;
	// ���s���[�h�𒼐��ɂ���
	run_mode = STRAIGHT_MODE;
	// �ǐ����L���ɂ���
	con_wall.enable = false;
	// �ڕW�������O���[�o���ϐ��ɑ������
	len_target = len;
	// �ڕW���x��ݒ�
	end_speed = end_sp;
	// �����x��ݒ�
	accel = acc;
	// �ō����x��ݒ�
	max_speed = max_sp;
	if (len_count == 0)
	{
		start_degree = degree;
	}

	// ���[�^�o�͂�ON
	MOT_POWER_ON;

	if ((end_speed != 0) && (len == SECTION))
	{
		r_wall_check = sen_r.is_wall;
		l_wall_check = sen_l.is_wall;
	}

	if (end_speed == 0)
	{ // �ŏI�I�ɒ�~����ꍇ

		// �����������n�߂�ׂ��ʒu�܂ŉ����A�葬��Ԃ𑱍s
		while (((len_target + 10) - len_mouse) < 1000.0 * ((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed)) / (float)(2.0 * accel))
			;
		// BEEP();
		//  ���������J�n
		accel = -acc; // �������邽�߂ɉ����x�𕉂̒l�ɂ���

		while (len_mouse > len_target)
		{ // ��~�����������̏�����O�܂Ōp��
			if (speed_new_r == 0 && speed_new_l == 0)
			{
				break;
			}
			// ��葬�x�܂Ō���������Œ�쓮�g���N�ő��s
			if (tar_speed >= -MIN_SPEED)
			{ // �ڕW���x���Œᑬ�x�ɂȂ�����A�����x��0�ɂ���
				accel = 0;
				tar_speed = -MIN_SPEED;
			}
		}
		accel = 0;
		tar_speed = 0;
		// ���x��0�ȉ��ɂȂ�܂ŋt�]����
		while (speed <= 0.0)
		{
			if (speed_new_r == 0 && speed_new_l == 0)
			{
				break;
			}
		}
	}
	else
	{
		// �����������n�߂�ׂ��ʒu�܂ŉ����A�葬��Ԃ𑱍s
		while (((len_target + 10) - len_mouse) < 1000.0 * ((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed)) / (float)(2.0 * accel))
		{

			if (len == SECTION)
			{
				if ((sen_r.is_wall == false) && (r_wall_check == true) && (hosei_f == 0))
				{
					len_mouse = (len_mouse + r_hosei) / 2;
					hosei_f = 1;
				}
				if ((sen_l.is_wall == false) && (l_wall_check == true) && (hosei_f == 0))
				{
					len_mouse = (len_mouse + l_hosei) / 2;
					hosei_f = 1;
				}
			}
		}
		// BEEP();
		//  ���������J�n
		accel = -acc;
		// �������邽�߂ɉ����x�𕉂̒l�ɂ���
		while (len_mouse > len_target)
		{ // ��~�����������̏�����O�܂Ōp��

			// ��葬�x�܂Ō���������Œ�쓮�g���N�ő��s
			if (tar_speed <= end_speed)
			{ // �ڕW���x���Œᑬ�x�ɂȂ�����A�����x��0�ɂ���
				accel = 0;
				// tar_speed = end_speed;
			}
		}
	}
	// �����x��0�ɂ���
	accel = 0;
	// ���݋�����0�Ƀ��Z�b�g
	len_mouse = 0;

	/*if (len_target == 90)
	{
		len_count++;
	}*/
}

void turn(int deg, float ang_accel, float max_ang_velocity, short dir)
{
	wait_ms(WAIT_TIME);
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	tar_degree = 0;
	degree = 0;

	float local_degree = 0;
	accel = 0;
	tar_speed = 0;
	tar_ang_vel = 0;
	// ���s���[�h���X�����[�����[�h�ɂ���
	run_mode = TURN_MODE;
	con_wall.enable = false;

	// ��]������`
	TURN_DIR = dir;

	// �ԑ̂̌��݊p�x���擾
	local_degree = degree;
	tar_degree = 0;

	sum = sum_len_mouse;
	count = len_count;
	sum_len_mouse = 0;
	len_count = 0;

	// �p�����x�A�����x�A�ō��p���x�ݒ�
	MOT_POWER_ON;
	if (dir == LEFT)
	{
		ang_acc = ang_accel; // �p�����x��ݒ�
		max_ang_vel = max_ang_velocity;
		max_degree = deg;
		while ((max_degree - (degree - local_degree)) * PI / 180.0 > (tar_ang_vel * tar_ang_vel / (2.0 * ang_acc)))
			;
	}
	else if (dir == RIGHT)
	{
		ang_acc = -ang_accel; // �p�����x��ݒ�
		max_ang_vel = -max_ang_velocity;
		max_degree = -deg;
		while (-(float)(max_degree - (degree - local_degree)) * PI / 180.0 > (float)(tar_ang_vel * tar_ang_vel / (float)(2.0 * -ang_acc)))
			;
	}

	// BEEP();
	// �p������Ԃɓ��邽�߁A�p�����x�ݒ�
	MOT_POWER_ON;
	if (dir == LEFT)
	{
		ang_acc = -ang_accel; // �p�����x��ݒ�
		// ������ԑ��s
		while ((degree - local_degree) < max_degree)
		{
			if (tar_ang_vel < TURN_MIN_SPEED)
			{
				ang_acc = 0;
				tar_ang_vel = TURN_MIN_SPEED;
			}
		}

		ang_acc = 0;
		tar_ang_vel = 0;
		tar_degree = max_degree;
	}
	else if (dir == RIGHT)
	{
		ang_acc = +ang_accel; // �p�����x��ݒ�
		// ������ԑ��s
		while ((degree - local_degree) > max_degree)
		{
			if (-tar_ang_vel < TURN_MIN_SPEED)
			{
				ang_acc = 0;
				tar_ang_vel = -TURN_MIN_SPEED;
			}
		}
		ang_acc = 0;
		tar_ang_vel = 0;
		tar_degree = max_degree;
	}

	while (ang_vel >= 0.05 || ang_vel <= -0.05)
		;

	tar_ang_vel = 0;
	ang_acc = 0;
	// ���݋�����0�Ƀ��Z�b�g
	len_mouse = 0;
	wait_ms(WAIT_TIME);
}

// �Ǔ��ėp
void check_straight(float len, float acc, float max_sp, float end_sp)
{

	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	I_start_degree = 0;
	I_degree = 0;
	TH_R_len_mouse = 0;
	TH_L_len_mouse = 0;
	len_mouse = 0;
	// ���s���[�h�𒼐��ɂ���
	run_mode = STRAIGHT_MODE;
	// �ǐ����L���ɂ���
	con_wall.enable = false;

	// ���E�G���R�[�_�����l�̌��ݒl
	bef_diff_pulse_r = locate_r;
	bef_diff_pulse_l = locate_l;
	// �ڕW�������O���[�o���ϐ��ɑ������
	len_target = len;
	// �ڕW���x��ݒ�
	end_speed = end_sp;
	// �����x��ݒ�
	accel = acc / 3;
	// �ō����x��ݒ�
	max_speed = max_sp;
	start_degree = degree;

	// ���[�^�o�͂�ON
	MOT_POWER_ON;

	// �����������n�߂�ׂ��ʒu�܂ŉ����A�葬��Ԃ𑱍s
	while (len_mouse < len_target)
	{
		wait_ms(100);
		if (speed_new_r == 0 && speed_new_l == 0)
		{
			break;
		}
	}
	accel = -acc * 2;
	while (speed >= 0.0)
		;

	accel = 0;
	tar_speed = 0;

	// �����x��0�ɂ���
	accel = 0;
	// ���݋�����0�Ƀ��Z�b�g
	len_mouse = 0;
}

void slalom(int deg, float ang_accel, float max_ang_velocity, short dir)
{

	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	I_start_degree = 0;
	I_degree = 0;
	TH_R_len_mouse = 0;
	TH_L_len_mouse = 0;
	r_len_mouse = 0;
	l_len_mouse = 0;
	degree = 0;
	len_mouse = 0;
	len_count = 0;

	float local_degree = 0;
	accel = 0;
	tar_speed = 0;
	tar_ang_vel = 0;
	ang_vel = 0;
	start_degree = 0;
	p_ang_vel = 0;
	max_ang_vel = 0;
	max_degree = 0;

	// ���s���[�h���X�����[�����[�h�ɂ���
	run_mode = SLA_MODE;
	con_wall.enable = false;

	// ��]������`
	TURN_DIR = dir;

	// �ԑ̂̌��݊p�x���擾
	local_degree = degree;
	tar_degree = 0;


	// �p�����x�A�����x�A�ō��p���x�ݒ�
	MOT_POWER_ON;

	len_target = OFFSET_PRE;
	accel = S_SEARCH_ACCEL;
	tar_speed = S_SEARCH_SPEED;
	
	// �����������n�߂�ׂ��ʒu�܂ŉ����A�葬��Ԃ𑱍s
	
	while (len_mouse < len_target)
	{ // ��~�����������̏�����O�܂Ōp��

        
		/*if (sen_fr.is_wall == true && sen_fl.is_wall == true)
		{
			if (sen_fr.value < 670 && sen_fl.value < 400)  //�E�O�Z���T��670�ȉ����A���O�Z���T��420�ȉ��Ȃ烋�[�v�i�ǂ��炩����ł��������甲����j
			{
				while (sen_fr.value <= 670 && sen_fl.value <= 400)
					;
			}
		}*/
	}

	// �����x��0�ɂ���
	accel = 0;
	// ���݋�����0�Ƀ��Z�b�g
	len_mouse = 0;
	
	

	tar_speed = S_SEARCH_SPEED;

	if (dir == LEFT)
	{
       
		max_ang_vel = max_ang_velocity;
		max_degree = deg;
		ang_acc = ang_accel; // �p�����x��ݒ�
		if(sen_fr.value == true && sen_fl.value == true){
        if (sen_fr.value > 700 && sen_fl.value > 420)  //�E�O�Z���T��670�ȉ����A���O�Z���T��420�ȉ��Ȃ烋�[�v�i�ǂ��炩����ł��������甲����j
			{
				max_ang_vel = SLALOM_SPEED_2;
		        max_degree = deg;
		        ang_acc = SLALOM_ACCEL_2; // �p�����x��ݒ�
			}
		}
		while ((max_degree - (degree - local_degree)) * PI / 180.0 > (tar_ang_vel * tar_ang_vel / (2.0 * ang_acc)))
			;
	}
	else if (dir == RIGHT)
	{
		max_ang_vel = -max_ang_velocity;
		max_degree = -deg;
		ang_acc = -ang_accel; // �p�����x��ݒ�
		if(sen_fr.value == true && sen_fl.value == true){
        if (sen_fr.value > 700 && sen_fl.value > 420)  //�E�O�Z���T��670�ȉ����A���O�Z���T��420�ȉ��Ȃ烋�[�v�i�ǂ��炩����ł��������甲����j
			{
				max_ang_vel = -SLALOM_SPEED_2;
		        max_degree = -deg;
		        ang_acc = -SLALOM_ACCEL_2; // �p�����x��ݒ�
			}
		}
		while (-(float)(max_degree - (degree - local_degree)) * PI / 180.0 > (float)(tar_ang_vel * tar_ang_vel / (float)(2.0 * -ang_acc)))
			;
	}

	// BEEP();
	// �p������Ԃɓ��邽�߁A�p�����x�ݒ�
	MOT_POWER_ON;
	if (dir == LEFT)
	{
		ang_acc = -ang_accel; // �p�����x��ݒ�
		// ������ԑ��s
		while ((degree - local_degree) < max_degree)
		{
			if (tar_ang_vel < SLA_MIN_SPEED)
			{
				ang_acc = 0;
				tar_ang_vel = SLA_MIN_SPEED;
			}
			
		}

		ang_acc = 0;
		tar_ang_vel = 0;
		tar_degree = max_degree;
	}
	else if (dir == RIGHT)
	{
		ang_acc = +ang_accel; // �p�����x��ݒ�
		// ������ԑ��s
		while ((degree - local_degree) > max_degree)
		{
			if (-tar_ang_vel < SLA_MIN_SPEED)
			{
				ang_acc = 0;
				tar_ang_vel = -SLA_MIN_SPEED;
			}
			
		}
		ang_acc = 0;
		tar_ang_vel = 0;
		tar_degree = max_degree;
	}
	
	 //while(ang_vel >= 0.05 || ang_vel <= -0.05 );
    

	

	
	len_target = OFFSET_FOL;
	accel = SEARCH_ACCEL;
	tar_speed = S_SEARCH_SPEED;
	len_mouse = 0;


	// �����������n�߂�ׂ��ʒu�܂ŉ����A�葬��Ԃ𑱍s
	while (len_mouse < len_target)
	{ // ��~�����������̏�����O�܂Ōp��
	  tar_ang_vel = 0;
      ang_vel = 0;
	  I_tar_ang_vel = 0;
	  I_ang_vel = 0;
	  p_ang_vel = 0;
	  if (sen_fr.is_wall == true && sen_fl.is_wall == true)
		{
			if (sen_fr.value < 360 && sen_fl.value < 208)
			{
				while (sen_fr.value <= 360 && sen_fl.value <= 208)
					;
			}
		}
		
	}

	// �����x��0�ɂ���
	accel = 0;
	// ���݋�����0�Ƀ��Z�b�g
	len_mouse = 0;

	tar_ang_vel = 0;
	ang_acc = 0;
	// ���݋�����0�Ƀ��Z�b�g
	len_mouse = 0;
	degree = 0;

	tar_speed = S_SEARCH_SPEED;
}

// �����p
void slalom_straight(float len, float acc, float max_sp, float end_sp)
{
	tar_ang_vel = 0;
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	I_start_degree = 0;
	I_degree = 0;
	TH_R_len_mouse = 0;
	TH_L_len_mouse = 0;
	// ���s���[�h�𒼐��ɂ���
	run_mode = SLA_MODE;
	// �ǐ����L���ɂ���
	con_wall.enable = false;
	// �ڕW�������O���[�o���ϐ��ɑ������
	len_target = len;
	// �ڕW���x��ݒ�
	end_speed = end_sp;
	// �����x��ݒ�
	accel = acc;
	// �ō����x��ݒ�
	max_speed = max_sp;

	degree = 0;
	// start_degree = degree;

	// ���[�^�o�͂�ON
	MOT_POWER_ON;

	if (end_speed == 0)
	{ // �ŏI�I�ɒ�~����ꍇ

		// �����������n�߂�ׂ��ʒu�܂ŉ����A�葬��Ԃ𑱍s
		while (((len_target - 10) - len_mouse) > 1000.0 * ((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed)) / (float)(2.0 * accel))
		{
		}

		// ���������J�n
		accel = -acc; // �������邽�߂ɉ����x�𕉂̒l�ɂ���

		while (len_mouse < len_target)
		{ // ��~�����������̏�����O�܂Ōp��

			// ��葬�x�܂Ō���������Œ�쓮�g���N�ő��s
			if (tar_speed <= MIN_SPEED)
			{ // �ڕW���x���Œᑬ�x�ɂȂ�����A�����x��0�ɂ���
				accel = 0;
				tar_speed = MIN_SPEED;
			}
		}
		accel = 0;
		tar_speed = 0;
		// ���x��0�ȉ��ɂȂ�܂ŋt�]����
		while (speed >= 0.0)
			;
	}
	else
	{
		// �����������n�߂�ׂ��ʒu�܂ŉ����A�葬��Ԃ𑱍s
		while (((len_target - 10) - len_mouse) > 1000.0 * ((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed)) / (float)(2.0 * accel))
			;

		// ���������J�n
		accel = -acc;
		// �������邽�߂ɉ����x�𕉂̒l�ɂ���
		while (len_mouse < len_target)
		{ // ��~�����������̏�����O�܂Ōp��

			// ��葬�x�܂Ō���������Œ�쓮�g���N�ő��s
			if (tar_speed <= end_speed)
			{ // �ڕW���x���Œᑬ�x�ɂȂ�����A�����x��0�ɂ���
				accel = 0;
				// tar_speed = end_speed;
			}
		}
	}
	// �����x��0�ɂ���
	accel = 0;
	// ���݋�����0�Ƀ��Z�b�g
	len_mouse = 0;

	degree = 0;
}

void slalom_straight_2(float len, float acc, float max_sp, float end_sp)
{
	char r_wall_check = 0, l_wall_check = 0, hosei_f = 0;
	tar_ang_vel = 0;
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	// tar_speed = 0;
	I_start_degree = 0;
	I_degree = 0;
	p_ang_vel = 0;
	TH_R_len_mouse = 0;
	TH_L_len_mouse = 0;
	// ���s���[�h�𒼐��ɂ���
	run_mode = STRAIGHT_MODE;
	// �ǐ����L���ɂ���
	con_wall.enable = false;
	if (con_wall.r_flag == 0 && con_wall.l_flag == 0)
	{ // ���ǖ����̎��̂�false
		con_wall.enable = false;
	}
	else
	{
		con_wall.enable = true;
	}
	// �ڕW�������O���[�o���ϐ��ɑ������
	len_target = len;
	// �ڕW���x��ݒ�
	end_speed = end_sp;
	// �����x��ݒ�
	accel = acc;
	// �ō����x��ݒ�
	max_speed = max_sp;
	if (len_count == 0)
	{
		start_degree = degree;
	}

	// ���[�^�o�͂�ON
	MOT_POWER_ON;

	if ((end_speed != 0) && (len == SECTION))
	{
		r_wall_check = sen_r.is_wall;
		l_wall_check = sen_l.is_wall;
	}
	else if ((end_speed != 0) && (len == HALF_SECTION))
	{
		r_wall_check = sen_r.is_wall;
		l_wall_check = sen_l.is_wall;
	}
	else
	{
	}

	if (end_speed == 0)
	{ // �ŏI�I�ɒ�~����ꍇ

		// �����������n�߂�ׂ��ʒu�܂ŉ����A�葬��Ԃ𑱍s
		while (((len_target - 10) - len_mouse) > 1000.0 * ((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed)) / (float)(2.0 * accel))
		{
			if ((sen_fr.value > r_wall) && (sen_fl.value > l_wall))
			{ // �ǂ�5mm��O
				break;
			}
		}

		// ���������J�n
		accel = -acc; // �������邽�߂ɉ����x�𕉂̒l�ɂ���

		while (len_mouse < len_target)
		{ // ��~�����������̏�����O�܂Ōp��
			if ((sen_fr.value > r_wall) && (sen_fl.value > l_wall))
			{ // �ǂ�5mm��O
				break;
			}
			else if (((sen_r.is_wall == false) && (TH_R_len_mouse > r_hosei)) || ((sen_l.is_wall == false) && (TH_L_len_mouse > l_hosei)))
			{
				break;
			}
			// ��葬�x�܂Ō���������Œ�쓮�g���N�ő��s
			if (tar_speed <= MIN_SPEED)
			{ // �ڕW���x���Œᑬ�x�ɂȂ�����A�����x��0�ɂ���
				accel = 0;
				tar_speed = MIN_SPEED;
			}
		}
		accel = 0;
		tar_speed = 0;
		// ���x��0�ȉ��ɂȂ�܂ŋt�]����
		while (speed >= 0.0)
			;
	}
	else
	{
		// �����������n�߂�ׂ��ʒu�܂ŉ����A�葬��Ԃ𑱍s
		while (((len_target - 10) - len_mouse) > 1000.0 * ((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed)) / (float)(2.0 * accel))
		{

			if (len == SECTION)
			{
				if ((sen_r.is_wall == false) && (r_wall_check == true) && (hosei_f == 0))
				{
					len_mouse = r_hosei;
					hosei_f = 1;
				}
				if ((sen_l.is_wall == false) && (l_wall_check == true) && (hosei_f == 0))
				{
					len_mouse = l_hosei;
					hosei_f = 1;
				}
			}
			if (len == HALF_SECTION)
			{
				if ((sen_r.is_wall == false) && (r_wall_check == true) && (hosei_f == 0))
				{
					len_mouse = half_r_hosei;
					hosei_f = 1;
				}
				if ((sen_l.is_wall == false) && (l_wall_check == true) && (hosei_f == 0))
				{
					len_mouse = half_l_hosei;
					hosei_f = 1;
				}
			}
			if(con_wall.r_flag == 0 && con_wall.l_flag == 0){
				con_wall.enable = false;
			}
		}

		// ���������J�n
		accel = -acc;
		// �������邽�߂ɉ����x�𕉂̒l�ɂ���
		while (len_mouse < len_target)
		{ // ��~�����������̏�����O�܂Ōp��

			if ((len_count >= 4) && (len_mouse > (len_target - 2)))
			{
				break;
			}
			// ��葬�x�܂Ō���������Œ�쓮�g���N�ő��s
			if (tar_speed <= end_speed)
			{ // �ڕW���x���Œᑬ�x�ɂȂ�����A�����x��0�ɂ���
				accel = 0;
				// tar_speed = end_speed;
			}
			if (len == HALF_SECTION)
			{
				if ((sen_r.is_wall == false) && (r_wall_check == true) && (hosei_f == 0))
				{
					len_mouse = half_r_hosei;
					hosei_f = 1;
				}
				if ((sen_l.is_wall == false) && (l_wall_check == true) && (hosei_f == 0))
				{
					len_mouse = half_l_hosei;
					hosei_f = 1;
				}
			}
			if(con_wall.r_flag == 0 && con_wall.l_flag == 0){
				con_wall.enable = false;
			}
		}
	}
	// �����x��0�ɂ���
	accel = 0;
	// ���݋�����0�Ƀ��Z�b�g
	len_mouse = 0;

	if (len_target == SECTION)
	{
		len_count++;
	}
}
