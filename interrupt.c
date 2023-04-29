#include "iodefine.h"
#include "interrupt.h"
#include "glob_var.h"
#include "parameters.h"
#include "spi.h"
#include "i2c.h"
#include "run.h"
#include "mytypedef.h"
#include "portdef.h"
#include "interface.h"
#include "machine.h"
#include "my_sin.h"
#include "my_cos.h"

void int_cmt0(void)
{
	/*****************************************************************************************
	�ڕW���x����
		tar_speed�̐���
		�Ȃ�
	*****************************************************************************************/
	// �����̏ꍇ�̖ڕW���x����
	if (run_mode == STRAIGHT_MODE || run_mode == BACK_STRAIGHT_MODE)
	{
		tar_speed += accel / 1000.0; // �ڕW���x��ݒ�����x�ōX�V
		// �ō����x����
		if (max_speed > 0)
		{ // �O�i��
			if (tar_speed > max_speed)
			{
				tar_speed = max_speed; // �ڕW���x��ݒ�ō����x�ɐݒ�
			}
		}
		else
		{ // ��i��
			if (tar_speed < max_speed)
			{
				tar_speed = max_speed; // �ڕW���x��ݒ�ō����x�ɐݒ�
			}
		}
	}
	else if (run_mode == TURN_MODE || run_mode == SLA_MODE || run_mode == ORBIT_FOLLOWING_MODE)
	{

		// �ԑ̒��S���x�X�V
		tar_speed += accel / 1000.0;
		// �ō����x����
		if (tar_speed > max_speed)
		{
			tar_speed = max_speed; // �ڕW���x��ݒ�ō����x�ɐݒ�
		}

        
		// �p�����x�X�V
		tar_ang_vel += ang_acc / 1000.0; // �ڕW�p���x��ݒ�����x�ōX�V
		tar_degree += (tar_ang_vel * 180.0 / PI) / 1000.0;

		// ����]�̏ꍇ
		if (TURN_DIR == LEFT)
		{
			// �ō��p���x����
			if (tar_ang_vel > max_ang_vel)
			{
				tar_ang_vel = max_ang_vel; // �ڕW���x��ݒ�ō����x�ɐݒ�
			}
			if (tar_degree > max_degree)
			{
				tar_degree = max_degree;
			}
		}
		else if (TURN_DIR == RIGHT)
		{
			// �E��]�̏ꍇ
			// �ō��p���x����
			if (tar_ang_vel < max_ang_vel)
			{
				tar_ang_vel = max_ang_vel; // �ڕW���x��ݒ�ō����x�ɐݒ�
			}
			if (tar_degree < max_degree)
			{
				tar_degree = max_degree;
			}
		}
		
	}
	else if (run_mode == NON_CON_MODE)
	{
		// �������Ȃ�
		nop();
	}
	else
	{
		// �������Ȃ�
		nop();
	}

	/*****************************************************************************************
	�ǐ���
		���ǃZ���T�ɂ��ڕW�p�x����
	*****************************************************************************************/
	if (run_mode == STRAIGHT_MODE)
	{
		if (con_wall.enable == true && sen_fr.value + sen_fl.value <= (TH_SEN_FR + TH_SEN_FL) * 5) // �ǐ��䂪������Ă��邩�`�F�b�N
		{

			con_wall.p_error = con_wall.error; // �ߋ��̕΍���ۑ�

			// ���E�̃Z���T���A���ꂼ��g�����Ԃł��邩�ǂ����`�F�b�N���āA�p������̕΍����v�Z
			if (con_wall.r_flag == 1 && con_wall.l_flag == 1)
			{ // �����Ƃ��L���������ꍇ�̕΍����v�Z
				con_wall.error = sen_r.error - sen_l.error;
			}
			else if (con_wall.r_flag == 1 && con_wall.l_flag == 0)
			{ // �E�ǂ̂�
				con_wall.error = sen_r.error;
			}
			else if (con_wall.r_flag == 0 && con_wall.l_flag == 1)
			{ // ���ǂ̂�
				con_wall.error = (-sen_l.error);
			}

			else
			{
				nop();
			}

			// DI����v�Z
			con_wall.diff = con_wall.error - con_wall.p_error; // �΍��̔����l���v�Z
			con_wall.sum += con_wall.error;					   // �΍��̐ϕ��l���v�Z

			if (con_wall.sum > con_wall.sum_max) // �΍��̐ϕ��l�̍ő�l�𐧌�
			{
				con_wall.sum = con_wall.sum_max;
			}
			else if (con_wall.sum < (-con_wall.sum_max)) // �΍��̐ϕ��l�̍Œ�l�𐧌�
			{
				con_wall.sum = -con_wall.sum_max;
			}

			con_wall.p_omega = con_wall.omega;
			con_wall.omega = con_wall.kp * con_wall.error * 0.5 + con_wall.p_omega * 0.5; // ���݂̖ڕW�p���x[rad/s]���v�Z
			tar_ang_vel = con_wall.omega;
		}
		else
		{
			tar_ang_vel = 0;
		}
	}
	else if (run_mode == NON_CON_MODE)
	{
		// �������Ȃ�
		nop();
	}
	else
	{
		nop();
	}

	/*****************************************************************************************
	�ǐ��䂩�X�����[���̗��z�l�ɂ���Đ������ꂽ
	�ڕW���x�ƖڕW�p�p�x�̐ϕ�
	*****************************************************************************************/
	I_tar_speed += tar_speed;
	if (I_tar_speed > 30 * 10000000000)
	{
		I_tar_speed = 30 * 10000000000;
	}
	else if (I_tar_speed < -1 * 10000000000)
	{
		I_tar_speed = 1 * 10000000000;
	}

	I_tar_ang_vel += tar_ang_vel;
	if (I_tar_ang_vel > 30 * 10000000000)
	{
		I_tar_ang_vel = 30 * 10000000000;
	}
	else if (I_tar_ang_vel < -1 * 10000000000)
	{
		I_tar_ang_vel = 1 * 10000000000;
	}

	I_start_degree += start_degree;
	if (I_start_degree > 30 * 10000000000)
	{
		I_start_degree = 30 * 10000000000;
	}
	else if (I_start_degree < -1 * 10000000000)
	{
		I_start_degree = -1 * 10000000000;
	}
	/*****************************************************************************************
	�ڕW���x�̕΍�����o�͓d���Ƀt�B�[�h�o�b�N

	*****************************************************************************************/
	// �t�B�[�h�o�b�N����
	V_r = V_l = 0.0;
	if (run_mode == STRAIGHT_MODE || run_mode == TURN_MODE || run_mode == SLA_MODE || run_mode == BACK_STRAIGHT_MODE || run_mode == ORBIT_FOLLOWING_MODE)
	{
		// ���i���̃t�B�[�h�o�b�N����

		// ���E���[�^�̃t�B�[�h�o�b�N
		// ���x�ɑ΂���P����

		V_r += 1 * (tar_speed - speed) * SPEED_KP / 1.0; // 15�ڕW�l�t�߂Ŕ��U
		V_l += 1 * (tar_speed - speed) * SPEED_KP / 1.0;
		// ���x�ɑ΂���I����

		V_r += 1 * (I_tar_speed - I_speed) * SPEED_KI / 1.0; //(0.4-0.3)*0.1 -> 0.01
		V_l += 1 * (I_tar_speed - I_speed) * SPEED_KI / 1.0;
		// ���x�ɑ΂���D����

		V_r -= 1 * (p_speed - speed) * SPEED_KD / 1.0; //(0.4-0.3)*0.1 -> 0.01
		V_l -= 1 * (p_speed - speed) * SPEED_KD / 1.0;
        
		// �p���x�ɑ΂���P����

		V_r += 1 * (tar_ang_vel - ang_vel) * (OMEGA_KP / 100.0);
		V_l -= 1 * (tar_ang_vel - ang_vel) * (OMEGA_KP / 100.0);
		// �p���x�ɑ΂���I����

		V_r += 1 * (I_tar_ang_vel - I_ang_vel) * (OMEGA_KI / 100.0); //(0.4-0.3)*0.1 -> 0.01
		V_l -= 1 * (I_tar_ang_vel - I_ang_vel) * (OMEGA_KI / 100.0);
		// �p���x�ɑ΂���D����

		V_r += 1 * (p_ang_vel - ang_vel) * (OMEGA_KD / 100.0); //(0.4-0.3)*0.1 -> 0.01
		V_l -= 1 * (p_ang_vel - ang_vel) * (OMEGA_KD / 100.0);

		if (run_mode == STRAIGHT_MODE && con_wall.enable == false)
		{ // IMU����i�����Ƃ��ǂ�������ΗL���j

			V_r += 1 * (start_degree - degree) * (DEGREE_KP / 1000);
			V_l -= 1 * (start_degree - degree) * (DEGREE_KP / 1000);

			V_r += 1 * (I_start_degree - I_degree) * (DEGREE_KI / 1000);
			V_l -= 1 * (I_start_degree - I_degree) * (DEGREE_KI / 1000);
		}
	}
	else if (run_mode == NON_CON_MODE)
	{
		// �������Ȃ�
		nop();
	}
	else
	{
		// �������Ȃ�
		nop();
	}

	/*****************************************************************************************
	�o�͓d�����烂�[�^�̉�]����������
		�d���͐�Βl����͂��邽�߁A
		���̒l�̎��̓��[�^�𐳓]�ݒ�ɂ��āA�d���͂��̂܂�
		���̒l�̎��̓��[�^���t�]�ݒ�ɂ��āA�d���𔽓]����
	*****************************************************************************************/
	if (run_mode != TEST_MODE)
	{
		// �E���[�^�̏o�͓d�������̏ꍇ
		if (V_r > 0)
		{
			// ���[�^�𐳓]�ɐݒ�
			MOT_CWCCW_R = MOT_R_FORWARD; // �E���[�^�𐳓]�ɐݒ�
			// V_r = V_r;			//�d���͐��Ȃ̂ł��̂܂�
		}
		else
		{
			// �E���[�^�̏o�͓d�������̏ꍇ
			MOT_CWCCW_R = MOT_R_BACK; // �E���[�^���t��]�ɐݒ�
			V_r = -V_r;				  // �d���𐳂̒l�֔��]
		}

		// �����[�^�̏o�͓d�������̏ꍇ
		if (V_l > 0)
		{
			// ���[�^�𐳓]�ɐݒ�
			MOT_CWCCW_L = MOT_L_FORWARD; // �����[�^�𐳓]�ɐݒ�
			// V_l = V_l;			//�d���͐��Ȃ̂ł��̂܂�
		}
		else
		{
			// �����[�^�̏o�͓d�������̏ꍇ
			MOT_CWCCW_L = MOT_L_BACK; // �����[�^���t��]�ɐݒ�
			V_l = -V_l;				  // �d���𐳂̒l�֔��]
		}
	}
	/*****************************************************************************************
	�o�͓d���̐���
		���[�^�ւ̏o�͓d���̏����2V�ɐ���
	*****************************************************************************************/
	// �E���[�^�Ɉ������d����2.0V�𒴂����狭���I��2.0V�ɕύX
	if (V_r > 2.0)
	{
		V_r = 2.0;
	}
	// �����[�^�Ɉ������d����2.0V�𒴂����狭���I��2.0V�ɕύX
	if (V_l > 2.0)
	{
		V_l = 2.0;
	}

	/*****************************************************************************************
	���[�^�֏o��

	*****************************************************************************************/

	// �o�b�e���[�d������f���[�e�B���v�Z
	Duty_r = V_r / V_bat;
	Duty_l = V_l / V_bat;

	// ���[�^��PWM���o��
	if (run_mode != TEST_MODE)
	{
		MOT_OUT_R = (short)(240.0 * Duty_r);
		MOT_OUT_L = (short)(240.0 * Duty_l);
	}

	/*****************************************************************************************
	�^�C�}�̃J�E���g

	*****************************************************************************************/
	timer++;
	cnt++;

}

void int_cmt1(void) // �Z���T�ǂݍ��ݗp�荞��
{
	/*****************************************************************************************
	A/D�ϊ�
		�Z���T�ƃo�b�e���[�d���擾
	*****************************************************************************************/
	static int state = 0; // �ǂݍ��ރZ���T�̃��[�e�[�V�����Ǘ��p�ϐ�
	int i;
	// long log_cnt;

	switch (state)
	{
	case 0: // �E�Z���T�ǂݍ���

		// �����t�B���^
		S12AD.ADANS0.BIT.ANS0 = 0x0004; // AN002
		S12AD.ADCSR.BIT.ADST = 1;		// AD�ϊ��J�n
		while (S12AD.ADCSR.BIT.ADST)
			;						 // AD�ϊ��I���܂ő҂�
		sen_r.d_value = S12AD.ADDR2; // ���l��ۑ�

		SLED_R = 1; // LED�_��
		for (i = 0; i < WAITLOOP_SLED; i++)
			;							// �t�H�g�g�����W�X�^�̉����҂����[�v
		S12AD.ADANS0.BIT.ANS0 = 0x0004; // AN002
		S12AD.ADCSR.BIT.ADST = 1;		// AD�ϊ��J�n
		while (S12AD.ADCSR.BIT.ADST)
			;		// AD�ϊ��I���܂ő҂�
		SLED_R = 0; // LED����

		sen_r.value = (S12AD.ADDR2 - sen_r.d_value); // �l��ۑ�

		if (sen_r.value > sen_r.th_wall) // �ǂ̗L���𔻒f
		{
			sen_r.is_wall = true; // �E�ǂ���
		}
		else
		{
			sen_r.is_wall = false; // �E�ǂȂ�
		}

		if (sen_r.value > sen_r.th_control) // ����������邩�ۂ��𔻒f
		{
			sen_r.error = sen_r.value - sen_r.ref; // �����������ꍇ�͕΍����v�Z
			sen_r.is_control = true;			   // �E�Z���T�𐧌�Ɏg��
		}
		else
		{
			sen_r.error = 0;		  // ����Ɏg��Ȃ��ꍇ�͕΍���0�ɂ��Ă���
			sen_r.is_control = false; // �E�Z���T�𐧌�Ɏg��Ȃ�
		}

		break;

	case 1: // �O���Z���T�ǂݍ���

		// �����t�B���^
		S12AD.ADANS0.BIT.ANS0 = 0x0001; // AN000
		S12AD.ADCSR.BIT.ADST = 1;		// AD�ϊ��J�n
		while (S12AD.ADCSR.BIT.ADST)
			;						  // AD�ϊ��I���܂ő҂�
		sen_fl.d_value = S12AD.ADDR0; // �l��ۑ�

		SLED_FL = 1; // LED�_��
		for (i = 0; i < WAITLOOP_SLED; i++)
			;							// �t�H�g�g�����W�X�^�̉����҂����[�v
		S12AD.ADANS0.BIT.ANS0 = 0x0001; // AN000
		S12AD.ADCSR.BIT.ADST = 1;		// AD�ϊ��J�n
		while (S12AD.ADCSR.BIT.ADST)
			;		 // AD�ϊ��I���܂ő҂�
		SLED_FL = 0; // LED����

		sen_fl.value = (S12AD.ADDR0 - sen_fl.d_value); // �l��ۑ�

		if (sen_fl.value > sen_fl.th_wall) // �ǂ̗L���𔻒f
		{
			sen_fl.is_wall = true; // ���O�ǂ���
		}
		else
		{
			sen_fl.is_wall = false; // ���O�ǂȂ�
		}
		break;

	case 2: // �O�E�Z���T�ǂݍ���

		// �����t�B���^
		S12AD.ADANS0.BIT.ANS0 = 0x0040; // AN006
		S12AD.ADCSR.BIT.ADST = 1;		// AD�ϊ��J�n
		while (S12AD.ADCSR.BIT.ADST)
			;						  // AD�ϊ��I���܂ő҂�
		sen_fr.d_value = S12AD.ADDR6; // �l��ۑ�

		SLED_FR = 1; // LED�_��
		for (i = 0; i < WAITLOOP_SLED; i++)
			;							// �t�H�g�g�����W�X�^�̉����҂����[�v
		S12AD.ADANS0.BIT.ANS0 = 0x0040; // AN006
		S12AD.ADCSR.BIT.ADST = 1;		// AD�ϊ��J�n
		while (S12AD.ADCSR.BIT.ADST)
			;		 // AD�ϊ��I���܂ő҂�
		SLED_FR = 0; // LED����

		sen_fr.value = (S12AD.ADDR6 - sen_fr.d_value); // �l��ۑ�

		if (sen_fr.value > sen_fr.th_wall) // �ǂ̗L���𔻒f
		{
			sen_fr.is_wall = true; // �E�O�ǂ���
		}
		else
		{
			sen_fr.is_wall = false; // �E�O�ǂȂ�
		}
		break;

	case 3: // ���Z���T�ǂݍ���

		// �����t�B���^
		S12AD.ADANS0.BIT.ANS0 = 0x0002; // AN001
		S12AD.ADCSR.BIT.ADST = 1;		// AD�ϊ��J�n
		while (S12AD.ADCSR.BIT.ADST)
			;						 // AD�ϊ��I���܂ő҂�
		sen_l.d_value = S12AD.ADDR1; // �l��ۑ�

		SLED_L = 1; // LED�_��
		for (i = 0; i < WAITLOOP_SLED; i++)
			;							// �t�H�g�g�����W�X�^�̉����҂����[�v
		S12AD.ADANS0.BIT.ANS0 = 0x0002; // AN001
		S12AD.ADCSR.BIT.ADST = 1;		// AD�ϊ��J�n
		while (S12AD.ADCSR.BIT.ADST)
			;		// AD�ϊ��I���܂ő҂�
		SLED_L = 0; // LED����

		sen_l.value = (S12AD.ADDR1 - sen_l.d_value); // �l��ۑ�

		if (sen_l.value > sen_l.th_wall) // �ǂ̗L���𔻒f
		{
			sen_l.is_wall = true; // ���ǂ���
		}
		else
		{
			sen_l.is_wall = false; // ���ǂȂ�
		}

		if (sen_l.value > sen_l.th_control) // ����������邩�ۂ��𔻒f
		{
			sen_l.error = sen_l.value - sen_l.ref; // �����������ꍇ�͕΍����v�Z����
			sen_l.is_control = true;			   // ���Z���T�𐧌�Ɏg��
		}
		else
		{
			sen_l.error = 0;		  // ����Ɏg��Ȃ��ꍇ�͕΍���0�ɂ��Ă���
			sen_l.is_control = false; // ���Z���T�𐧌�Ɏg��Ȃ�
		}

		break;
	}

	state++; // 4�񂲂ƂɌJ��Ԃ�
	if (state > 3)
	{
		state = 0;
	}

	if (sen_r.value > TH_SEN_R_POLE)
	{
		con_wall.r_flag = 1;
	}
	if (sen_l.value > TH_SEN_L_POLE)
	{
		con_wall.l_flag = 1;
	}
	if (sen_r.value < TH_SEN_R_POLE)
	{
		con_wall.r_flag = 0;
	}
	if (sen_l.value < TH_SEN_L_POLE)
	{
		con_wall.l_flag = 0;
	}

	S12AD.ADANS0.BIT.ANS0 = 0x0200; // AN009
	S12AD.ADCSR.BIT.ADST = 1;		// AD�ϊ��J�n
	while (S12AD.ADCSR.BIT.ADST)
		; // AD�ϊ��I���܂ő҂�
	V_bat = (2.0 * 3.3 * (float)(S12AD.ADDR9 / 4095.0));
	if (V_bat < 3.3)
	{

		// ���[�^�~�߂�
		Duty_r = 0;
		Duty_l = 0;
		MOT_POWER_OFF; // PC6(SLEEP�s��)

		// �u�U�[�炵������
		while (1)
		{
			BEEP();
		}
	}
	/*****************************************************************************************
	1kHz���ƂɃ��O���擾

	*****************************************************************************************/

	if (log_timer % 4 == 0 && log_flag == 1)
	{
		if (log_timer < (LOG_CNT * 4))
		{

			logs[0][log_timer / 4] = (int)(x_e);
			logs[1][log_timer / 4] = (int)(y_e);
			logs[2][log_timer / 4] = (int)(theta_e);
			logs[3][log_timer / 4] = (int)(now_dir);
			logs[4][log_timer / 4] = (int)(tar_x);
			logs[5][log_timer / 4] = (int)(tar_y);
			logs[6][log_timer / 4] = (int)(tar_th);
			logs[7][log_timer / 4] = (int)(degree * 10);
			logs[8][log_timer / 4] = (int)(tar_ang_vel*1000);
			logs[9][log_timer / 4] = (int)(tar_speed*1000);
			logs[10][log_timer / 4] = (long)(x_position);
			logs[11][log_timer / 4] = (long)(y_position);
		}
	}

	log_timer++;
}

void int_cmt2(void)
{
	static unsigned int enc_data_r; // �G���R�[�_�̐��f�[�^
	static unsigned int enc_data_l; // �G���R�[�_�̐��f�[�^
	static short state;
	/*****************************************************************************************
	�G���R�[�_�֘A
		�l�̎擾�@���x�X�V�@�����ϕ��Ȃ�
	*****************************************************************************************/
	if (state == 0)
	{
		RSPI0.SPCMD0.BIT.SSLA = 0x00; // SSL�M���A�T�[�g�ݒ�(SSL0���g��)
		preprocess_spi_enc(0xFFFF);	  // Read Angle
		enc_data_r = Get_enc_data();
		state = 1;
	}
	else
	{
		RSPI0.SPCMD0.BIT.SSLA = 0x02; // SSL�M���A�T�[�g�ݒ�(SSL2���g��)
		preprocess_spi_enc(0xFFFF);	  // Read Angle
		enc_data_l = Get_enc_data();

		// ���E�G���R�[�_����p�x�擾
		// 4096�ň��](360deg = 0deg)
		locate_r = enc_data_r;
		locate_l = enc_data_l;

		// �E�G���R�[�_�̌��݂̈ʒu��,1msec�O�̈ʒu�Ƃ̍������v�Z
		// �P�ʎ��ԁi1msec�j������̕ψʗʂ��v�Z
		diff_pulse_r = (locate_r - before_locate_r);
		// �ω��_��1023����0//�ֈړ������Ƃ��̕␳
		if ((diff_pulse_r > ENC_RES_HALF || diff_pulse_r < -ENC_RES_HALF) && before_locate_r > ENC_RES_HALF)
		{
			diff_pulse_r = (((ENC_RES_MAX - 1) - before_locate_r) + locate_r);
		}
		// �ω��_��0����1023�ֈړ������Ƃ��̕␳
		else if ((diff_pulse_r > ENC_RES_HALF || diff_pulse_r < -ENC_RES_HALF) && before_locate_r <= ENC_RES_HALF)
		{
			diff_pulse_r = 1 * (before_locate_r + ((ENC_RES_MAX - 1) - locate_r));
		}

		// ���G���R�[�_�̌��݂̈ʒu��,1msec�O�̈ʒu�Ƃ̍������v�Z
		// �P�ʎ��ԁi1msec�j������̕ψʗʂ��v�Z
		diff_pulse_l = (-locate_l + before_locate_l);
		// �ω��_��1023����0//�ֈړ������Ƃ��̕␳
		if ((diff_pulse_l > ENC_RES_HALF || diff_pulse_l < -ENC_RES_HALF) && before_locate_l > ENC_RES_HALF)
		{
			diff_pulse_l = 1 * (((ENC_RES_MAX - 1) - before_locate_l) + locate_l);
		}
		// �ω��_��0����1023�ֈړ������Ƃ��̕␳
		else if ((diff_pulse_l > ENC_RES_HALF || diff_pulse_l < -ENC_RES_HALF) && before_locate_l <= ENC_RES_HALF)
		{
			diff_pulse_l = (before_locate_l + ((ENC_RES_MAX - 1) - locate_l));
		}

		// ���ݑ��x���Z�o
		speed_new_r = (float)((float)diff_pulse_r * (float)MMPP);
		speed_new_l = (float)((float)diff_pulse_l * (float)MMPP);

		// �ߋ��̒l��ۑ�
		speed_old_r = speed_r;
		speed_old_l = speed_l;

		// ���x�̃��[�p�X�t�B���^
		speed_r = speed_new_r * 0.1 + speed_old_r * 0.9;
		speed_l = speed_new_l * 0.1 + speed_old_l * 0.9;

		p_speed = speed;
		// �ԑ̑��x���v�Z
		speed = ((speed_r + speed_l) / 2.0);

		// I�����̃I�[�o�[�t���[�ƃA���_�[�t���[�΍�
		I_speed += speed;
		if (I_speed > 30 * 10000000000)
		{
			I_speed = 30 * 10000000000;
		}
		else if (I_speed < -1 * 10000000000)
		{
			I_speed = -1 * 10000000000;
		}

		// �����̌v�Z
		len_mouse += (speed_new_r + speed_new_l) / 2.0;
		

		if (a < 0)
		{   
            if (a > -360)
			{
				th = -a;
			}
			else
			{
				th = -(a % 360); 
			}
			x_position += ((speed_new_r + speed_new_l) / 2.0) * ((float)cos_table[th] /10000.0);
			y_position += ((speed_new_r + speed_new_l) / 2.0) * ((float)-sin_table[th] /10000.0);
		}
		else
		{
			if (a < 360)
			{
				th = a;
			}
			else
			{
				th = (a % 360);
			}
			x_position += ((speed_new_r + speed_new_l) / 2.0) * ((float)cos_table[th] /10000.0);
		    y_position += ((speed_new_r + speed_new_l) / 2.0) * ((float)sin_table[th] /10000.0);
		}

		
        
		// �����̌v�Z���̓�
		sum_len_mouse += (speed_new_r + speed_new_l) / 2.0;
		// ���E�̃^�C���̈ړ�����
		//r_len_mouse += speed_new_r;
		//l_len_mouse += speed_new_l;
		// ���E�̃Z���T�̒l��臒l�ȉ��ɂȂ��Ă���̈ړ�����
		/*if (sen_r.value < TH_SEN_R)
		{
			TH_R_len_mouse += (speed_new_r + speed_new_l) / 2.0;
		}
		if (sen_l.value < TH_SEN_L)
		{
			TH_L_len_mouse += (speed_new_r + speed_new_l) / 2.0;
		}*/

		// �ߋ��̒l��ۑ�
		before_locate_r = locate_r;
		before_locate_l = locate_l;

		state = 0;
	}

	/*****************************************************************************************
	�W���C���֘A(���[��)
		�l�̎擾�@�p�x�̐ϕ��@
	*****************************************************************************************/
	if (state == 1)
	{
		// �W���C���Z���T�̒l�̍X�V
		preprocess_spi_gyro(0xB70000);

		// LowPass Filter
		gyro_x_new = (float)((short)(read_gyro_data() & 0x0000FFFF));
		gyro_x = (gyro_x_new - gyro_ref);

		// �p���x�̍X�V
		p_ang_vel = ang_vel;
		ang_vel = ((2000.0 * (gyro_x) / 32767.0)) * PI / 180.0;
		// �ϕ��l�̍X�V
		I_ang_vel += ang_vel;
		if (I_ang_vel > 30 * 10000000000)
		{
			I_ang_vel = 30 * 10000000000;
		}
		else if (I_ang_vel < -1 * 10000000000)
		{
			I_ang_vel = -1 * 10000000000;
		}

		// �W���C���̒l���p�x�ɕϊ�
		degree += (2.0 * (gyro_x_new - gyro_ref) / 32767.0);

		I_degree += degree;
		if (I_degree > 30 * 10000000000)
		{
			I_degree = 30 * 10000000000;
		}
		else if (I_degree < -1 * 10000000000)
		{
			I_degree = -1 * 10000000000;
		}

		
		a = (int)degree;
	}
}
