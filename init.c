/***********************************************************************/
/*                                                                     */
/*  FILE        :init.c                                                */
/*  DATE        :2017/6/27	                                       */
/*  DESCRIPTION :Main Program                                          */
/*  CPU TYPE    :RX631 48P                                             */
/*                                                                     */
/*  NOTE:Initiarise micro controler.                                   */
/*                                                                     */
/***********************************************************************/
#include "iodefine.h"
#include "sci.h"
#include "spi.h"
#include "glob_var.h"
#include "DataFlash.h"
#include "i2c.h"

extern wait_ms(int wtime);

/*****************************************************************************************
�N���b�N�ݒ�
*****************************************************************************************/
void clock_init(void){

	SYSTEM.PRCR.WORD = 0xa50b;		//�N���b�N�\�[�X�I���̕ی�̉���

	SYSTEM.PLLCR.WORD = 0x0F00;		/* PLL ���{�~16 ����1���� (12.000MHz * 16 = 192MHz)*/
	SYSTEM.PLLCR2.BYTE = 0x00;		/* PLL ENABLE */
	
	SYSTEM.PLLWTCR.BYTE     = 0x0F;		/* 4194304cycle(Default) */
	
	
	// ICK   : 192/2 = 96MHz 		// �V�X�e���N���b�N CPU DMAC DTC ROM RAM
	// PCLKA : 192/2 = 96MHz 		// ���Ӄ��W���[���N���b�NA ETHERC�AEDMAC�ADEU
	// PCLKB : 192/4 = 48MHz 		// ���Ӄ��W���[���N���b�NB ��L�ȊO PCLKB=PCLK
/*	
	SYSTEM.SCKCR.BIT.FCK=0x02;		//FCLK MAX 50MHz  192/4
	SYSTEM.SCKCR.BIT.ICK=0x01;		//ICLK MAX 100MHz 192/2
	SYSTEM.SCKCR.BIT.PSTOP1=0x01;		//BCLK �o�͒�~
	SYSTEM.SCKCR.BIT.PSTOP0=0x01;		//SDCLK �o�͒�~
	SYSTEM.SCKCR.BIT.BCK=0x02;		//BCLK MAX 100MHz ICLK�ȉ��ɂ���K�v������192/4
	SYSTEM.SCKCR.BIT.PCKA=0x01;		//PCLKA MAX 100MHz 192/2
	SYSTEM.SCKCR.BIT.PCKB=0x02;		//PCLKB MAX 50MHz 192/4
	//��L�̐ݒ�ł͐�����clock�ݒ肪�ł��Ȃ����߉��L�̂悤�Ɉꊇ�Őݒ肷�邱��
*/
	SYSTEM.SCKCR.LONG = 0x21C21211;		//FCK1/4 ICK1/2 BCLK��~ SDCLK��~ BCK1/4 PCLKA1/2 PCLKB1/4
/*
	SYSTEM.SCKCR2.BIT.UCK=0x03;		//UCLK MAX 48MHz 192/4
	SYSTEM.SCKCR2.BIT.IEBCK=0x02;		//IECLK MAX 50MHz 192/4
*/
	SYSTEM.SCKCR2.WORD = 0x0032;		/* UCLK1/4 IEBCK1/4 */
	SYSTEM.BCKCR.BYTE = 0x01;		/* BCLK = 1/2 */
	
	SYSTEM.SCKCR3.WORD = 0x0400;		//PLL��H�I��
	
}

/*****************************************************************************************
CMT�̐ݒ�
		
*****************************************************************************************/
void init_cmt(void)
{
	SYSTEM.PRCR.WORD = 0xA502;
	MSTP(CMT0) = 0;
	MSTP(CMT1) = 0;
	MSTP(CMT2) = 0;
    	SYSTEM.PRCR.WORD = 0xA500;	
	
	//CMT0�͐��䊄�荞�ݗp�^�C�}�Ƃ��Ďg�p
	CMT0.CMCR.BIT.CKS=1;	// PCLK/32 1.5MHz
	CMT0.CMCR.BIT.CMIE=1;	//���荞�݂�����
	CMT0.CMCNT=0;		//�J�E���^�[�̃N���A
	CMT0.CMCOR=1500-1;	//1kHz

	IEN(CMT0,CMI0) = 1;	//���荞�ݗv�������� 
	IPR(CMT0,CMI0) = 15;	//���荞�ݗD��x 15���ō�
	IR(CMT0,CMI0)=0;	//���荞�݃X�e�[�^�t���O���N���A
	
	//CMT1�̓Z���T�[����p�^�C�}�Ƃ��Ďg�p
	CMT1.CMCR.BIT.CKS=1;	// PCLK/32 1.5MHz
	CMT1.CMCR.BIT.CMIE=1;	//���荞�݂�����
	CMT1.CMCNT=0;		//�J�E���^�[�̃N���A
	CMT1.CMCOR=(1500/4)-1;	//4kHz

	IEN(CMT1,CMI1) = 1;	//���荞�ݗv�������� 
	IPR(CMT1,CMI1) = 14;	//���荞�ݗD��x�����_�ɐݒ�
	IR(CMT1,CMI1)=0;	//���荞�݃X�e�[�^�t���O���N���A

	//CMT2�̓Z���T�[����p�^�C�}�Ƃ��Ďg�p
	CMT2.CMCR.BIT.CKS=1;	// PCLK/32 1.5MHz
	CMT2.CMCR.BIT.CMIE=1;	//���荞�݂�����
	CMT2.CMCNT=0;		//�J�E���^�[�̃N���A
	CMT2.CMCOR=(1500/2)-1;	//2kHz

	IEN(CMT2,CMI2) = 1;	//���荞�ݗv�������� 
	IPR(CMT2,CMI2) = 13;	//���荞�ݗD��x�����_�ɐݒ�
	IR(CMT2,CMI2)=0;	//���荞�݃X�e�[�^�t���O���N���A
	
	CMT.CMSTR0.BIT.STR0=1;	//�J�E���g�X�^�[�g
	CMT.CMSTR0.BIT.STR1=1;	//�J�E���g�X�^�[�g
	CMT.CMSTR1.BIT.STR2=1;	//�J�E���g�X�^�[�g
	
}

/*****************************************************************************************
I/O�ݒ�
	LED�̐ݒ�	
*****************************************************************************************/
void io_init(void){
	
	//�u�U�[�֘A
	PORTB.PDR.BIT.B5 = 1;
		
	
	//�ԊOLED�̃s���ݒ�
	PORTA.PDR.BIT.B3 = 1;	//PA3���o�͗p�ɐݒ�
	PORT1.PDR.BIT.B5 = 1;	//P15���o�͗p�ɐݒ�
	PORT1.PDR.BIT.B4 = 1;	//P14���o�͗p�ɐݒ�
	PORT3.PDR.BIT.B1 = 1;	//P31���o�͗p�ɐݒ�
}


/*****************************************************************************************
A/DC�̐ݒ�
	���Z���T�ƃo�b�e���d��
*****************************************************************************************/
void sensor_init(void){

	//A/D�ϊ��p�̃s���ݒ�
	SYSTEM.PRCR.WORD = 0xA502;
	MSTP_S12AD = 0;
	SYSTEM.PRCR.WORD = 0xA500;
	
	//A/D�|�[�g��PMR�ݒ�
	PORT4.PMR.BIT.B6=1;	//P46�����Ӌ@��Ƃ��Ďg�p
	PORT4.PMR.BIT.B2=1;	//P42�����Ӌ@��Ƃ��Ďg�p
	PORT4.PMR.BIT.B1=1;	//P41�����Ӌ@��Ƃ��Ďg�p
	PORT4.PMR.BIT.B0=1;	//P40�����Ӌ@��Ƃ��Ďg�p
	PORTE.PMR.BIT.B7=1;	//PE7�����Ӌ@��Ƃ��Ďg�p
	//A/D�|�[�g��PFS�ݒ�
	MPC.PWPR.BYTE=0x00;	//�v���e�N�g����
	MPC.PWPR.BYTE=0x40;	//�v���e�N�g����
	MPC.P46PFS.BIT.ASEL=1;	//A/D SEN_FR	AN006���g�p
	MPC.P42PFS.BIT.ASEL=1;	//A/D SEN_R 	AN002���g�p
	MPC.P41PFS.BIT.ASEL=1;	//A/D SEN_FR	AN001���g�p
	MPC.P40PFS.BIT.ASEL=1;	//A/D SEN_R 	AN000���g�p
	MPC.PWPR.BYTE=0x80;	//�v���e�N�g�쓮
	
	//A/D�ϊ�(�f�t�H���g�ŃV���O�����[�h)
	//S12AD.ADCSR.BYTE = 0x0c;	//A/D�ϊ��N���b�N��PCLKB(48M[ha])
	S12AD.ADCSR.BIT.CKS = 3;	//A/D�ϊ��̃N���b�N��PCLK��1����(48M[Hz])�ɐݒ�
	S12AD.ADANS0.WORD = 0x0047;	//A/D�ϊ���AN006�̂݋�����
	S12AD.ADCSR.BIT.ADCS = 0;	//�V���O���X�L�������[�h�ɐݒ�
}

/*****************************************************************************************
���[�^�̐ݒ�
	���E���[�^
*****************************************************************************************/
void motor_init(void){
	
	//���[�^�n�s���ݒ�
	//MOT_POWER
	PORTC.PDR.BIT.B6 = IO_OUT;//motor SLEEP (STBY)
	//MOT_CWCCW
	PORTC.PDR.BIT.B5 = IO_OUT;//Rmotor PH (���ۂ͍�)
	PORTB.PDR.BIT.B3 = IO_OUT;//Rmotor EN (���ۂ͍�)
	PORTC.PDR.BIT.B4 = IO_OUT;//Lmotor PH (���ۂ͉E)
	PORTB.PDR.BIT.B1 = IO_OUT;//Lmotor EN (���ۂ͉E)
	
	//�@�\�s���ݒ�	
	MPC.PWPR.BIT.B0WI=0;
	MPC.PWPR.BIT.PFSWE=1;
	MPC.PB1PFS.BIT.PSEL=1;	//PWM R MTIOC0C
	MPC.PB3PFS.BIT.PSEL=1;	//PWM L MTIOC0A
	MPC.PWPR.BYTE=0x80;
	
	//MTU�̃C�j�V�����C�Y
	SYSTEM.PRCR.WORD = 0xA502;
	MSTP(MTU) = 0;//MTU���W���[��ON
	SYSTEM.PRCR.WORD = 0xA500;	
	
	//�s����@�\�ݒ莞�ɂ̓^�C�}�X�g�b�v
	MTU.TSTR.BYTE=0;	//�^�C�}����X�g�b�v
	
	//���E���[�^�pMTU0 PWM2 ���萔��=L/R=17uH/(1.07+0.5+0.3)=110kHz
	MTU0.TCR.BIT.TPSC=0;	//PCLK/1 48MHz
	MTU0.TCR.BIT.CCLR=6;	//PWM TGRD�̃R���y�A�}�b�`��TCNT�N���A
	MTU0.TIORH.BIT.IOA=5;	//�����o��0�R���y�A�}�b�`0�o��
	MTU0.TIORL.BIT.IOC=5;	//�����o��0�R���y�A�}�b�`0�o��
	MTU0.TIORL.BIT.IOD=2;	//�����o��0�R���y�A�}�b�`1�o��
	MTU0.TGRA = 0;		//4�ȉ��͓��삵�Ȃ�
	MTU0.TGRC = 0;
	MTU0.TGRD = 240;	//���� 200kHz
	MTU0.TMDR.BIT.MD=3;	//PWM2
	
	PORTB.PMR.BIT.B3=1;	//�EPWM
	PORTB.PMR.BIT.B1=1;	//��PWM
	MTU0.TGRA = 0;		//��
	MTU0.TGRC = 0;		//�E
	MTU.TSTR.BIT.CST0 =1; 
	
	MOT_POWER_OFF;
	MOT_CWCCW_R  = 1;	//R_PH
	MOT_CWCCW_L  = 1;	//L_PH
}

/*****************************************************************************************
���Z���T�[�n�̃p�����[�^������
	���t�@�����X�Ƃ��ǂ�臒l�Ƃ�
*****************************************************************************************/
void init_parameters(void)
{
			
	sen_r.ref = REF_SEN_R;				//�E�Z���T�̃��t�@�����X�l��������
	sen_l.ref = REF_SEN_L;				//���Z���T�̃��t�@�����X�l��������
	
	sen_r.th_wall = TH_SEN_R;			//�E�Z���T�̕ǗL�����f��臒l��������
	sen_l.th_wall = TH_SEN_L;			//���Z���T�̕ǗL�����f��臒l��������
	
	sen_fr.th_wall = TH_SEN_FR;			//�E�O�Z���T�̕ǗL�����f��臒l��������
	sen_fl.th_wall = TH_SEN_FL;			//���O�Z���T�̕ǗL�����f��臒l��������
	
	sen_r.th_control = CONTH_SEN_R;			//�E�Z���T�̕ǐ��䂩���邩�ۂ���臒l��������
	sen_l.th_control = CONTH_SEN_L;			//���Z���T�̕ǐ��䂩���邩�ۂ���臒l��������
	
	con_wall.kp = CON_WALL_KP/10000.0;			//�ǔ�ᐧ��̔��萔��������
}

/*****************************************************************************************
���H���̏�����
*****************************************************************************************/
void init_maze(void)	//���H���̏�����
{
	int i,j;
	
	for(i = 0; i < MAZESIZE_X; i++)
	{
		for(j = 0; j < MAZESIZE_Y; j++)
		{
			wall[i][j].north = wall[i][j].east = wall[i][j].south = wall[i][j].west = UNKNOWN;	//���H�̑S�̂��킩��Ȃ�����ݒ肷��
		}
	}
	
	for(i = 0; i < MAZESIZE_X; i++)
	{
		wall[i][0].south = WALL;		//�l���̕ǂ�ǉ�����(��)
		wall[i][MAZESIZE_Y-1].north = WALL;	//�l���̕ǂ�ǉ�����(�k)
	}
	
	for(j = 0; j < MAZESIZE_Y; j++)
	{
		wall[0][j].west = WALL;			//�l���̕ǂ�ǉ�����(��)
		wall[MAZESIZE_X-1][j].east = WALL;	//�l���̕ǂ�ǉ�����(��)
	}
	
	wall[0][0].east = wall[1][0].west = WALL;	//�X�^�[�g�n�_�̉E�̕ǂ�ǉ�����
	
}


/*****************************************************************************************
�W���C���̃��t�@�����X�擾
*****************************************************************************************/
void gyro_get_ref(void){
	long i = 0;
	float gyro_ref_temp = 0;
	gyro_ref = 0;
	//�W���C���̃��t�@�����X�擾
	for(i = 0; i < 2500; i++){
		gyro_ref_temp += (float)gyro_x_new;
		wait_ms(1);
	}
	gyro_ref = (gyro_ref_temp/2500.0);
	degree = 0;
	wait_ms(100);
}

/*****************************************************************************************
�S�Ă̋@�\�̃C�j�V�����C�Y
	
*****************************************************************************************/
void init_all(void){
	int i;
	clock_init();
	io_init();
	sensor_init();
	motor_init();
	init_sci();
	init_spi_gyro();
	init_spi_enc();
	init_I2C();
	IOex_SETTING();
	void LED(short led_num);
	LED(0);
	MOT_POWER_OFF;
	init_parameters();
	init_maze();
	//Encoder�̏�����
	/*
	RSPI0.SPCMD0.BIT.SSLA = 	0x00;	//SSL�M���A�T�[�g�ݒ�(SSL0���g��)
	preprocess_spi_enc(0x7E40);
	for(i = 0; i < 100*1000*10; i++);
	preprocess_spi_enc(0x5040);
	for(i = 0; i < 100*1000*10; i++);
	RSPI0.SPCMD0.BIT.SSLA = 	0x02;	//SSL�M���A�T�[�g�ݒ�(SSL2���g��)	preprocess_spi_enc(0x7E40);
	for(i = 0; i < 100*1000*10; i++);
	preprocess_spi_enc(0x5040);
	for(i = 0; i < 100*1000*10; i++);
	*/
	//Gyro�����ݒ�
	preprocess_spi_gyro_2byte(0x0681);		//�W���C�����Z�b�g
	for(i = 0; i < 100*1000*10; i++);
	preprocess_spi_gyro_2byte(0x0601);		//Low Power Mode OFF
	for(i = 0; i < 100*1000*10; i++);

	//�W���C���̐ݒ�
	preprocess_spi_gyro_2byte(0x7F20);		//User Bank2�ɕύX
	for(i = 0; i < 100*1000*10; i++);
	preprocess_spi_gyro_2byte(0x0107);		//Range ���ő�2000dps�֕ύX
	for(i = 0; i < 100*1000*10; i++);
	preprocess_spi_gyro_2byte(0x7F00);		//User Bank0�ɕύX
	for(i = 0; i < 100*1000*10; i++);

	preprocess_spi_gyro_2byte(0x0621);		//�W���C���X�^�[�g
	//�ϐ�������
	timer = 0;
	
	//�R���y�A�}�b�`�^�C�}�J�n
	init_cmt();	

	//E2�t���b�V���̏�����
	hw_dflash_init();

	
}





