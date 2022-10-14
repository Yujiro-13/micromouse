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

#include <machine.h>
#include <stdarg.h>
#include "iodefine.h"
#include "sci.h"
#include "glob_var.h"

int gyro_data = 0;
int gyro_address = 0;
//unsigned int enc_data = 0;
int ovre = 0;
int read = 0;
int gyro_write_cnt = 0;

//For encoder
int enc_r_data = 0;
int enc_r_address = 0;
int enc_write_cnt = 0;


void init_spi_gyro(void)
{
	/*****************************************************************************************
	SPI�̐ݒ�
		�W���C���̐ݒ�	
	*****************************************************************************************/

	//RSPI1�̃v���e�N�g����
	SYSTEM.PRCR.WORD = 0xA502;
	MSTP_RSPI1 = 0;
	SYSTEM.PRCR.WORD = 0xA500;
	
	RSPI1.SSLP.BYTE	= 	0x0e;	//SSLB0�̂݃A�N�e�B�uLOW�@�c��̓A�N�e�B�uHIGH
	RSPI1.SPPCR.BYTE=	0x00;	//RSPI�ʏ탂�[�h�@MOSI�̏o�͒l�͑O��]���̍ŏI�f�[�^
	
	//�r�b�g���[�g�ݒ�
	//�r�b�g���[�g= PCLK/(2*(n+1)*2^N
	//PCLK = 48[MHz] n = 5, N = 2
	//�r�b�g���[�g = 48/(2*6*4) = 1[MHz]
	RSPI1.SPBR =		5;	//n = 5, N = 2(��قǐݒ�)
	
	//RSPI�f�[�^�R���g���[�����W�X�^�ݒ�
	RSPI1.SPDCR.BIT.SPFC =	 0x00;	//�g�p����t���[�����̐ݒ� ���̂����ň�̃t���[�����i�[
	RSPI1.SPDCR.BIT.SPRDTD = 0x00;	//SPDR�͎�M�o�b�t�@��ǂݏo��
	RSPI1.SPDCR.BIT.SPLW =	 0x01;	//SPDR���W�X�^�ւ̓����O�A�N�Z�X(SPLE = 1�̎��̓����O���[�h�A�N�Z�X)
	
	//RSPCK�x���l�̐ݒ�
	RSPI1.SPCR.BIT.MSTR = 	0x0;	//RSPCK��SSLND���W�X�^�̏������݃G���[���p�ɐݒ�
	RSPI1.SPCR.BIT.SPE = 	0x0;	//RSPCK��SSLND���W�X�^�̏������݃G���[���p�ɐݒ�
	RSPI1.SPCKD.BYTE= 	0x02;	//RSPI�̒x����3RSPCK�ɐݒ�
	
	//SSL�l�Q�[�g�x���l�̐ݒ�
	RSPI1.SSLND.BYTE= 	0x02;	//RSPI��SSL�l�Q�[�g�x����3RSPCK�ɐݒ�
	
	//���A�N�Z�X�x���l�̐ݒ�
	RSPI1.SPND.BYTE = 	0x00;	//���A�N�Z�X�x���� 1RSPCK + 2PCLK �ɐݒ�
	
	//�p���e�B�@�\�̐ݒ�
	//���荞�݃}�X�N�̐ݒ�
	RSPI1.SPCR2.BYTE = 	0x00;	//�p���e�B�@�\�𖳌��@�A�C�h�����荞�ݗv�����֎~
	
	//�V�[�P���X���̐ݒ�
	RSPI1.SPSCR.BYTE = 	0x00;	//�V�[�P���X��1�ɐݒ�
	
	//�R�}���h���W�X�^�̐ݒ�
	//�V�[�P���X���̒����������s��
	RSPI1.SPCMD0.BIT.CPHA = 	0x01;	//�����G�b�W�Ńf�[�^�T���v���A��G�b�W�Ńf�[�^�ω�
	RSPI1.SPCMD0.BIT.CPOL = 	0x01;	//�A�C�h������RSPCK��High
	RSPI1.SPCMD0.BIT.BRDV = 	0x03;	//�r�b�g���[�g�������x�[�X�̃r�b�g���[�g(�r�b�g���[�g�ݒ��N�̐���=8)��I��
	RSPI1.SPCMD0.BIT.SSLA = 	0x00;	//SSL�M���A�T�[�g�ݒ�(SSL0���g��)
	RSPI1.SPCMD0.BIT.SSLKP = 	0x00;	//�]���I���ォ�玟�A�N�Z�X�J�n�܂�SSL�M�����x����ێ�
	RSPI1.SPCMD0.BIT.SPB =	 	0x1;	//RSPI�f�[�^����24�r�b�g�ɐݒ�
	RSPI1.SPCMD0.BIT.LSBF = 	0x00;	//MSB�t�@�[�X�g
	RSPI1.SPCMD0.BIT.SPNDEN = 	0x00;	//RSPI���A�N�Z�X�x�����r�b�g
	RSPI1.SPCMD0.BIT.SLNDEN = 	0x01;	//SSL�l�Q�[�g�̒x����SSLND�̐ݒ�l
	RSPI1.SPCMD0.BIT.SCKDEN = 	0x01;	//RSPCK�̒x����SPCKD�̐ݒ�l
	
	//���荞�ݗD�惌�x���ݒ�
	ICU.IPR[42].BYTE = 0x0F;
	ICU.IPR[43].BYTE = 0x0F;
	ICU.IPR[44].BYTE = 0x0F;
	
	//���荞�݂̋���
	ICU.IER[5].BIT.IEN2 = 1;	//SPRI1�̊��荞�݋���
	ICU.IER[5].BIT.IEN3 = 1;	//SPTI1�̊��荞�݋���
	ICU.IER[5].BIT.IEN4 = 1;	//SPII1�̊��荞�݋���
	
	//I/O�|�[�g�̐ݒ�
	PORT2.PMR.BIT.B7 =	0x01;	//P27�����Ӌ@�\�Ƃ��Ďg�p
	PORTE.PMR.BIT.B2 =	0x01;	//PE2�����Ӌ@�\�Ƃ��Ďg�p
	PORTE.PMR.BIT.B3 =	0x01;	//PE3�����Ӌ@�\�Ƃ��Ďg�p
	PORTE.PMR.BIT.B4 =	0x01;	//PE4�����Ӌ@�\�Ƃ��Ďg�p

	
	//���Ӌ@�\�̐ݒ�
	MPC.PWPR.BYTE=0x00;		//�v���e�N�g����
	MPC.PWPR.BYTE=0x40;		//�v���e�N�g����
	
	MPC.P27PFS.BIT.PSEL=	0x0D;	//P27��RSPCKB�Ƃ��ė��p
	MPC.PE2PFS.BIT.PSEL=	0x0E;	//PE2��MOSIB�Ƃ��ė��p
	MPC.PE3PFS.BIT.PSEL=	0x0D;	//PE3��MISOB�Ƃ��ė��p
	MPC.PE4PFS.BIT.PSEL=	0x0D;	//PE4��SSLBB�Ƃ��ė��p
	
	MPC.PWPR.BYTE=0x80;	//�v���e�N�g�쓮
	
	//RSPI���䃌�W�X�^�̐ݒ�
	RSPI1.SPCR.BIT.SPMS = 		0x00;	//SPI���쓮��
	RSPI1.SPCR.BIT.TXMD = 		0x00;	//�S��d�������V���A���ʐM
	RSPI1.SPCR.BIT.MODFEN =		0x00;	//���[�h�t�H���g�G���[���o���֎~
	RSPI1.SPCR.BIT.MSTR =		0x01;	//�}�X�^�[���[�h
	RSPI1.SPCR.BIT.SPE =		0x00;	//RSPI�@�\������
	RSPI1.SPCR.BIT.SPEIE =		0x00;	//RSPI�G���[���荞�ݗv���̔������֎~
	RSPI1.SPCR.BIT.SPTIE =		0x00;	//RSPI���M���荞�ݗv���̔������֎~
	RSPI1.SPCR.BIT.SPRIE =		0x00;	//RSPI��M���荞�ݗv���̔������֎~
	//SCI_printf("init_spi\n\r");
}



void preprocess_spi_gyro(int address)
{
	/*****************************************************************************************
	SPI�]���O����
		�W���C���p
	*****************************************************************************************/
	//SCI_printf("preprocess start\n\r");
	
	long dummy = 0;
	dummy = RSPI1.SPDR.LONG;
	
	RSPI1.SPCR.BIT.SPE = 0;		//RSPI�@�\������
	RSPI1.SPCMD0.BIT.SPB =	 	0x1;	//RSPI�f�[�^����24�r�b�g�ɐݒ�
	
	//�G���[�v���̃N���A
	RSPI1.SPSR.BIT.MODF = 0;
	RSPI1.SPSR.BIT.OVRF = 0;
	RSPI1.SPSR.BIT.PERF = 0;
	
	//SPII���荞�݂��֎~
	RSPI1.SPCR2.BIT.SPIIE = 0;
	
	gyro_write_cnt = 0;
	//SPE�r�b�g�̋��@�����ɕK�v�Ȋ��荞�݂�����
	///*
	RSPI1.SPCR.BIT.SPTIE = 1;	//RSPI���M���荞�ݗv���̔���������
	RSPI1.SPCR.BIT.SPRIE = 1;	//RSPI��M���荞�ݗv���̔���������
	RSPI1.SPCR.BIT.SPEIE = 1;	//RSPI�G���[���荞�ݗv���̔���������
	RSPI1.SPCR.BIT.SPE = 1;		//RSPI�@�\���L��
	//*/
	//RSPI1.SPCR.BYTE |= 0xF0;
	gyro_address = address;
	
}

void preprocess_spi_gyro_2byte(int address)
{
	/*****************************************************************************************
	SPI�]���O����
		�W���C���p
	*****************************************************************************************/
	//SCI_printf("preprocess start\n\r");
	long dummy = 0;
	dummy = RSPI1.SPDR.LONG;
	
	RSPI1.SPCR.BIT.SPE = 0;		//RSPI�@�\���L��
	RSPI1.SPCMD0.BIT.SPB =	 	0xF;	//RSPI�f�[�^����16�r�b�g�ɐݒ�
	//�G���[�v���̃N���A
	RSPI1.SPSR.BIT.MODF = 0;
	RSPI1.SPSR.BIT.OVRF = 0;
	RSPI1.SPSR.BIT.PERF = 0;
	
	//SPII���荞�݂��֎~
	RSPI1.SPCR2.BIT.SPIIE = 0;
	
	gyro_write_cnt = 0;
	//SPE�r�b�g�̋��@�����ɕK�v�Ȋ��荞�݂�����
	///*
	RSPI1.SPCR.BIT.SPTIE = 1;	//RSPI���M���荞�ݗv���̔���������
	RSPI1.SPCR.BIT.SPRIE = 1;	//RSPI��M���荞�ݗv���̔���������
	RSPI1.SPCR.BIT.SPEIE = 1;	//RSPI�G���[���荞�ݗv���̔���������
	RSPI1.SPCR.BIT.SPE = 1;		//RSPI�@�\���L��
	//*/
	//RSPI1.SPCR.BYTE |= 0xF0;
	gyro_address = address;
	
}

void write_spdr_gyro(void)
{
	/*****************************************************************************************
	���M�p
	SPDR�ɑ��M�f�[�^���������ފ֐�
	�����who am I �̃A�h���X�ɃA�N�Z�X
	*****************************************************************************************/
	if(gyro_write_cnt == 0){
		RSPI1.SPDR.LONG = gyro_address;	//�A�h���X���M 
		RSPI1.SPCR.BIT.SPTIE = 0;		//RSPI���M���荞�ݗv���̔������֎~
		RSPI1.SPCR2.BIT.SPIIE = 1;		//RSPI�A�C�h�����荞�ݗv���̔���������
	}
	gyro_write_cnt++;
}

void spii_int_gyro(void)
{
	/*****************************************************************************************
	���M�p
	SPII���荞��
	*****************************************************************************************/
	RSPI1.SPCR.BIT.SPE = 0;		//RSPI�@�\�͖���
	RSPI1.SPCR2.BIT.SPIIE = 0;	//RSPI�A�C�h�����荞�ݗv���̔������֎~
	
}

void read_spdr_gyro(void)
{
	/*****************************************************************************************
	��M�p
	SPDR�����M�f�[�^��ǂݍ��ފ֐�
	*****************************************************************************************/
	read++;
	gyro_data = RSPI1.SPDR.LONG;	//�f�[�^�̓ǂݍ���
	//gyro_data = 0xffff;
	RSPI1.SPCR.BIT.SPRIE = 0;	//RSPI��M���荞�ݗv���̔������֎~
	RSPI1.SPCR.BIT.SPEIE = 0;	//RSPI�G���[���荞�ݗv���̔������֎~
	
}


int ovre_check(void)
{
	return ovre;	
}

int gyro_read_check(void)
{
	return read;	
}

int gyro_write_check(void)
{
	return gyro_write_cnt;	
}

long read_gyro_data(void)
{
	return gyro_data;
}

/*****************************************************************************************
SPI�]������
	�G���R�[�_�p
*****************************************************************************************/
void init_spi_enc(void)
{
	/*****************************************************************************************
	SPI�̐ݒ�
		�G���R�[�_�̐ݒ�	
	*****************************************************************************************/
	//RSPI1�̃v���e�N�g����
	SYSTEM.PRCR.WORD = 0xA502;
	MSTP_RSPI0 = 0;
	SYSTEM.PRCR.WORD = 0xA500;
	
	RSPI0.SSLP.BYTE	= 	0x00;	//SSLA0,1,2,3 �S�ăA�N�e�B�uLow
	RSPI0.SPPCR.BYTE=	0x00;	//RSPI�ʏ탂�[�h�@MOSI�̏o�͒l�͑O��]���̍ŏI�f�[�^
	
	//�r�b�g���[�g�ݒ�
	//�r�b�g���[�g= PCLK/(2*(n+1)*2^N
	//PCLK = 48[MHz] n = 5, N = 2
	//�r�b�g���[�g = 48/(2*6*4) = 1[MHz]
	RSPI0.SPBR =		5;	//n = 5, N = 2(��قǐݒ�)
	
	//RSPI�f�[�^�R���g���[�����W�X�^�ݒ�
	RSPI0.SPDCR.BIT.SPFC =	 0x00;	//�g�p����t���[�����̐ݒ� ���̂����ň�̃t���[�����i�[
	RSPI0.SPDCR.BIT.SPRDTD = 0x00;	//SPDR�͎�M�o�b�t�@��ǂݏo��
	RSPI0.SPDCR.BIT.SPLW =	 0x00;	//SPDR���W�X�^�ւ̓��[�h�A�N�Z�X(SPLE = 1�̎��̓����O���[�h�A�N�Z�X)
	
	//RSPCK�x���l�̐ݒ�
	RSPI0.SPCR.BIT.MSTR = 	0x0;	//SPCKD��SSLND���W�X�^�̏������݃G���[���p�ɐݒ�
	RSPI0.SPCR.BIT.SPE = 	0x0;	//SPCKD��SSLND���W�X�^�̏������݃G���[���p�ɐݒ�
	RSPI0.SPCKD.BYTE= 	0x02;	//RSPI�̒x����3RSPCK�ɐݒ�
	
	//SSL�l�Q�[�g�x���l�̐ݒ�
	RSPI0.SSLND.BYTE= 	0x02;	//RSPI��SSL�l�Q�[�g�x����3RSPCK�ɐݒ�
	
	//���A�N�Z�X�x���l�̐ݒ�
	RSPI0.SPND.BYTE = 	0x02;	//���A�N�Z�X�x���� 3RSPCK + 2PCLK �ɐݒ�
	
	//�p���e�B�@�\�̐ݒ�
	//���荞�݃}�X�N�̐ݒ�
	RSPI0.SPCR2.BYTE = 	0x01;	//�p���e�B�@�\��L���@�����p���e�B�@�A�C�h�����荞�ݗv�����֎~
	
	//�V�[�P���X���̐ݒ�
	RSPI0.SPSCR.BYTE = 	0x00;	//�V�[�P���X��1�ɐݒ�
	
	//�R�}���h���W�X�^�̐ݒ�
	//�V�[�P���X���̒����������s��
	RSPI0.SPCMD0.BIT.CPHA = 	0x01;	//�����G�b�W�Ńf�[�^�T���v���A��G�b�W�Ńf�[�^�ω�
	RSPI0.SPCMD0.BIT.CPOL = 	0x01;	//�A�C�h������RSPCK��High
	RSPI0.SPCMD0.BIT.BRDV = 	0x03;	//�r�b�g���[�g�������x�[�X�̃r�b�g���[�g(�r�b�g���[�g�ݒ��N�̐���=8)��I��
	RSPI0.SPCMD0.BIT.SSLA = 	0x02;	//SSL�M���A�T�[�g�ݒ�(SSL0���g��)(2 SSL2)
	RSPI0.SPCMD0.BIT.SSLKP = 	0x00;	//�]���I���ォ�玟�A�N�Z�X�J�n�܂�SSL�M�����x�����l�Q�[�g
	RSPI0.SPCMD0.BIT.SPB =	 	0x9;	//RSPI�f�[�^����10�r�b�g�ɐݒ�(9~16�܂Őݒ�\���̐ݒ肪�G���R�[�_�̕���\�ƂȂ�)
	RSPI0.SPCMD0.BIT.LSBF = 	0x00;	//MSB�t�@�[�X�g
	RSPI0.SPCMD0.BIT.SPNDEN = 	0x01;	//RSPI���A�N�Z�X�x�����r�b�g
	RSPI0.SPCMD0.BIT.SLNDEN = 	0x01;	//SSL�l�Q�[�g�̒x����SSLND�̐ݒ�l
	RSPI0.SPCMD0.BIT.SCKDEN = 	0x01;	//RSPCK�̒x����SPCKD�̐ݒ�l
	
	//���荞�݂̋֎~(IPR�̃G���[����̂���)
	ICU.IER[4].BIT.IEN7 = 0;	//SPRI0�̊��荞�݋֎~
	ICU.IER[5].BIT.IEN0 = 0;	//SPTI0�̊��荞�݋֎~
	ICU.IER[5].BIT.IEN1 = 0;	//SPII0�̊��荞�݋֎~
	
	///*
	//���荞�ݗD�惌�x���ݒ�
	ICU.IPR[39].BYTE = 0x0F;	//SPRI0
	//ICU.IPR[40].BYTE = 0x0F;	//SPTI0
	//ICU.IPR[41].BYTE = 0x0F;	//SPII0
	
	//���荞�݂̋���
	ICU.IER[4].BIT.IEN7 = 1;	//SPRI0�̊��荞�݋���
	ICU.IER[5].BIT.IEN0 = 1;	//SPTI0�̊��荞�݋���
	ICU.IER[5].BIT.IEN1 = 1;	//SPII0�̊��荞�݋���
	//*/
	
	//I/O�|�[�g�̐ݒ�
	//PORT�؊���
	PORT.PSRB.BIT.PSEL0 =	0;	//PB0��L��
	PORTA.PMR.BIT.B1 =	0x01;	//PA1�����Ӌ@�\�Ƃ��Ďg�p
	PORTA.PMR.BIT.B4 =	0x01;	//PA4�����Ӌ@�\�Ƃ��Ďg�p
	PORTA.PMR.BIT.B6 =	0x01;	//PA6�����Ӌ@�\�Ƃ��Ďg�p
	PORTB.PMR.BIT.B0 =	0x01;	//PB0�����Ӌ@�\�Ƃ��Ďg�p
	PORTC.PMR.BIT.B7 =	0x01;	//PC7�����Ӌ@�\�Ƃ��Ďg�p
	//PORTC.PMR.BIT.B5 = 	0x01;	//test
	
	//���Ӌ@�\�̐ݒ�
	MPC.PWPR.BYTE=0x00;		//�v���e�N�g����
	MPC.PWPR.BYTE=0x40;		//�v���e�N�g����
	
	MPC.PA1PFS.BIT.PSEL=	0x0D;	//PA1��SSLA2�Ƃ��ė��p
	MPC.PA4PFS.BIT.PSEL=	0x0D;	//PA4��SSLA0�Ƃ��ė��p
	MPC.PA6PFS.BIT.PSEL=	0x0D;	//PA6��MOSIA�Ƃ��ė��p
	MPC.PB0PFS.BIT.PSEL=	0x0D;	//PB0��RSPCKA�Ƃ��ė��p
	MPC.PC7PFS.BIT.PSEL=	0x0D;	//PC7��MISOA�Ƃ��ė��p
	//MPC.PC5PFS.BIT.PSEL=	0x0D;	//test
	
	MPC.PWPR.BYTE=0x80;	//�v���e�N�g�쓮
	
	//RSPI���䃌�W�X�^�̐ݒ�
	RSPI0.SPCR.BIT.SPMS = 		0x00;	//SPI����(4����)
	RSPI0.SPCR.BIT.TXMD = 		0x00;	//�S��d�������V���A���ʐM
	RSPI0.SPCR.BIT.MODFEN =		0x00;	//���[�h�t�H���g�G���[���o���֎~
	RSPI0.SPCR.BIT.MSTR =		0x01;	//�}�X�^�[���[�h
	RSPI0.SPCR.BIT.SPE =		0x00;	//RSPI�@�\������
	RSPI0.SPCR.BIT.SPEIE =		0x00;	//RSPI�G���[���荞�ݗv���̔������֎~
	RSPI0.SPCR.BIT.SPTIE =		0x00;	//RSPI���M���荞�ݗv���̔������֎~
	RSPI0.SPCR.BIT.SPRIE =		0x00;	//RSPI��M���荞�ݗv���̔������֎~
	//SCI_printf("init_spi_enc\n\r");
}


void preprocess_spi_enc(int address)
{
	/*****************************************************************************************
	SPI�]���O����
		�G���R�[�_�p
	*****************************************************************************************/
	int dummy = 0;
	
	enc_r_address = address;
	enc_write_cnt = 0;
	
	//�G���[�v���̃N���A
	RSPI0.SPSR.BIT.MODF = 0;
	RSPI0.SPSR.BIT.OVRF = 0;
	RSPI0.SPSR.BIT.PERF = 0;
	
	//SPII���荞�݂��֎~
	RSPI0.SPCR2.BIT.SPIIE = 0;
	
	//dummy read
	dummy = RSPI0.SPDR.WORD.H;
	
	//SPE�r�b�g�̋��@�����ɕK�v�Ȋ��荞�݂�����
	RSPI0.SPCR.BIT.SPTIE = 1;	//RSPI���M���荞�ݗv���̔���������
	RSPI0.SPCR.BIT.SPRIE = 1;	//RSPI��M���荞�ݗv���̔���������
	RSPI0.SPCR.BIT.SPEIE = 1;	//RSPI�G���[���荞�ݗv���̔���������
	RSPI0.SPCR.BIT.SPE = 1;		//RSPI�@�\���L��	
}

void write_spdr_enc(void)
{
	/*****************************************************************************************
	���M�p
	SPDR�ɑ��M�f�[�^���������ފ֐�
	�����who am I �̃A�h���X�ɃA�N�Z�X
	*****************************************************************************************/
	if(enc_write_cnt == 0){
		RSPI0.SPDR.WORD.H = 0x0000;		//�A�h���X���M 
		RSPI0.SPCR.BIT.SPTIE = 0;		//RSPI���M���荞�ݗv���̔������֎~
		RSPI0.SPCR2.BIT.SPIIE = 1;		//RSPI�A�C�h�����荞�ݗv���̔���������
		//SCI_printf("write_spdr\n\r");
	}
	enc_write_cnt++;
}

void spii_int_enc(void)
{
	/*****************************************************************************************
	���M�p
	SPII���荞��
	*****************************************************************************************/
	RSPI0.SPCR.BIT.SPE = 0;		//RSPI�@�\�͖���
	RSPI0.SPCR2.BIT.SPIIE = 0;	//RSPI�A�C�h�����荞�ݗv���̔������֎~
	//SCI_printf("send_spdrdata\n\r");
}

void read_spdr_enc(void)
{
	/*****************************************************************************************
	��M�p
	SPDR�����M�f�[�^��ǂݍ��ފ֐�
	*****************************************************************************************/
	enc_r_data = RSPI0.SPDR.WORD.H;	//�f�[�^�̓ǂݍ���
	RSPI0.SPCR.BIT.SPRIE = 0;	//RSPI��M���荞�ݗv���̔������֎~
	//SCI_printf("read_spdr\n\r");
}

int enc_write_cnt_check(void)
{
	return enc_write_cnt;	
}

int Get_enc_data(void)
{
	return enc_r_data;	
}