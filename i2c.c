/***********************************************
RX631 I2C accel
          Gyro
	  touchSensor(and LED)
MPU6050
I2C Address 110_1000(0x68)
Gyro_Z register 0x47 0x48  read
Accel_X register 0x3B 0x3C read
Accel_Y register 0x3D 0x3E read

PIC1822
I2C Address 000_1000(0x08)
TouchSensor register 0x02 0x03 16bit read Timer Counter
TouchSensor register 0x04       8bit read Touch on/off
LED         register 0x06       8bit read/write LED0,LED1
TouchSensor register 0x08,0x09 16bit read/write Timer Counter Threshold


Start     Address R/W ACK  DATA ACK DATA ACK STOP
Condition 1-7     8    9   1-8  9  1-8   9   Condition

Write Sequence
Master S AD+W     RA      DATA     DATA     P
Slave         ACK    ACK       ACK      ACK

Read Sequence
Master S AD+W     RA     S AD+R          ACK       NACK P
Slave         ACK    ACK        ACK DATA      DATA

S:  StartCondition
AD: Address
ACK:Acknowledge
RA: Register Address
P:  StopCondition
NACK:not ackowlege

R/W
0:Write
1:Read

	  
************************************************/
#include "iodefine.h"
#include "sci.h"

struct str_IIC_data
{
	union {
		unsigned short WORD;
		struct {
			unsigned char GYRO_Z_H;
			unsigned char GYRO_Z_L;
		} BYTE;
	} GYRO_Z;
	union {
		unsigned short WORD;
		struct {
			unsigned char ACC_X_H;
			unsigned char ACC_X_L;
		} BYTE;
	} ACC_X;
	union {
		unsigned short WORD;
		struct {
			unsigned char ACC_Y_H;
			unsigned char ACC_Y_L;
		} BYTE;
	} ACC_Y;
	union {
		unsigned short WORD;
		struct {
			unsigned char TCNT_H;
			unsigned char TCNT_L;
		} BYTE;
	} TCNT;
	unsigned char Touch_Data;
	unsigned char LED_Data;
	union {
		unsigned short WORD;
		struct {
			unsigned char TCNT_TH_H;
			unsigned char TCNT_TH_L;
		} BYTE;
	} TCNT_TH;
}iic_data;

struct str_IIC
{
	unsigned char	SlvAdr;		/* Slave Address, Don't set bit0. It's a Read/Write bit */
	unsigned char   RW;		/* read:1 write:0 */
	unsigned char	RA;		/*Register Address*/
//	unsigned char	PreCnt;		/* Number of Predata */
//	unsigned char	read_buf[10];	/* read data buffer */
	unsigned char	RWCnt;		/* Number of Data */
	unsigned char	write_buf[10];	/* write data buffer */
	 unsigned char	trm_cnt;	/*transmission counter*/
	unsigned char   rcv_cnt;	/*reception counter*/
}iic_buff;

//char iic_trm_cnt;
//char iic_rcv_cnt;

//struct str_IIC iic_buf;
//struct str_IIC *sp;
/*
typedef struct str_IIC_API_T IIC_API_T;
extern IIC_API_T iic_buff;
extern unsigned char I2C_BUF[2];
extern unsigned char GYRO_ACCE_F;
*/

void I2C_START(void);
void I2C_RESTART(void);
void I2C_STOP(void);
void I2C_PUT(unsigned char aaa);
void GYRO_SETTING(void);

void init_I2C(void)
{
		
	//I2C�̃C�j�V�����C�Y
	//r_iic_drv_io_open
	PORT1.PCR.BIT.B6=0;//���̓v���A�b�v����
	PORT1.PDR.BIT.B6=0;//Input
	PORT1.PCR.BIT.B7=0;//���̓v���A�b�v����
	PORT1.PDR.BIT.B7=0;//Input

	//r_iic_drv_mpc_disable	
	PORT1.PMR.BIT.B6= 0 ;//SCL�@�ėp�|�[�g
	PORT1.PMR.BIT.B7= 0 ;//SDA�@�ėp�|�[�g
    	MPC.PWPR.BIT.B0WI  = 0;
    	MPC.PWPR.BIT.PFSWE = 1;
    	MPC.P16PFS.BIT.PSEL = 0x0;//�ėp�|�[�g
    	MPC.P17PFS.BIT.PSEL = 0x0;//�ėp�|�[�g
    	MPC.PWPR.BIT.PFSWE = 0x80;
	
	//r_iic_drv_cancel_mstp
	SYSTEM.PRCR.WORD = 0xA502;
	MSTP(RIIC2) = 0;
	SYSTEM.PRCR.WORD = 0xA500;
	
	//r_iic_drv_iic_disable
	RIIC2.ICCR1.BIT.ICE =0;		//SCLn SDAn�[�q��쓮���
	RIIC2.ICCR1.BIT.IICRST=1;	//IICRST ����I2C���Z�b�g��� IICRST=1 & ICE=0��RIIC�S���W�X�^���Z�b�g
	
	//�p���[�I�����̖\���΍�Ŋ��荞�݋֎~�ɐݒ�
	//r_iic_drv_int_disable
	IEN(RIIC2,EEI2) = 0;
	IEN(RIIC2,RXI2) = 0;
	IEN(RIIC2,TXI2) = 0;
	IEN(RIIC2,TEI2) = 0;
	IPR (RIIC2,EEI2) =0x02;//���荞�ݏ���
	IPR (RIIC2,RXI2) =0x02;
	IPR (RIIC2,TXI2) =0x02;
	IPR (RIIC2,TEI2) =0x02;
	
	//r_iic_drv_int_icier_setting
	RIIC2.ICIER.BYTE = 0x00;//���荞�݋֎~

	RIIC2.ICCR1.BIT.IICRST = 0;//�����Z�b�g����
	
	RIIC2.ICSER.BYTE = 0x00;//SARL0,SARL1����
	
	RIIC2.ICMR1.BYTE = 0x28;//bitcounter=9 PCLK/4=10MHz 0.08us 		12M -> 28, 10M -> 38
	RIIC2.ICBRL.BYTE = 0xF0;//�]�����x=1/{[ICBRH+1)+(ICBRL+1)]/IIC��}	12M -> F0, 10M -> FD
	RIIC2.ICBRH.BYTE = 0xE7;//IIC��=PCLKx10^6�~������@Tr=0�̏ꍇ��400kHz	12M -> E7, 10M -> F8
	
	RIIC2.ICMR2.BYTE = 0x00;
	RIIC2.ICMR3.BYTE = 0x00;
	RIIC2.ICMR3.BIT.ACKWP =0x01;//ACKBT(�A�N�m���b�W�̏������݋���
	
	IPR (RIIC2,EEI2) =0x02;//���荞�ݏ���
	IPR (RIIC2,RXI2) =0x02;
	IPR (RIIC2,TXI2) =0x02;
	IPR (RIIC2,TEI2) =0x02;	

	RIIC2.ICFER.BYTE =0x72;
	
    	MPC.PWPR.BIT.B0WI  = 0;
    	MPC.PWPR.BIT.PFSWE = 1;
    	MPC.P16PFS.BIT.PSEL = 0x0F;//SCL
    	MPC.P17PFS.BIT.PSEL = 0x0F;//SDA
    	MPC.PWPR.BIT.PFSWE = 0x80;
	
	PORT1.PMR.BIT.B6= 1 ;//SCL
	PORT1.PMR.BIT.B7= 1 ;//SDA

	
	IR(RIIC2,RXI2) =0x00;//clear
	IR(RIIC2,TXI2) =0x00;//clear
//	RIIC2.ICIER.BYTE =0xfc;//���M�f�[�^�G���v�e�B�A���M�I�����荞�� ��M�f�[�^�t�����荞�݁ANACK��M���荞�݁A�X�g�b�v�R���f�B�V�������o���荞�݁A�X�^�[�g�R���f�B�V�������o���荞��

	RIIC2.ICCR1.BYTE |= 0x80;//ICE ����I2C�]������\
	
}

unsigned char IOex_SWITCH(void){
	unsigned char sw;
	I2C_START();
	I2C_PUT((0x18)<<1);//�A�h���X write
	I2C_PUT(0x00);//�R���g���[�����W�X�^
	I2C_RESTART();
	I2C_PUT((0x18<<1)|0x01);//�A�h���X read

	while(RIIC2.ICSR2.BIT.RDRF==0);
	if(RIIC2.ICSR2.BIT.NACKF==1){	RIIC2.ICSR2.BIT.STOP=0;}
	
	RIIC2.ICMR3.BIT.WAIT=1;
	RIIC2.ICMR3.BIT.ACKBT=1;
	RIIC2.ICDRR;//dummy
	
	while(RIIC2.ICSR2.BIT.RDRF==0);
	sw=RIIC2.ICDRR;
	
	while(RIIC2.ICSR2.BIT.RDRF==0);
	RIIC2.ICSR2.BIT.STOP=0;
	RIIC2.ICCR2.BIT.SP=1;
	RIIC2.ICDRR;//dummy
	RIIC2.ICMR3.BIT.WAIT=0;
	
	while(RIIC2.ICSR2.BIT.STOP==0);
	RIIC2.ICSR2.BIT.NACKF=0;
	RIIC2.ICSR2.BIT.STOP=0;
	
	return sw;
	//SCI_printf("who=%x\n\r",who);
	
}

void IOex_LED(short led_num){
//�|�[�����O�ɂ��ݒ�
//*
	I2C_START();//Startbit����
	I2C_PUT((0x18<<1));//�A�h���X write
	I2C_PUT(0x01);//PWR_MGMT_2
	I2C_PUT(0x00 | led_num);//sleep����
	I2C_STOP();
}

void IOex_SETTING(void){
//�|�[�����O�ɂ��ݒ�
//*
	I2C_START();//Startbit����
	I2C_PUT((0x18<<1));//�A�h���X write
	I2C_PUT(0x03);//PWR_MGMT_2
	I2C_PUT(0xF0);//sleep����
	I2C_STOP();
}

//���荞�݂ɂ��f�[�^���M
void IIC_PIC_LED_WRITE(char data){
	if(RIIC0.ICCR2.BIT.BBSY !=0){//bus���r�W�[�Ȃ��M���Ȃ�
		return;
	}
	iic_buff.SlvAdr=0x08;	//PIC�̃X���[�u�A�h���X
	iic_buff.RW=0;		//0:write,1:read
	iic_buff.RA=0x06;	//���W�X�^�̃A�h���X
	iic_buff.RWCnt=1;	//�f�[�^��
	iic_buff.write_buf[0] = data;
	iic_buff.trm_cnt=0;	//���M�f�[�^�J�E���^
	iic_buff.rcv_cnt=0;	//��M�f�[�^�J�E���^
	
	RIIC2.ICIER.BIT.TIE  = 0;
	
	RIIC2.ICIER.BIT.STIE=1;
	RIIC2.ICSR2.BIT.START = 0;
	RIIC2.ICCR2.BIT.ST = 1;//���荞�݃t���O:START ���荞�ݗv��:EEI
	
	
}	

//���荞�݂ɂ��f�[�^��M
void IIC_PIC_TOUCH_READ(char data){
	if(RIIC0.ICCR2.BIT.BBSY !=0){//bus���r�W�[�Ȃ��M���Ȃ�
		return;
	}
	iic_buff.SlvAdr=0x08;	//PIC�̃X���[�u�A�h���X
	iic_buff.RW=1;		//0:write,1:read
	if(data==0){//on/off�f�[�^�̊m�F
		iic_buff.RA=0x04;	//���W�X�^�̃A�h���X
		iic_buff.RWCnt=1;	//�f�[�^��
	}else if(data==1){
		iic_buff.RA=0x02;	//���W�X�^�̃A�h���X
		iic_buff.RWCnt=1;	//�f�[�^��
	}else{
		iic_buff.RA=0x03;	//���W�X�^�̃A�h���X
		iic_buff.RWCnt=1;	//�f�[�^��
	}
	iic_buff.trm_cnt=0;	//���M�f�[�^�J�E���^
	iic_buff.rcv_cnt=0;	//��M�f�[�^�J�E���^
	
	RIIC2.ICIER.BIT.TIE  = 0;
	
	RIIC2.ICIER.BIT.STIE=1;
	RIIC2.ICSR2.BIT.START = 0;
	RIIC2.ICCR2.BIT.ST = 1;//���荞�݃t���O:START ���荞�ݗv��:EEI
	
	
}	
	

void I2C_START(void){
	while(RIIC2.ICCR2.BIT.BBSY==1){	
	}//1:�o�X�r�W�[ 0:�o�X�t���[�@�p�X���t���[�ɂȂ�܂ő҂�
	RIIC2.ICSR2.BIT.START = 0;//START�R���f�B�V���������o
	RIIC2.ICCR2.BIT.ST    = 1;//�X�^�[�g�R���f�B�V�������s
	
}

void I2C_RESTART(void){
	while(RIIC2.ICSR2.BIT.TDRE == 0);
	while(RIIC2.ICSR2.BIT.TEND==0);
	if(RIIC2.ICSR2.BIT.TDRE == 0);
	if(RIIC2.ICCR2.BIT.BBSY==1);
	RIIC2.ICSR2.BIT.START = 0;//START�R���f�B�V���������o
	RIIC2.ICCR2.BIT.RS = 1;//���X�^�[�g�R���f�B�V�����̔��s��v������
	while(RIIC2.ICCR2.BIT.RS);

}


void I2C_STOP(void){
	while(RIIC2.ICSR2.BIT.TEND==0);
	
	RIIC2.ICSR2.BIT.STOP=0;
	RIIC2.ICCR2.BIT.SP=1;//�X�g�b�v�R���f�B�V�����̔��s

	while(RIIC2.ICSR2.BIT.STOP==0);

	RIIC2.ICSR2.BIT.NACKF=0;
	RIIC2.ICSR2.BIT.STOP=0;
}

void I2C_PUT(unsigned char aaa){
	if(RIIC2.ICSR2.BIT.NACKF==1){	RIIC2.ICSR2.BIT.STOP=0;}//NACK���o�Ȃ�stop�R���f�B�V�����𖢌��o�ɂ���
	while(RIIC2.ICSR2.BIT.TDRE==0);//���M���W�X�^����check
	RIIC2.ICDRT = aaa;
}

//===================================���荞��===================================
//�ʐM�G���[/�C�x���g�����ɂ�銄�荞��
//AL    AL=1 & ALIE=1
//NACKF NACKF=1 & NAKIE=1
//TMOF  TMOF=1 & TMOIE=1
//START START=1 & STIE=1
//STOP  STOP=1 & SPIE=1
void int_i2c_ee(void){
	/* Check stop condition detection */
	if((RIIC2.ICSR2.BIT.STOP!=0) && (RIIC2.ICIER.BIT.SPIE!=0)){
		/* Clear each status flag */
		RIIC2.ICSR2.BIT.NACKF = 0;
		RIIC2.ICSR2.BIT.STOP = 0;
		/* Enable/Disable each interrupt */
		RIIC2.ICIER.BYTE = 0xBB;//TEIE��STIE�̊��荞�݂��֎~

		/* Initialize ram for RIIC */
		iic_buff.trm_cnt = 0;			/* Clear the internal transmission counter for IIC */
		iic_buff.rcv_cnt = 0;			/* Clear the internal reception counter for IIC */
	}

	/* Check NACK reception */
	if((RIIC2.ICSR2.BIT.NACKF != 0) && (RIIC2.ICIER.BIT.NAKIE!=0)){
		RIIC2.ICIER.BIT.NAKIE = 0;
		/* Generate Stop Condition */
		RIIC2.ICSR2.BIT.STOP = 0;
		RIIC2.ICCR2.BIT.SP = 1;
	}

	/* Check start condition detection restart*/
	if((RIIC2.ICSR2.BIT.START != 0) && (RIIC2.ICIER.BIT.STIE!=0)){
		/* Disable Start Condition Detection Interrupt */
		RIIC2.ICSR2.BIT.START = 0;
		RIIC2.ICIER.BIT.STIE = 0;
		RIIC2.ICIER.BIT.TIE = 1;
		/* Transfer slave device address */
		if((iic_buff.trm_cnt==0)&&(iic_buff.rcv_cnt==0)){//�ŏ��̃X�^�[�g�R���f�B�V����
			RIIC2.ICDRT = (iic_buff.SlvAdr<<1);		/* When master transfer, b0 must be '1' */
		}else{
			RIIC2.ICDRT = (iic_buff.SlvAdr<<1)| 0x01 ;	/* When master transfer, b0 must be '1' */
		}			
	}
}
	
void int_i2c_rx(void){
	volatile unsigned char tmp;

	/* Increase internal reception counter on RAM. It is a receive data counter. */
	iic_buff.rcv_cnt++;

	
	if(iic_buff.rcv_cnt == iic_buff.RWCnt){
			RIIC2.ICMR3.BIT.WAIT = 1;
			RIIC2.ICMR3.BIT.ACKBT = 1;
	}
	
	if(iic_buff.rcv_cnt == 1){
		/* dummy read */
		tmp = RIIC2.ICDRR;
	}else if(iic_buff.rcv_cnt == (iic_buff.RWCnt+1)){//�f�[�^��M�����
		if(iic_buff.SlvAdr==0x08){
			if(iic_buff.RA==0x04){
				iic_data.Touch_Data=RIIC2.ICDRR;
			}else if(iic_buff.RA==0x02){
				iic_data.TCNT.BYTE.TCNT_H=RIIC2.ICDRR;
			}else if(iic_buff.RA==0x03){
				iic_data.TCNT.BYTE.TCNT_L=RIIC2.ICDRR;
			}
		}
		RIIC2.ICSR2.BIT.STOP = 0;
		RIIC2.ICCR2.BIT.SP = 1;
		RIIC2.ICMR3.BIT.WAIT = 0;
		
	}else{
		if(iic_buff.SlvAdr==0x08){
			iic_data.TCNT.BYTE.TCNT_H=RIIC2.ICDRR;
		}
	}
		
	
//
//	}else if(iic_buff.RWCnt == 2){
//		RIIC2.ICMR3.BIT.WAIT = 1;
//		RIIC2.ICMR3.BIT.ACKBT = 1;
//		*iic_buff.pRWData++ = RIIC2.ICDRR;
//		iic_buff.RWCnt--;
//	}else if(iic_buff.RWCnt == 1){
//		RIIC2.ICSR2.BIT.STOP = 0;
//		RIIC2.ICCR2.BIT.SP = 1;
//		*iic_buff.pRWData++ = RIIC2.ICDRR;	/* Read final data */
//		iic_buff.RWCnt--;
//		RIIC2.ICMR3.BIT.WAIT = 0;
//		tmp2=(I2C_BUF[1]<<8) & 0xff00;
//		tmp3=I2C_BUF[0] & 0x00ff;
//		if(GYRO_ACCE_F==0){
//			GYRO16=( tmp2 | tmp3) >> 4;
//		}else{
//			ACCE16=( tmp2 | tmp3) >> 4;
//		}
//			
//	}else{
//		*iic_buff.pRWData++ = RIIC0.ICDRR;
//		iic_buff.RWCnt--;
//	}

}

//���M�f�[�^ �G���v�e�B
//TDRE=1 & TIE=1
void int_i2c_tx(void){
	/* Increase internal transmission counter */
	iic_buff.trm_cnt++;
	if(iic_buff.trm_cnt == 1){
	/* Transfer slave device register address */
		RIIC2.ICDRT = iic_buff.RA;
		if(iic_buff.RW==1){//read �̂��߃��X�^�[�g�̏���
			RIIC2.ICIER.BIT.TEIE = 1;	/* Enable Transmit End Interrupt */
		}
	}else if(iic_buff.trm_cnt == 2){
		if(iic_buff.RW==0){
			RIIC2.ICDRT = iic_buff.write_buf[0];
		}
//		if(GYRO_ACCE_F==0){
//			RIIC2.ICDRT = 0xac;//out_z_L ??read �ŏ�ʃA�h���X��1�̏ꍇ�A�A�h���X��auto incr
//		}else{
//			RIIC2.ICDRT = 0xaa;//out_y_L ??read
//		}
	}else if(iic_buff.trm_cnt == (iic_buff.RWCnt+2)) {//�������݃f�[�^�I���
		if(iic_buff.RW==0){
			RIIC2.ICIER.BIT.TEIE = 1;	/* Enable Transmit End Interrupt */
		}
	}
}

void int_i2c_te(void){
	if(iic_buff.RW==0){//write�̂�
		RIIC2.ICIER.BIT.SPIE=1;
		RIIC2.ICSR2.BIT.STOP=0; //STOP�R���f�B�V��������
		RIIC2.ICCR2.BIT.SP=1;
	}else{//read
		RIIC2.ICSR2.BIT.START = 0;		/* Clear Start Condition Detection flag */
		RIIC2.ICIER.BIT.STIE = 1;		/* Enable Start Condition Detection Interrupt */
		RIIC2.ICIER.BIT.TEIE = 0;		/* Disable Transmit End Interrupt */
		/* Generate restart condition */
		RIIC2.ICCR2.BIT.RS = 1;
	}
}
