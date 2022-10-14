#include <machine.h>
#include <stdarg.h>
#include "iodefine.h"

char SCI_putc(unsigned char c);
short SCI_printf(char *string , ...);
short SCI_getc(unsigned char *c);



void init_sci(void){
	SYSTEM.PRCR.WORD = 0xA502;
        MSTP(SCI1) = 0;
	SYSTEM.PRCR.WORD = 0xA500;

	SCI1.SCR.BYTE = 0x00;
	while (0x00 != (SCI1.SCR.BYTE & 0xF0));	//���荞�ݗv�����֎~�����܂ő҂�
	PORT2.PODR.BIT.B6 = 1;			//TXD��Dirction�̐؂�ւ���̒l��high
    	PORT2.PDR.BIT.B6 = 1;			//�o�͂ɐݒ�
    	PORT3.PDR.BIT.B0 = 0;			//���͂ɐݒ�
	PORT2.PMR.BIT.B6 = 0;			//�ėp�|�[�g�ɐݒ�
    	PORT3.PMR.BIT.B0 = 0;			//�ėp�|�[�g�ɐݒ�
    	MPC.PWPR.BIT.B0WI  = 0;
    	MPC.PWPR.BIT.PFSWE = 1;
    	MPC.P26PFS.BIT.PSEL = 0x0A;		//TXD1
    	MPC.P30PFS.BIT.PSEL = 0x0A;		//RXD1
    	MPC.PWPR.BIT.PFSWE = 0;
    	MPC.PWPR.BIT.B0WI  = 1;
    	PORT3.PMR.BIT.B0 = 1;			//���Ӌ@�\(RXD1)�Ƃ��Ďg�p
        SCI1.SCR.BIT.CKE = 0;
    	SCI1.SMR.BYTE = 0x00;			//1stopbit parity�Ȃ��@8bit ��������
        SCI1.SCMR.BYTE = 0xF2;			//S=32clock
    	SCI1.SEMR.BYTE = 0x00;
    	SCI1.BRR =38; 				//@48MHz 38400bps
    	SCI1.SCR.BYTE =0x30;			//���M���荞�݋֎~
	PORT2.PMR.BIT.B6 = 1;			//���Ӌ@�\(TXD1)�Ƃ��Ďg�p
	SCI1.SCR.BIT.TE = 1;
	SCI1.SCR.BIT.RE = 1;
}		



char SCI_putc(unsigned char c){
	//summary
	//  1�������M
	//parameter
	//	c : ���M����
	//return
	//	0:error
	//	1:success

	unsigned long i;
	
	for(i=0;i<2000000;i++){
		if(SCI1.SSR.BIT.TEND){					//�t���O�m�F
			SCI1.TDR = c;					//���M�f�[�^��������
			return 1;
		}
	}
	return 0;							//�^�C���A�E�g
}


static void int_to_Dec(unsigned long n, char *buf){
	//summary
	//	���l���L�����N�^�i10�i�@�j�ɕϊ�
    char c;
    short length = 0;
    short i, half;

    do{
        if(n == 0)  i = 0;
        else        i = n % 10;
        buf[length] = (char)(i + '0');
        length++;
        n /= 10;
    }while(n != 0);

												    // �����̕��я��𒼂�
    half = length >> 1;
    for(i=0; i < half; i++){
        c = buf[i];
        buf[i] = buf[(length-1)-i];
        buf[(length-1)-i] = c;
    }
    buf[length]='\0';								//�I�[�R�[�h
}

// --------------------------------------------------------
//  ���l��16�i�������ϊ�
// --------------------------------------------------------
static void int_to_Hex(unsigned long n, short upper, char *buf){
	//summary
	//	���l���L�����N�^�i16�i�@�j�ɕϊ�
    char c;
    char a = 'a';
    short len = 0;
    short i, half;

    // �啶��/�������̐ݒ�
    if(upper) a = 'A';
    
    // 16�i������֕ϊ������������J�E���g
    do{
        i = n & 0x0F;
        if(i > 9)  buf[len] = (unsigned char)(i + a - 10);
        else       buf[len] = (unsigned char)(i + '0');
        len++;
        n >>= 4;
    }while(n != 0);

    // �����̕��я��𒼂�
    half = len >> 1;
    for(i=0; i < half; i++){
        c = buf[i];
        buf[i] = buf[(len-1)-i];
        buf[(len-1)-i] = c;
    }

    buf[len]='\0';   								//�I�[�R�[�h�̑}�� 
}

static char *PrintFormat(char *format,void *value){
	//summary
	//  %5d�@�ȂǁA���l�����w�肵���t�H�[�}�b�g�ŏo�͂���
	//parameter
	//	-
	//return
	//	-
	short n = 0;							//����
	short m = 0;							//���̐���
	short z = 0;							//0�Ŗ��߂邩�ǂ����̃t���O
	short i = 0;							//��Ɨp�ϐ�
	char buf[20];
	char *ptr = buf;
	unsigned char length=0; 
	char base = ' ';

	if(*format=='0'){						//�[���Ŗ��߂�
		 z = 1;
		 format++;
	}
	if((*format>='0')&&(*format<='9')){				//�����w��
		n = *format-'0';
		format++;
	}

	switch(*format){
		case 'd':						// �������� 10�i��(short�^)
			if((signed short)value < 0){			//����
				m = 1;
				int_to_Dec((unsigned long)(-(signed short)value) ,buf); 
			}
			else
			{
				int_to_Dec((unsigned long)value , buf);
			}
			break;
		case 'l': 						// �������� 10�i��(long�^)
			if((signed long)value < 0){			//����
				m = 1;
				int_to_Dec((unsigned long)(-(signed long)value) ,buf); 
			}
			else
			{
				int_to_Dec((unsigned long)value , buf);
			}
			break;
		case 'u':   						// �����Ȃ� 10�i��
			int_to_Dec((unsigned short)value ,buf);
			break;
		case 'x':   						// �����Ȃ� 16 �i�����B"abcdef" ���g�p�B
			int_to_Hex((unsigned long)value ,0 ,buf);
			break;
		case 'X':   						// �����Ȃ� 16 �i�����B"ABCDEF" ���g�p�B
			int_to_Hex((unsigned long)value ,1 ,buf);
			break;
		case 's':   						// String�o��
			ptr = (char *)value;
			break;
		case 'c':   						// 1byte����
			buf[0] = (char)((unsigned long)value & 0xFF);
			buf[1] = '\0';					//�I�[�R�[�h
			break;
		default: 
			buf[0] = '\0';					//�I�[�R�[�h
			break;
	}
	
									//�\��
	if(m){length ++;}						//�����̏ꍇ����������
	for(;buf[length]!='\0';length++){}				//�������̃J�E���g
	if(n>length){n -=length;}					//�����̒���
    else{n  = 0;}

	if(z){								//�O�Ŗ��߂�ꍇ
        base = '0';
        if(m){	        						// �}�C�i�X�\��
            SCI_putc('-');
            m = 0;
			//length --;
        }
	}
    for(i=0; i<n; i++){							//�������킹
	    SCI_putc(base);
	}
    if(m){								//�}�C�i�X�̕\��
		SCI_putc('-');
		//length --;
	}
	for(i=0;i<length;i++){						//�f�[�^�̕\��
		if(!SCI_putc(buf[i])) break;
	}

	format++;
	return(format);
}


short SCI_printf(char *string , ...){
	//summary
	//  �ȈՕ����񑗐M
	//parameter
	//	������A
	//return
	//	0:	error
	//	else:���M������
	va_list ap;
	char *ptr;

//	unsigned long value = 0;
	
	ptr = string;
	va_start(ap,string);
	for(;;){
	    while (*ptr != '%') {					//���ʂ̕�����𑗐M
	      if (!*ptr) {						//�S���M
	        va_end (ap);
	        return (ptr-string);					//���M��������Ԃ�
	      }
		  else {
			SCI_putc(*ptr);					//1�������M
			ptr++;
	      }
	    }
		ptr++;							//%�ɑ��������Ŕ��f����
		if(*ptr=='%'){ SCI_putc('%'); ptr++;	}		//%%  -> %�ƕ\��
		else{
			ptr = PrintFormat(ptr,va_arg(ap,void *));
			if(ptr == 0) return 0;
		}
	
	}
}

 /*   char buf[100];
	va_list ap;
	char length=0,i;

	va_start(ap,string);
	length=vsprintf(buf,string,ap);
	va_end(ap);
	for(i=0;i<length;i++){
		SCI_putc(buf[i]);
	}
}	
*/