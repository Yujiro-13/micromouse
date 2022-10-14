#include "portdef.h"

#define SET_BUZZER_FREQ(f)	MTU0.TGRB=(unsigned short)(12000000/(f));MTU0.TGRA=(unsigned short)(6000000/(f))	//�u�U�[�̔��U���g�����Z�o���āA�ݒ�

#define ENABLE_BUZZER		PORTB.PMR.BIT.B3=1;MTU.TSTR.BIT.CST0=1	//�u�U�[�̔��U���J�n
#define DISABLE_BUZZER		PORTB.PMR.BIT.B3=0;MTU.TSTR.BIT.CST0=0	//�u�U�[�̔��U���~

#define TIRE_CIRCUIT	(PI*TIRE_DIAMETER)			//�^�C���̉~���𒼌a����Z�o(PI�͉~����)
#define SPEED2GREG(v)	(7500/(((v)/TIRE_CIRCUIT)))		//�X�s�[�h����W�F�l�������W�X�^�̒l���v�Z

#define LEN2STEP(l)	(2*400*(l)/TIRE_CIRCUIT)		//���s����(mm)����X�e�b�s���O���[�^�̃X�e�b�v�����Z�o
#define STEP2LEN(s) 	(TIRE_CIRCUIT*(s)/(2*400))		//�X�e�b�s���O���[�^�̃X�e�b�v�����瑖�s�������Z�o

#define CONV_SEN2WALL(w) ((w) ? WALL : NOWALL)			//�Z���T��񂩂�Ǐ��֕ϊ�
