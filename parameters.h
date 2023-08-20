#ifndef _PARAMETER

#include "static_parameters.h"

//�����I�ȃp�����[�^
#define TIRE_DIAMETER	(13.0)				//�^�C���̒��a	[mm]
#define TIRE_RADIUS	(TIRE_DIAMETER/2.0)		//�^�C���̔��a	[mm]
#define MMPP 		(TIRE_DIAMETER*PI)/(ENC_RES_MAX)	//�G���R�[�_1�p���X������ɐi�ދ���[mm](TIRE_DIAMETER*PI)/(ENC_MAX)
#define ENC_RES_MAX	(1024)
#define ENC_RES_HALF	(512)
#define MASS         (0.020)
#define REDUCTION_RATIO (0.2368)
#define Kt (0.000594)
#define Ke (0.000062)
#define R (5.0)
#define Ka (0.4626)      ///accel=0�g�p�� 0.4626 2.0747
#define Kv (2.20)

#define V_ref		3.8				//���[�^����̊�d��

//���O�p�̃p�����[�^
#define LOG_CNT		2000				//���O���Ƃ���B 1mms�Ŏ擾���Ă���̂ŁA�擾����[s]�͂��̐���1000����1 (���̐��𑝂₷(1200���炢�܂�)�A�������͕ʂɔz����쐬����ƃG���[)
//#define LOG_CNT2    2000

//�Z���T�֘A�p�����[�^
#define WAITLOOP_SLED	180				//LED�����点�Ă���AD�ϊ����J�n����܂ł̎��ԉ҂��p�萔

#define REF_SEN_R	1250				//�}�E�X����H�����ɒu�������̃Z���T�̒l
#define REF_SEN_L	1420				//�}�E�X����H�����ɒu�������̃Z���T�̒l

#define TH_SEN_R	    152				//�ǂ����邩�ۂ���臒l	�ԑ̂����̍��֊񂹂����̃Z���T�l(�ǂ���)
#define TH_SEN_L	    172				//�ǂ����邩�ۂ���臒l	�ԑ̂����̉E�֊񂹂����̃Z���T�l(�ǂ���)
#define TH_SEN_FR	    83				//�ǂ����邩�ۂ���臒l	
#define TH_SEN_FL	    53				//�ǂ����邩�ۂ���臒l
#define TH_SEN_R_POLE   500             //�������邩�ۂ���臒l�E�E
#define TH_SEN_L_POLE   600             //�������邩�ۂ���臒l�E��
#define TH_SEN_R_BE_T   2021            //�Ǔ��Ē��O��臒l�B�E
#define TH_SEN_L_BE_T   2276            //�Ǔ��Ē��O��臒l�B��

#define CONTH_SEN_R	TH_SEN_R			//����������邩�ۂ���臒l
#define CONTH_SEN_L	TH_SEN_L			//����������邩�ۂ���臒l
#define CON_WALL_KP	(10.0)				//�ǃZ���T�ɂ��p������̔�ᐧ��̔��萔

//�t�B�[�h�o�b�N�Q�C���p�����[�^
//P�Q�C���@�ŏ��ɒ�������	�����x���ڕW���x�𒆐S�Ƃ��Čy���U��������x�ɒ���
//I�Q�C���@�Ō�ɒ�������	�ϕ��l�������悤�ɂ�����x�B
//D�Q�C���@��Ԗڂɒ�������B	P����ɂ���Ĕ��������U����}��������x�ɒ���
//#���Ӂ@���̃v���O�����ł͐���������܂߂���Ԃ̃Q�C�����g�p���Ă��邽�߁A�V�X�e������œ��o�����Q�C�����g�p����ꍇ�ɂ�I�Q�C����1000�Ŋ������l�A�܂�D�Q�C����1000���|�����l���g�p����

//�ԑ̒��S�ɂ�������i�������x�Ɋւ���t�B�[�h�o�b�N�Q�C��           �V�X�e������ŋ��߂��l
#define SPEED_KP	(71.06)				//P�Q�C��  70.0                71.06                  53.9119
#define SPEED_KI	(3.395)				//I�Q�C��   2.0                3.395                  2.577
#define SPEED_KD	(0)				//D�Q�C���@ 0.01                     0
//�ԑ̒��S�ɂ������]�������x�Ɋւ���t�B�[�h�o�b�N�Q�C��           �V�X�e������ŋ��߂��l
#define OMEGA_KP	(69.19)				//P�Q�C��  75.0                39.19
#define OMEGA_KI	(1.826)			//I�Q�C��   4.0                1826
#define OMEGA_KD	(0.07266)			    //D�Q�C��   7.0                0.07266
//�p�x�Ɋւ���t�B�[�h�o�b�N�Q�C��
#define DEGREE_KP   (80.0) 
#define DEGREE_KI   (1.0)
//�ڕW�O���Ƃ̍��Ɋւ���t�B�[�h�o�b�N�Q�C��
#define Kx           (0.0) //0.05:1.0 //0.0001:0.3:1.2
#define Ky           (0.0)  //0.0001:0.0001:0.0
#define Ktheta       (0.0)

//���s�p�����[�^
#define SEARCH_SPEED	(0.2)				//�T�����s�̑��x	[m/s]
#define SEARCH_ACCEL	(1.0)				//�T�����s�̉����x	[m/s^2]
#define FAST_SPEED	(1.0)				//�ŒZ���s�̑��x	[m/s]
#define FAST_ACCEL	(2.0)				//�ŒZ���s�̉����x	[m/s^2]
#define MIN_SPEED	(0.1)				//�Œᑬ�x	[m/s]
#define S_SEARCH_SPEED (0.25)                //�X�����[�����s�̑��x   [m/s]
#define S_SEARCH_ACCEL (0.8)                //�X�����[�����s�̉����x [m/s/s]
#define S_FAST_SPEED	(0.3)				//�ŒZ���s�̑��x	[m/s]
#define S_FAST_ACCEL	(2.0)				//�ŒZ���s�̉����x	[m/s^2]
#define S_MIN_SPEED	(0.1)				//�Œᑬ�x	[m/s]

#define TURN_ACCEL	(PI*8)				//���M�n����̉����x	[rad/s^2]
#define	TURN_SPEED	(PI*2)				//���M�n����̍ō����x	[rad/s] (PI[rad/s]->180deg/s)
#define TURN_MIN_SPEED	(PI/100.0)			//���M�n����̍Œᑬ�x	[rad/s]
#define SLALOM_ACCEL  (130.9)        //143.82  150.45  153.24  148.35 91.28
#define SLALOM_SPEED  (11.65)          //10.05    10.23  10.26  10.24 9.77
#define SLA_MIN_SPEED (0.08)
#define SLALOM_ACCEL_2  (69.81)        //68.07 20��A����]�Ō덷��8�x�قǊm�F
#define SLALOM_SPEED_2  (6.98)         //6.87 
#define SLA_MIN_SPEED_2 (0.07)           //0.99
#define KANAYAMA_SPEED (10.05)




#define WAIT_TIME	500				//�e�����̑ҋ@����	[ms]

//���H�֘A�p�����[�^
#define GOAL_X	7		//�S�[�����W(x)
#define GOAL_Y	10		//�S�[�����W(y)
#define GOAL_X1	7		//�S�[�����W(x)
#define GOAL_Y1	11		//�S�[�����W(y)
#define GOAL_X2	8		//�S�[�����W(x)
#define GOAL_Y2	10		//�S�[�����W(y)
#define GOAL_X3	8		//�S�[�����W(x)
#define GOAL_Y3	11		//�S�[�����W(y)
/*#define GOAL_X 3
#define GOAL_Y 3
#define GOAL_X1 3
#define GOAL_Y1 3
#define GOAL_X2 3
#define GOAL_Y2 3
#define GOAL_X3 3
#define GOAL_Y3 3*/

#define PASSED 255
#define STACK_SIZE (MAZESIZE_X)*(MAZESIZE_Y)

//����p�Z���T�p�����[�^
#define	SEN_DECISION	2000	//���j���[����p�̌��Z���T臒l

#define _PARAMETER

#endif
