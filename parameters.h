#ifndef _PARAMETER

#include "static_parameters.h"

//�����I�ȃp�����[�^
#define TIRE_DIAMETER	(13.0)				//�^�C���̒��a	[mm]
#define TIRE_RADIUS	(TIRE_DIAMETER/2.0)		//�^�C���̔��a	[mm]
#define MMPP 		(TIRE_DIAMETER*PI)/(ENC_RES_MAX)	//�G���R�[�_1�p���X������ɐi�ދ���[mm](TIRE_DIAMETER*PI)/(ENC_MAX)
#define ENC_RES_MAX	(1024)
#define ENC_RES_HALF	(512)

#define V_ref		3.8				//���[�^����̊�d��

//���O�p�̃p�����[�^
#define LOG_CNT		1000				//���O���Ƃ���B 1mms�Ŏ擾���Ă���̂ŁA�擾����[s]�͂��̐���1000����1

//�Z���T�֘A�p�����[�^
#define WAITLOOP_SLED	180				//LED�����点�Ă���AD�ϊ����J�n����܂ł̎��ԉ҂��p�萔

#define REF_SEN_R	1136				//�}�E�X����H�����ɒu�������̃Z���T�̒l
#define REF_SEN_L	1435				//�}�E�X����H�����ɒu�������̃Z���T�̒l

#define TH_SEN_R	    35				//�ǂ����邩�ۂ���臒l	�ԑ̂����̍��֊񂹂����̃Z���T�l(�ǂ���)
#define TH_SEN_L	    71				//�ǂ����邩�ۂ���臒l	�ԑ̂����̉E�֊񂹂����̃Z���T�l(�ǂ���)
#define TH_SEN_FR	    50				//�ǂ����邩�ۂ���臒l	
#define TH_SEN_FL	    50				//�ǂ����邩�ۂ���臒l
#define TH_SEN_R_POLE   200             //�������邩�ۂ���臒l�E�E
#define TH_SEN_L_POLE   300             //�������邩�ۂ���臒l�E��
#define TH_SEN_R_BE_T   2021            //�Ǔ��Ē��O��臒l�B�E
#define TH_SEN_L_BE_T   2076

#define CONTH_SEN_R	TH_SEN_R			//����������邩�ۂ���臒l
#define CONTH_SEN_L	TH_SEN_L			//����������邩�ۂ���臒l
#define CON_WALL_KP	(10.0)				//�ǃZ���T�ɂ��p������̔�ᐧ��̔��萔

//�t�B�[�h�o�b�N�Q�C���p�����[�^
//P�Q�C���@�ŏ��ɒ�������	�����x���ڕW���x�𒆐S�Ƃ��Čy���U��������x�ɒ���
//I�Q�C���@�Ō�ɒ�������	�ϕ��l�������悤�ɂ�����x�B
//D�Q�C���@��Ԗڂɒ�������B	P����ɂ���Ĕ��������U����}��������x�ɒ���
//�ԑ̒��S�ɂ�������i�������x�Ɋւ���t�B�[�h�o�b�N�Q�C��
#define SPEED_KP	(70.0)				//P�Q�C��
#define SPEED_KI	(1.0)				//I�Q�C��
#define SPEED_KD	(0.0)				//D�Q�C���@
//�ԑ̒��S�ɂ������]�������x�Ɋւ���t�B�[�h�o�b�N�Q�C��
#define OMEGA_KP	(40.0)				//P�Q�C��
#define OMEGA_KI	(0.2)				//I�Q�C��
#define OMEGA_KD	(0.2)				//D�Q�C��
//�p�x�Ɋւ���t�B�[�h�o�b�N�Q�C��
#define DEGREE_KP   (80.0) //40,1
#define DEGREE_KI   (1.0)

//���s�p�����[�^
#define SEARCH_SPEED	(0.3)				//�T�����s�̑��x	[m/s]
#define SEARCH_ACCEL	(1.0)				//�T�����s�̉����x	[m/s^2]
#define FAST_SPEED	(1.0)				//�ŒZ���s�̑��x	[m/s]
#define FAST_ACCEL	(2.0)				//�ŒZ���s�̉����x	[m/s^2]
#define MIN_SPEED	(0.1)				//�Œᑬ�x	[m/s]

#define TURN_ACCEL	(PI*4)				//���M�n����̉����x	[rad/s^2]
#define	TURN_SPEED	(PI*2)				//���M�n����̍ō����x	[rad/s]
#define TURN_MIN_SPEED	(PI/10.0)			//���M�n����̍Œᑬ�x	[rad/s]

#define WAIT_TIME	500				//�e�����̑ҋ@����	[ms]

//���H�֘A�p�����[�^
#define GOAL_X	8		//�S�[�����W(x)
#define GOAL_Y	8		//�S�[�����W(y)

//����p�Z���T�p�����[�^
#define	SEN_DECISION	2000	//���j���[����p�̌��Z���T臒l

#define _PARAMETER

#endif