
#ifndef _PARAMETER

#include "static_parameters.h"

//��????�ȃp�����[�^
#define TIRE_DIAMETER (13.0)                      //�^�C���̒�???	[mm]
#define TIRE_RADIUS (TIRE_DIAMETER / 2.0)         //�^�C���̔�??	[mm]
#define MMPP (TIRE_DIAMETER * PI) / (ENC_RES_MAX) //�G���R�[�_1�p���X������ɐi�ދ���[mm](TIRE_DIAMETER*PI)/(ENC_MAX)
#define ENC_RES_MAX (1024)
#define ENC_RES_HALF (512)

#define V_ref 3.8 //���[�^����̊�d��

//���O�p�̃p�����[�^
#define LOG_CNT 1000 //���O���Ƃ��?? 1mms�Ŏ擾����??��???�ŁA�擾����[s]�͂�???����1000??��1

//�Z���T�֘A�p�����[�^
#define WAITLOOP_SLED 180 // LED��???�点�Ă���AD�ϊ����J�n����܂ł̎��ԉ҂��p�萔

#define REF_SEN_R 1120 //�}�E�X����H�����ɒu??�����̃Z���T�̒l
#define REF_SEN_L 1273 //�}�E�X����H�����ɒu??�����̃Z���T�̒l

#define TH_SEN_R 50 //�ǂ����邩�ۂ�???臒l	�ԑ̂����̍���??������???�Z���T�l(�ǂ�??)
#define TH_SEN_L 50 //�ǂ����邩�ۂ�???臒l	�ԑ̂����̉E��??������???�Z���T�l(�ǂ�??)
#define TH_SEN_FR 50 //�ǂ����邩�ۂ�???臒l
#define TH_SEN_FL 50 //�ǂ����邩�ۂ�???臒l
                                    
#define CONTH_SEN_R TH_SEN_R //����������邩�ۂ�???臒l
#define CONTH_SEN_L TH_SEN_L //����������邩�ۂ�???臒l
#define CON_WALL_KP (10.0)   //�ǃZ���T�ɂ��p������̔�ᐧ��̔��萔

//�t�B�[�h�o??�N�Q�C���p�����[�^
// P�Q�C���@�ŏ��ɒ�������	�����x���ڕW���x��??�Ƃ��Čy���U��������x�ɒ���
// I�Q�C���@�Ō�ɒ�������	��???�l����??�悤�ɂ�����x??
// D�Q�C���@��Ԗڂɒ�������??	P����ɂ���Ĕ��������U����}��������x�ɒ���
//�ԑ̒�??�ɂ�������i�������x�Ɋւ���t�B�[�h�o??�N�Q�C��
#define SPEED_KP (40.0) // P�Q�C��
#define SPEED_KI (1.0)  // I�Q�C��
#define SPEED_KD (0.0)  // D�Q�C���@
//�ԑ̒�??�ɂ������]�������x�Ɋւ���t�B�[�h�o??�N�Q�C��
#define OMEGA_KP (40.0) // P�Q�C��
#define OMEGA_KI (0.2)  // I�Q�C��
#define OMEGA_KD (0.2)  // D�Q�C��

//���s�p�����[�^
#define SEARCH_SPEED (0.3) //�T�����s???���x	[m/s]
#define SEARCH_ACCEL (1.0) //�T�����s?????���x	[m/s^2]
#define FAST_SPEED (1.0)   //�ŒZ���s???���x	[m/s]
#define FAST_ACCEL (2.0)   //�ŒZ���s?????���x	[m/s^2]
#define MIN_SPEED (0.1)    //�Œᑬ�x	[m/s]

#define TURN_ACCEL (PI * 2)        //??�M�n�����??���x	[rad/s^2]
#define TURN_SPEED (PI)            //??�M�n����̍ō����x	[rad/s]
#define TURN_MIN_SPEED (PI / 10.0) //??�M�n����̍Œᑬ�x	[rad/s]
;
//#define WAIT_TIME 500 //??����???????�@��??	[ms]

//���H�֘A�p�����[�^
#define GOAL_X 8//�S�[����??(x)
#define GOAL_Y 8 //�S�[����???(y)

#define RIGHT_90 385
#define LEFT_90 280

#define FR_BORDER 1810
#define FL_BORDER 1620

//����p�Z���T�p�����[�^
#define SEN_DECISION 2000 //���j���[����p�̌��Z���T臒l

#define _PARAMETER

#endif
