
#define IO_OUT	(1)				//PFC��Input/Output ���W�X�^��1��ݒ肷��Əo�͂ɂȂ�
#define IO_IN	(0)				//PFC��Input/Output ���W�X�^��0��ݒ肷��Ɠ��͂ɂȂ�

#define BATT_MAX		2552//12.4V
#define BATT_MIN		2034//10V

#define SW_OFF	(1)				//�X�C�b�`��OFF�̎��Ɏ��l
#define SW_ON	(0)				//�X�C�b�`��ON�̎��Ɏ��l

#define CHATTERING_WAIT	(50)			//�`���^�����O���p�҂�����

#define INC_FREQ	(2000)			//���[�h�������������ɖ炷���̎��g��
#define DEC_FREQ	(1000)			//���[�h�������������ɖ炷���̎��g��

#define MOT_R_FORWARD	(0)			//���[�^�h���C�o��CWCCW�[�q��LOW���o�͂���ƑO�i����
#define MOT_R_BACK	(1)			//���[�^�h���C�o��CWCCW�[�q��HIGH���o�͂���ƃo�b�N����#define MOT_FORWARD	(0)			//���[�^�h���C�o��CWCCW�[�q��LOW���o�͂���ƑO�i����
#define MOT_L_FORWARD	(1)			//���[�^�h���C�o��CWCCW�[�q��LOW���o�͂���ƑO�i����
#define MOT_L_BACK	(0)			//���[�^�h���C�o��CWCCW�[�q��HIGH���o�͂���ƃo�b�N����#define MOT_FORWARD	(0)			//���[�^�h���C�o��CWCCW�[�q��LOW���o�͂���ƑO�i����

//#define MIN_SPEED	(0.3)			//�Œᑬ�x.�W�F�l�������W�X�^��16bit�ł��邱�ƂƁAMTU�̓�����g�����狁�߂���l�������悻18mm/s�Ȃ̂ŁA�]�T��������30mm/s

#define PI (3.141592653589793)			//�~����

//����
#define RIGHT	(0)
#define LEFT	(1)
#define FRONT	(2)
#define REAR	(3)

//�ŒZ���s�I���t���O
#define END_FAST	(4)

#define HALF_SECTION	(45)			//�����̋���
#define SECTION			(90)		//����̋���

#define MAZESIZE_X		(8)		//���H�̑傫��(MAZESIZE_X * MAZESIZE_Y)���H
#define MAZESIZE_Y		(8)		//���H�̑傫��(MAZESIZE_X * MAZESIZE_Y)���H

#define UNKNOWN	2				//�ǂ����邩�Ȃ�������Ȃ���Ԃ̏ꍇ�̒l
#define NOWALL	0				//�ǂ��Ȃ��΂����̒l
#define WALL	1				//�ǂ�����ꍇ�̒l
#define VWALL	3				//���z�ǂ̒l(���g�p)

#define STRAIGHT_MODE	0			//���i���̃��[�h
#define TURN_MODE	1			//���M�n���񎞂̃��[�h
#define SLA_MODE	2			//�X�����[�����[�h
#define NON_CON_MODE	3			//�񐧌䃂�[�h
#define TEST_MODE	4			//�e�X�g���[�h(���荞�ݗp���[�^�����؂郂�[�h)
#define F_WALL_MODE	5

#define MASK_SEARCH		0x01		//�T�����s�p�}�X�N�l.�Ǐ��Ƃ��̒l��AND�l���O�iNOWALL�j�Ȃ�ǂȂ�or���T�����
#define MASK_SECOND		0x03		//�ŒZ���s�p�}�X�N�l.�Ǐ��Ƃ��̒l��AND�l���O�iNOWALL�j�Ȃ�ǂȂ�