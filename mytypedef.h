
#ifdef _MYTYPEDEF

#else

typedef enum
{
	false = 0,	//�U
	true = 1,	//�^
}t_bool;		//�^�U�l����舵���񋓌^


typedef struct
{
	short value;		//���݂̒l
	short d_value;		//�����t�B���^�p
	short p_value;		//�PmS�ߋ��̒l
	short p2_value;
	short error;		//value - ref
	short ref;		//���t�@�����X�l
	short th_wall;		//�ǂ����邩�ۂ���臒l
	short th_control;	//����������邩�ۂ���臒l
	t_bool is_wall;		//�ǂ����邩������ ( true = �ǂ��� false = �ǂȂ� )
	t_bool is_control;	//����Ɏg�����ۂ�
}t_sensor;			//�Z���T�\����

typedef struct
{
	float control;		//�ŏI�I�Ȑ����
	float omega;		//�ڕW�p���x
	float p_omega;
	float theta;		//�ڕW�p�x
	float p_theta;		//�ߋ��̖ڕW�p�x
	float error;		//�΍�
	float p_error;		//�ߋ��̕΍�
	float diff;		//�΍��̔����l
	float sum;		//�΍��̐ϕ��l
	float sum_max;		//�ϕ��l�̐����l
	float kp;		//��ᐧ��萔
	float kd;		//��������萔
	float ki;		//�ϕ�����萔
	t_bool enable;		//�����on/off
}t_control;			//����\����

typedef struct
{
	
	t_bool enable;
}t_testmode;

typedef enum
{
	front=0,		//�O
	right=1,		//�E
	rear=2,			//��
	left=3,			//��
	unknown,		//�����s��
}t_local_dir;	//�������猩�������������񋓌^

typedef enum
{
	north=0,
	east=1,
	south=2,
	west=3,
}t_direction;

typedef struct
{
	short x;
	short y;
	t_direction dir;
}t_position;

typedef struct
{
	unsigned char north:2;	//�k�̕Ǐ��
	unsigned char east:2;	//���̕Ǐ��
	unsigned char south:2;	//��̕Ǐ��
	unsigned char west:2;	//���̕Ǐ��
}t_wall;			//�Ǐ����i�[����\����(�r�b�g�t�B�[���h)


#define _MYTYPEDEF

#endif
