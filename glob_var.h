//�O���[�o���ϐ�����ӏ��Ő錾����B
//glob_var.c�Ŏ��̂����(_GLOB_VAR��define)�A���̃O���[�o���ϐ����g�p����t�@�C�������glob_var.h��include����B

#include "mytypedef.h"
#include "parameters.h"

//global�ϐ����ꊇ�Ŏ��̂�extern�錾����B
//include����_GLOB_VAR����`����Ă���Ύ��̐錾�A��`����Ă��Ȃ���ΊO���錾�ƂȂ�B
#ifdef _GLOB_VAR
#define GLOBAL
#else
#define GLOBAL extern
#endif

//�\���̌n�̃O���[�o���ϐ�
GLOBAL t_sensor			sen_r, sen_l, sen_fr, sen_fl;		//�Z���T�\����
GLOBAL t_control		con_wall;				//����\����
GLOBAL t_control        con_r_wall;              //����\����
GLOBAL t_control        con_l_wall;              //����\����
GLOBAL t_control		con_fwall;				//����\����
GLOBAL t_position		mypos;					//���ȍ��W
GLOBAL t_wall			wall[MAZESIZE_X][MAZESIZE_Y];		//�ǂ̏����i�[����\���̔z��
GLOBAL unsigned char	map[MAZESIZE_X][MAZESIZE_Y];		//�����}�b�v
GLOBAL unsigned char    maze[MAZESIZE_X][MAZESIZE_Y];
GLOBAL STACK_T          stack;
GLOBAL POS_T            route[MAZESIZE_X][MAZESIZE_Y];
GLOBAL t_bool           stack_flag;


//���s�n�̃O���[�o���ϐ�
GLOBAL int run_mode;							//�^����������]���̑��s���[�h�i����n�̐؂�ւ��Ɏg�p�j

//�ԑ̑��x�n�̃O���[�o���ϐ�
GLOBAL float			fast_speed;				//�ŒZ���s���̍ō����x	[m/s]
GLOBAL float			tar_ang_vel;				//�ڕW�p���x		[rad/s]
GLOBAL float			tar_degree;				//�ڕW�p�x		[deg]
GLOBAL float			max_degree;				//���񎞂̍ő�p�x	[deg]
GLOBAL float			start_degree;				//���s�i�����̎ԑ̊p�x	[deg]
GLOBAL float			ang_vel;				//���݊p���x		[rad/s]
GLOBAL float			p_ang_vel;				//�ߋ��p���x		[rad/s]
GLOBAL float			max_ang_vel;				//�ō��p���x		[rad/s]
GLOBAL float			ang_acc;				//�p�����x		[rad/ss]
GLOBAL float            ff_ang_acc;
GLOBAL float			accel;					//�����x		[m/ss]
GLOBAL float            ff_accel;
GLOBAL float            ff_decel;
GLOBAL float            tar_accel;
GLOBAL float			max_speed;				//�ō����x		[m/s]
GLOBAL float			speed_r;				//���݂̉E�^�C�����x	[m/s]
GLOBAL float			speed_l;				//���݂̍��^�C�����x	[m/s]
GLOBAL float			speed_old_r;				//�E�^�C���̉ߋ��̑��x	[m/s]
GLOBAL float			speed_new_r;				//�E�^�C���̍ŐV�̑��x	[m/s]
GLOBAL float			speed_old_l;				//���^�C���̉ߋ��̑��x	[m/s]
GLOBAL float			speed_new_l;				//���^�C���̍ŐV�̑��x	[m/s]				
GLOBAL float			speed;					//���ݎԑ̑��x		[m/s]
GLOBAL float			p_speed;				//�ߋ��̎ԑ̑��x	[m/s]
GLOBAL float			tar_speed;				//�ڕW�ԑ̑��x		[m/s]
GLOBAL float			end_speed;				//�I�[�ԑ̑��x		[m/s]
GLOBAL float			V_r;					//�E���[�^�̏o�͓d��	[V]
GLOBAL float			V_l;					//�����[�^�̏o�͓d��	[V]
GLOBAL float            error;                  //�΍�
GLOBAL float            p_error;                //�ߋ��̕΍�
GLOBAL float            I_error;                //�΍���I����
GLOBAL float            T_motor;                //���[�^�̃g���N
GLOBAL float            Motor_speed;            //���[�^�̉�]��
GLOBAL float            box_A;
GLOBAL float            box_B;

//�G���R�[�_�p�x�n�̃O���[�o���ϐ�
GLOBAL unsigned int			angle;					//���݂̎Ԏ��p�x	[deg]
GLOBAL unsigned int			locate_l;				//���݂̎Ԏ��ʒu	[������]
GLOBAL unsigned int			locate_r;				//���݂̎Ԏ��ʒu	[������]
GLOBAL unsigned int			before_locate_r;			//�ߋ��̎Ԏ��ʒu	[������]
GLOBAL unsigned int			before_locate_l;			//�ߋ��̎Ԏ��ʒu	[������]
GLOBAL int			diff_pulse_r;				//�Ԏ��ʒu�̔����l(�Ԏ��̉�]���x[pulse/ms])
GLOBAL int			diff_pulse_l;				//�Ԏ��ʒu�̔����l(�Ԏ��̉�]���x[pulse/ms])
GLOBAL int          bef_diff_pulse_r;           //�ߋ��̎Ԏ��ʒu�̔����l
GLOBAL int          bef_diff_pulse_l;           //�ߋ��̎Ԏ��ʒu�̔����l
				
//�^�C�}�n�O���[�o���ϐ�
GLOBAL unsigned long		timer;					//1mS(0.001s)���ƂɃJ�E���g�A�b�v�����ϐ�.

//�d���Ď��p�O���[�o���ϐ�
GLOBAL long 			cnt;					//���荞�ݒ��̃J�E���g
GLOBAL float			V_bat;					//�d���d��[V]

//�W���C���n�̃O���[�o���ϐ�
GLOBAL float			gyro_x;					//���[���W���C���̌��݂̒l	[������]
GLOBAL float			gyro_x_new;				//���[���W���C���̍ŐV�̒l	[������]
GLOBAL float			gyro_ref;				//��[���W���C���̃��t�@�����X�l[������]
GLOBAL float			degree;					//���݂̎ԑ̊p�x		[degree]
GLOBAL float            last_degree;
GLOBAL float            radian;                  //[rad]
GLOBAL float            accel_x;
GLOBAL float            accel_x_new;
GLOBAL unsigned int     hit_flag;                //�Փ˔���
          


//�ԑ̈ړ������n�̃O���[�o���ϐ�
GLOBAL float			len_mouse;				//�}�E�X�̈ړ�����		[mm]
GLOBAL float			len_target;				//�}�E�X�̖ڕW�ړ�����		[mm]
GLOBAL int              len_count;               //���i���J��Ԃ����񐔂̃J�E���g turn�܂���slalom�̓x��reset
GLOBAL float            sum_len_mouse;
GLOBAL int              sum;
GLOBAL int              count;
GLOBAL int              slalom_count;
     
//�I�h���g���̃O���[�o���ϐ�
GLOBAL float           x_position;             //x���W
GLOBAL float           y_position;             //y���W
GLOBAL float           last_x_pos;             //�ߋ���x���W(1��������O�̒l��ۑ�)
GLOBAL float           last_y_pos;             //�ߋ���y���W(1��������O�̒l��ۑ�)
GLOBAL unsigned int    now_dir;                //���݌����Ă�����p

//���O�p�̃O���[�o���ϐ�
//GLOBAL int			logs[12][LOG_CNT];			//���O�p�̔z��
GLOBAL int          loga[6][LOG_CNT];               //log*2  LOG_CNT 7200  log*3 LOG_CNT 4800
GLOBAL long			log_timer;				//���O���悤�̃^�C�}
GLOBAL int			log_flag;				//���O�擾�̃^�C�~���O�p

//�t���O�n�̃O���[�o���ϐ�
GLOBAL char			TURN_DIR;				//�^�[�������t���O

//���[�^�̃f���[�e�B����p�O���[�o���ϐ�
GLOBAL float			Duty_r;					//�o�͂̃f���[�e�B��				[%]
GLOBAL float			Duty_l;					//�o�͂̃f���[�e�B��				[%]

//����p�O���[�o���ϐ�
GLOBAL float			I_tar_speed;				//�ڕW���x��I����
GLOBAL float			I_speed;				//�����x��I����
GLOBAL float			I_tar_ang_vel;				//�ڕW�p���x��I����
GLOBAL float			I_ang_vel;				//���p���x��I����
GLOBAL float            I_start_degree;             //�ڕW�p�x��I����
GLOBAL float            I_degree;               //���p�x��I����
GLOBAL unsigned int     FB_flag;                //����p�t���O

//�O���Ǐ]�p�O���[�o���ϐ�
GLOBAL float            x_e;
GLOBAL float            y_e;
GLOBAL float            theta_e;
GLOBAL float            th_e;
GLOBAL float            tar_x;
GLOBAL float            tar_y;
GLOBAL float            tar_th;
GLOBAL float            tar_d_th;
GLOBAL float            tar_speed_s;
GLOBAL float            tar_ang_vel_s;
GLOBAL float            K_x;
GLOBAL float            K_y;
GLOBAL float            K_th;

//UI�p�O���[�o���ϐ�
GLOBAL unsigned char		push_switch;				//�X�C�b�`�������ꂽ���ǂ����̕ϐ�

GLOBAL long long a;
GLOBAL long long th;
GLOBAL int M_l;
GLOBAL int M_r;
GLOBAL int M_sig;
GLOBAL unsigned int bef_x;
GLOBAL unsigned int bef_y;



