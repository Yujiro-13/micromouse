//グローバル変数を一�?所で宣言する�?
//glob_var.cで実体を作り(_GLOB_VARをdefine)、他�?�グローバル変数を使用するファイルからはglob_var.hをincludeする�?

#include "mytypedef.h"
#include "parameters.h"

//global変数を一括で実体とextern宣言する�?
//include�?で_GLOB_VARが定義されて�?れ�?�実体宣言、定義されて�?なければ外部宣言となる�?
#ifdef _GLOB_VAR
#define GLOBAL
#else
#define GLOBAL extern
#endif

//構�?体系のグローバル変数
GLOBAL t_sensor			sen_r, sen_l, sen_fr, sen_fl;		//センサ構�?�?
GLOBAL t_control		con_wall;				//制御構�?�?
GLOBAL t_control		con_fwall;				//制御構�?�?
GLOBAL t_position		mypos;					//自己座�?
GLOBAL t_wall			wall[MAZESIZE_X][MAZESIZE_Y];		//壁�?��?報を�?�納する構�?体�?��??
GLOBAL unsigned char		map[MAZESIZE_X][MAZESIZE_Y];		//歩数マッ�?

//走行系のグローバル変数
GLOBAL int run_mode;							//真っ直ぐか回転か�?�走行モード（制御系の�?り替えに使用?�?

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
GLOBAL float			accel;					//�����x		[m/ss]
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

//エンコーダ角度系のグローバル変数
GLOBAL unsigned int			angle;					//現在の車軸角度	[deg]
GLOBAL unsigned int			locate_l;				//現在の車軸位置	[無次元]
GLOBAL unsigned int			locate_r;				//現在の車軸位置	[無次元]
GLOBAL unsigned int			before_locate_r;			//過去の車軸位置	[無次元]
GLOBAL unsigned int			before_locate_l;			//過去の車軸位置	[無次元]
GLOBAL int			diff_pulse_r;				//車軸位置の微�?値(車軸の回転速度[pulse/ms])
GLOBAL int			diff_pulse_l;				//車軸位置の微�?値(車軸の回転速度[pulse/ms])
				
//タイマ系グローバル変数
GLOBAL unsigned int		timer;					//1mSごとにカウントア�?プされる変数.

//電圧監視用グローバル変数
GLOBAL long 			cnt;					//割り込み中のカウン�?
GLOBAL float			V_bat;					//電源電圧[V]

//�W���C���n�̃O���[�o���ϐ�
GLOBAL float			gyro_x;					//���[���W���C���̌��݂̒l	[������]
GLOBAL float			gyro_x_new;				//���[���W���C���̍ŐV�̒l	[������]
GLOBAL float			gyro_ref;				//��[���W���C���̃��t�@�����X�l[������]
GLOBAL float			degree;					//���݂̎ԑ̊p�x		[degree]
          


//�ԑ̈ړ������n�̃O���[�o���ϐ�
GLOBAL float			len_mouse;				//�}�E�X�̈ړ�����		[mm]
GLOBAL float			len_target;				//�}�E�X�̖ڕW�ړ�����		[mm]
GLOBAL float            len_count;
GLOBAL float            TH_R_len_mouse;
GLOBAL float            TH_L_len_mouse;
GLOBAL float            sum_len_mouse;
GLOBAL int              sum;
GLOBAL int              count;

//ログ用のグローバル変数
GLOBAL int			log[12][LOG_CNT];			//ログ用の配�??
GLOBAL long			log_timer;				//ログ取りようのタイ�?
GLOBAL int			log_flag;				//ログ取得�?�タイミング用

//ログ用のグローバル変数其の�?
//GLOBAL int          log2[12][LOG_CNT];

//フラグ系のグローバル変数
GLOBAL char			TURN_DIR;				//ターン方向フラグ

//����p�O���[�o���ϐ�
GLOBAL float			I_tar_speed;				//�ڕW���x��I����
GLOBAL float			I_speed;				//�����x��I����
GLOBAL float			I_tar_ang_vel;				//�ڕW�p���x��I����
GLOBAL float			I_ang_vel;				//���p���x��I����
GLOBAL float            I_start_degree;             //�ڕW�p�x��I����
GLOBAL float            I_degree;               //���p�x��I����

//UI�p�O���[�o���ϐ�
GLOBAL unsigned char		push_switch;				//�X�C�b�`�������ꂽ���ǂ����̕ϐ�
