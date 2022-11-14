//ã‚°ãƒ­ãƒ¼ãƒãƒ«å¤‰æ•°ã‚’ä¸€ç®?æ‰€ã§å®£è¨€ã™ã‚‹ã€?
//glob_var.cã§å®Ÿä½“ã‚’ä½œã‚Š(_GLOB_VARã‚’define)ã€ä»–ã?®ã‚°ãƒ­ãƒ¼ãƒãƒ«å¤‰æ•°ã‚’ä½¿ç”¨ã™ã‚‹ãƒ•ã‚¡ã‚¤ãƒ«ã‹ã‚‰ã¯glob_var.hã‚’includeã™ã‚‹ã€?

#include "mytypedef.h"
#include "parameters.h"

//globalå¤‰æ•°ã‚’ä¸€æ‹¬ã§å®Ÿä½“ã¨externå®£è¨€ã™ã‚‹ã€?
//includeå…?ã§_GLOB_VARãŒå®šç¾©ã•ã‚Œã¦ã?ã‚Œã?°å®Ÿä½“å®£è¨€ã€å®šç¾©ã•ã‚Œã¦ã?ãªã‘ã‚Œã°å¤–éƒ¨å®£è¨€ã¨ãªã‚‹ã€?
#ifdef _GLOB_VAR
#define GLOBAL
#else
#define GLOBAL extern
#endif

//æ§‹é€?ä½“ç³»ã®ã‚°ãƒ­ãƒ¼ãƒãƒ«å¤‰æ•°
GLOBAL t_sensor			sen_r, sen_l, sen_fr, sen_fl;		//ã‚»ãƒ³ã‚µæ§‹é€?ä½?
GLOBAL t_control		con_wall;				//åˆ¶å¾¡æ§‹é€?ä½?
GLOBAL t_control		con_fwall;				//åˆ¶å¾¡æ§‹é€?ä½?
GLOBAL t_position		mypos;					//è‡ªå·±åº§æ¨?
GLOBAL t_wall			wall[MAZESIZE_X][MAZESIZE_Y];		//å£ã?®æƒ?å ±ã‚’æ?¼ç´ã™ã‚‹æ§‹é€?ä½“é?å??
GLOBAL unsigned char		map[MAZESIZE_X][MAZESIZE_Y];		//æ­©æ•°ãƒãƒƒãƒ?

//èµ°è¡Œç³»ã®ã‚°ãƒ­ãƒ¼ãƒãƒ«å¤‰æ•°
GLOBAL int run_mode;							//çœŸã£ç›´ãã‹å›è»¢ã‹ã?®èµ°è¡Œãƒ¢ãƒ¼ãƒ‰ï¼ˆåˆ¶å¾¡ç³»ã®åˆ?ã‚Šæ›¿ãˆã«ä½¿ç”¨?¼?

//Ô‘Ì‘¬“xŒn‚ÌƒOƒ[ƒoƒ‹•Ï”
GLOBAL float			fast_speed;				//Å’Z‘–s‚ÌÅ‚‘¬“x	[m/s]
GLOBAL float			tar_ang_vel;				//–Ú•WŠp‘¬“x		[rad/s]
GLOBAL float			tar_degree;				//–Ú•WŠp“x		[deg]
GLOBAL float			max_degree;				//ù‰ñ‚ÌÅ‘åŠp“x	[deg]
GLOBAL float			start_degree;				//‘–si“ü‚ÌÔ‘ÌŠp“x	[deg]
GLOBAL float			ang_vel;				//Œ»İŠp‘¬“x		[rad/s]
GLOBAL float			p_ang_vel;				//‰ß‹Šp‘¬“x		[rad/s]
GLOBAL float			max_ang_vel;				//Å‚Šp‘¬“x		[rad/s]
GLOBAL float			ang_acc;				//Šp‰Á‘¬“x		[rad/ss]
GLOBAL float			accel;					//‰Á‘¬“x		[m/ss]
GLOBAL float			max_speed;				//Å‚‘¬“x		[m/s]
GLOBAL float			speed_r;				//Œ»İ‚Ì‰Eƒ^ƒCƒ„‘¬“x	[m/s]
GLOBAL float			speed_l;				//Œ»İ‚Ì¶ƒ^ƒCƒ„‘¬“x	[m/s]
GLOBAL float			speed_old_r;				//‰Eƒ^ƒCƒ„‚Ì‰ß‹‚Ì‘¬“x	[m/s]
GLOBAL float			speed_new_r;				//‰Eƒ^ƒCƒ„‚ÌÅV‚Ì‘¬“x	[m/s]
GLOBAL float			speed_old_l;				//¶ƒ^ƒCƒ„‚Ì‰ß‹‚Ì‘¬“x	[m/s]
GLOBAL float			speed_new_l;				//¶ƒ^ƒCƒ„‚ÌÅV‚Ì‘¬“x	[m/s]				
GLOBAL float			speed;					//Œ»İÔ‘Ì‘¬“x		[m/s]
GLOBAL float			p_speed;				//‰ß‹‚ÌÔ‘Ì‘¬“x	[m/s]
GLOBAL float			tar_speed;				//–Ú•WÔ‘Ì‘¬“x		[m/s]
GLOBAL float			end_speed;				//I’[Ô‘Ì‘¬“x		[m/s]
GLOBAL float			V_r;					//‰Eƒ‚[ƒ^‚Ìo—Í“dˆ³	[V]
GLOBAL float			V_l;					//¶ƒ‚[ƒ^‚Ìo—Í“dˆ³	[V]
GLOBAL float            error;                  //•Î·
GLOBAL float            p_error;                //‰ß‹‚Ì•Î·
GLOBAL float            I_error;                //•Î·‚ÌI¬•ª

//ã‚¨ãƒ³ã‚³ãƒ¼ãƒ€è§’åº¦ç³»ã®ã‚°ãƒ­ãƒ¼ãƒãƒ«å¤‰æ•°
GLOBAL unsigned int			angle;					//ç¾åœ¨ã®è»Šè»¸è§’åº¦	[deg]
GLOBAL unsigned int			locate_l;				//ç¾åœ¨ã®è»Šè»¸ä½ç½®	[ç„¡æ¬¡å…ƒ]
GLOBAL unsigned int			locate_r;				//ç¾åœ¨ã®è»Šè»¸ä½ç½®	[ç„¡æ¬¡å…ƒ]
GLOBAL unsigned int			before_locate_r;			//éå»ã®è»Šè»¸ä½ç½®	[ç„¡æ¬¡å…ƒ]
GLOBAL unsigned int			before_locate_l;			//éå»ã®è»Šè»¸ä½ç½®	[ç„¡æ¬¡å…ƒ]
GLOBAL int			diff_pulse_r;				//è»Šè»¸ä½ç½®ã®å¾®åˆ?å€¤(è»Šè»¸ã®å›è»¢é€Ÿåº¦[pulse/ms])
GLOBAL int			diff_pulse_l;				//è»Šè»¸ä½ç½®ã®å¾®åˆ?å€¤(è»Šè»¸ã®å›è»¢é€Ÿåº¦[pulse/ms])
				
//ã‚¿ã‚¤ãƒç³»ã‚°ãƒ­ãƒ¼ãƒãƒ«å¤‰æ•°
GLOBAL unsigned int		timer;					//1mSã”ã¨ã«ã‚«ã‚¦ãƒ³ãƒˆã‚¢ãƒ?ãƒ—ã•ã‚Œã‚‹å¤‰æ•°.

//é›»åœ§ç›£è¦–ç”¨ã‚°ãƒ­ãƒ¼ãƒãƒ«å¤‰æ•°
GLOBAL long 			cnt;					//å‰²ã‚Šè¾¼ã¿ä¸­ã®ã‚«ã‚¦ãƒ³ãƒ?
GLOBAL float			V_bat;					//é›»æºé›»åœ§[V]

//ƒWƒƒƒCƒŒn‚ÌƒOƒ[ƒoƒ‹•Ï”
GLOBAL float			gyro_x;					//ƒˆ[²ƒWƒƒƒCƒ‚ÌŒ»İ‚Ì’l	[–³ŸŒ³]
GLOBAL float			gyro_x_new;				//ƒˆ[²ƒWƒƒƒCƒ‚ÌÅV‚Ì’l	[–³ŸŒ³]
GLOBAL float			gyro_ref;				//‚æ[²ƒWƒƒƒCƒ‚ÌƒŠƒtƒ@ƒŒƒ“ƒX’l[–³ŸŒ³]
GLOBAL float			degree;					//Œ»İ‚ÌÔ‘ÌŠp“x		[degree]
          


//Ô‘ÌˆÚ“®‹——£Œn‚ÌƒOƒ[ƒoƒ‹•Ï”
GLOBAL float			len_mouse;				//ƒ}ƒEƒX‚ÌˆÚ“®‹——£		[mm]
GLOBAL float			len_target;				//ƒ}ƒEƒX‚Ì–Ú•WˆÚ“®‹——£		[mm]
GLOBAL float            len_count;
GLOBAL float            TH_R_len_mouse;
GLOBAL float            TH_L_len_mouse;
GLOBAL float            sum_len_mouse;
GLOBAL int              sum;
GLOBAL int              count;

//ãƒ­ã‚°ç”¨ã®ã‚°ãƒ­ãƒ¼ãƒãƒ«å¤‰æ•°
GLOBAL int			log[12][LOG_CNT];			//ãƒ­ã‚°ç”¨ã®é…å??
GLOBAL long			log_timer;				//ãƒ­ã‚°å–ã‚Šã‚ˆã†ã®ã‚¿ã‚¤ãƒ?
GLOBAL int			log_flag;				//ãƒ­ã‚°å–å¾—ã?®ã‚¿ã‚¤ãƒŸãƒ³ã‚°ç”¨

//ãƒ­ã‚°ç”¨ã®ã‚°ãƒ­ãƒ¼ãƒãƒ«å¤‰æ•°å…¶ã®äº?
//GLOBAL int          log2[12][LOG_CNT];

//ãƒ•ãƒ©ã‚°ç³»ã®ã‚°ãƒ­ãƒ¼ãƒãƒ«å¤‰æ•°
GLOBAL char			TURN_DIR;				//ã‚¿ãƒ¼ãƒ³æ–¹å‘ãƒ•ãƒ©ã‚°

//§Œä—pƒOƒ[ƒoƒ‹•Ï”
GLOBAL float			I_tar_speed;				//–Ú•W‘¬“x‚ÌI¬•ª
GLOBAL float			I_speed;				//À‘¬“x‚ÌI¬•ª
GLOBAL float			I_tar_ang_vel;				//–Ú•WŠp‘¬“x‚ÌI¬•ª
GLOBAL float			I_ang_vel;				//ÀŠp‘¬“x‚ÌI¬•ª
GLOBAL float            I_start_degree;             //–Ú•WŠp“x‚ÌI¬•ª
GLOBAL float            I_degree;               //ÀŠp“x‚ÌI¬•ª

//UI—pƒOƒ[ƒoƒ‹•Ï”
GLOBAL unsigned char		push_switch;				//ƒXƒCƒbƒ`‚ª‰Ÿ‚³‚ê‚½‚©‚Ç‚¤‚©‚Ì•Ï”
