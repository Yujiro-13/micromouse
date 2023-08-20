#ifndef _PARAMETER

#include "static_parameters.h"

//物理的なパラメータ
#define TIRE_DIAMETER	(13.0)				//タイヤの直径	[mm]
#define TIRE_RADIUS	(TIRE_DIAMETER/2.0)		//タイヤの半径	[mm]
#define MMPP 		(TIRE_DIAMETER*PI)/(ENC_RES_MAX)	//エンコーダ1パルスあたりに進む距離[mm](TIRE_DIAMETER*PI)/(ENC_MAX)
#define ENC_RES_MAX	(1024)
#define ENC_RES_HALF	(512)
#define MASS         (0.020)
#define REDUCTION_RATIO (0.2368)
#define Kt (0.000594)
#define Ke (0.000062)
#define R (5.0)
#define Ka (0.4626)      ///accel=0使用時 0.4626 2.0747
#define Kv (2.20)

#define V_ref		3.8				//モータ制御の基準電圧

//ログ用のパラメータ
#define LOG_CNT		2000				//ログをとる個数。 1mmsで取得しているので、取得時間[s]はこの数の1000分の1 (この数を増やす(1200くらいまで)、もしくは別に配列を作成するとエラー)
//#define LOG_CNT2    2000

//センサ関連パラメータ
#define WAITLOOP_SLED	180				//LEDを光らせてからAD変換を開始するまでの時間稼ぎ用定数

#define REF_SEN_R	1250				//マウスを迷路中央に置いた時のセンサの値
#define REF_SEN_L	1420				//マウスを迷路中央に置いた時のセンサの値

#define TH_SEN_R	    152				//壁があるか否かの閾値	車体を区画の左へ寄せた時のセンサ値(壁あり)
#define TH_SEN_L	    172				//壁があるか否かの閾値	車体を区画の右へ寄せた時のセンサ値(壁あり)
#define TH_SEN_FR	    83				//壁があるか否かの閾値	
#define TH_SEN_FL	    53				//壁があるか否かの閾値
#define TH_SEN_R_POLE   500             //柱があるか否かの閾値・右
#define TH_SEN_L_POLE   600             //柱があるか否かの閾値・左
#define TH_SEN_R_BE_T   2021            //壁当て直前の閾値。右
#define TH_SEN_L_BE_T   2276            //壁当て直前の閾値。左

#define CONTH_SEN_R	TH_SEN_R			//制御をかけるか否かの閾値
#define CONTH_SEN_L	TH_SEN_L			//制御をかけるか否かの閾値
#define CON_WALL_KP	(10.0)				//壁センサによる姿勢制御の比例制御の比例定数

//フィードバックゲインパラメータ
//Pゲイン　最初に調整する	実速度が目標速度を中心として軽く振動する程度に調整
//Iゲイン　最後に調整する	積分値が合うようにする程度。
//Dゲイン　二番目に調整する。	P制御によって発生した振動を抑えられる程度に調整
//#注意　このプログラムでは制御周期を含めた状態のゲインを使用しているため、システム同定で導出したゲインを使用する場合にはIゲインを1000で割った値、またDゲインに1000を掛けた値を使用する

//車体中心における並進方向速度に関するフィードバックゲイン           システム同定で求めた値
#define SPEED_KP	(71.06)				//Pゲイン  70.0                71.06                  53.9119
#define SPEED_KI	(3.395)				//Iゲイン   2.0                3.395                  2.577
#define SPEED_KD	(0)				//Dゲイン　 0.01                     0
//車体中心における回転方向速度に関するフィードバックゲイン           システム同定で求めた値
#define OMEGA_KP	(69.19)				//Pゲイン  75.0                39.19
#define OMEGA_KI	(1.826)			//Iゲイン   4.0                1826
#define OMEGA_KD	(0.07266)			    //Dゲイン   7.0                0.07266
//角度に関するフィードバックゲイン
#define DEGREE_KP   (80.0) 
#define DEGREE_KI   (1.0)
//目標軌道との差に関するフィードバックゲイン
#define Kx           (0.0) //0.05:1.0 //0.0001:0.3:1.2
#define Ky           (0.0)  //0.0001:0.0001:0.0
#define Ktheta       (0.0)

//走行パラメータ
#define SEARCH_SPEED	(0.2)				//探索走行の速度	[m/s]
#define SEARCH_ACCEL	(1.0)				//探索走行の加速度	[m/s^2]
#define FAST_SPEED	(1.0)				//最短走行の速度	[m/s]
#define FAST_ACCEL	(2.0)				//最短走行の加速度	[m/s^2]
#define MIN_SPEED	(0.1)				//最低速度	[m/s]
#define S_SEARCH_SPEED (0.25)                //スラローム走行の速度   [m/s]
#define S_SEARCH_ACCEL (0.8)                //スラローム走行の加速度 [m/s/s]
#define S_FAST_SPEED	(0.3)				//最短走行の速度	[m/s]
#define S_FAST_ACCEL	(2.0)				//最短走行の加速度	[m/s^2]
#define S_MIN_SPEED	(0.1)				//最低速度	[m/s]

#define TURN_ACCEL	(PI*8)				//超信地旋回の加速度	[rad/s^2]
#define	TURN_SPEED	(PI*2)				//超信地旋回の最高速度	[rad/s] (PI[rad/s]->180deg/s)
#define TURN_MIN_SPEED	(PI/100.0)			//超信地旋回の最低速度	[rad/s]
#define SLALOM_ACCEL  (130.9)        //143.82  150.45  153.24  148.35 91.28
#define SLALOM_SPEED  (11.65)          //10.05    10.23  10.26  10.24 9.77
#define SLA_MIN_SPEED (0.08)
#define SLALOM_ACCEL_2  (69.81)        //68.07 20回連続回転で誤差約8度ほど確認
#define SLALOM_SPEED_2  (6.98)         //6.87 
#define SLA_MIN_SPEED_2 (0.07)           //0.99
#define KANAYAMA_SPEED (10.05)




#define WAIT_TIME	500				//各動作後の待機時間	[ms]

//迷路関連パラメータ
#define GOAL_X	7		//ゴール座標(x)
#define GOAL_Y	10		//ゴール座標(y)
#define GOAL_X1	7		//ゴール座標(x)
#define GOAL_Y1	11		//ゴール座標(y)
#define GOAL_X2	8		//ゴール座標(x)
#define GOAL_Y2	10		//ゴール座標(y)
#define GOAL_X3	8		//ゴール座標(x)
#define GOAL_Y3	11		//ゴール座標(y)
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

//決定用センサパラメータ
#define	SEN_DECISION	2000	//メニュー決定用の光センサ閾値

#define _PARAMETER

#endif
