
#ifndef _PARAMETER

#include "static_parameters.h"　

//物�?�?なパラメータ
#define TIRE_DIAMETER (13.0)                      //タイヤの直�?	[mm]
#define TIRE_RADIUS (TIRE_DIAMETER / 2.0)         //タイヤの半�?	[mm]
#define MMPP (TIRE_DIAMETER * PI) / (ENC_RES_MAX) //エンコーダ1パルスあたりに進む距離[mm](TIRE_DIAMETER*PI)/(ENC_MAX)
#define ENC_RES_MAX (1024)
#define ENC_RES_HALF (512)

#define V_ref 3.8 //モータ制御の基準電圧

//ログ用のパラメータ
#define LOG_CNT 1000 //ログをとる個数�? 1mmsで取得して�?る�?�で、取得時間[s]はこ�?�数の1000�?の1

//センサ関連パラメータ
#define WAITLOOP_SLED 180 // LEDを�?�らせてからAD変換を開始するまでの時間稼ぎ用定数

#define REF_SEN_R 1140 //マウスを迷路中央に置�?た時のセンサの値
#define REF_SEN_L 1176 //マウスを迷路中央に置�?た時のセンサの値

#define TH_SEN_R 274 //壁があるか否か�?�閾値	車体を区画の左へ�?せた時�?�センサ値(壁あ�?)
#define TH_SEN_L 267 //壁があるか否か�?�閾値	車体を区画の右へ�?せた時�?�センサ値(壁あ�?)
#define TH_SEN_FR 88 //壁があるか否か�?�閾値
#define TH_SEN_FL 67 //壁があるか否か�?�閾値

#define CONTH_SEN_R TH_SEN_R //制御をかけるか否か�?�閾値
#define CONTH_SEN_L TH_SEN_L //制御をかけるか否か�?�閾値
#define CON_WALL_KP (10.0)   //壁センサによる姿勢制御の比例制御の比例定数

//フィードバ�?クゲインパラメータ
// Pゲイン　最初に調整する	実速度が目標速度を中�?として軽く振動する程度に調整
// Iゲイン　最後に調整する	積�??値が合�?ようにする程度�?
// Dゲイン　二番目に調整する�?	P制御によって発生した振動を抑えられる程度に調整
//車体中�?における並進方向速度に関するフィードバ�?クゲイン
#define SPEED_KP (40.0) // Pゲイン
#define SPEED_KI (1.0)  // Iゲイン
#define SPEED_KD (0.0)  // Dゲイン　
//車体中�?における回転方向速度に関するフィードバ�?クゲイン
#define OMEGA_KP (40.0) // Pゲイン
#define OMEGA_KI (0.2)  // Iゲイン
#define OMEGA_KD (0.2)  // Dゲイン

//走行パラメータ
#define SEARCH_SPEED (0.3) //探索走行�?�速度	[m/s]
#define SEARCH_ACCEL (1.0) //探索走行�?��?速度	[m/s^2]
#define FAST_SPEED (1.0)   //最短走行�?�速度	[m/s]
#define FAST_ACCEL (2.0)   //最短走行�?��?速度	[m/s^2]
#define MIN_SPEED (0.1)    //最低速度	[m/s]

#define TURN_ACCEL (PI * 2)        //�?信地旋回の�?速度	[rad/s^2]
#define TURN_SPEED (PI)            //�?信地旋回の最高速度	[rad/s]
#define TURN_MIN_SPEED (PI / 10.0) //�?信地旋回の最低速度	[rad/s]
;
//#define WAIT_TIME 500 //�?動作後�?��?機時�?	[ms]

//迷路関連パラメータ
#define GOAL_X 8//ゴール座�?(x)
#define GOAL_Y 8 //ゴール座�?(y)

#define RIGHT_90 385
#define LEFT_90 280

#define FR_BORDER 1810
#define FL_BORDER 1620

//決定用センサパラメータ
#define SEN_DECISION 2000 //メニュー決定用の光センサ閾値

#define _PARAMETER

#endif
