/***********************************************************************/
/*                                                                     */
/*  FILE        :spi.c			                               */
/*  DATE        :Tue, Jun 08, 2017                                     */
/*  DESCRIPTION :SPI Program                                           */
/*  CPU TYPE    :                                                      */
/*                                                                     */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/*                                                                     */
/***********************************************************************/
#include "iodefine.h"
#include "mathf.h"
#include "sci.h"
#include "init.h"
#include "spi.h"
#include "parameters.h"
#include "glob_var.h"
#include "mytypedef.h"
#include "portdef.h"
#include "interface.h"
#include "odom.h"
#include "my_sin.h"
#include "my_cos.h"
#include "stdlib.h"

#define r_wall 3750
#define l_wall 3750
#define r_hosei 63 // 60
#define l_hosei 63
#define half_r_hosei 19
#define half_l_hosei 18
#define tar_min_len 40.06
#define tar_max_len 101.32
#define OFFSET_PRE 5
#define OFFSET_FOL 5

extern wait_ms(int wtime);

void straight(float len, float acc, float max_sp, float end_sp)
{
	char r_wall_check = 0, l_wall_check = 0, hosei_f = 0;
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	// tar_speed = 0;
	I_start_degree = 0;
	I_degree = 0;
	TH_R_len_mouse = 0;
	TH_L_len_mouse = 0;
	// 走行モードを直線にする
	run_mode = STRAIGHT_MODE;
	// 壁制御を有効にする
	con_wall.enable = false;
	/*if (con_wall.r_flag == 1 && con_wall.l_flag == 1)
	{
		con_wall.enable = true;
	}
	else
	{
		con_wall.enable = false;
	}*/
	// 目標距離をグローバル変数に代入する
	len_target = len;
	// 目標速度を設定
	end_speed = end_sp;
	// 加速度を設定
	accel = acc;
	// 最高速度を設定
	max_speed = max_sp;
	if (len_count == 0)
	{
		start_degree = degree;
	}

	// モータ出力をON
	MOT_POWER_ON;

	if ((end_speed != 0) && (len == SECTION))
	{
		r_wall_check = sen_r.is_wall;
		l_wall_check = sen_l.is_wall;
	}

	if (end_speed == 0)
	{ // 最終的に停止する場合

		// 減速処理を始めるべき位置まで加速、定速区間を続行
		while (((len_target - 10) - len_mouse) < 1000.0 * ((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed)) / (float)(2.0 * accel))
		{
		}

		// 減速処理開始
		accel = -acc; // 減速するために加速度を負の値にする

		while (len_mouse < len_target)
		{ // 停止したい距離の少し手前まで継続
			if ((sen_fr.value > r_wall) && (sen_fl.value > l_wall))
			{ // 目標の位置の5mm手前
				break;
			}

			// 一定速度まで減速したら最低駆動トルクで走行
			if (tar_speed <= MIN_SPEED)
			{ // 目標速度が最低速度になったら、加速度を0にする
				accel = 0;
				tar_speed = MIN_SPEED;
			}
		}
		accel = 0;
		tar_speed = 0;
		// 速度が0以下になるまで逆転する
		while (speed >= 0.0)
			;
	}
	else
	{
		// 減速処理を始めるべき位置まで加速、定速区間を続行
		while (((len_target - 10) - len_mouse) > 1000.0 * ((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed)) / (float)(2.0 * accel))
		{

			if (len == SECTION)
			{
				if ((sen_r.is_wall == false) && (r_wall_check == true) && (hosei_f == 0))
				{
					len_mouse = (len_mouse + r_hosei) / 2;
					hosei_f = 1;
				}
				if ((sen_l.is_wall == false) && (l_wall_check == true) && (hosei_f == 0))
				{
					len_mouse = (len_mouse + l_hosei) / 2;
					hosei_f = 1;
				}
			}
		}

		// 減速処理開始
		accel = -acc;
		// 減速するために加速度を負の値にする
		while (len_mouse < len_target)
		{ // 停止したい距離の少し手前まで継続

			// 一定速度まで減速したら最低駆動トルクで走行
			if (tar_speed <= end_speed)
			{ // 目標速度が最低速度になったら、加速度を0にする
				accel = 0;
				// tar_speed = end_speed;
			}
		}
	}
	// 加速度を0にする
	accel = 0;
	// 現在距離を0にリセット
	len_mouse = 0;

	if (len_target == 90)
	{
		len_count++;
	}
}

void back_straight(float len, float acc, float max_sp, float end_sp)
{ // 実験用
	char r_wall_check = 0, l_wall_check = 0, hosei_f = 0;
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	// tar_speed = 0;
	I_start_degree = 0;
	I_degree = 0;
	TH_R_len_mouse = 0;
	TH_L_len_mouse = 0;
	len_mouse = 0;
	// 走行モードを直線にする
	run_mode = BACK_STRAIGHT_MODE;
	// 壁制御を無効にする
	con_wall.enable = false;
	// 目標距離をグローバル変数に代入する
	len_target = len;
	// 目標速度を設定
	end_speed = end_sp;
	// 加速度を設定
	accel = acc;
	// 最高速度を設定
	max_speed = max_sp;
	/*if (len_count == 0)
	{
		start_degree = degree;
	}*/

	// モータ出力をON
	MOT_POWER_ON;

	if ((end_speed != 0) && (len == SECTION))
	{
		r_wall_check = sen_r.is_wall;
		l_wall_check = sen_l.is_wall;
	}

	if (end_speed == 0)
	{ // 最終的に停止する場合

		// 減速処理を始めるべき位置まで加速、定速区間を続行
		while (((len_target + 10) - len_mouse) < 1000.0 * ((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed)) / (float)(2.0 * accel))
			;
		// BEEP();
		//  減速処理開始
		accel = -acc; // 減速するために加速度を負の値にする

		while (len_mouse > len_target)
		{ // 停止したい距離の少し手前まで継続
			if (speed_new_r == 0 && speed_new_l == 0)
			{
				break;
			}
			// 一定速度まで減速したら最低駆動トルクで走行
			if (tar_speed >= -MIN_SPEED)
			{ // 目標速度が最低速度になったら、加速度を0にする
				accel = 0;
				tar_speed = -MIN_SPEED;
			}
		}
		wait_ms(100);
		accel = 0;
		tar_speed = 0;
		// 速度が0以下になるまで逆転する
		while (speed <= 0.0)
		{
			if (speed_new_r == 0 && speed_new_l == 0)
			{

				break;
			}
		}
	}
	else
	{
		// 減速処理を始めるべき位置まで加速、定速区間を続行
		while (((len_target + 10) - len_mouse) < 1000.0 * ((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed)) / (float)(2.0 * accel))
		{

			if (len == SECTION)
			{
				if ((sen_r.is_wall == false) && (r_wall_check == true) && (hosei_f == 0))
				{
					len_mouse = (len_mouse + r_hosei) / 2;
					hosei_f = 1;
				}
				if ((sen_l.is_wall == false) && (l_wall_check == true) && (hosei_f == 0))
				{
					len_mouse = (len_mouse + l_hosei) / 2;
					hosei_f = 1;
				}
			}
		}
		// BEEP();
		//  減速処理開始
		accel = -acc;
		// 減速するために加速度を負の値にする
		while (len_mouse > len_target)
		{ // 停止したい距離の少し手前まで継続

			// 一定速度まで減速したら最低駆動トルクで走行
			if (tar_speed <= end_speed)
			{ // 目標速度が最低速度になったら、加速度を0にする
				accel = 0;
				// tar_speed = end_speed;
			}
		}
	}
	// 加速度を0にする
	accel = 0;
	// 現在距離を0にリセット
	len_mouse = 0;

	/*if (len_target == 90)
	{
		len_count++;
	}*/
}

void turn(int deg, float ang_accel, float max_ang_velocity, short dir)
{
	wait_ms(WAIT_TIME);
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	tar_degree = 0;
	//degree = 0;

	float local_degree = 0;
	accel = 0;
	tar_speed = 0;
	tar_ang_vel = 0;
	// 走行モードをスラロームモードにする
	run_mode = TURN_MODE;
	con_wall.enable = false;

	// 回転方向定義
	TURN_DIR = dir;

	// 車体の現在角度を取得
	local_degree = degree;
	tar_degree = 0;

	sum = sum_len_mouse;
	count = len_count;
	sum_len_mouse = 0;
	len_count = 0;

	// 角加速度、加速度、最高角速度設定
	MOT_POWER_ON;
	if (dir == LEFT)
	{
		ang_acc = ang_accel; // 角加速度を設定
		max_ang_vel = max_ang_velocity;
		max_degree = deg;
		while ((max_degree - (degree - local_degree)) * PI / 180.0 > (tar_ang_vel * tar_ang_vel / (2.0 * ang_acc)))
			;
	}
	else if (dir == RIGHT)
	{
		ang_acc = -ang_accel; // 角加速度を設定
		max_ang_vel = -max_ang_velocity;
		max_degree = -deg;
		while (-(float)(max_degree - (degree - local_degree)) * PI / 180.0 > (float)(tar_ang_vel * tar_ang_vel / (float)(2.0 * -ang_acc)))
			;
	}

	// BEEP();
	// 角減速区間に入るため、角加速度設定
	MOT_POWER_ON;
	if (dir == LEFT)
	{
		ang_acc = -ang_accel; // 角加速度を設定
		// 減速区間走行
		while ((degree - local_degree) < max_degree)
		{
			if (tar_ang_vel < TURN_MIN_SPEED)
			{
				ang_acc = 0;
				tar_ang_vel = TURN_MIN_SPEED;
			}
		}

		ang_acc = 0;
		tar_ang_vel = 0;
		tar_degree = max_degree;
	}
	else if (dir == RIGHT)
	{
		ang_acc = +ang_accel; // 角加速度を設定
		// 減速区間走行
		while ((degree - local_degree) > max_degree)
		{
			if (-tar_ang_vel < TURN_MIN_SPEED)
			{
				ang_acc = 0;
				tar_ang_vel = -TURN_MIN_SPEED;
			}
		}
		ang_acc = 0;
		tar_ang_vel = 0;
		tar_degree = max_degree;
	}

	while (ang_vel >= 0.05 || ang_vel <= -0.05)
		;

	tar_ang_vel = 0;
	ang_acc = 0;
	// 現在距離を0にリセット
	len_mouse = 0;
	wait_ms(WAIT_TIME);
}

// 壁当て用
void check_straight(float len, float acc, float max_sp, float end_sp)
{

	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	I_start_degree = 0;
	I_degree = 0;
	TH_R_len_mouse = 0;
	TH_L_len_mouse = 0;
	len_mouse = 0;
	// 走行モードを直線にする
	run_mode = STRAIGHT_MODE;
	// 壁制御を有効にする
	con_wall.enable = false;

	// 左右エンコーダ微分値の現在値
	bef_diff_pulse_r = locate_r;
	bef_diff_pulse_l = locate_l;
	// 目標距離をグローバル変数に代入する
	len_target = len;
	// 目標速度を設定
	end_speed = end_sp;
	// 加速度を設定
	accel = acc;
	// 最高速度を設定
	max_speed = max_sp;
	start_degree = degree;

	// モータ出力をON
	MOT_POWER_ON;

	// 減速処理を始めるべき位置まで加速、定速区間を続行
	while (1)
		;

	accel = -acc * 2;
	while (speed >= 0.0)
		;

	accel = 0;
	tar_speed = 0;

	// 加速度を0にする
	accel = 0;
	// 現在距離を0にリセット
	len_mouse = 0;
}

void slalom(int deg, float ang_accel, float max_ang_velocity, short dir)
{

	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	I_start_degree = 0;
	I_degree = 0;
	TH_R_len_mouse = 0;
	TH_L_len_mouse = 0;
	r_len_mouse = 0;
	l_len_mouse = 0;
	degree = 0;
	len_mouse = 0;
	len_count = 0;

	float local_degree = 0;
	accel = 0;
	tar_speed = 0;
	tar_ang_vel = 0;
	ang_vel = 0;
	start_degree = 0;
	p_ang_vel = 0;
	max_ang_vel = 0;
	max_degree = 0;

	// 走行モードをスラロームモードにする
	run_mode = SLA_MODE;
	con_wall.enable = false;

	// 回転方向定義
	TURN_DIR = dir;

	// 車体の現在角度を取得
	local_degree = degree;
	tar_degree = 0;

	// 角加速度、加速度、最高角速度設定
	MOT_POWER_ON;

	len_target = OFFSET_PRE;
	accel = S_SEARCH_ACCEL;
	tar_speed = S_SEARCH_SPEED;

	// 減速処理を始めるべき位置まで加速、定速区間を続行

	while (len_mouse < len_target)
	{ // 停止したい距離の少し手前まで継続

		if (sen_fr.is_wall == true && sen_fl.is_wall == true)
		{
			if (sen_fr.value < 670 && sen_fl.value < 400) // 右前センサが670以下かつ、左前センサが420以下ならループ（どちらか一方でも超えたら抜ける）
			{
				while (sen_fr.value <= 670 && sen_fl.value <= 400)
					;
			}
		}
	}

	// 加速度を0にする
	accel = 0;
	// 現在距離を0にリセット
	len_mouse = 0;

	tar_speed = S_SEARCH_SPEED;

	if (dir == LEFT)
	{

		max_ang_vel = max_ang_velocity;
		max_degree = deg;
		ang_acc = ang_accel; // 角加速度を設定
		/*if(sen_fr.value == true && sen_fl.value == true){
		if (sen_fr.value > 700 && sen_fl.value > 420)  //右前センサが670以下かつ、左前センサが420以下ならループ（どちらか一方でも超えたら抜ける）
			{
				max_ang_vel = SLALOM_SPEED_2;
				max_degree = deg;
				ang_acc = SLALOM_ACCEL_2; // 角加速度を設定
			}
		}*/
		while ((max_degree - (degree - local_degree)) * PI / 180.0 > (tar_ang_vel * tar_ang_vel / (2.0 * ang_acc)))
			;
	}
	else if (dir == RIGHT)
	{
		max_ang_vel = -max_ang_velocity;
		max_degree = -deg;
		ang_acc = -ang_accel; // 角加速度を設定
		/*if(sen_fr.value == true && sen_fl.value == true){
		if (sen_fr.value > 700 && sen_fl.value > 420)  //右前センサが670以下かつ、左前センサが420以下ならループ（どちらか一方でも超えたら抜ける）
			{
				max_ang_vel = -SLALOM_SPEED_2;
				max_degree = -deg;
				ang_acc = -SLALOM_ACCEL_2; // 角加速度を設定
			}
		}*/
		while (-(float)(max_degree - (degree - local_degree)) * PI / 180.0 > (float)(tar_ang_vel * tar_ang_vel / (float)(2.0 * -ang_acc)))
			;
	}

	// BEEP();
	// 角減速区間に入るため、角加速度設定
	MOT_POWER_ON;
	if (dir == LEFT)
	{
		ang_acc = -ang_accel; // 角加速度を設定
		// 減速区間走行
		while ((degree - local_degree) < max_degree)
		{
			if (tar_ang_vel < SLA_MIN_SPEED)
			{
				ang_acc = 0;
				tar_ang_vel = SLA_MIN_SPEED;
			}
		}

		ang_acc = 0;
		tar_ang_vel = 0;
		tar_degree = max_degree;
	}
	else if (dir == RIGHT)
	{
		ang_acc = +ang_accel; // 角加速度を設定
		// 減速区間走行
		while ((degree - local_degree) > max_degree)
		{
			if (-tar_ang_vel < SLA_MIN_SPEED)
			{
				ang_acc = 0;
				tar_ang_vel = -SLA_MIN_SPEED;
			}
		}
		ang_acc = 0;
		tar_ang_vel = 0;
		tar_degree = max_degree;
	}

	// while(ang_vel >= 0.05 || ang_vel <= -0.05 );

	len_target = OFFSET_FOL;
	accel = SEARCH_ACCEL;
	tar_speed = S_SEARCH_SPEED;
	len_mouse = 0;

	// 減速処理を始めるべき位置まで加速、定速区間を続行
	while (len_mouse < len_target)
	{ // 停止したい距離の少し手前まで継続
		/*tar_ang_vel = 0;
		ang_vel = 0;
		I_tar_ang_vel = 0;
		I_ang_vel = 0;
		p_ang_vel = 0;*/
		if (sen_fr.is_wall == true && sen_fl.is_wall == true)
		{
			if (sen_fr.value < 360 && sen_fl.value < 208)
			{
				while (sen_fr.value <= 360 && sen_fl.value <= 208)
					;
			}
		}
	}

	// 加速度を0にする
	accel = 0;
	// 現在距離を0にリセット
	len_mouse = 0;

	tar_ang_vel = 0;
	ang_acc = 0;
	// 現在距離を0にリセット
	len_mouse = 0;
	degree = 0;

	tar_speed = S_SEARCH_SPEED;
}

void slalom_2(int deg, float ang_accel, float max_ang_velocity, short dir)
{

	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	I_start_degree = 0;
	I_degree = 0;
	TH_R_len_mouse = 0;
	TH_L_len_mouse = 0;
	r_len_mouse = 0;
	l_len_mouse = 0;
	degree = 0;
	len_mouse = 0;
	len_count = 0;

	float local_degree = 0;
	accel = 0;
	tar_speed = 0;
	tar_ang_vel = 0;
	ang_vel = 0;
	start_degree = 0;
	p_ang_vel = 0;
	max_ang_vel = 0;
	max_degree = 0;

	// 走行モードをスラロームモードにする
	run_mode = SLA_MODE;
	con_wall.enable = false;

	// 回転方向定義
	TURN_DIR = dir;

	// 車体の現在角度を取得
	local_degree = degree;
	tar_degree = 0;

	// 角加速度、加速度、最高角速度設定
	MOT_POWER_ON;

	len_target = OFFSET_PRE;
	accel = SEARCH_ACCEL;
	tar_speed = SEARCH_SPEED;

	// 減速処理を始めるべき位置まで加速、定速区間を続行

	while (len_mouse <= len_target)
	{ // 停止したい距離の少し手前まで継続

		if (sen_fr.is_wall == true && sen_fl.is_wall == true)
		{
			if (sen_fr.value < 670 && sen_fl.value < 400) // 右前センサが670以下かつ、左前センサが420以下ならループ（どちらか一方でも超えたら抜ける）
			{
				while (sen_fr.value <= 670 && sen_fl.value <= 400)
					;
			}
		}
	}

	// 加速度を0にする
	accel = 0;
	// 現在距離を0にリセット
	len_mouse = 0;

	tar_speed = SEARCH_SPEED;

	if (dir == LEFT)
	{

		max_ang_vel = max_ang_velocity;
		max_degree = deg;
		ang_acc = ang_accel; // 角加速度を設定
		/*if(sen_fr.value == true && sen_fl.value == true){
		if (sen_fr.value > 700 && sen_fl.value > 420)  //右前センサが670以下かつ、左前センサが420以下ならループ（どちらか一方でも超えたら抜ける）
			{
				max_ang_vel = SLALOM_SPEED_2;
				max_degree = deg;
				ang_acc = SLALOM_ACCEL_2; // 角加速度を設定
			}
		}*/
		while ((max_degree - (degree - local_degree)) * PI / 180.0 > (tar_ang_vel * tar_ang_vel / (2.0 * ang_acc)))
			;
	}
	else if (dir == RIGHT)
	{
		max_ang_vel = -max_ang_velocity;
		max_degree = -deg;
		ang_acc = -ang_accel; // 角加速度を設定
		/*if(sen_fr.value == true && sen_fl.value == true){
		if (sen_fr.value > 700 && sen_fl.value > 420)  //右前センサが670以下かつ、左前センサが420以下ならループ（どちらか一方でも超えたら抜ける）
			{
				max_ang_vel = -SLALOM_SPEED_2;
				max_degree = -deg;
				ang_acc = -SLALOM_ACCEL_2; // 角加速度を設定
			}
		}*/
		while (-(float)(max_degree - (degree - local_degree)) * PI / 180.0 > (float)(tar_ang_vel * tar_ang_vel / (float)(2.0 * -ang_acc)))
			;
	}

	// BEEP();
	// 角減速区間に入るため、角加速度設定
	MOT_POWER_ON;
	if (dir == LEFT)
	{
		ang_acc = -ang_accel; // 角加速度を設定
		// 減速区間走行
		while ((degree - local_degree) < max_degree)
		{
			if (tar_ang_vel < SLA_MIN_SPEED_2)
			{
				ang_acc = 0;
				tar_ang_vel = SLA_MIN_SPEED_2;
			}
		}

		ang_acc = 0;
		tar_ang_vel = 0;
		tar_degree = max_degree;
	}
	else if (dir == RIGHT)
	{
		ang_acc = +ang_accel; // 角加速度を設定
		// 減速区間走行
		while ((degree - local_degree) > max_degree)
		{
			if (-tar_ang_vel < SLA_MIN_SPEED_2)
			{
				ang_acc = 0;
				tar_ang_vel = -SLA_MIN_SPEED_2;
			}
		}
		ang_acc = 0;
		tar_ang_vel = 0;
		tar_degree = max_degree;
	}

	// while(ang_vel >= 0.05 || ang_vel <= -0.05 );

	len_target = OFFSET_FOL;
	accel = SEARCH_ACCEL;
	tar_speed = SEARCH_SPEED;
	len_mouse = 0;

	// 減速処理を始めるべき位置まで加速、定速区間を続行
	while (len_mouse <= len_target)
	{ // 停止したい距離の少し手前まで継続
		/*tar_ang_vel = 0;
		ang_vel = 0;
		I_tar_ang_vel = 0;
		I_ang_vel = 0;
		p_ang_vel = 0;*/
		if (sen_fr.is_wall == true && sen_fl.is_wall == true)
		{
			if (sen_fr.value < 400 && sen_fl.value < 230)
			{
				while (sen_fr.value <= 400 && sen_fl.value <= 230)
					;
			}
		}
	}

	// 加速度を0にする
	accel = 0;
	// 現在距離を0にリセット
	len_mouse = 0;

	tar_ang_vel = 0;
	ang_acc = 0;
	// 現在距離を0にリセット
	len_mouse = 0;
	degree = 0;

	tar_speed = SEARCH_SPEED;
}

// 実験用
void slalom_straight(float len, float acc, float max_sp, float end_sp)
{
	tar_ang_vel = 0;
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	tar_degree = 0;
	I_start_degree = 0;
	I_degree = 0;
	TH_R_len_mouse = 0;
	TH_L_len_mouse = 0;
	x_e = 0;
	y_e = 0;
	theta_e = 0;
	// 走行モードを直線にする
	run_mode = STRAIGHT_MODE;
	// 壁制御を有効にする
	con_wall.enable = false;
	// 目標距離をグローバル変数に代入する
	len_target = len;
	// 目標速度を設定
	end_speed = end_sp;
	// 加速度を設定
	accel = acc;
	// 最高速度を設定
	max_speed = max_sp;

	// degree = 0;
	start_degree = degree;

	// モータ出力をON
	MOT_POWER_ON;

	if (end_speed == 0)
	{ // 最終的に停止する場合

		// 減速処理を始めるべき位置まで加速、定速区間を続行
		while (((len_target - 20) - len_mouse) > 1000.0 * ((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed)) / (float)(2.0 * accel))
		{
		}

		// 減速処理開始
		accel = -acc; // 減速するために加速度を負の値にする

		while (len_mouse < len_target - 1)
		{ // 停止したい距離の少し手前まで継続

			// 一定速度まで減速したら最低駆動トルクで走行
			if (tar_speed <= MIN_SPEED)
			{ // 目標速度が最低速度になったら、加速度を0にする
				accel = 0;
				tar_speed = MIN_SPEED;
			}
		}
		accel = 0;
		tar_speed = 0;
		// 速度が0以下になるまで逆転する
		while (speed >= 0.0)
			;
	}
	else
	{
		// 減速処理を始めるべき位置まで加速、定速区間を続行
		while (((len_target - 10) - len_mouse) > 1000.0 * ((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed)) / (float)(2.0 * accel))
			;

		// 減速処理開始
		accel = -acc;
		// 減速するために加速度を負の値にする
		while (len_mouse < len_target)
		{ // 停止したい距離の少し手前まで継続

			// 一定速度まで減速したら最低駆動トルクで走行
			if (tar_speed <= end_speed)
			{ // 目標速度が最低速度になったら、加速度を0にする
				accel = 0;
				// tar_speed = end_speed;
			}
		}
	}
	// 加速度を0にする
	accel = 0;
	// 現在距離を0にリセット
	len_mouse = 0;

	degree = 0;
}

void slalom_straight_2(float len, float acc, float max_sp, float end_sp)
{
	char r_wall_check = 0, l_wall_check = 0, hosei_f = 0;
	tar_ang_vel = 0;
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	// tar_speed = 0;
	I_start_degree = 0;
	I_degree = 0;
	p_ang_vel = 0;
	TH_R_len_mouse = 0;
	TH_L_len_mouse = 0;
	// 走行モードを直線にする
	run_mode = STRAIGHT_MODE;
	// 壁制御を有効にする
	con_wall.enable = false;
	/*if (con_wall.r_flag == 0 && con_wall.l_flag == 0)
	{ // 両壁無しの時のみfalse
		con_wall.enable = false;
	}
	else
	{
		con_wall.enable = true;
	}*/
	// 目標距離をグローバル変数に代入する
	len_target = len;
	// 目標速度を設定
	end_speed = end_sp;
	// 加速度を設定
	accel = acc;
	// 最高速度を設定
	max_speed = max_sp;
	if (len_count == 0)
	{
		start_degree = degree;
	}

	len_mouse = 0;

	// モータ出力をON
	MOT_POWER_ON;

	if ((end_speed != 0) && (len == SECTION))
	{
		r_wall_check = sen_r.is_wall;
		l_wall_check = sen_l.is_wall;
	}
	/*else if ((end_speed != 0) && (len == HALF_SECTION))
	{
		r_wall_check = sen_r.is_wall;
		l_wall_check = sen_l.is_wall;
	}
	else
	{
	}*/

	if (end_speed == 0)
	{ // 最終的に停止する場合

		// 減速処理を始めるべき位置まで加速、定速区間を続行
		while (((len_target - 10) - len_mouse) > 1000.0 * ((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed)) / (float)(2.0 * accel))
		{
			if ((sen_fr.value > r_wall) && (sen_fl.value > l_wall))
			{ // 壁の5mm手前
				break;
			}
		}

		// 減速処理開始
		accel = -acc; // 減速するために加速度を負の値にする

		while (len_mouse < len_target)
		{ // 停止したい距離の少し手前まで継続
			if ((sen_fr.value > r_wall) && (sen_fl.value > l_wall))
			{ // 壁の5mm手前
				break;
			}
			/*else if (((sen_r.is_wall == false) && (TH_R_len_mouse > r_hosei)) || ((sen_l.is_wall == false) && (TH_L_len_mouse > l_hosei)))
			{
				break;
			}*/
			// 一定速度まで減速したら最低駆動トルクで走行
			if (tar_speed <= MIN_SPEED)
			{ // 目標速度が最低速度になったら、加速度を0にする
				accel = 0;
				tar_speed = MIN_SPEED;
			}
		}
		accel = 0;
		tar_speed = 0;
		// 速度が0以下になるまで逆転する
		while (speed >= 0.0)
			;
	}
	else
	{
		// 減速処理を始めるべき位置まで加速、定速区間を続行
		while (((len_target - 10) - len_mouse) > 1000.0 * ((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed)) / (float)(2.0 * accel))
		{

			/*if (len == SECTION)
			{
				if ((sen_r.is_wall == false) && (r_wall_check == true) && (hosei_f == 0))
				{
					len_mouse = r_hosei;
					hosei_f = 1;
				}
				if ((sen_l.is_wall == false) && (l_wall_check == true) && (hosei_f == 0))
				{
					len_mouse = l_hosei;
					hosei_f = 1;
				}
			}*/
			/*if (len == HALF_SECTION)
			{
				if ((sen_r.is_wall == false) && (r_wall_check == true) && (hosei_f == 0))
				{
					len_mouse = half_r_hosei;
					hosei_f = 1;
				}
				if ((sen_l.is_wall == false) && (l_wall_check == true) && (hosei_f == 0))
				{
					len_mouse = half_l_hosei;
					hosei_f = 1;
				}
			}*/
			if (con_wall.r_flag == 0 && con_wall.l_flag == 0)
			{
				con_wall.enable = false;
			}
		}

		// 減速処理開始
		accel = -acc;
		// 減速するために加速度を負の値にする
		while (len_mouse < len_target)
		{ // 停止したい距離の少し手前まで継続

			if ((len_count >= 4) && (len_mouse > (len_target - 3)))
			{
				break;
			}
			// 一定速度まで減速したら最低駆動トルクで走行
			if (tar_speed <= end_speed)
			{ // 目標速度が最低速度になったら、加速度を0にする
				accel = 0;
				// tar_speed = end_speed;
			}
			/*if (len == HALF_SECTION)
			{
				if ((sen_r.is_wall == false) && (r_wall_check == true) && (hosei_f == 0))
				{
					len_mouse = half_r_hosei;
					hosei_f = 1;
				}
				if ((sen_l.is_wall == false) && (l_wall_check == true) && (hosei_f == 0))
				{
					len_mouse = half_l_hosei;
					hosei_f = 1;
				}
			}*/
			if (con_wall.r_flag == 0 && con_wall.l_flag == 0)
			{
				con_wall.enable = false;
			}
		}
	}
	// 加速度を0にする
	accel = 0;
	// 現在距離を0にリセット
	len_mouse = 0;

	if (len_target == SECTION)
	{
		len_count++;
	}
}

void make_traject(short dir)
{
	// TURN_DIR = dir;

	switch (now_dir)
	{

	case north:
		if (dir == LEFT)
		{
			tar_x = s90[cnt].x_pos;
			tar_y = s90[cnt].y_pos;
			tar_th = s90[cnt].deg;
			tar_d_th = (s90[cnt].d_theta * (PI / 180.0)) * 1000;
			K_x = Kx;
			K_y = Ky;
			K_th = Ktheta;
		}
		else if (dir == RIGHT)
		{
			tar_x = s90[cnt].x_pos;
			tar_y = -s90[cnt].y_pos;
			tar_th = -s90[cnt].deg;
			tar_d_th = -(s90[cnt].d_theta * (PI / 180.0)) * 1000;
			K_x = Kx;
			K_y = Ky;
			K_th = Ktheta;
		}
		break;

	case east:
		if (dir == LEFT)
		{
			tar_x = s90[cnt].y_pos;
			tar_y = -s90[cnt].x_pos;
			tar_th = s90[cnt].deg;
			tar_d_th = (s90[cnt].d_theta * (PI / 180.0)) * 1000;
			K_x = Ky;
			K_y = Kx;
			K_th = Ktheta;
		}
		else if (dir == RIGHT)
		{
			tar_x = -s90[cnt].y_pos;
			tar_y = -s90[cnt].x_pos;
			tar_th = -s90[cnt].deg;
			tar_d_th = -(s90[cnt].d_theta * (PI / 180.0)) * 1000;
			K_x = Ky;
			K_y = Kx;
			K_th = Ktheta;
		}
		break;

	case south:
		if (dir == LEFT)
		{
			tar_x = -s90[cnt].x_pos;
			tar_y = -s90[cnt].y_pos;
			tar_th = s90[cnt].deg;
			tar_d_th = (s90[cnt].d_theta * (PI / 180.0)) * 1000;
			K_x = -Kx; //-kx
			K_y = -Ky; //-ky
			K_th = Ktheta;
		}
		else if (dir == RIGHT)
		{
			tar_x = -s90[cnt].x_pos;
			tar_y = s90[cnt].y_pos;
			tar_th = -s90[cnt].deg;
			tar_d_th = -(s90[cnt].d_theta * (PI / 180.0)) * 1000;
			K_x = -Kx; //-kx
			K_y = -Ky; //-ky
			K_th = Ktheta;
		}
		break;

	case west:
		if (dir == LEFT)
		{
			tar_x = -s90[cnt].y_pos;
			tar_y = s90[cnt].x_pos;
			tar_th = s90[cnt].deg;
			tar_d_th = (s90[cnt].d_theta * (PI / 180.0)) * 1000;
			K_x = Ky;
			K_y = Kx;
			K_th = Ktheta;
		}
		else if (dir == RIGHT)
		{
			tar_x = s90[cnt].y_pos;
			tar_y = s90[cnt].x_pos;
			tar_th = -s90[cnt].deg;
			tar_d_th = -(s90[cnt].d_theta * (PI / 180.0)) * 1000;
			K_x = Ky;
			K_y = Kx;
			K_th = Ktheta;
		}
		break;

		/*case north:
		if (dir == LEFT)
		{
			tar_x = s90[cnt].x_pos;
			tar_y = s90[cnt].y_pos;
			tar_th = s90[cnt].deg;
		}else if(dir == RIGHT){
			tar_x = s90[cnt].x_pos;
			tar_y = -s90[cnt].y_pos;
			tar_th = -s90[cnt].deg;
		}

		case east:
		if (dir == LEFT)
		{
			tar_x = -s90[cnt].y_pos;
			tar_y = s90[cnt].x_pos;
			tar_th = s90[cnt].deg;
		}else if(dir == RIGHT){
			tar_x = -s90[cnt].y_pos;
			tar_y = -s90[cnt].x_pos;
			tar_th = -s90[cnt].deg;
		}

		case south:
		if (dir == LEFT)
		{
			tar_x = -s90[cnt].x_pos;
			tar_y = -s90[cnt].y_pos;
			tar_th = s90[cnt].deg;
		}else if(dir == RIGHT){
			tar_x = -s90[cnt].x_pos;
			tar_y = s90[cnt].y_pos;
			tar_th = -s90[cnt].deg;
		}

		case west:
		if (dir == LEFT)
		{
			tar_x = s90[cnt].y_pos;
			tar_y = -s90[cnt].x_pos;
			tar_th = s90[cnt].deg;
		}else if(dir == RIGHT){
			tar_x = s90[cnt].y_pos;
			tar_y = s90[cnt].x_pos;
			tar_th = -s90[cnt].deg;
		}
		*/
	}
}

void orb_follow_sla(short dir)
{
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	I_start_degree = 0;
	I_degree = 0;
	TH_R_len_mouse = 0;
	TH_L_len_mouse = 0;
	r_len_mouse = 0;
	l_len_mouse = 0;
	// degree = 0;
	// len_mouse = 0;
	len_count = 0;

	float local_degree = 0;
	accel = 0;
	tar_speed = 0;
	tar_ang_vel = 0;
	ang_vel = 0;
	start_degree = 0;
	p_ang_vel = 0;
	p_speed = 0;
	max_ang_vel = 0;
	max_degree = 0;
	tar_degree = 0;

	x_e = 0;
	y_e = 0;
	theta_e = 0;
	th_e = 0;

	run_mode = ORBIT_FOLLOWING_MODE;
	TURN_DIR = dir;
	con_wall.enable = false;
	cnt = 0;

	last_x_pos = x_position;
	last_y_pos = y_position;
	last_degree = degree;

	max_speed = S_SEARCH_SPEED;
	// accel = S_SEARCH_ACCEL;
	// ang_acc = SLALOM_ACCEL;
	tar_speed_s = S_SEARCH_SPEED;
	// tar_ang_vel_s = SLALOM_SPEED;

	TURN_DIR = dir;
	if (dir == LEFT)
	{
		max_ang_vel = SLALOM_SPEED;
	}
	else if (dir == RIGHT)
	{
		max_ang_vel = -SLALOM_SPEED;
	}

	MOT_POWER_ON;
	cnt = 0;

	while (cnt < 306)
	{

		// tar_speed = tar_speed_s;

		if (theta_e < 0)
		{
			tar_speed = tar_speed_s * ((float)cos_table[(int)th_e] / 10000.0) + K_x * x_e;
			// max_speed = tar_speed_s;
			tar_ang_vel = tar_d_th + tar_speed_s * ((K_y * y_e) + (Ktheta * (-((float)sin_table[(int)th_e] / 10000.0))));
			// max_ang_vel = tar_d_th;
		}
		else
		{

			tar_speed = tar_speed_s * ((float)cos_table[(int)theta_e] / 10000.0) + K_x * x_e;
			// max_speed = tar_speed_s;
			tar_ang_vel = tar_d_th + tar_speed_s * ((K_y * y_e) + (Ktheta * ((float)sin_table[(int)theta_e] / 10000.0)));
			// max_ang_vel = tar_d_th;
		}

		if (cnt % 4 == 0)
		{
			make_traject(dir);

			/*if (now_dir == 1 || now_dir == 3)
			{
				if (last_x_pos < 0)
				{
					x_e = (last_x_pos - tar_x) - x_position;
				}
				else
				{
					x_e = (last_x_pos + tar_x) - x_position;
				}
				if (last_y_pos < 0)
				{
					y_e = (last_y_pos - tar_y) - y_position;
				}
				else
				{
					y_e = (last_y_pos + tar_y) - y_position;
				}
			}
			else
			{
				x_e = (last_x_pos + tar_x) - x_position;
				y_e = (last_y_pos + tar_y) - y_position;
			}*/
			    x_e = (last_x_pos + tar_x) - x_position;
				y_e = (last_y_pos + tar_y) - y_position;

			theta_e = (last_degree + tar_th) - degree;
			if (theta_e < 0)
			{
				th_e = -theta_e;
			}
		}
		if (dir == LEFT)
		{
			if (last_degree + 90 < degree)
			{
				// theta_e = 0;
				tar_ang_vel = 0;

				// tar_d_th = 0;
			}
		}
		else if (dir == RIGHT)
		{
			if (last_degree - 90 > degree)
			{
				// tar_d_th = 0;
				// theta_e = 0;
				tar_ang_vel = 0;
			}
		}

		/*if(cnt < 18 || cnt > 238){
			tar_ang_vel = 0;


		}*/
		/*if(last_degree + 20 < degree){
			tar_ang_vel = max_ang_vel;
		}else if(last_degree + 70 < degree){
			tar_ang_vel = 0;
		}*/
	}

	cnt = 0;

	x_e = 0;
	y_e = 0;
	theta_e = 0;
	tar_d_th = 0;
	len_mouse = 0;
	len_target = 2;

	tar_ang_vel = 0;
	max_ang_vel = 0;

	accel = S_SEARCH_ACCEL;
	tar_speed = S_SEARCH_SPEED;

	while (len_target > len_mouse)
		;

	len_mouse = 0;
	tar_speed = S_SEARCH_SPEED;

	len_count = 0;
}
