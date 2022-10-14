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

extern wait_ms(int wtime);

#define r_hosei 31.168919
#define l_hosei 32.006469
void straight(float len, float acc, float max_sp, float end_sp){
	char r_wall_check=0, l_wall_check=0, hosei_f=0;
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	//走行モードを直線にする
	run_mode = STRAIGHT_MODE;
	//壁制御を有効にする
	con_wall.enable = true;
	//目標距離をグローバル変数に代入する
	len_target = len;
	//目標速度を設定
	end_speed = end_sp;
	//加速度を設定
	accel = acc;
	//最高速度を設定
	max_speed = max_sp;
	
	//モータ出力をON
	MOT_POWER_ON;

	if((end_speed!=0) && (len==SECTION)){
		r_wall_check = sen_r.is_wall;
		l_wall_check = sen_l.is_wall;
	}
	
   
    /*if(sen_fl.value > FL_BORDER && sen_fr.value > FR_BORDER){
		//BEEP();
		accel = 0;
		tar_speed = 0;			
	}
     while(speed >= 0.0);
	if(sen_fl.value==FL_BORDER && sen_fr.value==FR_BORDER && speed != 0){
		accel = -acc;
		tar_speed = 0;			
	}*/

	if(end_speed == 0){	//最終的に停止する場合
	    //BEEP();
		//減速処理を始めるべき位置まで加速、定速区間を続行
		while( ((len_target -10) - len_mouse) >  1000.0*((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed))/(float)(2.0*accel));
		//減速処理開始
		accel = -acc*2;					//減速するために加速度を負の値にする	
		while(len_mouse < len_target -5){		//停止したい距離の少し手前まで継続
			//一定速度まで減速したら最低駆動トルクで走行
			if(tar_speed <= MIN_SPEED){	//目標速度が最低速度になったら、加速度を0にする
				accel = 0;
				tar_speed = MIN_SPEED;
			}
			
		}
		accel = 0;
		tar_speed = 0;
		//速度が0以下になるまで逆転する
		while(speed >= 0.0);
			
	}else{
		//減速処理を始めるべき位置まで加速、定速区間を続行
		while( ((len_target-10) - len_mouse) >  1000.0*((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed))/(float)(2.0*accel));
		    //BEEP();
		    /* if(len==SECTION){
				if((sen_r.is_wall==false) && (r_wall_check==true) && (hosei_f==0)){
					len_mouse = (len_mouse*0.5+r_hosei*0.5);
					hosei_f=1;
				}
				if((sen_l.is_wall==false) && (l_wall_check==true) && (hosei_f==0)){
					len_mouse = (len_mouse*0.5+l_hosei*0.5);
					hosei_f=1;
				}
			 }*/
		    
		//減速処理開始
		accel = -acc*3;					//減速するために加速度を負の値にする	
		while(len_mouse < len_target + 10){		//停止したい距離の少し手前まで継続
		    //BEEP();
		    if((sen_fr.is_wall==false) && (sen_fl.is_wall==false) && (len_mouse > len_target)) {//not frontwall
				break;
			}
			if((sen_fr.value > RIGHT_90) && (sen_fl.value > LEFT_90 ) && (len_mouse > (len_target -5))) {//mokuhyou 5mm temae
				break;
			}
			//一定速度まで減速したら最低駆動トルクで走行
			if(tar_speed <= end_speed){	//目標速度が最低速度になったら、加速度を0にする
				accel = 0;
				//tar_speed = end_speed;
			}
			if(sen_fl.value==FL_BORDER && sen_fr.value==FR_BORDER){
					break;
				}
			}
	
	}
	
	
	

	//加速度を0にする
	accel = 0;
	//現在距離を0にリセット
	len_mouse = 0;
}

void turn(int deg, float ang_accel, float max_ang_velocity, short dir){
	wait_ms(WAIT_TIME);
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	tar_degree = 0;

	float	local_degree = 0;
	accel = 0;
	tar_speed = 0;
	tar_ang_vel = 0;
	//走行モードをスラロームモードにする
	run_mode = TURN_MODE;

	//回転方向定義
	TURN_DIR = dir;	
	
	//車体の現在角度を取得
	local_degree = degree;
	tar_degree = 0;
	
	//角加速度、加速度、最高角速度設定
	MOT_POWER_ON;
	if(dir == LEFT){
		ang_acc = ang_accel;			//角加速度を設定
		max_ang_vel = max_ang_velocity;
		max_degree = deg;
		while( (max_degree - (degree - local_degree))*PI/180.0 > (tar_ang_vel*tar_ang_vel/(2.0 * ang_acc)));
		
	}else if(dir == RIGHT){
		ang_acc = -ang_accel;			//角加速度を設定
		max_ang_vel = -max_ang_velocity;
		max_degree = -deg;
		while(-(float)(max_degree - (degree - local_degree))*PI/180.0 > (float)(tar_ang_vel*tar_ang_vel/(float)(2.0 * -ang_acc)));
	}
    
	
	//BEEP();
	//角減速区間に入るため、角加速度設定
	MOT_POWER_ON;
	if(dir == LEFT){
		ang_acc = -ang_accel;			//角加速度を設定
		//減速区間走行
		while((degree - local_degree) < max_degree){
			if(tar_ang_vel < TURN_MIN_SPEED){
				ang_acc = 0;
				tar_ang_vel = TURN_MIN_SPEED;
			}
		}
		
		ang_acc = 0;
		tar_ang_vel = 0;
		tar_degree = max_degree;
		
	}else if(dir == RIGHT){
		ang_acc = +ang_accel;			//角加速度を設定
		//減速区間走行
		while((degree - local_degree) > max_degree){
			if(-tar_ang_vel < TURN_MIN_SPEED){
				ang_acc = 0;
				tar_ang_vel = -TURN_MIN_SPEED;
			}
		}
		ang_acc = 0;
		tar_ang_vel = 0;
		tar_degree = max_degree;


	}
	
	while(ang_vel >= 0.05 || ang_vel <= -0.05 );
	
	tar_ang_vel = 0;
	ang_acc = 0;
	//現在距離を0にリセット
	len_mouse = 0;
	wait_ms(WAIT_TIME);
}

float r_adjust_len, l_adjust_len;
void check_straight(float end_sp)
{
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	//走行モードを直線にする
	run_mode = STRAIGHT_MODE;
	//壁制御を有効にする
	con_wall.enable = false;
	//目標速度を設定
	end_speed = end_sp;
	//最高速度を設定
	max_speed = end_sp;
	//access = 0;
	
	//モータ出力をON
	MOT_POWER_ON;	
	r_adjust_len = l_adjust_len = 0;
	while(len_mouse < 90.0){
		if((sen_r.is_wall==false) && (r_adjust_len==0)){
			r_adjust_len =  len_mouse;
		}
		if((sen_l.is_wall==false) && (l_adjust_len==0)){
			l_adjust_len = len_mouse;
		}
	}
	len_mouse = 0;
}

void get_adjust_len(float * r_len, float * l_len){
	*r_len = r_adjust_len;
	*l_len = l_adjust_len;
}
