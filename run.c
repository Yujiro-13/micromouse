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
	//���s���[�h�𒼐��ɂ���
	run_mode = STRAIGHT_MODE;
	//�ǐ����L���ɂ���
	con_wall.enable = true;
	//�ڕW�������O���[�o���ϐ��ɑ������
	len_target = len;
	//�ڕW���x��ݒ�
	end_speed = end_sp;
	//�����x��ݒ�
	accel = acc;
	//�ō����x��ݒ�
	max_speed = max_sp;
	
	//���[�^�o�͂�ON
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

	if(end_speed == 0){	//�ŏI�I�ɒ�~����ꍇ
	    //BEEP();
		//�����������n�߂�ׂ��ʒu�܂ŉ����A�葬��Ԃ𑱍s
		while( ((len_target -10) - len_mouse) >  1000.0*((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed))/(float)(2.0*accel));
		//���������J�n
		accel = -acc*2;					//�������邽�߂ɉ����x�𕉂̒l�ɂ���	
		while(len_mouse < len_target -5){		//��~�����������̏�����O�܂Ōp��
			//��葬�x�܂Ō���������Œ�쓮�g���N�ő��s
			if(tar_speed <= MIN_SPEED){	//�ڕW���x���Œᑬ�x�ɂȂ�����A�����x��0�ɂ���
				accel = 0;
				tar_speed = MIN_SPEED;
			}
			
		}
		accel = 0;
		tar_speed = 0;
		//���x��0�ȉ��ɂȂ�܂ŋt�]����
		while(speed >= 0.0);
			
	}else{
		//�����������n�߂�ׂ��ʒu�܂ŉ����A�葬��Ԃ𑱍s
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
		    
		//���������J�n
		accel = -acc*3;					//�������邽�߂ɉ����x�𕉂̒l�ɂ���	
		while(len_mouse < len_target + 10){		//��~�����������̏�����O�܂Ōp��
		    //BEEP();
		    if((sen_fr.is_wall==false) && (sen_fl.is_wall==false) && (len_mouse > len_target)) {//not frontwall
				break;
			}
			if((sen_fr.value > RIGHT_90) && (sen_fl.value > LEFT_90 ) && (len_mouse > (len_target -5))) {//mokuhyou 5mm temae
				break;
			}
			//��葬�x�܂Ō���������Œ�쓮�g���N�ő��s
			if(tar_speed <= end_speed){	//�ڕW���x���Œᑬ�x�ɂȂ�����A�����x��0�ɂ���
				accel = 0;
				//tar_speed = end_speed;
			}
			if(sen_fl.value==FL_BORDER && sen_fr.value==FR_BORDER){
					break;
				}
			}
	
	}
	
	
	

	//�����x��0�ɂ���
	accel = 0;
	//���݋�����0�Ƀ��Z�b�g
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
	//���s���[�h���X�����[�����[�h�ɂ���
	run_mode = TURN_MODE;

	//��]������`
	TURN_DIR = dir;	
	
	//�ԑ̂̌��݊p�x���擾
	local_degree = degree;
	tar_degree = 0;
	
	//�p�����x�A�����x�A�ō��p���x�ݒ�
	MOT_POWER_ON;
	if(dir == LEFT){
		ang_acc = ang_accel;			//�p�����x��ݒ�
		max_ang_vel = max_ang_velocity;
		max_degree = deg;
		while( (max_degree - (degree - local_degree))*PI/180.0 > (tar_ang_vel*tar_ang_vel/(2.0 * ang_acc)));
		
	}else if(dir == RIGHT){
		ang_acc = -ang_accel;			//�p�����x��ݒ�
		max_ang_vel = -max_ang_velocity;
		max_degree = -deg;
		while(-(float)(max_degree - (degree - local_degree))*PI/180.0 > (float)(tar_ang_vel*tar_ang_vel/(float)(2.0 * -ang_acc)));
	}
    
	
	//BEEP();
	//�p������Ԃɓ��邽�߁A�p�����x�ݒ�
	MOT_POWER_ON;
	if(dir == LEFT){
		ang_acc = -ang_accel;			//�p�����x��ݒ�
		//������ԑ��s
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
		ang_acc = +ang_accel;			//�p�����x��ݒ�
		//������ԑ��s
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
	//���݋�����0�Ƀ��Z�b�g
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
	//���s���[�h�𒼐��ɂ���
	run_mode = STRAIGHT_MODE;
	//�ǐ����L���ɂ���
	con_wall.enable = false;
	//�ڕW���x��ݒ�
	end_speed = end_sp;
	//�ō����x��ݒ�
	max_speed = end_sp;
	//access = 0;
	
	//���[�^�o�͂�ON
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
