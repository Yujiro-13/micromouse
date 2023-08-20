/***********************************************************************/
/*                                                                     */
/*  FILE        :Main.c or Main.cpp                                    */
/*  DATE        :2017/6/27	                                       */
/*  DESCRIPTION :Main Program                                          */
/*  CPU TYPE    :RX631 48P                                             */
/*                                                                     */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/*                                                                     */
/***********************************************************************/
//#include "typedefine.h"
#include "iodefine.h"
#include "mathf.h"
#include "sci.h"
#include "init.h"
#include "spi.h"
#include "i2c.h"
#include "parameters.h"
#include "glob_var.h"
#include "run.h"
#include "interface.h"
#include "DataFlash.h"
#include "portdef.h"
#include "fast.h"
#include "search.h"
#include "math.h"


#ifdef __cplusplus
//#include <ios>                        // Remove the comment when you use ios
//_SINT ios_base::Init::init_cnt;       // Remove the comment when you use ios
#endif

void main(void);
#ifdef __cplusplus
extern "C" {
void abort(void);
}
#endif


extern void wait_ms(int wtime);
extern void adjust(void);

void main(void)
{

	init_all();
	unsigned long i = 0;
	//int i = 0;
	int flash_time = 0;
	char flash;
	short ad_mode = 1;
	

	
	//�u�U�[
	//BEEP();
	//�ŏ���0���Ă���
	speed_r=0;
	speed_l=0;
	
	//�N�����̃��O�͂Ƃ�Ȃ�
	log_flag = 0;
	short mode = 1;
	while(1){
		I_tar_ang_vel = 0;
		I_ang_vel = 0;
		I_tar_speed = 0;
		I_speed = 0;

		switch(mode){
			
			case 1:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	O	X	X	X	*
				*					*
				*****************************************/
				
				//�Z���T�[�̑O�Ɏ���������ăX�^�[�g
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					BEEP();
					degree = 0;
					timer = 0;
					log_timer = 0;
					gyro_get_ref();
					BEEP();
					mypos.x = mypos.y = 0;			//���W��������
					mypos.dir = north;			//���p��������
					log_flag = 1;
					log_timer = 0;
					search_adachi(GOAL_X,GOAL_Y,GOAL_X1,GOAL_Y1,GOAL_X2,GOAL_Y2,GOAL_X3,GOAL_Y3);		//�S�[���܂ő����@
					if(sen_r.is_wall == true && sen_l.is_wall == true && sen_fr.is_wall == true && sen_fl.is_wall == true){
                    turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
                    back_straight(-28,-SEARCH_ACCEL,-SEARCH_SPEED,0);
					straight(21,SEARCH_ACCEL,SEARCH_SPEED,0);
                    turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
                    back_straight(-28,-SEARCH_ACCEL,-SEARCH_SPEED,0);
					straight(18,S_SEARCH_ACCEL,S_SEARCH_SPEED,0);
                    }
                    else if(sen_fr.is_wall == true && sen_fl.is_wall ==true){
                         turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
                         back_straight(-28,-SEARCH_ACCEL,-SEARCH_SPEED,0);
				         straight(18,S_SEARCH_ACCEL,S_SEARCH_SPEED,0);
                    }     
                    else{
                        turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
                        }			                    	//�S�[��������180�x��]����
					mypos.dir = (mypos.dir+6) % 4;		//���p���X�V
					map_write();
					BEEP();
					wait_ms(100);
					BEEP();//�S�[���������Ƃ��A�s�[��
					wait_ms(100);
					BEEP();//�S�[���������Ƃ��A�s�[��
					search_adachi(0,0,0,0,0,0,0,0);			//�X�^�[�g�n�_�܂ő����@�ŋA���Ă���	
					turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
					MOT_POWER_OFF;
					map_write();
					log_flag = 0;
					BEEP();
				}
				
				break;
				
			case 2:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	X	O	X	X	*
				*					*
				*****************************************/	
			
				//�Z���T�[�̑O�Ɏ���������ăX�^�[�g
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					BEEP();
					degree = 0;
					timer = 0;
					log_timer = 0;
					gyro_get_ref();
					BEEP();
					mypos.x = mypos.y = 0;			//���W��������
					mypos.dir = north;			//���p��������
					log_flag = 1;
					log_timer = 0;
					slalom_search_adachi(GOAL_X,GOAL_Y,GOAL_X1,GOAL_Y1,GOAL_X2,GOAL_Y2,GOAL_X3,GOAL_Y3);		//�S�[���܂ő����@
					if(sen_r.is_wall == true && sen_l.is_wall == true && sen_fr.is_wall == true && sen_fl.is_wall == true){
                    turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
                    back_straight(-28,-SEARCH_ACCEL,-SEARCH_SPEED,0);
					straight(21,SEARCH_ACCEL,SEARCH_SPEED,0);
                    turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
                    back_straight(-28,-SEARCH_ACCEL,-SEARCH_SPEED,0);
					straight(18,S_SEARCH_ACCEL,S_SEARCH_SPEED,0);
                }
                else if(sen_fr.is_wall == true && sen_fl.is_wall ==true){
                   turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
                   back_straight(-28,-SEARCH_ACCEL,-SEARCH_SPEED,0);
				   straight(18,S_SEARCH_ACCEL,S_SEARCH_SPEED,0);
                }
                else{
                    turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
                    }		                  			//�S�[��������180�x��]����
					mypos.dir = (mypos.dir+6) % 4;		//���p���X�V
					map_write();
					BEEP();
					wait_ms(100);
					BEEP();//�S�[���������Ƃ��A�s�[��
					wait_ms(100);
					BEEP();//�S�[���������Ƃ��A�s�[��
					slalom_search_adachi(0,0,0,0,0,0,0,0);			//�X�^�[�g�n�_�܂ő����@�ŋA���Ă���
					turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);	
					MOT_POWER_OFF;
					map_write();
					log_flag = 0;
					BEEP();
					wait_ms(500);
				}
				
				break;
				
			case 3:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	O	O	X	X	*
				*					*
				*****************************************/
			
				//�Z���T�[�̑O�Ɏ���������ăX�^�[�g
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					BEEP();
					degree = 0;
					timer = 0;
					log_timer = 0;
					gyro_get_ref();
					BEEP();
					mypos.x = mypos.y = 0;			//���W��������
					mypos.dir = north;			//���p��������
					log_flag = 1;
					log_timer = 0;
					slalom_search_adachi_2(GOAL_X,GOAL_Y,GOAL_X1,GOAL_Y1,GOAL_X2,GOAL_Y2,GOAL_X3,GOAL_Y3);		//�S�[���܂ő����@
					/*if(sen_r.is_wall == true && sen_l.is_wall == true && sen_fr.is_wall == true && sen_fl.is_wall == true){
                    turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
                    back_straight(-28,-SEARCH_ACCEL,-SEARCH_SPEED,0);
					straight(21,SEARCH_ACCEL,SEARCH_SPEED,0);
                    turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
                    back_straight(-28,-SEARCH_ACCEL,-SEARCH_SPEED,0);
					straight(18,SEARCH_ACCEL,SE
					ARCH_SPEED,0);
                }
                else if(sen_fr.is_wall == true && sen_fl.is_wall ==true){
                   turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
                   back_straight(-28,-SEARCH_ACCEL,-SEARCH_SPEED,0);
				   straight(18,SEARCH_ACCEL,SEARCH_SPEED,0);
                }
                else{
                    turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
                    }*/		                  			
					turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);     //�S�[��������180�x��]����
					mypos.dir = (mypos.dir+6) % 4;		//���p���X�V
					map_write();
					BEEP();
					wait_ms(100);
					BEEP();//�S�[���������Ƃ��A�s�[��
					wait_ms(100);
					BEEP();//�S�[���������Ƃ��A�s�[��
					slalom_search_adachi_2(0,0,0,0,0,0,0,0);			//�X�^�[�g�n�_�܂ő����@�ŋA���Ă���
					turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);	
					MOT_POWER_OFF;
					map_write();
					log_flag = 0;
					BEEP();
					wait_ms(500);
				}
				
				break;
				
			case 4:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	X	X	O	X	*
				*					*
				*****************************************/
			
				//�Z���T�[�̑O�Ɏ���������ăX�^�[�g
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					BEEP();
					map_copy();
					degree = 0;
					timer = 0;
					gyro_get_ref();
					BEEP();
					mypos.x = mypos.y = 0;			//���W��������
					mypos.dir = north;			//���p��������
					log_flag = 1;
					log_timer = 0;
					fast_slalom_run(GOAL_X,GOAL_Y);		//�S�[���܂ő����@
					turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);			//�S�[��������180�x��]����
					mypos.dir = (mypos.dir+6) % 4;		//���p���X�V
					map_write();
					BEEP();
					wait_ms(100);
					BEEP();//�S�[���������Ƃ��A�s�[��
					wait_ms(100);
					BEEP();//�S�[���������Ƃ��A�s�[��
					//search_adachi(0,0);			//�X�^�[�g�n�_�܂ő����@�ŋA���Ă���
					//turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);			//�A���Ă�����180�x��]	
					MOT_POWER_OFF;
					//map_write();
					log_flag = 0;
					BEEP();
					wait_ms(500);
				}
				
				break;
				
			case 5:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	O	X	O	X	*
				*					*
				*****************************************/
			
				//�Z���T�[�̑O�Ɏ���������ăX�^�[�g
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					BEEP();
					degree = 0;
					timer = 0;
					log_timer = 0;
					gyro_get_ref();
					BEEP();
					mypos.x = mypos.y = 0;			//���W��������
					mypos.dir = north;			//���p��������
					log_flag = 1;
					log_timer = 0;
					slalom_search_adachi(GOAL_X,GOAL_Y,GOAL_X1,GOAL_Y1,GOAL_X2,GOAL_Y2,GOAL_X3,GOAL_Y3);		//�S�[���܂ő����@
					if(sen_r.is_wall == true && sen_l.is_wall == true && sen_fr.is_wall == true && sen_fl.is_wall == true){
                    turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
                    back_straight(-28,-SEARCH_ACCEL,-SEARCH_SPEED,0);
					straight(21,SEARCH_ACCEL,SEARCH_SPEED,0);
                    turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
                    back_straight(-28,-SEARCH_ACCEL,-SEARCH_SPEED,0);
					straight(18,S_SEARCH_ACCEL,S_SEARCH_SPEED,0);
                }

                else if(sen_fr.is_wall == true && sen_fl.is_wall ==true){
                   turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
                   back_straight(-28,-SEARCH_ACCEL,-SEARCH_SPEED,0);
				   straight(18,S_SEARCH_ACCEL,S_SEARCH_SPEED,0);
                }
                else{
                    turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
                    }		                  			//�S�[��������180�x��]����
					mypos.dir = (mypos.dir+6) % 4;		//���p���X�V
					map_write();
					BEEP();
					wait_ms(100);
					BEEP();//�S�[���������Ƃ��A�s�[��
					wait_ms(100);
					BEEP();//�S�[���������Ƃ��A�s�[��
					//slalom_search_adachi(0,0);			//�X�^�[�g�n�_�܂ő����@�ŋA���Ă���
					//turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);	
					MOT_POWER_OFF;
					map_write();
					log_flag = 0;
					BEEP();
					wait_ms(500);		
				}
				
				break;
				
			case 6:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	X	O	O	X	*
				*					*
				*****************************************/
			
				//�Z���T�[�̑O�Ɏ���������ăX�^�[�g
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					//BEEP();
					degree = 0;
					timer = 0;
					log_timer = 0;
					gyro_get_ref();
					//BEEP();
					mypos.x = mypos.y = 0;			//���W��������
					mypos.dir = north;			//���p��������
					log_flag = 1;
					log_timer = 0;
					slalom_search_adachi_2(GOAL_X,GOAL_Y,GOAL_X1,GOAL_Y1,GOAL_X2,GOAL_Y2,GOAL_X3,GOAL_Y3);		//�S�[���܂ő����@
					/*if(sen_fl.is_wall == true && sen_fr.is_wall == true && sen_l.is_wall == true){
					slalom_straight_2(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,0);
					turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
                    back_straight(-28,-SEARCH_ACCEL,-SEARCH_SPEED,0);
					wait_ms(100);
					slalom_straight_2(20,SEARCH_ACCEL,SEARCH_SPEED/2,0);
                    turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
                    back_straight(-28,-SEARCH_ACCEL,-SEARCH_SPEED,0);
					wait_ms(100);
					slalom_straight_2(15,SEARCH_ACCEL,SEARCH_SPEED,0);
				}else if(sen_fl.is_wall == true && sen_fr.is_wall == true && sen_r.is_wall == true){
					slalom_straight_2(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,0);
					turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
                    back_straight(-28,-SEARCH_ACCEL,-SEARCH_SPEED,0);
					wait_ms(100);
					slalom_straight_2(20,SEARCH_ACCEL,SEARCH_SPEED/2,0);
                    turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
                    back_straight(-28,-SEARCH_ACCEL,-SEARCH_SPEED,0);
					wait_ms(100);
					slalom_straight_2(15,SEARCH_ACCEL,SEARCH_SPEED,0);
				}else if(sen_fr.is_wall == true && sen_fl.is_wall ==true){
					slalom_straight_2(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,0);
					turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
                    back_straight(-28,-SEARCH_ACCEL,-SEARCH_SPEED,0);
				    wait_ms(100);
				    slalom_straight_2(15,SEARCH_ACCEL,SEARCH_SPEED,0);
				}else{
					slalom_straight_2(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,0);
					turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
				}*/
					//slalom_straight_2(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,0);			                  			
					turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);			//�S�[��������180�x��]����//�S�[��������180�x��]����
					mypos.dir = (mypos.dir+6) % 4;		//���p���X�V
					map_write();
					//BEEP();
					wait_ms(100);
					//BEEP();//�S�[���������Ƃ��A�s�[��
					wait_ms(100);
					//BEEP();//�S�[���������Ƃ��A�s�[��
					//all_slalom_search_adachi(0,0);			//�X�^�[�g�n�_�܂ő����@�ŋA���Ă���
					//depth_first_search(0,0);
					//turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);	
					MOT_POWER_OFF;
					map_write();
					log_flag = 0;
					//BEEP();
				}
				
				break;
				
			case 7:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	O	O	O	X	*
				*					*
				*****************************************/
			
				//�Z���T�[�̑O�Ɏ���������ăX�^�[�g
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					//BEEP();
					degree = 0;
					timer = 0;
					log_timer = 0;
					gyro_get_ref();
					//BEEP();
					mypos.x = mypos.y = 0;			//���W��������
					x_position = y_position = 0;
					mypos.dir = north;			//���p��������
					log_flag = 1;
					log_timer = 0;
					slalom_search_adachi_t(GOAL_X,GOAL_Y,GOAL_X1,GOAL_Y1,GOAL_X2,GOAL_Y2,GOAL_X3,GOAL_Y3);		//�S�[���܂ő����@
					if(sen_fl.is_wall == true && sen_fr.is_wall == true && sen_l.is_wall == true){
					slalom_straight_2(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,0);
					turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
                    back_straight(-28,-SEARCH_ACCEL,-SEARCH_SPEED,0);
					wait_ms(100);
					slalom_straight_2(20,SEARCH_ACCEL,SEARCH_SPEED/2,0);
                    turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
                    back_straight(-28,-SEARCH_ACCEL,-SEARCH_SPEED,0);
					wait_ms(100);
					slalom_straight_2(20,SEARCH_ACCEL,SEARCH_SPEED,0);
				}else if(sen_fl.is_wall == true && sen_fr.is_wall == true && sen_r.is_wall == true){
					slalom_straight_2(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,0);
					turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
                    back_straight(-28,-SEARCH_ACCEL,-SEARCH_SPEED,0);
					wait_ms(100);
					slalom_straight_2(20,SEARCH_ACCEL,SEARCH_SPEED/2,0);
                    turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
                    back_straight(-28,-SEARCH_ACCEL,-SEARCH_SPEED,0);
					wait_ms(100);
					slalom_straight_2(20,SEARCH_ACCEL,SEARCH_SPEED,0);
				}else if(sen_fr.is_wall == true && sen_fl.is_wall ==true){
					slalom_straight_2(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,0);
					turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
                    back_straight(-28,-SEARCH_ACCEL,-SEARCH_SPEED,0);
				    wait_ms(100);
				    slalom_straight_2(20,SEARCH_ACCEL,SEARCH_SPEED,0);
				}else{
					slalom_straight_2(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,0);
					turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
				}
								                  			//�S�[��������180�x��]����
					mypos.dir = (mypos.dir+6) % 4;		//���p���X�V
					map_write();
					//BEEP();
					wait_ms(100);
					//BEEP();//�S�[���������Ƃ��A�s�[��
					wait_ms(100);
					//BEEP();//�S�[���������Ƃ��A�s�[��
					//all_slalom_search_adachi(0,0);			//�X�^�[�g�n�_�܂ő����@�ŋA���Ă���
					//turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);	
					MOT_POWER_OFF;
					map_write();
					log_flag = 0;
					//BEEP();
				}
				
				break;
				
			case 8:  //loop
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	X	X	X	O	*
				*					*
				*****************************************/
			
				//�Z���T�[�̑O�Ɏ���������ăX�^�[�g
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					//BEEP();
					BEEP();
					gyro_get_ref();
					BEEP();
					len_count = 0;
				
					log_flag = 1;
					log_timer = 0;
					len_mouse = 0;
                    x_position = y_position = 0;

					slalom_straight_2(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);
                    slalom_2(90,SLALOM_ACCEL_2,SLALOM_SPEED_2,RIGHT);
					slalom_2(90,SLALOM_ACCEL_2,SLALOM_SPEED_2,RIGHT);
                    slalom_2(90,SLALOM_ACCEL_2,SLALOM_SPEED_2,RIGHT);
					slalom_2(90,SLALOM_ACCEL_2,SLALOM_SPEED_2,RIGHT);
					slalom_2(90,SLALOM_ACCEL_2,SLALOM_SPEED_2,RIGHT);
					slalom_2(90,SLALOM_ACCEL_2,SLALOM_SPEED_2,RIGHT);
					slalom_2(90,SLALOM_ACCEL_2,SLALOM_SPEED_2,RIGHT);
					slalom_2(90,SLALOM_ACCEL_2,SLALOM_SPEED_2,RIGHT);
					slalom_2(90,SLALOM_ACCEL_2,SLALOM_SPEED_2,RIGHT);
					slalom_2(90,SLALOM_ACCEL_2,SLALOM_SPEED_2,RIGHT);
					slalom_2(90,SLALOM_ACCEL_2,SLALOM_SPEED_2,RIGHT);
					slalom_2(90,SLALOM_ACCEL_2,SLALOM_SPEED_2,RIGHT);
					slalom_2(90,SLALOM_ACCEL_2,SLALOM_SPEED_2,RIGHT);
					slalom_2(90,SLALOM_ACCEL_2,SLALOM_SPEED_2,RIGHT);
					slalom_2(90,SLALOM_ACCEL_2,SLALOM_SPEED_2,RIGHT);
					slalom_2(90,SLALOM_ACCEL_2,SLALOM_SPEED_2,RIGHT);
					slalom_2(90,SLALOM_ACCEL_2,SLALOM_SPEED_2,RIGHT);
					slalom_2(90,SLALOM_ACCEL_2,SLALOM_SPEED_2,RIGHT);
					slalom_2(90,SLALOM_ACCEL_2,SLALOM_SPEED_2,RIGHT);
					slalom_2(90,SLALOM_ACCEL_2,SLALOM_SPEED_2,RIGHT);
					slalom_straight_2(HALF_SECTION,S_SEARCH_ACCEL,S_SEARCH_SPEED,0);
					log_flag = 0;
					wait_ms(500);
				}
				
				break;
				
			case 9:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	O	X	X	O	*
				*					*
				*****************************************/
			
				//�Z���T�[�̑O�Ɏ���������ăX�^�[�g
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					BEEP();
					gyro_get_ref();
					BEEP();

					log_flag = 1;
					log_timer = 0;

					len_count = 0;
					
					len_mouse = 0;
					x_position = y_position = 0;

                    now_dir = north;

					straight(20,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);
					slalom_straight_2(HALF_SECTION,S_SEARCH_ACCEL,S_SEARCH_SPEED,S_SEARCH_SPEED);
					slalom_straight_2(HALF_SECTION,S_SEARCH_ACCEL,S_SEARCH_SPEED,S_SEARCH_SPEED);
					slalom_straight_2(HALF_SECTION,S_SEARCH_ACCEL,S_SEARCH_SPEED,S_SEARCH_SPEED);
					slalom_straight_2(HALF_SECTION,S_SEARCH_ACCEL,S_SEARCH_SPEED,S_SEARCH_SPEED);
					slalom_straight_2(HALF_SECTION,S_SEARCH_ACCEL,S_SEARCH_SPEED,S_SEARCH_SPEED);
                    Kanayama_sla(RIGHT);
					now_dir = east;
					slalom_straight_2(HALF_SECTION,S_SEARCH_ACCEL,S_SEARCH_SPEED,S_SEARCH_SPEED);
					slalom_straight_2(HALF_SECTION,S_SEARCH_ACCEL,S_SEARCH_SPEED,S_SEARCH_SPEED);
					slalom_straight_2(HALF_SECTION,S_SEARCH_ACCEL,S_SEARCH_SPEED,S_SEARCH_SPEED);
					slalom_straight_2(HALF_SECTION,S_SEARCH_ACCEL,S_SEARCH_SPEED,S_SEARCH_SPEED);
					Kanayama_sla(RIGHT);
					now_dir = south;
					slalom_straight_2(HALF_SECTION,S_SEARCH_ACCEL,S_SEARCH_SPEED,S_SEARCH_SPEED);
					slalom_straight_2(HALF_SECTION,S_SEARCH_ACCEL,S_SEARCH_SPEED,S_SEARCH_SPEED);
					slalom_straight_2(HALF_SECTION,S_SEARCH_ACCEL,S_SEARCH_SPEED,S_SEARCH_SPEED);
					slalom_straight_2(HALF_SECTION,S_SEARCH_ACCEL,S_SEARCH_SPEED,S_SEARCH_SPEED);
                    Kanayama_sla(RIGHT);
					now_dir = west;
					slalom_straight_2(HALF_SECTION,S_SEARCH_ACCEL,S_SEARCH_SPEED,S_SEARCH_SPEED);
					slalom_straight_2(HALF_SECTION,S_SEARCH_ACCEL,S_SEARCH_SPEED,S_SEARCH_SPEED);
					slalom_straight_2(HALF_SECTION,S_SEARCH_ACCEL,S_SEARCH_SPEED,S_SEARCH_SPEED);
					slalom_straight_2(HALF_SECTION,S_SEARCH_ACCEL,S_SEARCH_SPEED,S_SEARCH_SPEED);
					log_flag = 1;
					log_timer = 0;
                    Kanayama_sla(RIGHT);
					now_dir = north;
					slalom_straight_2(HALF_SECTION,S_SEARCH_ACCEL,S_SEARCH_SPEED,0);
					log_flag = 0;
					
					wait_ms(500);		
				}
				
				break;
				
			case 10:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	X	O	X	O	*
				*					*
				*****************************************/
			
				//�Z���T�[�̑O�Ɏ���������ăX�^�[�g
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					//BEEP();
					gyro_get_ref();
					//BEEP();
					wait_ms(100);
					degree = 0;
					len_mouse = 0;
					x_position = y_position = 0;
                    now_dir = north;
                    turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
					now_dir = south;
					
					slalom_straight_2(HALF_SECTION,S_SEARCH_ACCEL,S_SEARCH_SPEED,S_SEARCH_SPEED);
					
                    Kanayama_sla(LEFT);
					now_dir = east;
					slalom_straight_2(HALF_SECTION,S_SEARCH_ACCEL,S_SEARCH_SPEED,0);
					turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
					
					log_flag = 1;
					log_timer = 0;
					now_dir = west;
					slalom_straight_2(HALF_SECTION,S_SEARCH_ACCEL,S_SEARCH_SPEED,S_SEARCH_SPEED);
					Kanayama_sla(LEFT);
					
					now_dir = south;
					Kanayama_sla(LEFT);
                    
					now_dir = east;
					
					Kanayama_sla(RIGHT);
					log_flag = 0;
					now_dir = south;
                    Kanayama_sla(LEFT);
                    //log_flag = 0;
					
					slalom_straight_2(SECTION,S_SEARCH_ACCEL,S_SEARCH_SPEED,0);

					wait_ms(500);		
				}

				break;
				
			case 11:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	O	O	X	O	*
				*					*
				*****************************************/
			
				//�Z���T�[�̑O�Ɏ���������ăX�^�[�g
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					BEEP();
					degree = 0;
					timer = 0;
					log_timer = 0;
					gyro_get_ref();
					BEEP();
					mypos.x = mypos.y = 0;			//���W��������
					mypos.dir = north;			//���p��������
					log_flag = 1;
					log_timer = 0;
					depth_first_search(GOAL_X,GOAL_Y);
					turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
					
					
					wait_ms(500);			
				}
				
				break;
				
			case 12:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	X	X	O	O	*
				*					*
				*****************************************/
			
				//�Z���T�[�̑O�Ɏ���������ăX�^�[�g
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					//BEEP();

					gyro_get_ref();
                    degree = 0;
					len_mouse = 0;
					timer = 0;
					cnt = 0;
					x_position = 0;
					y_position = 0;
					while(1){
					
						//gyro
						SCI_printf("degree: %d\n\r",(int)degree*10);
						SCI_printf("radian: %d\n\r",(int)degree);			
						SCI_printf("gyro: %d\n\r", (int)(ang_vel*1000) );	

						//len_mouse
						SCI_printf("len_mouse: %d\n\r",(int)(len_mouse));
					
                        //x,y position
						SCI_printf("x_position: %d\n\r",(long)(x_position));
						SCI_printf("y_position: %d\n\r",(long)(y_position));
						SCI_printf("timer: %d\n\r", timer);
						SCI_printf("cnt: %d\n\r", cnt);

						wait_ms(100);
						//��ʃN���A�V�[�P���X
						SCI_printf("\x1b[2J");				//�N���A�X�N���[��[CLS]
						SCI_printf("\x1b[0;0H");			//�J�[�\����0,0�Ɉړ�
						
						//�v�b�V���X�C�b�`�p����
						push_switch = IOex_SWITCH();
			
						if(SW_C == 1){
							BEEP();
							break;	
						}
					}
					wait_ms(500);		
				}
				
				break;
				
				
			case 13:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	O	X	O	O	*
				*					*
				*****************************************/
			
				//�Z���T�[�̑O�Ɏ���������ăX�^�[�g
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					//BEEP();
                    SCI_printf("time[msec],x_postion,y_position\n\r");
					for(i = 0; i < LOG_CNT; i++){
						
						SCI_printf("%d,",i);//time[msec]
						SCI_printf("%d,",loga[0][i]);
						SCI_printf("%d\n\r",loga[1][i]);	
					/*SCI_printf("time[msec],x_e,y_e,theta_e,now_dir,tar_x,tar_y,tar_th,degree*10,tar_ang_vel*1000,tar_speed*1000,x_postion,y_position\n\r");
					for(i = 0; i < LOG_CNT; i++){
						
						SCI_printf("%d,",i);//time[msec]
						SCI_printf("%d,",logs[0][i]);
						SCI_printf("%d,",logs[1][i]);
						SCI_printf("%d,",logs[2][i]);
						SCI_printf("%d,",logs[3][i]);
						SCI_printf("%d,",logs[4][i]);
						SCI_printf("%d,",logs[5][i]);
						SCI_printf("%d,",logs[6][i]);
						SCI_printf("%d,",logs[7][i]);
						SCI_printf("%d,",logs[8][i]);
						SCI_printf("%d,",logs[9][i]);
						SCI_printf("%d,",logs[10][i]);
						SCI_printf("%d\n\r",logs[11][i]);		
					}
					wait_ms(500);*/		
					}
					wait_ms(500);	
				}
				
				break;
				
			case 14:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	X	O	O	O	*
				*					*
				*****************************************/
			
				//�Z���T�[�̑O�Ɏ���������ăX�^�[�g
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					//BEEP();
                    gyro_get_ref();
                    degree = 0;
					len_mouse = 0;
					timer = 0;
					cnt = 0;
					x_position = 0;
					y_position = 0;
					while(1){
					
						//gyro
						SCI_printf("degree*10: %d\n\r",(int)gyro_x);
						SCI_printf("degree: %d\n\r",(int)degree);			
						SCI_printf("ang_vel*1000: %d\n\r", (int)(ang_vel*1000) );	

						//acccel
						SCI_printf("accel_x_new: %d\n\r",(int)(accel_x_new));
                        SCI_printf("accel_x: %d\n\r",(int)(accel_x*1000));

						//len_mouse
						//SCI_printf("len_mouse: %d\n\r",(int)(len_mouse));
					
                        //x,y position
						//SCI_printf("x_position: %d\n\r",(long)(x_position));
						//SCI_printf("y_position: %d\n\r",(long)(y_position));
						//SCI_printf("timer: %d\n\r", timer);
						//SCI_printf("cnt: %d\n\r", cnt);

						wait_ms(100);
						//��ʃN���A�V�[�P���X
						SCI_printf("\x1b[2J");				//�N���A�X�N���[��[CLS]
						SCI_printf("\x1b[0;0H");			//�J�[�\����0,0�Ɉړ�
						
						//�v�b�V���X�C�b�`�p����
						push_switch = IOex_SWITCH();
			
						if(SW_C == 1){
							BEEP();
							break;	
						}
					}
					wait_ms(500);	
				}
				
				break;
				
			case 15:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	O	O	O	O	*
				*					*
				*****************************************/
			
				//�Z���T�[�̑O�Ɏ���������ăX�^�[�g
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					//BEEP();
					while(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4);
					adjust();
					
					//BEEP();
					wait_ms(500);
				}
				
				break;
				
			//mode0~15�ȊO�̏ꍇ�B�������Ȃ��B
			default:
				break;
			
		}
		
		//���[�h�؂�ւ��p����
		if(speed > 0.1){
			if(mode == 15){
				mode = 1;
			}else{
				mode ++;
			}
			for(i = 0; i < 100*1000*10; i++);
			//BEEP();
		}
		
		if(speed < -0.1){
			if(mode == 1){
				mode = 15;
			}else{
				mode --;
		}
			for(i = 0; i < 100*1000*10; i++);
			//BEEP(); 
		}
		if(flash_time > 0x00FF){
			flash_time = 0;
			if(flash == 0x08){
				flash = 0x00;
			}else{
				flash = 0x08;
			}
		}
			
		flash_time++;
		LED(mode | flash);
		//LED(mode);
		
		//�v�b�V���X�C�b�`�p����
		push_switch = IOex_SWITCH();
		MOT_POWER_OFF;
	
	}
	

}

#ifdef __cplusplus
void abort(void)
{

}
#endif
