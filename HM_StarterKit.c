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
					search_adachi(GOAL_X,GOAL_Y);		//�S�[���܂ő����@
					turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);			//�S�[��������180�x��]����
					mypos.dir = (mypos.dir+6) % 4;		//���p���X�V
					map_write();
					BEEP();
					wait_ms(100);
					BEEP();//�S�[���������Ƃ��A�s�[��
					wait_ms(100);
					BEEP();//�S�[���������Ƃ��A�s�[��
					search_adachi(0,0);			//�X�^�[�g�n�_�܂ő����@�ŋA���Ă���
					turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);			//�A���Ă�����180�x��]	
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
					//BEEP();
					map_copy();
					degree = 0;
					timer = 0;
					gyro_get_ref();
					BEEP();
					mypos.x = mypos.y = 0;			//���W��������
					mypos.dir = north;			//���p��������
					log_flag = 1;
					log_timer = 0;
					fast_run(GOAL_X,GOAL_Y);		//�S�[���܂ő����@
					turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);			//�S�[��������180�x��]����
					mypos.dir = (mypos.dir+6) % 4;		//���p���X�V
					map_write();
					BEEP();
					wait_ms(100);
					BEEP();//�S�[���������Ƃ��A�s�[��
					wait_ms(100);
					BEEP();//�S�[���������Ƃ��A�s�[��
					search_adachi(0,0);			//�X�^�[�g�n�_�܂ő����@�ŋA���Ă���
					turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);			//�A���Ă�����180�x��]	
					MOT_POWER_OFF;
					map_write();
					log_flag = 0;
					BEEP();
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
					//BEEP();
					SCI_printf("sen_r.value,sen_l.value,sen_fr.value,sen_fl.value,speed_r*100,speed_l*100,degree*10,V_bat*1000,con_wall.omega,ang_vel*1000,locate_r,locate_l\n\r");
					for(i = 0; i < LOG_CNT; i++){
						
						SCI_printf("%d,",i);//time[msec]						
						SCI_printf("%d,",log[12][i]);
						SCI_printf("%d,",log[13][i]);
						SCI_printf("%d,",log[14][i]);
						SCI_printf("%d,",log[15][i]);
						SCI_printf("%d,",log[16][i]);
						SCI_printf("%d,",log[17][i]);
						SCI_printf("%d,",log[18][i]);
						SCI_printf("%d,",log[19][i]);
						SCI_printf("%d,",log[20][i]);
						SCI_printf("%d,",log[21][i]);
						SCI_printf("%d,",log[22][i]);
						SCI_printf("%d\n\r",log[23][i]);	
					}	
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
					//BEEP();
					BEEP();
					gyro_get_ref();
					//BEEP();
					log_flag = 1;
					log_timer = 0;
					len_mouse = 0;
					straight(SECTION*3,SEARCH_ACCEL,SEARCH_SPEED,0);
					log_flag = 0;
					MOT_POWER_OFF;
					//BEEP();
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
					//BEEP();
	                BEEP();
					gyro_get_ref();
					//BEEP();
					log_flag = 1;
					log_timer = 0;
					len_mouse = 0;
					straight(SECTION,SEARCH_ACCEL,SEARCH_SPEED,0);
					turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);					//180�^�[��
				    straight(SECTION,SEARCH_ACCEL,SEARCH_SPEED,0);
					log_flag = 0;
					MOT_POWER_OFF;
					//BEEP();
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

					wait_ms(500);
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

					wait_ms(500);	
				}
				
				break;
				
			case 8:
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
					//BEEP();

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
					//BEEP();

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
		LED(mode);
		
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
