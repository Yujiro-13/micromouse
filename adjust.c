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

extern void wait_ms(int wtime);

void adjust(void)
{

	int i = 0;
	int flash_time = 0;
	char flash;
	short ad_mode = 1;
	while(1){
		switch(ad_mode){

			
			case 1:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	O	X	X	X	*
				*					*
				*****************************************/
				
				//センサーの前に手をかざしてスタート
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					//BEEP();
					//壁制御を有効にする
					con_wall.enable = true;
					degree = 0;
					while(1){
						//A/D sensor
						SCI_printf("sen_r.value: %d\n\r",sen_r.value);
						SCI_printf("sen_l.value: %d\n\r",sen_l.value);
						SCI_printf("sen_fr.value: %d\n\r",sen_fr.value);
						SCI_printf("sen_fl.value: %d\n\r",sen_fl.value);
						SCI_printf("V_bat: %d\n\r",(int)(V_bat*1000));
						SCI_printf("sen_r.th_wall: %d\n\r",sen_r.th_wall);
						SCI_printf("sen_l.th_wall: %d\n\r",sen_l.th_wall);
						SCI_printf("sen_fr.th_wall: %d\n\r",sen_fr.th_wall);
						SCI_printf("sen_fl.th_wall: %d\n\r",sen_fl.th_wall);
						SCI_printf("con_wall.omega: %d\n\r",(int)(con_wall.omega*1000));
						SCI_printf("speed_r: %d\n\r", (int)(speed_r*100));
						SCI_printf("speed_l: %d\n\r", (int)(speed_l*100));
						SCI_printf("len_mouse: %d\n\r", (int)(len_mouse));
						//gyro
						SCI_printf("degree: %d\n\r",(int)degree*10);
						SCI_printf("radian: %d\n\r",a);			
						SCI_printf("gyro: %d\n\r", (int)(ang_vel*1000) );
						//encoder
						SCI_printf("locate_r: %d\n\r",locate_r);
						SCI_printf("locate_l: %d\n\r",locate_l);	
					
						//switch
						SCI_printf("switchC: %d\n\r",SW_C);
						SCI_printf("switchU: %d\n\r",SW_U);
						SCI_printf("switchD: %d\n\r",SW_D);

                        //x,y position
						SCI_printf("x_position: %d\n\r",(int)x_position);
						SCI_printf("y_position: %d\n\r",(int)y_position);

						wait_ms(100);
						//画面クリアシーケンス
						SCI_printf("\x1b[2J");				//クリアスクリーン[CLS]
						SCI_printf("\x1b[0;0H");			//カーソルを0,0に移動
						
						//プッシュスイッチ用処理
						push_switch = IOex_SWITCH();
			
						if(SW_C == 1){
							BEEP();
							break;	
						}
					}
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
			
				//センサーの前に手をかざしてスタート
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					//BEEP();
					gyro_get_ref();
					//BEEP();
					log_flag = 1;
					log_timer = 0;
					len_mouse = 0;
					check_straight(90,0,0,0);
					log_flag = 0;
					MOT_POWER_OFF;
					//BEEP();
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
				//マップの表示
				//センサーの前に手をかざしてスタート
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					//BEEP();
					gyro_get_ref();
					//BEEP();
					log_flag = 1;
					log_timer = 0;
					turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
					log_flag = 0;
					MOT_POWER_OFF;
					//BEEP();
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
			
				//センサーの前に手をかざしてスタート
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					//gyro_get_ref();
					//BEEP();
					len_mouse = 0;
					x_position = 0;
					y_position = 0;
					len_count = 0;
					wait_ms(1000);
					timer = 0;
					log_flag = 1;
					log_timer = 0;
					
				    //run_test(1.0, 1.0, 2000);
					//run_test_2(500, 600, 700, 800, 900, 1000);
					check_FF_run(200,400,600);
					log_flag = 0;
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
			
				//センサーの前に手をかざしてスタート
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					//BEEP();
					//BEEP();
					gyro_get_ref();
					//BEEP();
					x_position = y_position = 0;
					len_mouse = 0;
					log_flag = 1;
					log_timer = 0;
					
					//back_straight(-28,-SEARCH_ACCEL,-SEARCH_SPEED,0);
					
					slalom_straight(SECTION,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);
					slalom_2(90,SLALOM_ACCEL_2,SLALOM_SPEED_2,RIGHT);
					slalom_2(90,SLALOM_ACCEL_2,SLALOM_SPEED_2,LEFT);
					slalom_2(90,SLALOM_ACCEL_2,SLALOM_SPEED_2,RIGHT);
					slalom_straight(SECTION,SEARCH_ACCEL,SEARCH_SPEED,0);
					//slalom_straight(SECTION,2.0,1.0,1.0);
					//slalom_straight(SECTION,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);
					//slalom_straight(SECTION,SEARCH_ACCEL,SEARCH_SPEED,0);
					MOT_POWER_OFF;
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
				//マップ表示
				//センサーの前に手をかざしてスタート
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					BEEP();
					map_copy();
					map_view();
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
				//ログ出力
				//センサーの前に手をかざしてスタート
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					//BEEP();
					SCI_printf("time[ms],V_bat,V_l*1000,tar_speed*1000,speed*1000,locate*1000,locate*1000\n\r");
					for(i = 0; i < LOG_CNT; i++){
						
						SCI_printf("%d,",i);//time[msec]
						SCI_printf("%d,",loga[0][i]);
						SCI_printf("%d,",loga[1][i]);
						SCI_printf("%d,",loga[2][i]);
						SCI_printf("%d,",loga[3][i]);
						SCI_printf("%d,",loga[4][i]);
						SCI_printf("%d\n\r",loga[5][i]);

					/*SCI_printf("time[msec],sen_r.value,sen_l.value,sen_fr.value,sen_fl.value,speed_r*100,speed_l*100,degree*10,1000*V_bat,len_mouse,ang_vel*1000,locate_r,locate_l\n\r");
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
						SCI_printf("%d\n\r",logs[11][i]);*/		
					}
					wait_ms(500);
				}				
				break;
			//mode0~7以外の場合。何もしない。
			default:
				break;
			
		}
		
		//モード切り替え用処理
		if(speed > 0.1){
			if(ad_mode == 7){
				ad_mode = 1;
			}else{
				ad_mode ++;
			}
			for(i = 0; i < 100*1000*10; i++);
			//BEEP();
		}
		
		if(speed < -0.1){
			if(ad_mode == 1){
				ad_mode = 7;
			}else{
				ad_mode --;
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
		LED(ad_mode | flash);
		
		//プッシュスイッチ用処理
		push_switch = IOex_SWITCH();
		MOT_POWER_OFF;
		
		if(SW_C == 1){
			BEEP();
			break;	
		}
	}
}