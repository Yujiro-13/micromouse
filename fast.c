
#include "portdef.h"
#include "iodefine.h"
#include "parameters.h"
#include "mytypedef.h"
#include "glob_var.h"
#include "run.h"
#include "interface.h"
#include "sci.h"

extern int get_nextdir(int x, int y, int mask, t_direction *dir);

void fast_run(int x, int y)
{
//引数の座標x,yに向かって最短走行する

	t_direction glob_nextdir;
	int straight_count=0;

	//現在の向きから、次に行くべき方向へ向く
	switch(get_nextdir(x,y,MASK_SECOND,&glob_nextdir))	//次に行く方向を戻り値とする関数を呼ぶ
	{
		case front:
			straight_count++;			//前向きだった場合は直線を走る距離を伸ばす
			break;
		
		case right:					//右に向く
			turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);				//右に曲がって
			straight_count = 1;
			break;
		
		case left:					//左に向く
			turn(90,TURN_ACCEL,TURN_SPEED,LEFT);				//左に曲がって
			straight_count = 1;
			break;
		
		case rear:					//後ろに向く
			turn(180,TURN_ACCEL,TURN_SPEED,LEFT);				//左に曲がって
			straight_count = 1;
			break;
	}

	mypos.dir = glob_nextdir;	//自分の向きを更新


	//向いた方向によって自分の座標を更新する
	switch(mypos.dir)
	{
		case north:
			mypos.y++;	//北を向いた時はY座標を増やす
			break;
			
		case east:
			mypos.x++;	//東を向いた時はX座標を増やす
			break;
			
		case south:
			mypos.y--;	//南を向いた時はY座標を減らす
			break;
		
		case west:
			mypos.x--;	//西を向いたときはX座標を減らす
			break;

	}


	
	while((mypos.x != x) || (mypos.y != y)){			//ゴールするまで繰り返す


		switch(get_nextdir(x,y,MASK_SECOND,&glob_nextdir))	//次に行く方向を戻り値とする関数を呼ぶ
		{
			case front:					//直線をまとめて走るようにする
				straight_count++;
				break;
			
			case right:
				straight(SECTION*straight_count,FAST_ACCEL,FAST_SPEED,0.0);
				turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);				//右に曲がって
				straight_count = 1;			//走る直線の距離をリセット
				break;
			
			case left:
				straight(SECTION*straight_count,FAST_ACCEL,FAST_SPEED,0.0);
				turn(90,TURN_ACCEL,TURN_SPEED,LEFT);				//左に曲がって
				straight_count = 1;			//走る直線の距離をリセット
				break;
			
			case rear:
				straight(SECTION*straight_count,FAST_ACCEL,FAST_SPEED,0.0);
				turn(180,TURN_ACCEL,TURN_SPEED,LEFT);				//左に曲がって
				straight_count = 1;			//走る直線の距離をリセット
				break;
		}
	
		mypos.dir = glob_nextdir;	//自分の向きを修正
		
		//向いた方向によって自分の座標を更新する
		switch(mypos.dir)
		{
			case north:
				mypos.y++;	//北を向いた時はY座標を増やす
				break;
				
			case east:
				mypos.x++;	//東を向いた時はX座標を増やす
				break;
				
			case south:
				mypos.y--;	//南を向いた時はY座標を減らす
				break;
			
			case west:
				mypos.x--;	//西を向いたときはX座標を減らす
				break;

		}
	}
	straight(SECTION*straight_count,FAST_ACCEL,FAST_SPEED,0.0);
}




void fast_slalom_run(int x, int y)
{
//引数の座標x,yに向かって最短走行する

	t_direction glob_nextdir;
	int straight_count=0;

	/*if(x != 0 && y != 0){
		straight(10,S_SEARCH_ACCEL,S_SEARCH_SPEED,S_SEARCH_SPEED);
	}*/

	//現在の向きから、次に行くべき方向へ向く
	switch(get_nextdir(x,y,MASK_SECOND,&glob_nextdir))	//次に行く方向を戻り値とする関数を呼ぶ
	{
		case front:
			straight_count++;			//前向きだった場合は直線を走る距離を伸ばす
			break;
		
		case right:					//右に向く
			turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);				//右に曲がって
			straight_count = 1;
			break;
		
		case left:					//左に向く
			turn(90,TURN_ACCEL,TURN_SPEED,LEFT);				//左に曲がって
			straight_count = 1;
			break;
		
		case rear:					//後ろに向く
			turn(180,TURN_ACCEL,TURN_SPEED,LEFT);				//左に曲がって
			straight_count = 1;
			break;
	}

	mypos.dir = glob_nextdir;	//自分の向きを更新


	//向いた方向によって自分の座標を更新する
	switch(mypos.dir)
	{
		case north:
			mypos.y++;	//北を向いた時はY座標を増やす
			break;
			
		case east:
			mypos.x++;	//東を向いた時はX座標を増やす
			break;
			
		case south:
			mypos.y--;	//南を向いた時はY座標を減らす
			break;
		
		case west:
			mypos.x--;	//西を向いたときはX座標を減らす
			break;

	}


	
	while((mypos.x != x) || (mypos.y != y)){			//ゴールするまで繰り返す


		switch(get_nextdir(x,y,MASK_SECOND,&glob_nextdir))	//次に行く方向を戻り値とする関数を呼ぶ
		{
			case front:					//直線をまとめて走るようにする
				straight_count++;
				break;
			
			case right:
				slalom_straight_2(SECTION*straight_count,S_FAST_ACCEL,S_FAST_SPEED,S_FAST_SPEED);
				slalom(90,SLALOM_ACCEL_2,SLALOM_SPEED_2,RIGHT);				//右に曲がって
				straight_count = 0;			//走る直線の距離をリセット
				break;
			
			case left:
				slalom_straight_2(SECTION*straight_count,S_FAST_ACCEL,S_FAST_SPEED,S_FAST_SPEED);
				slalom(90,SLALOM_ACCEL_2,SLALOM_SPEED_2,LEFT);				//左に曲がって
				straight_count = 0;			//走る直線の距離をリセット
				break;
			
			case rear:
				slalom_straight_2(SECTION*straight_count,S_FAST_ACCEL,S_FAST_SPEED,S_FAST_SPEED);
				turn(180,TURN_ACCEL,TURN_SPEED,LEFT);				//左に曲がって
				straight_count = 1;			//走る直線の距離をリセット
				break;
		}
	
		mypos.dir = glob_nextdir;	//自分の向きを修正
		
		//向いた方向によって自分の座標を更新する
		switch(mypos.dir)
		{
			case north:
				mypos.y++;	//北を向いた時はY座標を増やす
				break;
				
			case east:
				mypos.x++;	//東を向いた時はX座標を増やす
				break;
				
			case south:
				mypos.y--;	//南を向いた時はY座標を減らす
				break;
			
			case west:
				mypos.x--;	//西を向いたときはX座標を減らす
				break;

		}
	}
	slalom_straight_2(SECTION*straight_count,S_FAST_ACCEL,S_FAST_SPEED,S_FAST_SPEED);
	slalom_straight_2(HALF_SECTION,S_FAST_ACCEL,S_FAST_SPEED,0.0);

}

