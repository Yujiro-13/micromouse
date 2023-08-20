#include "mytypedef.h"
#include "parameters.h"
#include "iodefine.h"
#include "portdef.h"
#include "glob_var.h"
#include "macro.h"
#include "run.h"
#include "interface.h"
#include "depth_first_search.h"
// #include "odom.h"
// #include "my_sin.h"
// #include "my_cos.h"

void search_lefthand(void); // 左手法

void init_map(int x, int y, int x1, int y1, int x2, int y2, int x3, int y3);							   // 歩数Mapの初期化
void make_map(int x, int y, int x1, int y1, int x2, int y2, int x3, int y3, int mask);					   // 歩数マップ作成
void set_wall(int x, int y);							   // 壁情報を保存
t_bool is_unknown(int x, int y);						   // 未探索区間か否かを判定
int get_priority(int x, int y, t_direction dir);		   // 優先度を取得(未探索、前方向が優先される)
int get_nextdir(int x, int y, int x1, int y1, int x2, int y2, int x3, int y3, int mask, t_direction *dir); // 次に行くべき方向を取得する
void search_adachi(int gx, int gy);						   // 足立法
void slalom_search_adachi(int gx, int gy);				   // スラロームによる足立法
void slalom_search_adachi_2(int gx, int gy, int gx1, int gy1, int gx2, int gy2, int gx3, int gy3);
void all_search_adachi(int gx, int gy);		   // 足立法(全面探索)
void all_slalom_search_adachi(int gx, int gy); // スラローム足立法(全面探索)
void init_map_all(int x, int y);
void make_map_all(int x, int y, int mask);
void set_wall_all(int x, int y);
int get_nextdir_all(int x, int y, int mask, t_direction *dir);
void slalom_search_adachi_t(int gx, int gy); // 軌道追従での探索
void init_map_D(int x, int y);
void make_map_D(int x, int y, int mask);
int get_nextdir_D(int x, int y, int mask, t_direction *dir);
void initStack(STACK_T *stack);
void push(STACK_T *stack, POS_T *input);
POS_T *pop(STACK_T *stack);
t_bool check(int x, int y);
void search_unknown_pos(int x, int y, int mask);
void get_next_target();
void depth_first_search(int gx, int gy);

extern void wait_ms(int wtime);

/* スタックを初期化する関数 */
void initStack(STACK_T *stack)
{

	/* スタックを空に設定 */
	stack->tail = -1;
}

void search_lefthand(void)
{

	max_speed = SEARCH_SPEED; // 探索の速度を指定
	accel = SEARCH_ACCEL;

	straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // まず、半区画進む

	while (1)
	{
		if (sen_l.is_wall == false) // 左に壁がなければ左に進む
		{
			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);			  // 半区画進んで
			turn(90, TURN_ACCEL, TURN_SPEED, LEFT);							  // 左に曲がって
			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む
		}
		else if ((sen_fl.is_wall == false) && (sen_fr.is_wall == false)) // 前に壁がなければ前に進む
		{
			straight(SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 一区画進む
		}
		else if (sen_r.is_wall == false) // 右に壁がなければ右に進む
		{
			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);			  // 半区画進む
			turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);						  // 右に曲がる
			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む
		}
		else // それ以外の場合、後ろに進む
		{
			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);			  // 半区画進む
			turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);						  // 後ろに向く
			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む
		}
	}
}

void init_map(int x, int y, int x1, int y1, int x2, int y2, int x3, int y3)
{
	// 迷路の歩数Mapを初期化する。全体を0xff、引数の座標x,yは0で初期化する

	int i, j;

	for (i = 0; i < MAZESIZE_X; i++) // 迷路の大きさ分ループ(x座標)　
	{
		for (j = 0; j < MAZESIZE_Y; j++) // 迷路の大きさ分ループ(y座標)
		{
			map[i][j] = 255; // すべて255で埋める  ex)map[1][1] = 255,map[1][2] = 255, ...map[1][9] = 255,map[2][1] = 255...
		}
	}

	map[x][y] = 0; // ゴール座標の歩数を０に設定
	map[x1][y1] = 0;
	map[x2][y2] = 0;
	map[x3][y3] = 0;
}

void make_map(int x, int y, int x1, int y1, int x2, int y2, int x3, int y3, int mask) // 歩数マップを作成する
{
	// 座標x,yをゴールとした歩数Mapを作成する。
	// maskの値(MASK_SEARCH or MASK_SECOND)によって、
	// 探索用の歩数Mapを作るか、最短走行の歩数Mapを作るかが切り替わる
	int i, j;
	t_bool change_flag; // Map作成終了を見極めるためのフラグ

	init_map(x, y, x1, y1, x2, y2, x3, y3); // Mapを初期化する

	do
	{
		change_flag = false;			 // 変更がなかった場合にはループを抜ける
		for (i = 0; i < MAZESIZE_X; i++) // 迷路の大きさ分ループ(x座標)
		{
			for (j = 0; j < MAZESIZE_Y; j++) // 迷路の大きさ分ループ(y座標)
			{
				if (map[i][j] == 255) // 255の場合は次へ
				{
					continue;
				}

				if (j < MAZESIZE_Y - 1) // 範囲チェック
				{
					if ((wall[i][j].north & mask) == NOWALL) // 壁がなければ(maskの意味はstatic_parametersを参照)
					{
						if (map[i][j + 1] == 255) // まだ値が入っていなければ
						{
							map[i][j + 1] = map[i][j] + 1; // 値を代入
							change_flag = true;			   // 値が更新されたことを示す
						}
					}
				}

				if (i < MAZESIZE_X - 1) // 範囲チェック
				{
					if ((wall[i][j].east & mask) == NOWALL) // 壁がなければ
					{
						if (map[i + 1][j] == 255) // 値が入っていなければ
						{
							map[i + 1][j] = map[i][j] + 1; // 値を代入
							change_flag = true;			   // 値が更新されたことを示す
						}
					}
				}

				if (j > 0) // 範囲チェック
				{
					if ((wall[i][j].south & mask) == NOWALL) // 壁がなければ
					{
						if (map[i][j - 1] == 255) // 値が入っていなければ
						{
							map[i][j - 1] = map[i][j] + 1; // 値を代入
							change_flag = true;			   // 値が更新されたことを示す
						}
					}
				}

				if (i > 0) // 範囲チェック
				{
					if ((wall[i][j].west & mask) == NOWALL) // 壁がなければ
					{
						if (map[i - 1][j] == 255) // 値が入っていなければ
						{
							map[i - 1][j] = map[i][j] + 1; // 値を代入
							change_flag = true;			   // 値が更新されたことを示す
						}
					}
				}
			}
		}

	} while (change_flag == true); // 全体を作り終わるまで待つ
}

void set_wall(int x, int y) // 壁情報を記録
{
	// 引数の座標x,yに壁情報を書き込む
	int n_write, s_write, e_write, w_write;

	// 自分の方向に応じて書き込むデータを生成
	// CONV_SEN2WALL()はmacro.hを参照
	switch (mypos.dir)
	{
	case north: // 北を向いている時

		n_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall); // 前壁の有無を判断
		e_write = CONV_SEN2WALL(sen_r.is_wall);					   // 右壁の有無を判断
		w_write = CONV_SEN2WALL(sen_l.is_wall);					   // 左壁の有無を判断
		s_write = NOWALL;										   // 後ろは必ず壁がない

		break;

	case east: // 東を向いているとき

		e_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall); // 前壁の有無を判断
		s_write = CONV_SEN2WALL(sen_r.is_wall);					   // 右壁の有無を判断
		n_write = CONV_SEN2WALL(sen_l.is_wall);					   // 左壁の有無を判断
		w_write = NOWALL;										   // 後ろは必ず壁がない

		break;

	case south: // 南を向いているとき

		s_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall); // 前壁の有無を判断
		w_write = CONV_SEN2WALL(sen_r.is_wall);					   // 右壁の有無を判断
		e_write = CONV_SEN2WALL(sen_l.is_wall);					   // 左壁の有無を判断
		n_write = NOWALL;										   // 後ろは必ず壁がない

		break;

	case west: // 西を向いているとき

		w_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall); // 前壁の有無を判断
		n_write = CONV_SEN2WALL(sen_r.is_wall);					   // 右壁の有無を判断
		s_write = CONV_SEN2WALL(sen_l.is_wall);					   // 左壁の有無を判断
		e_write = NOWALL;										   // 後ろは必ず壁がない

		break;
	}

	wall[x][y].north = n_write; // 実際に壁情報を書き込み
	wall[x][y].south = s_write; // 実際に壁情報を書き込み
	wall[x][y].east = e_write;	// 実際に壁情報を書き込み
	wall[x][y].west = w_write;	// 実際に壁情報を書き込み

	if (y < MAZESIZE_Y - 1) // 範囲チェック
	{
		wall[x][y + 1].south = n_write; // 反対側から見た壁を書き込み
	}

	if (x < MAZESIZE_X - 1) // 範囲チェック
	{
		wall[x + 1][y].west = e_write; // 反対側から見た壁を書き込み
	}

	if (y > 0) // 範囲チェック
	{
		wall[x][y - 1].north = s_write; // 反対側から見た壁を書き込み
	}

	if (x > 0) // 範囲チェック
	{
		wall[x - 1][y].east = w_write; // 反対側から見た壁を書き込み
	}
}

t_bool is_unknown(int x, int y) // 指定された区画が未探索か否かを判断する関数 未探索:true　探索済:false
{
	// 座標x,yが未探索区間か否かを調べる

	if ((wall[x][y].north == UNKNOWN) || (wall[x][y].east == UNKNOWN) || (wall[x][y].south == UNKNOWN) || (wall[x][y].west == UNKNOWN))
	{				 // どこかの壁情報が不明のままであれば
		return true; // 未探索
	}
	else
	{
		return false; // 探索済
	}
}

int get_priority(int x, int y, t_direction dir) // そのマスの情報から、優先度を算出する
{
	// 座標x,yと、向いている方角dirから優先度を算出する

	// 未探索が一番優先度が高い.(4)
	// それに加え、自分の向きと、行きたい方向から、
	// 前(2)横(1)後(0)の優先度を付加する。

	int priority; // 優先度を記録する変数

	priority = 0;

	if (mypos.dir == dir) // 行きたい方向が現在の進行方向と同じ場合
	{
		priority = 2;
	}
	else if (((4 + mypos.dir - dir) % 4) == 2) // 行きたい方向が現在の進行方向と逆の場合
	{
		priority = 0;
	}
	else // それ以外(左右どちらか)の場合
	{
		priority = 1;
	}

	if (is_unknown(x, y) == true)
	{
		priority += 4; // 未探索の場合優先度をさらに付加
	}

	return priority; // 優先度を返す
}

int get_nextdir(int x, int y, int x1, int y1, int x2, int y2, int x3, int y3, int mask, t_direction *dir)
{
	// ゴール座標x,yに向かう場合、今どちらに行くべきかを判断する。
	// 探索、最短の切り替えのためのmaskを指定、dirは方角を示す
	int little, priority, tmp_priority; // 最小の値を探すために使用する変数

	make_map(x, y, x1, y1, x2, y2, x3, y3, mask); // 歩数Map生成
	little = 255;		  // 最小歩数を255歩(mapがunsigned char型なので)に設定

	priority = 0; // 優先度の初期値は0

	// maskの意味はstatic_parameter.hを参照
	if ((wall[mypos.x][mypos.y].north & mask) == NOWALL) // 北に壁がなければ
	{
		tmp_priority = get_priority(mypos.x, mypos.y + 1, north); // 優先度を算出
		if (map[mypos.x][mypos.y + 1] < little)					  // 一番歩数が小さい方向を見つける
		{
			little = map[mypos.x][mypos.y + 1]; // ひとまず北が歩数が小さい事にする
			*dir = north;						// 方向を保存
			// now_dir = north;
			priority = tmp_priority; // 優先度を保存
		}
		else if (map[mypos.x][mypos.y + 1] == little) // 歩数が同じ場合は優先度から判断する
		{
			if (priority < tmp_priority) // 優先度を評価
			{
				*dir = north; // 方向を更新
				// now_dir = north;
				priority = tmp_priority; // 優先度を保存
			}
		}
	}

	if ((wall[mypos.x][mypos.y].east & mask) == NOWALL) // 東に壁がなければ
	{
		tmp_priority = get_priority(mypos.x + 1, mypos.y, east); // 優先度を算出
		if (map[mypos.x + 1][mypos.y] < little)					 // 一番歩数が小さい方向を見つける
		{
			little = map[mypos.x + 1][mypos.y]; // ひとまず東が歩数が小さい事にする
			*dir = east;						// 方向を保存
			// now_dir = east;
			priority = tmp_priority; // 優先度を保存
		}
		else if (map[mypos.x + 1][mypos.y] == little) // 歩数が同じ場合、優先度から判断
		{
			if (priority < tmp_priority) // 優先度を評価
			{
				*dir = east; // 方向を保存
				// now_dir = east;
				priority = tmp_priority; // 優先度を保存
			}
		}
	}

	if ((wall[mypos.x][mypos.y].south & mask) == NOWALL) // 南に壁がなければ
	{
		tmp_priority = get_priority(mypos.x, mypos.y - 1, south); // 優先度を算出
		if (map[mypos.x][mypos.y - 1] < little)					  // 一番歩数が小さい方向を見つける
		{
			little = map[mypos.x][mypos.y - 1]; // ひとまず南が歩数が小さい事にする
			*dir = south;						// 方向を保存
			// now_dir = south;
			priority = tmp_priority; // 優先度を保存
		}
		else if (map[mypos.x][mypos.y - 1] == little) // 歩数が同じ場合、優先度で評価
		{
			if (priority < tmp_priority) // 優先度を評価
			{
				*dir = south; // 方向を保存
				// now_dir = south;
				priority = tmp_priority; // 優先度を保存
			}
		}
	}

	if ((wall[mypos.x][mypos.y].west & mask) == NOWALL) // 西に壁がなければ
	{
		tmp_priority = get_priority(mypos.x - 1, mypos.y, west); // 優先度を算出
		if (map[mypos.x - 1][mypos.y] < little)					 // 一番歩数が小さい方向を見つける
		{
			little = map[mypos.x - 1][mypos.y]; // 西が歩数が小さい
			*dir = west;						// 方向を保存
			// now_dir = west;
			priority = tmp_priority; // 優先度を保存
		}
		else if (map[mypos.x - 1][mypos.y] == little) // 歩数が同じ場合、優先度で評価
		{
			*dir = west; // 方向を保存
			// now_dir = west;
			priority = tmp_priority; // 優先度を保存
		}
	}

	return ((int)((4 + *dir - mypos.dir) % 4)); // どっちに向かうべきかを返す。
												// 演算の意味はmytyedef.h内のenum宣言から。
}

void search_adachi(int gx, int gy, int gx1, int gy1, int gx2, int gy2, int gx3, int gy3)
{
	// 引数gx,gyに向かって足立法で迷路を探索する
	t_direction glob_nextdir; // 次に向かう方向を記録する変数

	accel = SEARCH_ACCEL;

	switch (get_nextdir(gx, gy, gx1, gy1, gx2, gy2, gx3, gy3, MASK_SEARCH, &glob_nextdir)) // 次に行く方向を戻り値とする関数を呼ぶ
	{
	case front:
		straight(20, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む
		break;

	case right:
		turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);						  // 右に曲がって
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む
		break;

	case left:
		turn(90, TURN_ACCEL, TURN_SPEED, LEFT);							  // 左に曲がって
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む
		break;

	case rear:
		turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);						  // 180ターン
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む
		break;
	}
	accel = SEARCH_ACCEL; // 加速度を設定
	// con_wall.enable = true;					//壁制御を有効にする
	// MOT_CWCCW_R = MOT_CWCCW_L = MOT_FORWARD;		//前方に進む
	len_mouse = 0; // 進んだ距離カウント用変数をリセット
	len_count = 0;
	MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // カウントスタート

	mypos.dir = glob_nextdir; // 方向を更新

	// 向いた方向によって自分の座標を更新する
	switch (mypos.dir)
	{
	case north:
		mypos.y++; // 北を向いた時はY座標を増やす
		break;

	case east:
		mypos.x++; // 東を向いた時はX座標を増やす
		break;

	case south:
		mypos.y--; // 南を向いた時はY座標を減らす
		break;

	case west:
		mypos.x--; // 西を向いたときはX座標を減らす
		break;
	}

	while ((mypos.x != gx) || (mypos.y != gy))
	{ // ゴールするまで繰り返す

		set_wall(mypos.x, mypos.y); // 壁をセット

		switch (get_nextdir(gx, gy, gx1, gy1, gx2, gy2, gx3, gy3, MASK_SEARCH, &glob_nextdir)) // 次に行く方向を戻り値とする関数を呼ぶ
		{
		case front:

			straight(SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む

			break;

		case right:
			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0); // 半区画進む
			if (sen_l.value > TH_SEN_L_BE_T && sen_fr.is_wall == true && sen_fl.is_wall == true)
			{
				turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);
				check_straight(HALF_SECTION, -SEARCH_ACCEL, SEARCH_SPEED, 0);
				straight(20, SEARCH_ACCEL, SEARCH_SPEED, 0);
				turn(90, TURN_ACCEL, TURN_SPEED, LEFT);
				check_straight(HALF_SECTION, -SEARCH_ACCEL, SEARCH_SPEED, 0);
				straight(18, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			}
			else if (sen_l.value > TH_SEN_L_BE_T)
			{
				turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);
				check_straight(HALF_SECTION, -SEARCH_ACCEL, SEARCH_SPEED, 0);
				straight(18, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			}
			else if (sen_fr.is_wall == true && sen_fl.is_wall == true)
			{
				turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);
				check_straight(HALF_SECTION, -SEARCH_ACCEL, SEARCH_SPEED, 0);
				straight(20, SEARCH_ACCEL, SEARCH_SPEED, 0);
				turn(90, TURN_ACCEL, TURN_SPEED, LEFT);
			}

			else
			{
				turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);
			}
			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			break;

		case left:

			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0); // 半区画進む
			if (sen_r.value > TH_SEN_R_BE_T && sen_fr.is_wall == true && sen_fl.is_wall == true)
			{
				turn(180, TURN_ACCEL, TURN_SPEED, LEFT);
				check_straight(HALF_SECTION, -SEARCH_ACCEL, SEARCH_SPEED, 0);
				straight(20, SEARCH_ACCEL, SEARCH_SPEED, 0);
				turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);
				check_straight(HALF_SECTION, -SEARCH_ACCEL, SEARCH_SPEED, 0);
				straight(18, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			}
			else if (sen_r.value > TH_SEN_R_BE_T)
			{
				turn(90, TURN_ACCEL, TURN_SPEED, LEFT);
				check_straight(HALF_SECTION, -SEARCH_ACCEL, SEARCH_SPEED, 0);
				straight(18, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			}
			else if (sen_fr.is_wall == true && sen_fl.is_wall == true)
			{
				turn(180, TURN_ACCEL, TURN_SPEED, LEFT);
				check_straight(HALF_SECTION, -SEARCH_ACCEL, SEARCH_SPEED, 0);
				straight(20, SEARCH_ACCEL, SEARCH_SPEED, 0);
				turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);
			}

			else
			{
				turn(90, TURN_ACCEL, TURN_SPEED, LEFT);
			}
			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			break;

		case rear:

			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0); // 半区画進む
			if (sen_r.is_wall == true && sen_l.is_wall == true && sen_fr.is_wall == true && sen_fl.is_wall == true)
			{
				turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);
				check_straight(HALF_SECTION, -SEARCH_ACCEL, SEARCH_SPEED, 0);
				straight(20, SEARCH_ACCEL, SEARCH_SPEED, 0);
				turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);
				check_straight(HALF_SECTION, -SEARCH_ACCEL, SEARCH_SPEED, 0);
				straight(18, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			}
			else if (sen_fr.is_wall == true && sen_fl.is_wall == true)
			{
				turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);
				check_straight(HALF_SECTION, -SEARCH_ACCEL, SEARCH_SPEED, 0);
				straight(18, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			}
			else
			{
				turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);
			}
			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			break;
		}

		// con_wall.enable = true;						//壁制御を有効にする
		len_mouse = 0;							   // 進んだ距離をカウントする変数をリセット
		MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // カウントスタート

		mypos.dir = glob_nextdir; // 方向を更新

		// 向いた方向によって自分の座標を更新する
		switch (mypos.dir)
		{
		case north:
			mypos.y++; // 北を向いた時はY座標を増やす
			break;

		case east:
			mypos.x++; // 東を向いた時はX座標を増やす
			break;

		case south:
			mypos.y--; // 南を向いた時はY座標を減らす
			break;

		case west:
			mypos.x--; // 西を向いたときはX座標を減らす
			break;
		}
	}
	set_wall(mypos.x, mypos.y); // 壁をセット
	straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);
}
void slalom_search_adachi(int gx, int gy, int gx1, int gy1, int gx2, int gy2, int gx3, int gy3)
{
	// 引数gx,gyに向かって足立法で迷路を探索する
	t_direction glob_nextdir; // 次に向かう方向を記録する変数

	accel = SEARCH_ACCEL;
	len_count = 0;
	slalom_count = 0;

	if (gx != 0 && gy != 0)
	{
		straight(20, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED);
	}

	switch (get_nextdir(gx, gy, gx1, gy1, gx2, gy2, gx3, gy3, MASK_SEARCH, &glob_nextdir)) // 次に行く方向を戻り値とする関数を呼ぶ
	{
	case front:

		straight(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED); // 半区画進む
		break;

	case right:

		turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);								// 右に曲がって
		straight(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED); // 半区画進む
		break;

	case left:

		turn(90, TURN_ACCEL, TURN_SPEED, LEFT);									// 左に曲がって
		straight(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED); // 半区画進む
		break;

	case rear:

		turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);								// 180ターン
		straight(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED); // 半区画進む
		break;
	}
	accel = S_SEARCH_ACCEL; // 加速度を設定
	// con_wall.enable = true;					//壁制御を有効にする
	// MOT_CWCCW_R = MOT_CWCCW_L = MOT_FORWARD;		//前方に進む
	len_mouse = 0;							   // 進んだ距離カウント用変数をリセット
	MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // カウントスタート

	mypos.dir = glob_nextdir; // 方向を更新

	// 向いた方向によって自分の座標を更新する
	switch (mypos.dir)
	{
	case north:
		mypos.y++; // 北を向いた時はY座標を増やす
		break;

	case east:
		mypos.x++; // 東を向いた時はX座標を増やす
		break;

	case south:
		mypos.y--; // 南を向いた時はY座標を減らす
		break;

	case west:
		mypos.x--; // 西を向いたときはX座標を減らす
		break;
	}

	while ((mypos.x != gx) || (mypos.y != gy))
	{ // ゴールするまで繰り返す

		set_wall(mypos.x, mypos.y); // 壁をセット

		switch (get_nextdir(gx, gy, gx1, gy1, gx2, gy2, gx3, gy3, MASK_SEARCH, &glob_nextdir)) // 次に行く方向を戻り値とする関数を呼ぶ
		{
		case front:

			slalom_straight_2(SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED); // 半区画進む
			// slalom_count = 0;
			break;

		case right:
			slalom(90, SLALOM_ACCEL, SLALOM_SPEED, RIGHT);
			// slalom_count++;
			break;

		case left:
			slalom(90, SLALOM_ACCEL, SLALOM_SPEED, LEFT);
			// slalom_count++;
			// slalom_count++;
			break;

		case rear:

			slalom_straight_2(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, 0); // 半区画進む
			if (sen_r.is_wall == true && sen_l.is_wall == true && sen_fr.is_wall == true && sen_fl.is_wall == true)
			{
				turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);
				back_straight(-28, -SEARCH_ACCEL / 2, -SEARCH_SPEED / 2, 0);
				straight(20, SEARCH_ACCEL / 2, SEARCH_SPEED / 2, 0);
				turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);
				back_straight(-28, -SEARCH_ACCEL / 2, -SEARCH_SPEED / 2, 0);
				straight(21, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED);
			}
			else if (sen_fr.is_wall == true && sen_fl.is_wall == true)
			{
				turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);
				back_straight(-28, -SEARCH_ACCEL / 2, -SEARCH_SPEED / 2, 0);
				straight(21, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED);
			}
			else
			{
				turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);
			}
			slalom_straight_2(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED);
			break;
		}

		// con_wall.enable = true;						//壁制御を有効にする
		len_mouse = 0;							   // 進んだ距離をカウントする変数をリセット
		MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // カウントスタート

		mypos.dir = glob_nextdir; // 方向を更新

		// 向いた方向によって自分の座標を更新する
		switch (mypos.dir)
		{
		case north:
			mypos.y++; // 北を向いた時はY座標を増やす
			break;

		case east:
			mypos.x++; // 東を向いた時はX座標を増やす
			break;

		case south:
			mypos.y--; // 南を向いた時はY座標を減らす
			break;

		case west:
			mypos.x--; // 西を向いたときはX座標を減らす
			break;
		}
	}
	set_wall(mypos.x, mypos.y); // 壁をセット
	straight(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, 0);
}

void slalom_search_adachi_2(int gx, int gy, int gx1, int gy1, int gx2, int gy2, int gx3, int gy3)
{
	// 引数gx,gyに向かって足立法で迷路を探索する
	t_direction glob_nextdir; // 次に向かう方向を記録する変数

	// スタックの初期化
	initStack(&stack);

	/* 次の探索候補としてスタート周りのマスの情報をスタックに格納 */
	search_unknown_pos(mypos.x, mypos.y, MASK_SEARCH);

	accel = SEARCH_ACCEL;
	len_count = 0;
	slalom_count = 0;
	x_position = y_position = 0;
	len_mouse = 0;

	if (gx != 0 && gy != 0)
	{
		slalom_straight_2(17, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
	}

	switch (get_nextdir(gx, gy, gx1, gy1, gx2, gy2, gx3, gy3, MASK_SEARCH, &glob_nextdir)) // 次に行く方向を戻り値とする関数を呼ぶ
	{
	case front:

		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む
		break;

	case right:

		turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);						  // 右に曲がって
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む
		break;

	case left:

		turn(90, TURN_ACCEL, TURN_SPEED, LEFT);							  // 左に曲がって
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む
		break;

	case rear:

		turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);						  // 180ターン
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む
		break;
	}
	accel = SEARCH_ACCEL; // 加速度を設定
	// con_wall.enable = true;					//壁制御を有効にする
	// MOT_CWCCW_R = MOT_CWCCW_L = MOT_FORWARD;		//前方に進む
	len_mouse = 0;							   // 進んだ距離カウント用変数をリセット
	MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // カウントスタート

	mypos.dir = glob_nextdir; // 方向を更新

	// 向いた方向によって自分の座標を更新する
	switch (mypos.dir)
	{
	case north:
		mypos.y++; // 北を向いた時はY座標を増やす
		break;

	case east:
		mypos.x++; // 東を向いた時はX座標を増やす
		break;

	case south:
		mypos.y--; // 南を向いた時はY座標を減らす
		break;

	case west:
		mypos.x--; // 西を向いたときはX座標を減らす
		break;
	}

	maze[mypos.x][mypos.y] = PASSED;

	//while ((mypos.x != gx) || (mypos.y != gy))
	while ((maze[gx][gy] != PASSED) || (maze[gx1][gy1] != PASSED) || (maze[gx2][gy2] != PASSED) || (maze[gx3][gy3] != PASSED))
	{ // ゴールするまで繰り返す

		set_wall(mypos.x, mypos.y);						   // 壁をセット
		search_unknown_pos(mypos.x, mypos.y, MASK_SEARCH); // スタックに新しい探索先をプッシュ
		// get_next_target();

		switch (get_nextdir(gx, gy, gx1, gy1, gx2, gy2, gx3, gy3, MASK_SEARCH, &glob_nextdir)) // 次に行く方向を戻り値とする関数を呼ぶ
		{
		case front:

			slalom_straight_2(SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む
			// slalom_count = 0;
			break;

		case right:
			slalom_2(90, SLALOM_ACCEL_2, SLALOM_SPEED_2, RIGHT);
			// slalom_count++;
			break;

		case left:
			slalom_2(90, SLALOM_ACCEL_2, SLALOM_SPEED_2, LEFT);
			// slalom_count++;
			// slalom_count++;
			break;

		case rear: // 壁当ては先判断の方がよい

			if (sen_fl.is_wall == true && sen_fr.is_wall == true && sen_l.is_wall == true)
			{
				slalom_straight_2(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);
				turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);
				back_straight(-28, -SEARCH_ACCEL, -SEARCH_SPEED, 0);
				wait_ms(100);
				slalom_straight_2(20, SEARCH_ACCEL, SEARCH_SPEED / 2, 0);
				turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);
				back_straight(-28, -SEARCH_ACCEL, -SEARCH_SPEED, 0);
				wait_ms(100);
				slalom_straight_2(20, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			}
			else if (sen_fl.is_wall == true && sen_fr.is_wall == true && sen_r.is_wall == true)
			{
				slalom_straight_2(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);
				turn(90, TURN_ACCEL, TURN_SPEED, LEFT);
				back_straight(-28, -SEARCH_ACCEL, -SEARCH_SPEED, 0);
				wait_ms(100);
				slalom_straight_2(20, SEARCH_ACCEL, SEARCH_SPEED / 2, 0);
				turn(90, TURN_ACCEL, TURN_SPEED, LEFT);
				back_straight(-28, -SEARCH_ACCEL, -SEARCH_SPEED, 0);
				wait_ms(100);
				slalom_straight_2(20, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			}
			else if (sen_fr.is_wall == true && sen_fl.is_wall == true)
			{
				slalom_straight_2(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);
				turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);
				back_straight(-28, -SEARCH_ACCEL, -SEARCH_SPEED, 0);
				wait_ms(100);
				slalom_straight_2(20, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			}
			else
			{
				slalom_straight_2(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);
				turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);
			}
			slalom_straight_2(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			break;
		}

		// con_wall.enable = true;						//壁制御を有効にする
		len_mouse = 0;							   // 進んだ距離をカウントする変数をリセット
		MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // カウントスタート

		mypos.dir = glob_nextdir; // 方向を更新

		// 向いた方向によって自分の座標を更新する
		switch (mypos.dir)
		{
		case north:
			mypos.y++; // 北を向いた時はY座標を増やす
			break;

		case east:
			mypos.x++; // 東を向いた時はX座標を増やす
			break;

		case south:
			mypos.y--; // 南を向いた時はY座標を減らす
			break;

		case west:
			mypos.x--; // 西を向いたときはX座標を減らす
			break;
		}
		maze[mypos.x][mypos.y] = PASSED;
	}

	set_wall(mypos.x, mypos.y); // 壁をセット
	straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);
}

void init_map_all(int x, int y)
{
	// 迷路の歩数Mapを初期化する。全体を0xff、引数の座標x,yは0で初期化する

	int i, j;

	for (i = 0; i < MAZESIZE_X; i++) // 迷路の大きさ分ループ(x座標)　
	{
		for (j = 0; j < MAZESIZE_Y; j++) // 迷路の大きさ分ループ(y座標)
		{
			if (is_unknown(i, j) == true)
			{ // そのマスにおいて、東西南北いずれかの方角に未発見の壁があればゴールに設定
				map[i][j] = 0;
			}
			else
			{
				map[i][j] = 255;
			}
		}
	}
	map[x][y] = 0; // ゴール座標の歩数を０に設定
}

void make_map_all(int x, int y, int mask) // 歩数マップを作成する
{
	// 座標x,yをゴールとした歩数Mapを作成する。
	// maskの値(MASK_SEARCH or MASK_SECOND)によって、
	// 探索用の歩数Mapを作るか、最短走行の歩数Mapを作るかが切り替わる
	int i, j;
	t_bool change_flag; // Map作成終了を見極めるためのフラグ

	init_map_all(x, y); // Mapを初期化する

	do
	{
		change_flag = false;			 // 変更がなかった場合にはループを抜ける
		for (i = 0; i < MAZESIZE_X; i++) // 迷路の大きさ分ループ(x座標)
		{
			for (j = 0; j < MAZESIZE_Y; j++) // 迷路の大きさ分ループ(y座標)
			{
				if (map[i][j] == 255) // 255の場合は次へ
				{
					continue;
				}

				if (j < MAZESIZE_Y - 1) // 範囲チェック
				{
					if ((wall[i][j].north & mask) == NOWALL) // 壁がなければ(maskの意味はstatic_parametersを参照)
					{
						if (map[i][j + 1] == 255) // まだ値が入っていなければ
						{
							map[i][j + 1] = map[i][j] + 1; // 値を代入
							change_flag = true;			   // 値が更新されたことを示す
						}
					}
				}

				if (i < MAZESIZE_X - 1) // 範囲チェック
				{
					if ((wall[i][j].east & mask) == NOWALL) // 壁がなければ
					{
						if (map[i + 1][j] == 255) // 値が入っていなければ
						{
							map[i + 1][j] = map[i][j] + 1; // 値を代入
							change_flag = true;			   // 値が更新されたことを示す
						}
					}
				}

				if (j > 0) // 範囲チェック
				{
					if ((wall[i][j].south & mask) == NOWALL) // 壁がなければ
					{
						if (map[i][j - 1] == 255) // 値が入っていなければ
						{
							map[i][j - 1] = map[i][j] + 1; // 値を代入
							change_flag = true;			   // 値が更新されたことを示す
						}
					}
				}

				if (i > 0) // 範囲チェック
				{
					if ((wall[i][j].west & mask) == NOWALL) // 壁がなければ
					{
						if (map[i - 1][j] == 255) // 値が入っていなければ
						{
							map[i - 1][j] = map[i][j] + 1; // 値を代入
							change_flag = true;			   // 値が更新されたことを示す
						}
					}
				}
			}
		}

	} while (change_flag == true); // 全体を作り終わるまで待つ
}

void set_wall_all(int x, int y) // 壁情報を記録
{
	// 引数の座標x,yに壁情報を書き込む
	int n_write, s_write, e_write, w_write;

	// 自分の方向に応じて書き込むデータを生成
	// CONV_SEN2WALL()はmacro.hを参照
	switch (mypos.dir)
	{
	case north: // 北を向いている時

		n_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall); // 前壁の有無を判断
		e_write = CONV_SEN2WALL(sen_r.is_wall);					   // 右壁の有無を判断
		w_write = CONV_SEN2WALL(sen_l.is_wall);					   // 左壁の有無を判断
		s_write = NOWALL;										   // 後ろは必ず壁がない

		break;

	case east: // 東を向いているとき

		e_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall); // 前壁の有無を判断
		s_write = CONV_SEN2WALL(sen_r.is_wall);					   // 右壁の有無を判断
		n_write = CONV_SEN2WALL(sen_l.is_wall);					   // 左壁の有無を判断
		w_write = NOWALL;										   // 後ろは必ず壁がない

		break;

	case south: // 南を向いているとき

		s_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall); // 前壁の有無を判断
		w_write = CONV_SEN2WALL(sen_r.is_wall);					   // 右壁の有無を判断
		e_write = CONV_SEN2WALL(sen_l.is_wall);					   // 左壁の有無を判断
		n_write = NOWALL;										   // 後ろは必ず壁がない

		break;

	case west: // 西を向いているとき

		w_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall); // 前壁の有無を判断
		n_write = CONV_SEN2WALL(sen_r.is_wall);					   // 右壁の有無を判断
		s_write = CONV_SEN2WALL(sen_l.is_wall);					   // 左壁の有無を判断
		e_write = NOWALL;										   // 後ろは必ず壁がない

		break;
	}

	wall[x][y].north = n_write; // 実際に壁情報を書き込み
	wall[x][y].south = s_write; // 実際に壁情報を書き込み
	wall[x][y].east = e_write;	// 実際に壁情報を書き込み
	wall[x][y].west = w_write;	// 実際に壁情報を書き込み

	if (y < MAZESIZE_Y - 1) // 範囲チェック
	{
		wall[x][y + 1].south = n_write; // 反対側から見た壁を書き込み
	}

	if (x < MAZESIZE_X - 1) // 範囲チェック
	{
		wall[x + 1][y].west = e_write; // 反対側から見た壁を書き込み
	}

	if (y > 0) // 範囲チェック
	{
		wall[x][y - 1].north = s_write; // 反対側から見た壁を書き込み
	}

	if (x > 0) // 範囲チェック
	{
		wall[x - 1][y].east = w_write; // 反対側から見た壁を書き込み
	}
}

int get_nextdir_all(int x, int y, int mask, t_direction *dir)
{
	// ゴール座標x,yに向かう場合、今どちらに行くべきかを判断する。
	// 探索、最短の切り替えのためのmaskを指定、dirは方角を示す
	int little, priority, tmp_priority; // 最小の値を探すために使用する変数

	make_map_all(x, y, mask); // 歩数Map生成
	little = 255;			  // 最小歩数を255歩(mapがunsigned char型なので)に設定

	priority = 0; // 優先度の初期値は0

	// maskの意味はstatic_parameter.hを参照
	if ((wall[mypos.x][mypos.y].north & mask) == NOWALL) // 北に壁がなければ
	{
		tmp_priority = get_priority(mypos.x, mypos.y + 1, north); // 優先度を算出
		if (map[mypos.x][mypos.y + 1] < little)					  // 一番歩数が小さい方向を見つける
		{
			little = map[mypos.x][mypos.y + 1]; // ひとまず北が歩数が小さい事にする
			*dir = north;						// 方向を保存
			priority = tmp_priority;			// 優先度を保存
		}
		else if (map[mypos.x][mypos.y + 1] == little) // 歩数が同じ場合は優先度から判断する
		{
			if (priority < tmp_priority) // 優先度を評価
			{
				*dir = north;			 // 方向を更新
				priority = tmp_priority; // 優先度を保存
			}
		}
	}

	if ((wall[mypos.x][mypos.y].east & mask) == NOWALL) // 東に壁がなければ
	{
		tmp_priority = get_priority(mypos.x + 1, mypos.y, east); // 優先度を算出
		if (map[mypos.x + 1][mypos.y] < little)					 // 一番歩数が小さい方向を見つける
		{
			little = map[mypos.x + 1][mypos.y]; // ひとまず東が歩数が小さい事にする
			*dir = east;						// 方向を保存
			priority = tmp_priority;			// 優先度を保存
		}
		else if (map[mypos.x + 1][mypos.y] == little) // 歩数が同じ場合、優先度から判断
		{
			if (priority < tmp_priority) // 優先度を評価
			{
				*dir = east;			 // 方向を保存
				priority = tmp_priority; // 優先度を保存
			}
		}
	}

	if ((wall[mypos.x][mypos.y].south & mask) == NOWALL) // 南に壁がなければ
	{
		tmp_priority = get_priority(mypos.x, mypos.y - 1, south); // 優先度を算出
		if (map[mypos.x][mypos.y - 1] < little)					  // 一番歩数が小さい方向を見つける
		{
			little = map[mypos.x][mypos.y - 1]; // ひとまず南が歩数が小さい事にする
			*dir = south;						// 方向を保存
			priority = tmp_priority;			// 優先度を保存
		}
		else if (map[mypos.x][mypos.y - 1] == little) // 歩数が同じ場合、優先度で評価
		{
			if (priority < tmp_priority) // 優先度を評価
			{
				*dir = south;			 // 方向を保存
				priority = tmp_priority; // 優先度を保存
			}
		}
	}

	if ((wall[mypos.x][mypos.y].west & mask) == NOWALL) // 西に壁がなければ
	{
		tmp_priority = get_priority(mypos.x - 1, mypos.y, west); // 優先度を算出
		if (map[mypos.x - 1][mypos.y] < little)					 // 一番歩数が小さい方向を見つける
		{
			little = map[mypos.x - 1][mypos.y]; // 西が歩数が小さい
			*dir = west;						// 方向を保存
			priority = tmp_priority;			// 優先度を保存
		}
		else if (map[mypos.x - 1][mypos.y] == little) // 歩数が同じ場合、優先度で評価
		{
			*dir = west;			 // 方向を保存
			priority = tmp_priority; // 優先度を保存
		}
	}

	return ((int)((4 + *dir - mypos.dir) % 4)); // どっちに向かうべきかを返す。
												// 演算の意味はmytyedef.h内のenum宣言から。
}

void all_search_adachi(int gx, int gy)
{
	// 引数gx,gyに向かって足立法で迷路を探索する
	t_direction glob_nextdir; // 次に向かう方向を記録する変数

	accel = SEARCH_ACCEL;

	switch (get_nextdir_all(gx, gy, MASK_SEARCH, &glob_nextdir)) // 次に行く方向を戻り値とする関数を呼ぶ
	{
	case front:
		straight(20, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む
		break;

	case right:
		turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);						  // 右に曲がって
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む
		break;

	case left:
		turn(90, TURN_ACCEL, TURN_SPEED, LEFT);							  // 左に曲がって
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む
		break;

	case rear:
		turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);						  // 180ターン
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む
		break;
	}
	accel = SEARCH_ACCEL; // 加速度を設定
	// con_wall.enable = true;					//壁制御を有効にする
	// MOT_CWCCW_R = MOT_CWCCW_L = MOT_FORWARD;		//前方に進む
	len_mouse = 0; // 進んだ距離カウント用変数をリセット
	len_count = 0;
	MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // カウントスタート

	mypos.dir = glob_nextdir; // 方向を更新

	// 向いた方向によって自分の座標を更新する
	switch (mypos.dir)
	{
	case north:
		mypos.y++; // 北を向いた時はY座標を増やす
		break;

	case east:
		mypos.x++; // 東を向いた時はX座標を増やす
		break;

	case south:
		mypos.y--; // 南を向いた時はY座標を減らす
		break;

	case west:
		mypos.x--; // 西を向いたときはX座標を減らす
		break;
	}

	while ((mypos.x != gx) || (mypos.y != gy))
	{ // ゴールするまで繰り返す

		set_wall_all(mypos.x, mypos.y); // 壁をセット

		switch (get_nextdir_all(gx, gy, MASK_SEARCH, &glob_nextdir)) // 次に行く方向を戻り値とする関数を呼ぶ
		{
		case front:

			straight(SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む

			break;

		case right:
			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0); // 半区画進む
			if (sen_l.value > TH_SEN_L_BE_T && sen_fr.is_wall == true && sen_fl.is_wall == true)
			{
				turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);
				check_straight(HALF_SECTION, -SEARCH_ACCEL, SEARCH_SPEED, 0);
				straight(20, SEARCH_ACCEL, SEARCH_SPEED, 0);
				turn(90, TURN_ACCEL, TURN_SPEED, LEFT);
				check_straight(HALF_SECTION, -SEARCH_ACCEL, SEARCH_SPEED, 0);
				straight(18, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			}
			else if (sen_l.value > TH_SEN_L_BE_T)
			{
				turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);
				check_straight(HALF_SECTION, -SEARCH_ACCEL, SEARCH_SPEED, 0);
				straight(18, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			}
			else if (sen_fr.is_wall == true && sen_fl.is_wall == true)
			{
				turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);
				check_straight(HALF_SECTION, -SEARCH_ACCEL, SEARCH_SPEED, 0);
				straight(20, SEARCH_ACCEL, SEARCH_SPEED, 0);
				turn(90, TURN_ACCEL, TURN_SPEED, LEFT);
			}

			else
			{
				turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);
			}
			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			break;

		case left:

			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0); // 半区画進む
			if (sen_r.value > TH_SEN_R_BE_T && sen_fr.is_wall == true && sen_fl.is_wall == true)
			{
				turn(180, TURN_ACCEL, TURN_SPEED, LEFT);
				check_straight(HALF_SECTION, -SEARCH_ACCEL, SEARCH_SPEED, 0);
				straight(20, SEARCH_ACCEL, SEARCH_SPEED, 0);
				turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);
				check_straight(HALF_SECTION, -SEARCH_ACCEL, SEARCH_SPEED, 0);
				straight(18, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			}
			else if (sen_r.value > TH_SEN_R_BE_T)
			{
				turn(90, TURN_ACCEL, TURN_SPEED, LEFT);
				check_straight(HALF_SECTION, -SEARCH_ACCEL, SEARCH_SPEED, 0);
				straight(18, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			}
			else if (sen_fr.is_wall == true && sen_fl.is_wall == true)
			{
				turn(180, TURN_ACCEL, TURN_SPEED, LEFT);
				check_straight(HALF_SECTION, -SEARCH_ACCEL, SEARCH_SPEED, 0);
				straight(20, SEARCH_ACCEL, SEARCH_SPEED, 0);
				turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);
			}

			else
			{
				turn(90, TURN_ACCEL, TURN_SPEED, LEFT);
			}
			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			break;

		case rear:

			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0); // 半区画進む
			if (sen_r.is_wall == true && sen_l.is_wall == true && sen_fr.is_wall == true && sen_fl.is_wall == true)
			{
				turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);
				check_straight(HALF_SECTION, -SEARCH_ACCEL, SEARCH_SPEED, 0);
				straight(20, SEARCH_ACCEL, SEARCH_SPEED, 0);
				turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);
				check_straight(HALF_SECTION, -SEARCH_ACCEL, SEARCH_SPEED, 0);
				straight(18, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			}
			else if (sen_fr.is_wall == true && sen_fl.is_wall == true)
			{
				turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);
				check_straight(HALF_SECTION, -SEARCH_ACCEL, SEARCH_SPEED, 0);
				straight(18, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			}
			else
			{
				turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);
			}
			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			break;
		}

		// con_wall.enable = true;						//壁制御を有効にする
		len_mouse = 0;							   // 進んだ距離をカウントする変数をリセット
		MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // カウントスタート

		mypos.dir = glob_nextdir; // 方向を更新

		// 向いた方向によって自分の座標を更新する
		switch (mypos.dir)
		{
		case north:
			mypos.y++; // 北を向いた時はY座標を増やす
			break;

		case east:
			mypos.x++; // 東を向いた時はX座標を増やす
			break;

		case south:
			mypos.y--; // 南を向いた時はY座標を減らす
			break;

		case west:
			mypos.x--; // 西を向いたときはX座標を減らす
			break;
		}
	}
	set_wall_all(mypos.x, mypos.y); // 壁をセット
	straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);
}
void all_slalom_search_adachi(int gx, int gy)
{
	// 引数gx,gyに向かって足立法で迷路を探索する
	t_direction glob_nextdir; // 次に向かう方向を記録する変数

	accel = SEARCH_ACCEL;
	len_count = 0;
	slalom_count = 0;

	/*if(gx != 0 && gy != 0){
		straight(20,S_SEARCH_ACCEL,S_SEARCH_SPEED,S_SEARCH_SPEED);
	}*/

	switch (get_nextdir_all(gx, gy, MASK_SEARCH, &glob_nextdir)) // 次に行く方向を戻り値とする関数を呼ぶ
	{
	case front:

		slalom_straight_2(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む
		break;

	case right:

		turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);						  // 右に曲がって
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む
		break;

	case left:

		turn(90, TURN_ACCEL, TURN_SPEED, LEFT);							  // 左に曲がって
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む
		break;

	case rear:

		turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);						  // 180ターン
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む
		break;
	}
	accel = SEARCH_ACCEL; // 加速度を設定
	// con_wall.enable = true;					//壁制御を有効にする
	// MOT_CWCCW_R = MOT_CWCCW_L = MOT_FORWARD;		//前方に進む
	len_mouse = 0;							   // 進んだ距離カウント用変数をリセット
	MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // カウントスタート

	mypos.dir = glob_nextdir; // 方向を更新

	// 向いた方向によって自分の座標を更新する
	switch (mypos.dir)
	{
	case north:
		mypos.y++; // 北を向いた時はY座標を増やす
		break;

	case east:
		mypos.x++; // 東を向いた時はX座標を増やす
		break;

	case south:
		mypos.y--; // 南を向いた時はY座標を減らす
		break;

	case west:
		mypos.x--; // 西を向いたときはX座標を減らす
		break;
	}

	while ((mypos.x != gx) || (mypos.y != gy))
	{ // ゴールするまで繰り返す

		set_wall_all(mypos.x, mypos.y); // 壁をセット

		switch (get_nextdir_all(gx, gy, MASK_SEARCH, &glob_nextdir)) // 次に行く方向を戻り値とする関数を呼ぶ
		{
		case front:

			slalom_straight_2(SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む
			break;

		case right:
			slalom_2(90, SLALOM_ACCEL_2, SLALOM_SPEED_2, RIGHT);
			break;

		case left:
			slalom_2(90, SLALOM_ACCEL_2, SLALOM_SPEED_2, LEFT);
			break;

		case rear:

			if (sen_fl.is_wall == true && sen_fr.is_wall == true && sen_l.is_wall == true)
			{
				slalom_straight_2(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);
				turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);
				back_straight(-28, -SEARCH_ACCEL, -SEARCH_SPEED, 0);
				wait_ms(100);
				slalom_straight_2(20, SEARCH_ACCEL, SEARCH_SPEED / 2, 0);
				turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);
				back_straight(-28, -SEARCH_ACCEL, -SEARCH_SPEED, 0);
				wait_ms(100);
				slalom_straight_2(20, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			}
			else if (sen_fl.is_wall == true && sen_fr.is_wall == true && sen_r.is_wall == true)
			{
				slalom_straight_2(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);
				turn(90, TURN_ACCEL, TURN_SPEED, LEFT);
				back_straight(-28, -SEARCH_ACCEL, -SEARCH_SPEED, 0);
				wait_ms(100);
				slalom_straight_2(20, SEARCH_ACCEL, SEARCH_SPEED / 2, 0);
				turn(90, TURN_ACCEL, TURN_SPEED, LEFT);
				back_straight(-28, -SEARCH_ACCEL, -SEARCH_SPEED, 0);
				wait_ms(100);
				slalom_straight_2(20, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			}
			else if (sen_fr.is_wall == true && sen_fl.is_wall == true)
			{
				slalom_straight_2(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);
				turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);
				back_straight(-28, -SEARCH_ACCEL, -SEARCH_SPEED, 0);
				wait_ms(100);
				slalom_straight_2(20, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			}
			else
			{
				slalom_straight_2(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);
				turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);
			}
			slalom_straight_2(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			break;
		}

		// con_wall.enable = true;						//壁制御を有効にする
		len_mouse = 0;							   // 進んだ距離をカウントする変数をリセット
		MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // カウントスタート

		mypos.dir = glob_nextdir; // 方向を更新

		// 向いた方向によって自分の座標を更新する
		switch (mypos.dir)
		{
		case north:
			mypos.y++; // 北を向いた時はY座標を増やす
			break;

		case east:
			mypos.x++; // 東を向いた時はX座標を増やす
			break;

		case south:
			mypos.y--; // 南を向いた時はY座標を減らす
			break;

		case west:
			mypos.x--; // 西を向いたときはX座標を減らす
			break;
		}
	}
	set_wall_all(mypos.x, mypos.y); // 壁をセット
	straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);
}

void slalom_search_adachi_t(int gx, int gy, int gx1, int gy1, int gx2, int gy2, int gx3, int gy3)
{
	// 引数gx,gyに向かって足立法で迷路を探索する
	t_direction glob_nextdir; // 次に向かう方向を記録する変数

	// accel=S_SEARCH_ACCEL;
	len_count = 0;
	slalom_count = 0;
	len_mouse = 0;
	now_dir = north;

	if (gx != 0 && gy != 0)
	{
		slalom_straight_2(10, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED);
	}

	switch (get_nextdir(gx, gy, gx1, gy1, gx2, gy2, gx3, gy3, MASK_SEARCH, &glob_nextdir)) // 次に行く方向を戻り値とする関数を呼ぶ
	{
	case front:

		slalom_straight_2(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED); // 半区画進む
		break;

	case right:

		turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);								// 右に曲がって
		straight(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED); // 半区画進む
		break;

	case left:

		turn(90, TURN_ACCEL, TURN_SPEED, LEFT);									// 左に曲がって
		straight(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED); // 半区画進む
		break;

	case rear:

		turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);								// 180ターン
		straight(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED); // 半区画進む
		break;
	}
	// accel=S_SEARCH_ACCEL;				//加速度を設定
	// con_wall.enable = true;					//壁制御を有効にする
	// MOT_CWCCW_R = MOT_CWCCW_L = MOT_FORWARD;		//前方に進む
	// len_mouse = 0;					//進んだ距離カウント用変数をリセット
	MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // カウントスタート

	mypos.dir = glob_nextdir; // 方向を更新
	now_dir = mypos.dir;

	// 向いた方向によって自分の座標を更新する
	switch (mypos.dir)
	{
	case north:
		mypos.y++; // 北を向いた時はY座標を増やす
		break;

	case east:
		mypos.x++; // 東を向いた時はX座標を増やす
		break;

	case south:
		mypos.y--; // 南を向いた時はY座標を減らす
		break;

	case west:
		mypos.x--; // 西を向いたときはX座標を減らす
		break;
	}

	while ((mypos.x != gx) || (mypos.y != gy))
	{ // ゴールするまで繰り返す

		set_wall(mypos.x, mypos.y); // 壁をセット

		switch (get_nextdir(gx, gy, gx1, gy1, gx2, gy2, gx3, gy3, MASK_SEARCH, &glob_nextdir)) // 次に行く方向を戻り値とする関数を呼ぶ
		{
		case front:

			slalom_straight_2(SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED); // 半区画進む
			break;

		case right:
			Kanayama_sla(RIGHT);
			break;

		case left:
			Kanayama_sla(LEFT);
			break;

		case rear:

			slalom_straight_2(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, 0); // 半区画進む
			turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);							// 180ターン
			slalom_straight_2(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED);
			break;
		}

		// con_wall.enable = true;						//壁制御を有効にする
		len_mouse = 0;							   // 進んだ距離をカウントする変数をリセット
		MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // カウントスタート

		mypos.dir = glob_nextdir; // 方向を更新
		now_dir = mypos.dir;

		// 向いた方向によって自分の座標を更新する
		switch (mypos.dir)
		{
		case north:
			mypos.y++; // 北を向いた時はY座標を増やす
			break;

		case east:
			mypos.x++; // 東を向いた時はX座標を増やす
			break;

		case south:
			mypos.y--; // 南を向いた時はY座標を減らす
			break;

		case west:
			mypos.x--; // 西を向いたときはX座標を減らす
			break;
		}
	}
	set_wall(mypos.x, mypos.y); // 壁をセット
	slalom_straight_2(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, 0);
}

void init_map_D(int x, int y)
{
	// 迷路の歩数Mapを初期化する。全体を0xff、引数の座標x,yは0で初期化する

	int i, j;

	for (i = 0; i < MAZESIZE_X; i++) // 迷路の大きさ分ループ(x座標)　
	{
		for (j = 0; j < MAZESIZE_Y; j++) // 迷路の大きさ分ループ(y座標)
		{
			map[i][j] = 255;

			/*if (is_unknown(i, j) == false)
			{ // 探索済みのマスを全て通過済みとしておく
				maze[i][j] = PASSED;
			}*/
		}
	}

	get_next_target();

	map[x][y] = 0; // ゴール座標の歩数を０に設定
}

void make_map_D(int x, int y, int mask) // 歩数マップを作成する
{
	// 座標x,yをゴールとした歩数Mapを作成する。
	// maskの値(MASK_SEARCH or MASK_SECOND)によって、
	// 探索用の歩数Mapを作るか、最短走行の歩数Mapを作るかが切り替わる
	int i, j;
	t_bool change_flag; // Map作成終了を見極めるためのフラグ

	init_map_D(x, y); // Mapを初期化する

	do
	{
		change_flag = false;			 // 変更がなかった場合にはループを抜ける
		for (i = 0; i < MAZESIZE_X; i++) // 迷路の大きさ分ループ(x座標)
		{
			for (j = 0; j < MAZESIZE_Y; j++) // 迷路の大きさ分ループ(y座標)
			{
				if (map[i][j] == 255) // 255の場合は次へ
				{
					continue;
				}

				if (j < MAZESIZE_Y - 1) // 範囲チェック
				{
					if ((wall[i][j].north & mask) == NOWALL) // 壁がなければ(maskの意味はstatic_parametersを参照)
					{
						if (map[i][j + 1] == 255) // まだ値が入っていなければ
						{
							map[i][j + 1] = map[i][j] + 1; // 値を代入
							change_flag = true;			   // 値が更新されたことを示す
						}
					}
				}

				if (i < MAZESIZE_X - 1) // 範囲チェック
				{
					if ((wall[i][j].east & mask) == NOWALL) // 壁がなければ
					{
						if (map[i + 1][j] == 255) // 値が入っていなければ
						{
							map[i + 1][j] = map[i][j] + 1; // 値を代入
							change_flag = true;			   // 値が更新されたことを示す
						}
					}
				}

				if (j > 0) // 範囲チェック
				{
					if ((wall[i][j].south & mask) == NOWALL) // 壁がなければ
					{
						if (map[i][j - 1] == 255) // 値が入っていなければ
						{
							map[i][j - 1] = map[i][j] + 1; // 値を代入
							change_flag = true;			   // 値が更新されたことを示す
						}
					}
				}

				if (i > 0) // 範囲チェック
				{
					if ((wall[i][j].west & mask) == NOWALL) // 壁がなければ
					{
						if (map[i - 1][j] == 255) // 値が入っていなければ
						{
							map[i - 1][j] = map[i][j] + 1; // 値を代入
							change_flag = true;			   // 値が更新されたことを示す
						}
					}
				}
			}
		}

	} while (change_flag == true); // 全体を作り終わるまで待つ
}

void set_wall_D(int x, int y) // 壁情報を記録
{
	// 引数の座標x,yに壁情報を書き込む
	int n_write, s_write, e_write, w_write;

	// 自分の方向に応じて書き込むデータを生成
	// CONV_SEN2WALL()はmacro.hを参照
	switch (mypos.dir)
	{
	case north: // 北を向いている時

		n_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall); // 前壁の有無を判断
		e_write = CONV_SEN2WALL(sen_r.is_wall);					   // 右壁の有無を判断
		w_write = CONV_SEN2WALL(sen_l.is_wall);					   // 左壁の有無を判断
		s_write = NOWALL;										   // 後ろは必ず壁がない

		break;

	case east: // 東を向いているとき

		e_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall); // 前壁の有無を判断
		s_write = CONV_SEN2WALL(sen_r.is_wall);					   // 右壁の有無を判断
		n_write = CONV_SEN2WALL(sen_l.is_wall);					   // 左壁の有無を判断
		w_write = NOWALL;										   // 後ろは必ず壁がない

		break;

	case south: // 南を向いているとき

		s_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall); // 前壁の有無を判断
		w_write = CONV_SEN2WALL(sen_r.is_wall);					   // 右壁の有無を判断
		e_write = CONV_SEN2WALL(sen_l.is_wall);					   // 左壁の有無を判断
		n_write = NOWALL;										   // 後ろは必ず壁がない

		break;

	case west: // 西を向いているとき

		w_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall); // 前壁の有無を判断
		n_write = CONV_SEN2WALL(sen_r.is_wall);					   // 右壁の有無を判断
		s_write = CONV_SEN2WALL(sen_l.is_wall);					   // 左壁の有無を判断
		e_write = NOWALL;										   // 後ろは必ず壁がない

		break;
	}

	wall[x][y].north = n_write; // 実際に壁情報を書き込み
	wall[x][y].south = s_write; // 実際に壁情報を書き込み
	wall[x][y].east = e_write;	// 実際に壁情報を書き込み
	wall[x][y].west = w_write;	// 実際に壁情報を書き込み

	if (y < MAZESIZE_Y - 1) // 範囲チェック
	{
		wall[x][y + 1].south = n_write; // 反対側から見た壁を書き込み
	}

	if (x < MAZESIZE_X - 1) // 範囲チェック
	{
		wall[x + 1][y].west = e_write; // 反対側から見た壁を書き込み
	}

	if (y > 0) // 範囲チェック
	{
		wall[x][y - 1].north = s_write; // 反対側から見た壁を書き込み
	}

	if (x > 0) // 範囲チェック
	{
		wall[x - 1][y].east = w_write; // 反対側から見た壁を書き込み
	}
}

int get_nextdir_D(int x, int y, int mask, t_direction *dir)
{
	// ゴール座標x,yに向かう場合、今どちらに行くべきかを判断する。
	// 探索、最短の切り替えのためのmaskを指定、dirは方角を示す
	int little, priority, tmp_priority; // 最小の値を探すために使用する変数

	make_map_D(x, y, mask); // 歩数Map生成
	little = 255;			// 最小歩数を255歩(mapがunsigned char型なので)に設定

	priority = 0; // 優先度の初期値は0

	// maskの意味はstatic_parameter.hを参照
	if ((wall[mypos.x][mypos.y].north & mask) == NOWALL) // 北に壁がなければ
	{
		tmp_priority = get_priority(mypos.x, mypos.y + 1, north); // 優先度を算出
		if (map[mypos.x][mypos.y + 1] < little)					  // 一番歩数が小さい方向を見つける
		{
			little = map[mypos.x][mypos.y + 1]; // ひとまず北が歩数が小さい事にする
			*dir = north;						// 方向を保存
			priority = tmp_priority;			// 優先度を保存
		}
		else if (map[mypos.x][mypos.y + 1] == little) // 歩数が同じ場合は優先度から判断する
		{
			if (priority < tmp_priority) // 優先度を評価
			{
				*dir = north;			 // 方向を更新
				priority = tmp_priority; // 優先度を保存
			}
		}
	}

	if ((wall[mypos.x][mypos.y].east & mask) == NOWALL) // 東に壁がなければ
	{
		tmp_priority = get_priority(mypos.x + 1, mypos.y, east); // 優先度を算出
		if (map[mypos.x + 1][mypos.y] < little)					 // 一番歩数が小さい方向を見つける
		{
			little = map[mypos.x + 1][mypos.y]; // ひとまず東が歩数が小さい事にする
			*dir = east;						// 方向を保存
			priority = tmp_priority;			// 優先度を保存
		}
		else if (map[mypos.x + 1][mypos.y] == little) // 歩数が同じ場合、優先度から判断
		{
			if (priority < tmp_priority) // 優先度を評価
			{
				*dir = east;			 // 方向を保存
				priority = tmp_priority; // 優先度を保存
			}
		}
	}

	if ((wall[mypos.x][mypos.y].south & mask) == NOWALL) // 南に壁がなければ
	{
		tmp_priority = get_priority(mypos.x, mypos.y - 1, south); // 優先度を算出
		if (map[mypos.x][mypos.y - 1] < little)					  // 一番歩数が小さい方向を見つける
		{
			little = map[mypos.x][mypos.y - 1]; // ひとまず南が歩数が小さい事にする
			*dir = south;						// 方向を保存
			priority = tmp_priority;			// 優先度を保存
		}
		else if (map[mypos.x][mypos.y - 1] == little) // 歩数が同じ場合、優先度で評価
		{
			if (priority < tmp_priority) // 優先度を評価
			{
				*dir = south;			 // 方向を保存
				priority = tmp_priority; // 優先度を保存
			}
		}
	}

	if ((wall[mypos.x][mypos.y].west & mask) == NOWALL) // 西に壁がなければ
	{
		tmp_priority = get_priority(mypos.x - 1, mypos.y, west); // 優先度を算出
		if (map[mypos.x - 1][mypos.y] < little)					 // 一番歩数が小さい方向を見つける
		{
			little = map[mypos.x - 1][mypos.y]; // 西が歩数が小さい
			*dir = west;						// 方向を保存
			priority = tmp_priority;			// 優先度を保存
		}
		else if (map[mypos.x - 1][mypos.y] == little) // 歩数が同じ場合、優先度で評価
		{
			*dir = west;			 // 方向を保存
			priority = tmp_priority; // 優先度を保存
		}
	}

	return ((int)((4 + *dir - mypos.dir) % 4)); // どっちに向かうべきかを返す。
												// 演算の意味はmytyedef.h内のenum宣言から。
}

/* PUSHする関数 */
void push(STACK_T *stack, POS_T *input)
{

	/* スタックが満杯なら何もせず関数終了 */
	if (stack->tail >= STACK_SIZE - 1)
	{
		// printf("スタックが満杯でPUSHできません\n");
		return 0;
	}

	/* データをデータの最後尾の１つ後ろに格納 */
	stack->data[stack->tail + 1] = *input;

	/* データの最後尾を１つ後ろに移動 */
	stack->tail = stack->tail + 1;
}

/* POPする関数 */
POS_T *pop(STACK_T *stack)
{
	POS_T *ret;

	/* スタックが空なら何もせずに関数終了 */
	if (stack->tail == -1)
	{
		// printf("スタックが空です\n");
		return 0;
	}

	/* データの最後尾からデータを取得 */
	ret = &(stack->data[stack->tail]);

	/* データの最後尾を１つ前にずらす */
	stack->tail = stack->tail - 1;

	/* 取得したデータを返却 */
	return ret;
}

/* (x,y) が通過可能なマスかどうかを確認する関数 */
t_bool check(int x, int y)
{
	if (x < 0 || x >= MAZESIZE_X || y < 0 || y >= MAZESIZE_Y)
	{
		/* (x,y) は迷路外なので通過不可 */
		return false;
	}
	else if (maze[x][y] == PASSED)
	{
		/* (x,y) は通過済みなので通過不可 */
		return false;
	}
	else
	{
		return true;
	}
}

/* 現在値(x, y)から目標を指定する関数*/
void search_unknown_pos(int x, int y, int mask)
{

	POS_T pos;
	stack_flag = false;

	/* 現在探索中のマスから辿れる次の探索候補をスタックに格納 */
		if ((check(x, y + 1) == true) && ((wall[x][y].north & mask) == NOWALL)) // 自分の向きを考慮する
		{
			/* 上方向に辿れる場合 */

			pos.x = x;	   /* 上方向のマスのi座標 */
			pos.y = y + 1; /* 上方向のマスのj座標 */

			/* 上方向のマスを探索候補としてスタックに格納*/
			push(&stack, &pos);
			stack_flag = true;
		}

		if ((check(x + 1, y) == true) && ((wall[x][y].east & mask) == NOWALL))
		{
			/* 右方向に辿れる場合 */
			pos.x = x + 1; /* 右方向のマスのi座標 */
			pos.y = y;	   /* 右方向のマスのj座標 */

			/* 右方向のマスを探索候補としてスタックに格納*/
			push(&stack, &pos);
			stack_flag = true;
		}

		if ((check(x - 1, y) == true) && ((wall[x][y].west & mask) == NOWALL))
		{
			/* 左方向に辿れる場合 */

			pos.x = x - 1; /* 左方向のマスのi座標 */
			pos.y = y;	   /* 左方向のマスのj座標 */

			/* 左方向のマスを探索候補としてスタックに格納*/
			push(&stack, &pos);
			stack_flag = true;
		}

		if ((check(x, y - 1) == true) && ((wall[x][y].south & mask) == NOWALL))
		{
			/* 下方向に辿れる場合 */

			pos.x = x;	   /* 下方向のマスのi座標 */
			pos.y = y - 1; /* 下方向のマスのj座標 */

			/* 下方向のマスを探索候補としてスタックに格納*/
			push(&stack, &pos);
			stack_flag = true;
		}

}

// スタックから取り出したマスを、次に向かう目標とする関数
void get_next_target()
{
	POS_T pos;
	unsigned int x, y;

	//if (stack_flag == true)  //スタックに情報が入れられなかったとき（周囲に未探索のマスがないとき）は更新されないようにする
	//{
	/* スタックから次の探索候補のマスを取得*/
	POS_T *next = pop(&stack);

	/* 探索するマスの座標を取得 */
	x = next->x;
	y = next->y;

	// 一度情報を保存
	bef_x = x;
	bef_y = y;
	//}

	// 探索するマスを目標として設定
	map[bef_x][bef_y] = 0;
}

void depth_first_search(int gx, int gy)
{
	// 引数gx,gyに向かって足立法で迷路を探索する
	t_direction glob_nextdir; // 次に向かう方向を記録する変数
	POS_T pos;

	accel = SEARCH_ACCEL;
	len_count = 0;
	slalom_count = 0;
	x_position = y_position = 0;
	len_mouse = 0;

	// スタックの初期化
	initStack(&stack);

	/* 次の探索候補としてスタート周りのマスの情報をスタックに格納 */
	search_unknown_pos(mypos.x, mypos.y, MASK_SEARCH);

	if (gx != 0 && gy != 0)
	{
		slalom_straight_2(10, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
	}

	switch (get_nextdir_D(gx, gy, MASK_SEARCH, &glob_nextdir)) // 次に行く方向を戻り値とする関数を呼ぶ
	{
	case front:

		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む
		break;

	case right:

		turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);						  // 右に曲がって
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む
		break;

	case left:

		turn(90, TURN_ACCEL, TURN_SPEED, LEFT);							  // 左に曲がって
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む
		break;

	case rear:

		turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);						  // 180ターン
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む
		break;
	}
	accel = SEARCH_ACCEL; // 加速度を設定
	// con_wall.enable = true;					//壁制御を有効にする
	// MOT_CWCCW_R = MOT_CWCCW_L = MOT_FORWARD;		//前方に進む
	len_mouse = 0;							   // 進んだ距離カウント用変数をリセット
	MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // カウントスタート

	mypos.dir = glob_nextdir; // 方向を更新

	// 向いた方向によって自分の座標を更新する
	switch (mypos.dir)
	{
	case north:
		mypos.y++; // 北を向いた時はY座標を増やす
		break;

	case east:
		mypos.x++; // 東を向いた時はX座標を増やす
		break;

	case south:
		mypos.y--; // 南を向いた時はY座標を減らす
		break;

	case west:
		mypos.x--; // 西を向いたときはX座標を減らす
		break;
	}

	maze[mypos.x][mypos.y] = PASSED;

	while ((mypos.x != gx) || (mypos.y != gy))
	{ // ゴールするまで繰り返す

		set_wall_D(mypos.x, mypos.y);					   // 壁をセット
		search_unknown_pos(mypos.x, mypos.y, MASK_SEARCH); // スタックに新しい探索先をプッシュ

		// get_next_target();   //スタックの更新が必要な時だけ呼ぶようにしたい + initmapに入れる

		switch (get_nextdir_D(gx, gy, MASK_SEARCH, &glob_nextdir)) // 次に行く方向を戻り値とする関数を呼ぶ
		{
		case front:

			slalom_straight_2(SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // 半区画進む
			break;

		case right:
			slalom_2(90, SLALOM_ACCEL_2, SLALOM_SPEED_2, RIGHT);
			break;

		case left:
			slalom_2(90, SLALOM_ACCEL_2, SLALOM_SPEED_2, LEFT);
			break;

		case rear: // 壁当ては先判断の方がよい

			if (sen_fl.is_wall == true && sen_fr.is_wall == true && sen_l.is_wall == true)
			{
				slalom_straight_2(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);
				turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);
				back_straight(-28, -SEARCH_ACCEL, -SEARCH_SPEED, 0);
				wait_ms(100);
				slalom_straight_2(20, SEARCH_ACCEL, SEARCH_SPEED / 2, 0);
				turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);
				back_straight(-28, -SEARCH_ACCEL, -SEARCH_SPEED, 0);
				wait_ms(100);
				slalom_straight_2(20, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			}
			else if (sen_fl.is_wall == true && sen_fr.is_wall == true && sen_r.is_wall == true)
			{
				slalom_straight_2(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);
				turn(90, TURN_ACCEL, TURN_SPEED, LEFT);
				back_straight(-28, -SEARCH_ACCEL, -SEARCH_SPEED, 0);
				wait_ms(100);
				slalom_straight_2(20, SEARCH_ACCEL, SEARCH_SPEED / 2, 0);
				turn(90, TURN_ACCEL, TURN_SPEED, LEFT);
				back_straight(-28, -SEARCH_ACCEL, -SEARCH_SPEED, 0);
				wait_ms(100);
				slalom_straight_2(20, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			}
			else if (sen_fr.is_wall == true && sen_fl.is_wall == true)
			{
				slalom_straight_2(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);
				turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);
				back_straight(-28, -SEARCH_ACCEL, -SEARCH_SPEED, 0);
				wait_ms(100);
				slalom_straight_2(20, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			}
			else
			{
				slalom_straight_2(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);
				turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);
			}
			slalom_straight_2(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
			break;
		}

		// con_wall.enable = true;						//壁制御を有効にする
		len_mouse = 0;							   // 進んだ距離をカウントする変数をリセット
		MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // カウントスタート

		mypos.dir = glob_nextdir; // 方向を更新

		// 向いた方向によって自分の座標を更新する
		switch (mypos.dir)
		{
		case north:
			mypos.y++; // 北を向いた時はY座標を増やす
			break;

		case east:
			mypos.x++; // 東を向いた時はX座標を増やす
			break;

		case south:
			mypos.y--; // 南を向いた時はY座標を減らす
			break;

		case west:
			mypos.x--; // 西を向いたときはX座標を減らす
			break;
		}

		maze[mypos.x][mypos.y] = PASSED; // 現在座標を通過済みとして保存
	}
	set_wall_D(mypos.x, mypos.y); // 壁をセット
	straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);
}
