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

void search_lefthand(void); // ����@

void init_map(int x, int y, int x1, int y1, int x2, int y2, int x3, int y3);							   // ����Map�̏�����
void make_map(int x, int y, int x1, int y1, int x2, int y2, int x3, int y3, int mask);					   // �����}�b�v�쐬
void set_wall(int x, int y);							   // �Ǐ���ۑ�
t_bool is_unknown(int x, int y);						   // ���T����Ԃ��ۂ��𔻒�
int get_priority(int x, int y, t_direction dir);		   // �D��x���擾(���T���A�O�������D�悳���)
int get_nextdir(int x, int y, int x1, int y1, int x2, int y2, int x3, int y3, int mask, t_direction *dir); // ���ɍs���ׂ��������擾����
void search_adachi(int gx, int gy);						   // �����@
void slalom_search_adachi(int gx, int gy);				   // �X�����[���ɂ�鑫���@
void slalom_search_adachi_2(int gx, int gy, int gx1, int gy1, int gx2, int gy2, int gx3, int gy3);
void all_search_adachi(int gx, int gy);		   // �����@(�S�ʒT��)
void all_slalom_search_adachi(int gx, int gy); // �X�����[�������@(�S�ʒT��)
void init_map_all(int x, int y);
void make_map_all(int x, int y, int mask);
void set_wall_all(int x, int y);
int get_nextdir_all(int x, int y, int mask, t_direction *dir);
void slalom_search_adachi_t(int gx, int gy); // �O���Ǐ]�ł̒T��
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

/* �X�^�b�N������������֐� */
void initStack(STACK_T *stack)
{

	/* �X�^�b�N����ɐݒ� */
	stack->tail = -1;
}

void search_lefthand(void)
{

	max_speed = SEARCH_SPEED; // �T���̑��x���w��
	accel = SEARCH_ACCEL;

	straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �܂��A�����i��

	while (1)
	{
		if (sen_l.is_wall == false) // ���ɕǂ��Ȃ���΍��ɐi��
		{
			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);			  // �����i���
			turn(90, TURN_ACCEL, TURN_SPEED, LEFT);							  // ���ɋȂ�����
			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��
		}
		else if ((sen_fl.is_wall == false) && (sen_fr.is_wall == false)) // �O�ɕǂ��Ȃ���ΑO�ɐi��
		{
			straight(SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // ����i��
		}
		else if (sen_r.is_wall == false) // �E�ɕǂ��Ȃ���ΉE�ɐi��
		{
			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);			  // �����i��
			turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);						  // �E�ɋȂ���
			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��
		}
		else // ����ȊO�̏ꍇ�A���ɐi��
		{
			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);			  // �����i��
			turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);						  // ���Ɍ���
			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��
		}
	}
}

void init_map(int x, int y, int x1, int y1, int x2, int y2, int x3, int y3)
{
	// ���H�̕���Map������������B�S�̂�0xff�A�����̍��Wx,y��0�ŏ���������

	int i, j;

	for (i = 0; i < MAZESIZE_X; i++) // ���H�̑傫�������[�v(x���W)�@
	{
		for (j = 0; j < MAZESIZE_Y; j++) // ���H�̑傫�������[�v(y���W)
		{
			map[i][j] = 255; // ���ׂ�255�Ŗ��߂�  ex)map[1][1] = 255,map[1][2] = 255, ...map[1][9] = 255,map[2][1] = 255...
		}
	}

	map[x][y] = 0; // �S�[�����W�̕������O�ɐݒ�
	map[x1][y1] = 0;
	map[x2][y2] = 0;
	map[x3][y3] = 0;
}

void make_map(int x, int y, int x1, int y1, int x2, int y2, int x3, int y3, int mask) // �����}�b�v���쐬����
{
	// ���Wx,y���S�[���Ƃ�������Map���쐬����B
	// mask�̒l(MASK_SEARCH or MASK_SECOND)�ɂ���āA
	// �T���p�̕���Map����邩�A�ŒZ���s�̕���Map����邩���؂�ւ��
	int i, j;
	t_bool change_flag; // Map�쐬�I�������ɂ߂邽�߂̃t���O

	init_map(x, y, x1, y1, x2, y2, x3, y3); // Map������������

	do
	{
		change_flag = false;			 // �ύX���Ȃ������ꍇ�ɂ̓��[�v�𔲂���
		for (i = 0; i < MAZESIZE_X; i++) // ���H�̑傫�������[�v(x���W)
		{
			for (j = 0; j < MAZESIZE_Y; j++) // ���H�̑傫�������[�v(y���W)
			{
				if (map[i][j] == 255) // 255�̏ꍇ�͎���
				{
					continue;
				}

				if (j < MAZESIZE_Y - 1) // �͈̓`�F�b�N
				{
					if ((wall[i][j].north & mask) == NOWALL) // �ǂ��Ȃ����(mask�̈Ӗ���static_parameters���Q��)
					{
						if (map[i][j + 1] == 255) // �܂��l�������Ă��Ȃ����
						{
							map[i][j + 1] = map[i][j] + 1; // �l����
							change_flag = true;			   // �l���X�V���ꂽ���Ƃ�����
						}
					}
				}

				if (i < MAZESIZE_X - 1) // �͈̓`�F�b�N
				{
					if ((wall[i][j].east & mask) == NOWALL) // �ǂ��Ȃ����
					{
						if (map[i + 1][j] == 255) // �l�������Ă��Ȃ����
						{
							map[i + 1][j] = map[i][j] + 1; // �l����
							change_flag = true;			   // �l���X�V���ꂽ���Ƃ�����
						}
					}
				}

				if (j > 0) // �͈̓`�F�b�N
				{
					if ((wall[i][j].south & mask) == NOWALL) // �ǂ��Ȃ����
					{
						if (map[i][j - 1] == 255) // �l�������Ă��Ȃ����
						{
							map[i][j - 1] = map[i][j] + 1; // �l����
							change_flag = true;			   // �l���X�V���ꂽ���Ƃ�����
						}
					}
				}

				if (i > 0) // �͈̓`�F�b�N
				{
					if ((wall[i][j].west & mask) == NOWALL) // �ǂ��Ȃ����
					{
						if (map[i - 1][j] == 255) // �l�������Ă��Ȃ����
						{
							map[i - 1][j] = map[i][j] + 1; // �l����
							change_flag = true;			   // �l���X�V���ꂽ���Ƃ�����
						}
					}
				}
			}
		}

	} while (change_flag == true); // �S�̂����I���܂ő҂�
}

void set_wall(int x, int y) // �Ǐ����L�^
{
	// �����̍��Wx,y�ɕǏ�����������
	int n_write, s_write, e_write, w_write;

	// �����̕����ɉ����ď������ރf�[�^�𐶐�
	// CONV_SEN2WALL()��macro.h���Q��
	switch (mypos.dir)
	{
	case north: // �k�������Ă��鎞

		n_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall); // �O�ǂ̗L���𔻒f
		e_write = CONV_SEN2WALL(sen_r.is_wall);					   // �E�ǂ̗L���𔻒f
		w_write = CONV_SEN2WALL(sen_l.is_wall);					   // ���ǂ̗L���𔻒f
		s_write = NOWALL;										   // ���͕K���ǂ��Ȃ�

		break;

	case east: // ���������Ă���Ƃ�

		e_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall); // �O�ǂ̗L���𔻒f
		s_write = CONV_SEN2WALL(sen_r.is_wall);					   // �E�ǂ̗L���𔻒f
		n_write = CONV_SEN2WALL(sen_l.is_wall);					   // ���ǂ̗L���𔻒f
		w_write = NOWALL;										   // ���͕K���ǂ��Ȃ�

		break;

	case south: // ��������Ă���Ƃ�

		s_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall); // �O�ǂ̗L���𔻒f
		w_write = CONV_SEN2WALL(sen_r.is_wall);					   // �E�ǂ̗L���𔻒f
		e_write = CONV_SEN2WALL(sen_l.is_wall);					   // ���ǂ̗L���𔻒f
		n_write = NOWALL;										   // ���͕K���ǂ��Ȃ�

		break;

	case west: // ���������Ă���Ƃ�

		w_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall); // �O�ǂ̗L���𔻒f
		n_write = CONV_SEN2WALL(sen_r.is_wall);					   // �E�ǂ̗L���𔻒f
		s_write = CONV_SEN2WALL(sen_l.is_wall);					   // ���ǂ̗L���𔻒f
		e_write = NOWALL;										   // ���͕K���ǂ��Ȃ�

		break;
	}

	wall[x][y].north = n_write; // ���ۂɕǏ�����������
	wall[x][y].south = s_write; // ���ۂɕǏ�����������
	wall[x][y].east = e_write;	// ���ۂɕǏ�����������
	wall[x][y].west = w_write;	// ���ۂɕǏ�����������

	if (y < MAZESIZE_Y - 1) // �͈̓`�F�b�N
	{
		wall[x][y + 1].south = n_write; // ���Α����猩���ǂ���������
	}

	if (x < MAZESIZE_X - 1) // �͈̓`�F�b�N
	{
		wall[x + 1][y].west = e_write; // ���Α����猩���ǂ���������
	}

	if (y > 0) // �͈̓`�F�b�N
	{
		wall[x][y - 1].north = s_write; // ���Α����猩���ǂ���������
	}

	if (x > 0) // �͈̓`�F�b�N
	{
		wall[x - 1][y].east = w_write; // ���Α����猩���ǂ���������
	}
}

t_bool is_unknown(int x, int y) // �w�肳�ꂽ��悪���T�����ۂ��𔻒f����֐� ���T��:true�@�T����:false
{
	// ���Wx,y�����T����Ԃ��ۂ��𒲂ׂ�

	if ((wall[x][y].north == UNKNOWN) || (wall[x][y].east == UNKNOWN) || (wall[x][y].south == UNKNOWN) || (wall[x][y].west == UNKNOWN))
	{				 // �ǂ����̕Ǐ�񂪕s���̂܂܂ł����
		return true; // ���T��
	}
	else
	{
		return false; // �T����
	}
}

int get_priority(int x, int y, t_direction dir) // ���̃}�X�̏�񂩂�A�D��x���Z�o����
{
	// ���Wx,y�ƁA�����Ă�����pdir����D��x���Z�o����

	// ���T������ԗD��x������.(4)
	// ����ɉ����A�����̌����ƁA�s��������������A
	// �O(2)��(1)��(0)�̗D��x��t������B

	int priority; // �D��x���L�^����ϐ�

	priority = 0;

	if (mypos.dir == dir) // �s���������������݂̐i�s�����Ɠ����ꍇ
	{
		priority = 2;
	}
	else if (((4 + mypos.dir - dir) % 4) == 2) // �s���������������݂̐i�s�����Ƌt�̏ꍇ
	{
		priority = 0;
	}
	else // ����ȊO(���E�ǂ��炩)�̏ꍇ
	{
		priority = 1;
	}

	if (is_unknown(x, y) == true)
	{
		priority += 4; // ���T���̏ꍇ�D��x������ɕt��
	}

	return priority; // �D��x��Ԃ�
}

int get_nextdir(int x, int y, int x1, int y1, int x2, int y2, int x3, int y3, int mask, t_direction *dir)
{
	// �S�[�����Wx,y�Ɍ������ꍇ�A���ǂ���ɍs���ׂ����𔻒f����B
	// �T���A�ŒZ�̐؂�ւ��̂��߂�mask���w��Adir�͕��p������
	int little, priority, tmp_priority; // �ŏ��̒l��T�����߂Ɏg�p����ϐ�

	make_map(x, y, x1, y1, x2, y2, x3, y3, mask); // ����Map����
	little = 255;		  // �ŏ�������255��(map��unsigned char�^�Ȃ̂�)�ɐݒ�

	priority = 0; // �D��x�̏����l��0

	// mask�̈Ӗ���static_parameter.h���Q��
	if ((wall[mypos.x][mypos.y].north & mask) == NOWALL) // �k�ɕǂ��Ȃ����
	{
		tmp_priority = get_priority(mypos.x, mypos.y + 1, north); // �D��x���Z�o
		if (map[mypos.x][mypos.y + 1] < little)					  // ��ԕ�����������������������
		{
			little = map[mypos.x][mypos.y + 1]; // �ЂƂ܂��k�����������������ɂ���
			*dir = north;						// ������ۑ�
			// now_dir = north;
			priority = tmp_priority; // �D��x��ۑ�
		}
		else if (map[mypos.x][mypos.y + 1] == little) // �����������ꍇ�͗D��x���画�f����
		{
			if (priority < tmp_priority) // �D��x��]��
			{
				*dir = north; // �������X�V
				// now_dir = north;
				priority = tmp_priority; // �D��x��ۑ�
			}
		}
	}

	if ((wall[mypos.x][mypos.y].east & mask) == NOWALL) // ���ɕǂ��Ȃ����
	{
		tmp_priority = get_priority(mypos.x + 1, mypos.y, east); // �D��x���Z�o
		if (map[mypos.x + 1][mypos.y] < little)					 // ��ԕ�����������������������
		{
			little = map[mypos.x + 1][mypos.y]; // �ЂƂ܂��������������������ɂ���
			*dir = east;						// ������ۑ�
			// now_dir = east;
			priority = tmp_priority; // �D��x��ۑ�
		}
		else if (map[mypos.x + 1][mypos.y] == little) // �����������ꍇ�A�D��x���画�f
		{
			if (priority < tmp_priority) // �D��x��]��
			{
				*dir = east; // ������ۑ�
				// now_dir = east;
				priority = tmp_priority; // �D��x��ۑ�
			}
		}
	}

	if ((wall[mypos.x][mypos.y].south & mask) == NOWALL) // ��ɕǂ��Ȃ����
	{
		tmp_priority = get_priority(mypos.x, mypos.y - 1, south); // �D��x���Z�o
		if (map[mypos.x][mypos.y - 1] < little)					  // ��ԕ�����������������������
		{
			little = map[mypos.x][mypos.y - 1]; // �ЂƂ܂��삪���������������ɂ���
			*dir = south;						// ������ۑ�
			// now_dir = south;
			priority = tmp_priority; // �D��x��ۑ�
		}
		else if (map[mypos.x][mypos.y - 1] == little) // �����������ꍇ�A�D��x�ŕ]��
		{
			if (priority < tmp_priority) // �D��x��]��
			{
				*dir = south; // ������ۑ�
				// now_dir = south;
				priority = tmp_priority; // �D��x��ۑ�
			}
		}
	}

	if ((wall[mypos.x][mypos.y].west & mask) == NOWALL) // ���ɕǂ��Ȃ����
	{
		tmp_priority = get_priority(mypos.x - 1, mypos.y, west); // �D��x���Z�o
		if (map[mypos.x - 1][mypos.y] < little)					 // ��ԕ�����������������������
		{
			little = map[mypos.x - 1][mypos.y]; // ����������������
			*dir = west;						// ������ۑ�
			// now_dir = west;
			priority = tmp_priority; // �D��x��ۑ�
		}
		else if (map[mypos.x - 1][mypos.y] == little) // �����������ꍇ�A�D��x�ŕ]��
		{
			*dir = west; // ������ۑ�
			// now_dir = west;
			priority = tmp_priority; // �D��x��ۑ�
		}
	}

	return ((int)((4 + *dir - mypos.dir) % 4)); // �ǂ����Ɍ������ׂ�����Ԃ��B
												// ���Z�̈Ӗ���mytyedef.h����enum�錾����B
}

void search_adachi(int gx, int gy, int gx1, int gy1, int gx2, int gy2, int gx3, int gy3)
{
	// ����gx,gy�Ɍ������đ����@�Ŗ��H��T������
	t_direction glob_nextdir; // ���Ɍ������������L�^����ϐ�

	accel = SEARCH_ACCEL;

	switch (get_nextdir(gx, gy, gx1, gy1, gx2, gy2, gx3, gy3, MASK_SEARCH, &glob_nextdir)) // ���ɍs��������߂�l�Ƃ���֐����Ă�
	{
	case front:
		straight(20, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��
		break;

	case right:
		turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);						  // �E�ɋȂ�����
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��
		break;

	case left:
		turn(90, TURN_ACCEL, TURN_SPEED, LEFT);							  // ���ɋȂ�����
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��
		break;

	case rear:
		turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);						  // 180�^�[��
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��
		break;
	}
	accel = SEARCH_ACCEL; // �����x��ݒ�
	// con_wall.enable = true;					//�ǐ����L���ɂ���
	// MOT_CWCCW_R = MOT_CWCCW_L = MOT_FORWARD;		//�O���ɐi��
	len_mouse = 0; // �i�񂾋����J�E���g�p�ϐ������Z�b�g
	len_count = 0;
	MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // �J�E���g�X�^�[�g

	mypos.dir = glob_nextdir; // �������X�V

	// �����������ɂ���Ď����̍��W���X�V����
	switch (mypos.dir)
	{
	case north:
		mypos.y++; // �k������������Y���W�𑝂₷
		break;

	case east:
		mypos.x++; // ��������������X���W�𑝂₷
		break;

	case south:
		mypos.y--; // �������������Y���W�����炷
		break;

	case west:
		mypos.x--; // �����������Ƃ���X���W�����炷
		break;
	}

	while ((mypos.x != gx) || (mypos.y != gy))
	{ // �S�[������܂ŌJ��Ԃ�

		set_wall(mypos.x, mypos.y); // �ǂ��Z�b�g

		switch (get_nextdir(gx, gy, gx1, gy1, gx2, gy2, gx3, gy3, MASK_SEARCH, &glob_nextdir)) // ���ɍs��������߂�l�Ƃ���֐����Ă�
		{
		case front:

			straight(SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��

			break;

		case right:
			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0); // �����i��
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

			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0); // �����i��
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

			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0); // �����i��
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

		// con_wall.enable = true;						//�ǐ����L���ɂ���
		len_mouse = 0;							   // �i�񂾋������J�E���g����ϐ������Z�b�g
		MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // �J�E���g�X�^�[�g

		mypos.dir = glob_nextdir; // �������X�V

		// �����������ɂ���Ď����̍��W���X�V����
		switch (mypos.dir)
		{
		case north:
			mypos.y++; // �k������������Y���W�𑝂₷
			break;

		case east:
			mypos.x++; // ��������������X���W�𑝂₷
			break;

		case south:
			mypos.y--; // �������������Y���W�����炷
			break;

		case west:
			mypos.x--; // �����������Ƃ���X���W�����炷
			break;
		}
	}
	set_wall(mypos.x, mypos.y); // �ǂ��Z�b�g
	straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);
}
void slalom_search_adachi(int gx, int gy, int gx1, int gy1, int gx2, int gy2, int gx3, int gy3)
{
	// ����gx,gy�Ɍ������đ����@�Ŗ��H��T������
	t_direction glob_nextdir; // ���Ɍ������������L�^����ϐ�

	accel = SEARCH_ACCEL;
	len_count = 0;
	slalom_count = 0;

	if (gx != 0 && gy != 0)
	{
		straight(20, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED);
	}

	switch (get_nextdir(gx, gy, gx1, gy1, gx2, gy2, gx3, gy3, MASK_SEARCH, &glob_nextdir)) // ���ɍs��������߂�l�Ƃ���֐����Ă�
	{
	case front:

		straight(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED); // �����i��
		break;

	case right:

		turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);								// �E�ɋȂ�����
		straight(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED); // �����i��
		break;

	case left:

		turn(90, TURN_ACCEL, TURN_SPEED, LEFT);									// ���ɋȂ�����
		straight(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED); // �����i��
		break;

	case rear:

		turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);								// 180�^�[��
		straight(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED); // �����i��
		break;
	}
	accel = S_SEARCH_ACCEL; // �����x��ݒ�
	// con_wall.enable = true;					//�ǐ����L���ɂ���
	// MOT_CWCCW_R = MOT_CWCCW_L = MOT_FORWARD;		//�O���ɐi��
	len_mouse = 0;							   // �i�񂾋����J�E���g�p�ϐ������Z�b�g
	MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // �J�E���g�X�^�[�g

	mypos.dir = glob_nextdir; // �������X�V

	// �����������ɂ���Ď����̍��W���X�V����
	switch (mypos.dir)
	{
	case north:
		mypos.y++; // �k������������Y���W�𑝂₷
		break;

	case east:
		mypos.x++; // ��������������X���W�𑝂₷
		break;

	case south:
		mypos.y--; // �������������Y���W�����炷
		break;

	case west:
		mypos.x--; // �����������Ƃ���X���W�����炷
		break;
	}

	while ((mypos.x != gx) || (mypos.y != gy))
	{ // �S�[������܂ŌJ��Ԃ�

		set_wall(mypos.x, mypos.y); // �ǂ��Z�b�g

		switch (get_nextdir(gx, gy, gx1, gy1, gx2, gy2, gx3, gy3, MASK_SEARCH, &glob_nextdir)) // ���ɍs��������߂�l�Ƃ���֐����Ă�
		{
		case front:

			slalom_straight_2(SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED); // �����i��
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

			slalom_straight_2(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, 0); // �����i��
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

		// con_wall.enable = true;						//�ǐ����L���ɂ���
		len_mouse = 0;							   // �i�񂾋������J�E���g����ϐ������Z�b�g
		MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // �J�E���g�X�^�[�g

		mypos.dir = glob_nextdir; // �������X�V

		// �����������ɂ���Ď����̍��W���X�V����
		switch (mypos.dir)
		{
		case north:
			mypos.y++; // �k������������Y���W�𑝂₷
			break;

		case east:
			mypos.x++; // ��������������X���W�𑝂₷
			break;

		case south:
			mypos.y--; // �������������Y���W�����炷
			break;

		case west:
			mypos.x--; // �����������Ƃ���X���W�����炷
			break;
		}
	}
	set_wall(mypos.x, mypos.y); // �ǂ��Z�b�g
	straight(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, 0);
}

void slalom_search_adachi_2(int gx, int gy, int gx1, int gy1, int gx2, int gy2, int gx3, int gy3)
{
	// ����gx,gy�Ɍ������đ����@�Ŗ��H��T������
	t_direction glob_nextdir; // ���Ɍ������������L�^����ϐ�

	// �X�^�b�N�̏�����
	initStack(&stack);

	/* ���̒T�����Ƃ��ăX�^�[�g����̃}�X�̏����X�^�b�N�Ɋi�[ */
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

	switch (get_nextdir(gx, gy, gx1, gy1, gx2, gy2, gx3, gy3, MASK_SEARCH, &glob_nextdir)) // ���ɍs��������߂�l�Ƃ���֐����Ă�
	{
	case front:

		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��
		break;

	case right:

		turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);						  // �E�ɋȂ�����
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��
		break;

	case left:

		turn(90, TURN_ACCEL, TURN_SPEED, LEFT);							  // ���ɋȂ�����
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��
		break;

	case rear:

		turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);						  // 180�^�[��
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��
		break;
	}
	accel = SEARCH_ACCEL; // �����x��ݒ�
	// con_wall.enable = true;					//�ǐ����L���ɂ���
	// MOT_CWCCW_R = MOT_CWCCW_L = MOT_FORWARD;		//�O���ɐi��
	len_mouse = 0;							   // �i�񂾋����J�E���g�p�ϐ������Z�b�g
	MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // �J�E���g�X�^�[�g

	mypos.dir = glob_nextdir; // �������X�V

	// �����������ɂ���Ď����̍��W���X�V����
	switch (mypos.dir)
	{
	case north:
		mypos.y++; // �k������������Y���W�𑝂₷
		break;

	case east:
		mypos.x++; // ��������������X���W�𑝂₷
		break;

	case south:
		mypos.y--; // �������������Y���W�����炷
		break;

	case west:
		mypos.x--; // �����������Ƃ���X���W�����炷
		break;
	}

	maze[mypos.x][mypos.y] = PASSED;

	//while ((mypos.x != gx) || (mypos.y != gy))
	while ((maze[gx][gy] != PASSED) || (maze[gx1][gy1] != PASSED) || (maze[gx2][gy2] != PASSED) || (maze[gx3][gy3] != PASSED))
	{ // �S�[������܂ŌJ��Ԃ�

		set_wall(mypos.x, mypos.y);						   // �ǂ��Z�b�g
		search_unknown_pos(mypos.x, mypos.y, MASK_SEARCH); // �X�^�b�N�ɐV�����T������v�b�V��
		// get_next_target();

		switch (get_nextdir(gx, gy, gx1, gy1, gx2, gy2, gx3, gy3, MASK_SEARCH, &glob_nextdir)) // ���ɍs��������߂�l�Ƃ���֐����Ă�
		{
		case front:

			slalom_straight_2(SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��
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

		case rear: // �Ǔ��Ă͐攻�f�̕����悢

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

		// con_wall.enable = true;						//�ǐ����L���ɂ���
		len_mouse = 0;							   // �i�񂾋������J�E���g����ϐ������Z�b�g
		MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // �J�E���g�X�^�[�g

		mypos.dir = glob_nextdir; // �������X�V

		// �����������ɂ���Ď����̍��W���X�V����
		switch (mypos.dir)
		{
		case north:
			mypos.y++; // �k������������Y���W�𑝂₷
			break;

		case east:
			mypos.x++; // ��������������X���W�𑝂₷
			break;

		case south:
			mypos.y--; // �������������Y���W�����炷
			break;

		case west:
			mypos.x--; // �����������Ƃ���X���W�����炷
			break;
		}
		maze[mypos.x][mypos.y] = PASSED;
	}

	set_wall(mypos.x, mypos.y); // �ǂ��Z�b�g
	straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);
}

void init_map_all(int x, int y)
{
	// ���H�̕���Map������������B�S�̂�0xff�A�����̍��Wx,y��0�ŏ���������

	int i, j;

	for (i = 0; i < MAZESIZE_X; i++) // ���H�̑傫�������[�v(x���W)�@
	{
		for (j = 0; j < MAZESIZE_Y; j++) // ���H�̑傫�������[�v(y���W)
		{
			if (is_unknown(i, j) == true)
			{ // ���̃}�X�ɂ����āA������k�����ꂩ�̕��p�ɖ������̕ǂ�����΃S�[���ɐݒ�
				map[i][j] = 0;
			}
			else
			{
				map[i][j] = 255;
			}
		}
	}
	map[x][y] = 0; // �S�[�����W�̕������O�ɐݒ�
}

void make_map_all(int x, int y, int mask) // �����}�b�v���쐬����
{
	// ���Wx,y���S�[���Ƃ�������Map���쐬����B
	// mask�̒l(MASK_SEARCH or MASK_SECOND)�ɂ���āA
	// �T���p�̕���Map����邩�A�ŒZ���s�̕���Map����邩���؂�ւ��
	int i, j;
	t_bool change_flag; // Map�쐬�I�������ɂ߂邽�߂̃t���O

	init_map_all(x, y); // Map������������

	do
	{
		change_flag = false;			 // �ύX���Ȃ������ꍇ�ɂ̓��[�v�𔲂���
		for (i = 0; i < MAZESIZE_X; i++) // ���H�̑傫�������[�v(x���W)
		{
			for (j = 0; j < MAZESIZE_Y; j++) // ���H�̑傫�������[�v(y���W)
			{
				if (map[i][j] == 255) // 255�̏ꍇ�͎���
				{
					continue;
				}

				if (j < MAZESIZE_Y - 1) // �͈̓`�F�b�N
				{
					if ((wall[i][j].north & mask) == NOWALL) // �ǂ��Ȃ����(mask�̈Ӗ���static_parameters���Q��)
					{
						if (map[i][j + 1] == 255) // �܂��l�������Ă��Ȃ����
						{
							map[i][j + 1] = map[i][j] + 1; // �l����
							change_flag = true;			   // �l���X�V���ꂽ���Ƃ�����
						}
					}
				}

				if (i < MAZESIZE_X - 1) // �͈̓`�F�b�N
				{
					if ((wall[i][j].east & mask) == NOWALL) // �ǂ��Ȃ����
					{
						if (map[i + 1][j] == 255) // �l�������Ă��Ȃ����
						{
							map[i + 1][j] = map[i][j] + 1; // �l����
							change_flag = true;			   // �l���X�V���ꂽ���Ƃ�����
						}
					}
				}

				if (j > 0) // �͈̓`�F�b�N
				{
					if ((wall[i][j].south & mask) == NOWALL) // �ǂ��Ȃ����
					{
						if (map[i][j - 1] == 255) // �l�������Ă��Ȃ����
						{
							map[i][j - 1] = map[i][j] + 1; // �l����
							change_flag = true;			   // �l���X�V���ꂽ���Ƃ�����
						}
					}
				}

				if (i > 0) // �͈̓`�F�b�N
				{
					if ((wall[i][j].west & mask) == NOWALL) // �ǂ��Ȃ����
					{
						if (map[i - 1][j] == 255) // �l�������Ă��Ȃ����
						{
							map[i - 1][j] = map[i][j] + 1; // �l����
							change_flag = true;			   // �l���X�V���ꂽ���Ƃ�����
						}
					}
				}
			}
		}

	} while (change_flag == true); // �S�̂����I���܂ő҂�
}

void set_wall_all(int x, int y) // �Ǐ����L�^
{
	// �����̍��Wx,y�ɕǏ�����������
	int n_write, s_write, e_write, w_write;

	// �����̕����ɉ����ď������ރf�[�^�𐶐�
	// CONV_SEN2WALL()��macro.h���Q��
	switch (mypos.dir)
	{
	case north: // �k�������Ă��鎞

		n_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall); // �O�ǂ̗L���𔻒f
		e_write = CONV_SEN2WALL(sen_r.is_wall);					   // �E�ǂ̗L���𔻒f
		w_write = CONV_SEN2WALL(sen_l.is_wall);					   // ���ǂ̗L���𔻒f
		s_write = NOWALL;										   // ���͕K���ǂ��Ȃ�

		break;

	case east: // ���������Ă���Ƃ�

		e_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall); // �O�ǂ̗L���𔻒f
		s_write = CONV_SEN2WALL(sen_r.is_wall);					   // �E�ǂ̗L���𔻒f
		n_write = CONV_SEN2WALL(sen_l.is_wall);					   // ���ǂ̗L���𔻒f
		w_write = NOWALL;										   // ���͕K���ǂ��Ȃ�

		break;

	case south: // ��������Ă���Ƃ�

		s_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall); // �O�ǂ̗L���𔻒f
		w_write = CONV_SEN2WALL(sen_r.is_wall);					   // �E�ǂ̗L���𔻒f
		e_write = CONV_SEN2WALL(sen_l.is_wall);					   // ���ǂ̗L���𔻒f
		n_write = NOWALL;										   // ���͕K���ǂ��Ȃ�

		break;

	case west: // ���������Ă���Ƃ�

		w_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall); // �O�ǂ̗L���𔻒f
		n_write = CONV_SEN2WALL(sen_r.is_wall);					   // �E�ǂ̗L���𔻒f
		s_write = CONV_SEN2WALL(sen_l.is_wall);					   // ���ǂ̗L���𔻒f
		e_write = NOWALL;										   // ���͕K���ǂ��Ȃ�

		break;
	}

	wall[x][y].north = n_write; // ���ۂɕǏ�����������
	wall[x][y].south = s_write; // ���ۂɕǏ�����������
	wall[x][y].east = e_write;	// ���ۂɕǏ�����������
	wall[x][y].west = w_write;	// ���ۂɕǏ�����������

	if (y < MAZESIZE_Y - 1) // �͈̓`�F�b�N
	{
		wall[x][y + 1].south = n_write; // ���Α����猩���ǂ���������
	}

	if (x < MAZESIZE_X - 1) // �͈̓`�F�b�N
	{
		wall[x + 1][y].west = e_write; // ���Α����猩���ǂ���������
	}

	if (y > 0) // �͈̓`�F�b�N
	{
		wall[x][y - 1].north = s_write; // ���Α����猩���ǂ���������
	}

	if (x > 0) // �͈̓`�F�b�N
	{
		wall[x - 1][y].east = w_write; // ���Α����猩���ǂ���������
	}
}

int get_nextdir_all(int x, int y, int mask, t_direction *dir)
{
	// �S�[�����Wx,y�Ɍ������ꍇ�A���ǂ���ɍs���ׂ����𔻒f����B
	// �T���A�ŒZ�̐؂�ւ��̂��߂�mask���w��Adir�͕��p������
	int little, priority, tmp_priority; // �ŏ��̒l��T�����߂Ɏg�p����ϐ�

	make_map_all(x, y, mask); // ����Map����
	little = 255;			  // �ŏ�������255��(map��unsigned char�^�Ȃ̂�)�ɐݒ�

	priority = 0; // �D��x�̏����l��0

	// mask�̈Ӗ���static_parameter.h���Q��
	if ((wall[mypos.x][mypos.y].north & mask) == NOWALL) // �k�ɕǂ��Ȃ����
	{
		tmp_priority = get_priority(mypos.x, mypos.y + 1, north); // �D��x���Z�o
		if (map[mypos.x][mypos.y + 1] < little)					  // ��ԕ�����������������������
		{
			little = map[mypos.x][mypos.y + 1]; // �ЂƂ܂��k�����������������ɂ���
			*dir = north;						// ������ۑ�
			priority = tmp_priority;			// �D��x��ۑ�
		}
		else if (map[mypos.x][mypos.y + 1] == little) // �����������ꍇ�͗D��x���画�f����
		{
			if (priority < tmp_priority) // �D��x��]��
			{
				*dir = north;			 // �������X�V
				priority = tmp_priority; // �D��x��ۑ�
			}
		}
	}

	if ((wall[mypos.x][mypos.y].east & mask) == NOWALL) // ���ɕǂ��Ȃ����
	{
		tmp_priority = get_priority(mypos.x + 1, mypos.y, east); // �D��x���Z�o
		if (map[mypos.x + 1][mypos.y] < little)					 // ��ԕ�����������������������
		{
			little = map[mypos.x + 1][mypos.y]; // �ЂƂ܂��������������������ɂ���
			*dir = east;						// ������ۑ�
			priority = tmp_priority;			// �D��x��ۑ�
		}
		else if (map[mypos.x + 1][mypos.y] == little) // �����������ꍇ�A�D��x���画�f
		{
			if (priority < tmp_priority) // �D��x��]��
			{
				*dir = east;			 // ������ۑ�
				priority = tmp_priority; // �D��x��ۑ�
			}
		}
	}

	if ((wall[mypos.x][mypos.y].south & mask) == NOWALL) // ��ɕǂ��Ȃ����
	{
		tmp_priority = get_priority(mypos.x, mypos.y - 1, south); // �D��x���Z�o
		if (map[mypos.x][mypos.y - 1] < little)					  // ��ԕ�����������������������
		{
			little = map[mypos.x][mypos.y - 1]; // �ЂƂ܂��삪���������������ɂ���
			*dir = south;						// ������ۑ�
			priority = tmp_priority;			// �D��x��ۑ�
		}
		else if (map[mypos.x][mypos.y - 1] == little) // �����������ꍇ�A�D��x�ŕ]��
		{
			if (priority < tmp_priority) // �D��x��]��
			{
				*dir = south;			 // ������ۑ�
				priority = tmp_priority; // �D��x��ۑ�
			}
		}
	}

	if ((wall[mypos.x][mypos.y].west & mask) == NOWALL) // ���ɕǂ��Ȃ����
	{
		tmp_priority = get_priority(mypos.x - 1, mypos.y, west); // �D��x���Z�o
		if (map[mypos.x - 1][mypos.y] < little)					 // ��ԕ�����������������������
		{
			little = map[mypos.x - 1][mypos.y]; // ����������������
			*dir = west;						// ������ۑ�
			priority = tmp_priority;			// �D��x��ۑ�
		}
		else if (map[mypos.x - 1][mypos.y] == little) // �����������ꍇ�A�D��x�ŕ]��
		{
			*dir = west;			 // ������ۑ�
			priority = tmp_priority; // �D��x��ۑ�
		}
	}

	return ((int)((4 + *dir - mypos.dir) % 4)); // �ǂ����Ɍ������ׂ�����Ԃ��B
												// ���Z�̈Ӗ���mytyedef.h����enum�錾����B
}

void all_search_adachi(int gx, int gy)
{
	// ����gx,gy�Ɍ������đ����@�Ŗ��H��T������
	t_direction glob_nextdir; // ���Ɍ������������L�^����ϐ�

	accel = SEARCH_ACCEL;

	switch (get_nextdir_all(gx, gy, MASK_SEARCH, &glob_nextdir)) // ���ɍs��������߂�l�Ƃ���֐����Ă�
	{
	case front:
		straight(20, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��
		break;

	case right:
		turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);						  // �E�ɋȂ�����
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��
		break;

	case left:
		turn(90, TURN_ACCEL, TURN_SPEED, LEFT);							  // ���ɋȂ�����
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��
		break;

	case rear:
		turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);						  // 180�^�[��
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��
		break;
	}
	accel = SEARCH_ACCEL; // �����x��ݒ�
	// con_wall.enable = true;					//�ǐ����L���ɂ���
	// MOT_CWCCW_R = MOT_CWCCW_L = MOT_FORWARD;		//�O���ɐi��
	len_mouse = 0; // �i�񂾋����J�E���g�p�ϐ������Z�b�g
	len_count = 0;
	MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // �J�E���g�X�^�[�g

	mypos.dir = glob_nextdir; // �������X�V

	// �����������ɂ���Ď����̍��W���X�V����
	switch (mypos.dir)
	{
	case north:
		mypos.y++; // �k������������Y���W�𑝂₷
		break;

	case east:
		mypos.x++; // ��������������X���W�𑝂₷
		break;

	case south:
		mypos.y--; // �������������Y���W�����炷
		break;

	case west:
		mypos.x--; // �����������Ƃ���X���W�����炷
		break;
	}

	while ((mypos.x != gx) || (mypos.y != gy))
	{ // �S�[������܂ŌJ��Ԃ�

		set_wall_all(mypos.x, mypos.y); // �ǂ��Z�b�g

		switch (get_nextdir_all(gx, gy, MASK_SEARCH, &glob_nextdir)) // ���ɍs��������߂�l�Ƃ���֐����Ă�
		{
		case front:

			straight(SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��

			break;

		case right:
			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0); // �����i��
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

			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0); // �����i��
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

			straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0); // �����i��
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

		// con_wall.enable = true;						//�ǐ����L���ɂ���
		len_mouse = 0;							   // �i�񂾋������J�E���g����ϐ������Z�b�g
		MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // �J�E���g�X�^�[�g

		mypos.dir = glob_nextdir; // �������X�V

		// �����������ɂ���Ď����̍��W���X�V����
		switch (mypos.dir)
		{
		case north:
			mypos.y++; // �k������������Y���W�𑝂₷
			break;

		case east:
			mypos.x++; // ��������������X���W�𑝂₷
			break;

		case south:
			mypos.y--; // �������������Y���W�����炷
			break;

		case west:
			mypos.x--; // �����������Ƃ���X���W�����炷
			break;
		}
	}
	set_wall_all(mypos.x, mypos.y); // �ǂ��Z�b�g
	straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);
}
void all_slalom_search_adachi(int gx, int gy)
{
	// ����gx,gy�Ɍ������đ����@�Ŗ��H��T������
	t_direction glob_nextdir; // ���Ɍ������������L�^����ϐ�

	accel = SEARCH_ACCEL;
	len_count = 0;
	slalom_count = 0;

	/*if(gx != 0 && gy != 0){
		straight(20,S_SEARCH_ACCEL,S_SEARCH_SPEED,S_SEARCH_SPEED);
	}*/

	switch (get_nextdir_all(gx, gy, MASK_SEARCH, &glob_nextdir)) // ���ɍs��������߂�l�Ƃ���֐����Ă�
	{
	case front:

		slalom_straight_2(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��
		break;

	case right:

		turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);						  // �E�ɋȂ�����
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��
		break;

	case left:

		turn(90, TURN_ACCEL, TURN_SPEED, LEFT);							  // ���ɋȂ�����
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��
		break;

	case rear:

		turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);						  // 180�^�[��
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��
		break;
	}
	accel = SEARCH_ACCEL; // �����x��ݒ�
	// con_wall.enable = true;					//�ǐ����L���ɂ���
	// MOT_CWCCW_R = MOT_CWCCW_L = MOT_FORWARD;		//�O���ɐi��
	len_mouse = 0;							   // �i�񂾋����J�E���g�p�ϐ������Z�b�g
	MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // �J�E���g�X�^�[�g

	mypos.dir = glob_nextdir; // �������X�V

	// �����������ɂ���Ď����̍��W���X�V����
	switch (mypos.dir)
	{
	case north:
		mypos.y++; // �k������������Y���W�𑝂₷
		break;

	case east:
		mypos.x++; // ��������������X���W�𑝂₷
		break;

	case south:
		mypos.y--; // �������������Y���W�����炷
		break;

	case west:
		mypos.x--; // �����������Ƃ���X���W�����炷
		break;
	}

	while ((mypos.x != gx) || (mypos.y != gy))
	{ // �S�[������܂ŌJ��Ԃ�

		set_wall_all(mypos.x, mypos.y); // �ǂ��Z�b�g

		switch (get_nextdir_all(gx, gy, MASK_SEARCH, &glob_nextdir)) // ���ɍs��������߂�l�Ƃ���֐����Ă�
		{
		case front:

			slalom_straight_2(SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��
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

		// con_wall.enable = true;						//�ǐ����L���ɂ���
		len_mouse = 0;							   // �i�񂾋������J�E���g����ϐ������Z�b�g
		MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // �J�E���g�X�^�[�g

		mypos.dir = glob_nextdir; // �������X�V

		// �����������ɂ���Ď����̍��W���X�V����
		switch (mypos.dir)
		{
		case north:
			mypos.y++; // �k������������Y���W�𑝂₷
			break;

		case east:
			mypos.x++; // ��������������X���W�𑝂₷
			break;

		case south:
			mypos.y--; // �������������Y���W�����炷
			break;

		case west:
			mypos.x--; // �����������Ƃ���X���W�����炷
			break;
		}
	}
	set_wall_all(mypos.x, mypos.y); // �ǂ��Z�b�g
	straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);
}

void slalom_search_adachi_t(int gx, int gy, int gx1, int gy1, int gx2, int gy2, int gx3, int gy3)
{
	// ����gx,gy�Ɍ������đ����@�Ŗ��H��T������
	t_direction glob_nextdir; // ���Ɍ������������L�^����ϐ�

	// accel=S_SEARCH_ACCEL;
	len_count = 0;
	slalom_count = 0;
	len_mouse = 0;
	now_dir = north;

	if (gx != 0 && gy != 0)
	{
		slalom_straight_2(10, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED);
	}

	switch (get_nextdir(gx, gy, gx1, gy1, gx2, gy2, gx3, gy3, MASK_SEARCH, &glob_nextdir)) // ���ɍs��������߂�l�Ƃ���֐����Ă�
	{
	case front:

		slalom_straight_2(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED); // �����i��
		break;

	case right:

		turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);								// �E�ɋȂ�����
		straight(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED); // �����i��
		break;

	case left:

		turn(90, TURN_ACCEL, TURN_SPEED, LEFT);									// ���ɋȂ�����
		straight(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED); // �����i��
		break;

	case rear:

		turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);								// 180�^�[��
		straight(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED); // �����i��
		break;
	}
	// accel=S_SEARCH_ACCEL;				//�����x��ݒ�
	// con_wall.enable = true;					//�ǐ����L���ɂ���
	// MOT_CWCCW_R = MOT_CWCCW_L = MOT_FORWARD;		//�O���ɐi��
	// len_mouse = 0;					//�i�񂾋����J�E���g�p�ϐ������Z�b�g
	MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // �J�E���g�X�^�[�g

	mypos.dir = glob_nextdir; // �������X�V
	now_dir = mypos.dir;

	// �����������ɂ���Ď����̍��W���X�V����
	switch (mypos.dir)
	{
	case north:
		mypos.y++; // �k������������Y���W�𑝂₷
		break;

	case east:
		mypos.x++; // ��������������X���W�𑝂₷
		break;

	case south:
		mypos.y--; // �������������Y���W�����炷
		break;

	case west:
		mypos.x--; // �����������Ƃ���X���W�����炷
		break;
	}

	while ((mypos.x != gx) || (mypos.y != gy))
	{ // �S�[������܂ŌJ��Ԃ�

		set_wall(mypos.x, mypos.y); // �ǂ��Z�b�g

		switch (get_nextdir(gx, gy, gx1, gy1, gx2, gy2, gx3, gy3, MASK_SEARCH, &glob_nextdir)) // ���ɍs��������߂�l�Ƃ���֐����Ă�
		{
		case front:

			slalom_straight_2(SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED); // �����i��
			break;

		case right:
			Kanayama_sla(RIGHT);
			break;

		case left:
			Kanayama_sla(LEFT);
			break;

		case rear:

			slalom_straight_2(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, 0); // �����i��
			turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);							// 180�^�[��
			slalom_straight_2(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, S_SEARCH_SPEED);
			break;
		}

		// con_wall.enable = true;						//�ǐ����L���ɂ���
		len_mouse = 0;							   // �i�񂾋������J�E���g����ϐ������Z�b�g
		MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // �J�E���g�X�^�[�g

		mypos.dir = glob_nextdir; // �������X�V
		now_dir = mypos.dir;

		// �����������ɂ���Ď����̍��W���X�V����
		switch (mypos.dir)
		{
		case north:
			mypos.y++; // �k������������Y���W�𑝂₷
			break;

		case east:
			mypos.x++; // ��������������X���W�𑝂₷
			break;

		case south:
			mypos.y--; // �������������Y���W�����炷
			break;

		case west:
			mypos.x--; // �����������Ƃ���X���W�����炷
			break;
		}
	}
	set_wall(mypos.x, mypos.y); // �ǂ��Z�b�g
	slalom_straight_2(HALF_SECTION, S_SEARCH_ACCEL, S_SEARCH_SPEED, 0);
}

void init_map_D(int x, int y)
{
	// ���H�̕���Map������������B�S�̂�0xff�A�����̍��Wx,y��0�ŏ���������

	int i, j;

	for (i = 0; i < MAZESIZE_X; i++) // ���H�̑傫�������[�v(x���W)�@
	{
		for (j = 0; j < MAZESIZE_Y; j++) // ���H�̑傫�������[�v(y���W)
		{
			map[i][j] = 255;

			/*if (is_unknown(i, j) == false)
			{ // �T���ς݂̃}�X��S�Ēʉߍς݂Ƃ��Ă���
				maze[i][j] = PASSED;
			}*/
		}
	}

	get_next_target();

	map[x][y] = 0; // �S�[�����W�̕������O�ɐݒ�
}

void make_map_D(int x, int y, int mask) // �����}�b�v���쐬����
{
	// ���Wx,y���S�[���Ƃ�������Map���쐬����B
	// mask�̒l(MASK_SEARCH or MASK_SECOND)�ɂ���āA
	// �T���p�̕���Map����邩�A�ŒZ���s�̕���Map����邩���؂�ւ��
	int i, j;
	t_bool change_flag; // Map�쐬�I�������ɂ߂邽�߂̃t���O

	init_map_D(x, y); // Map������������

	do
	{
		change_flag = false;			 // �ύX���Ȃ������ꍇ�ɂ̓��[�v�𔲂���
		for (i = 0; i < MAZESIZE_X; i++) // ���H�̑傫�������[�v(x���W)
		{
			for (j = 0; j < MAZESIZE_Y; j++) // ���H�̑傫�������[�v(y���W)
			{
				if (map[i][j] == 255) // 255�̏ꍇ�͎���
				{
					continue;
				}

				if (j < MAZESIZE_Y - 1) // �͈̓`�F�b�N
				{
					if ((wall[i][j].north & mask) == NOWALL) // �ǂ��Ȃ����(mask�̈Ӗ���static_parameters���Q��)
					{
						if (map[i][j + 1] == 255) // �܂��l�������Ă��Ȃ����
						{
							map[i][j + 1] = map[i][j] + 1; // �l����
							change_flag = true;			   // �l���X�V���ꂽ���Ƃ�����
						}
					}
				}

				if (i < MAZESIZE_X - 1) // �͈̓`�F�b�N
				{
					if ((wall[i][j].east & mask) == NOWALL) // �ǂ��Ȃ����
					{
						if (map[i + 1][j] == 255) // �l�������Ă��Ȃ����
						{
							map[i + 1][j] = map[i][j] + 1; // �l����
							change_flag = true;			   // �l���X�V���ꂽ���Ƃ�����
						}
					}
				}

				if (j > 0) // �͈̓`�F�b�N
				{
					if ((wall[i][j].south & mask) == NOWALL) // �ǂ��Ȃ����
					{
						if (map[i][j - 1] == 255) // �l�������Ă��Ȃ����
						{
							map[i][j - 1] = map[i][j] + 1; // �l����
							change_flag = true;			   // �l���X�V���ꂽ���Ƃ�����
						}
					}
				}

				if (i > 0) // �͈̓`�F�b�N
				{
					if ((wall[i][j].west & mask) == NOWALL) // �ǂ��Ȃ����
					{
						if (map[i - 1][j] == 255) // �l�������Ă��Ȃ����
						{
							map[i - 1][j] = map[i][j] + 1; // �l����
							change_flag = true;			   // �l���X�V���ꂽ���Ƃ�����
						}
					}
				}
			}
		}

	} while (change_flag == true); // �S�̂����I���܂ő҂�
}

void set_wall_D(int x, int y) // �Ǐ����L�^
{
	// �����̍��Wx,y�ɕǏ�����������
	int n_write, s_write, e_write, w_write;

	// �����̕����ɉ����ď������ރf�[�^�𐶐�
	// CONV_SEN2WALL()��macro.h���Q��
	switch (mypos.dir)
	{
	case north: // �k�������Ă��鎞

		n_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall); // �O�ǂ̗L���𔻒f
		e_write = CONV_SEN2WALL(sen_r.is_wall);					   // �E�ǂ̗L���𔻒f
		w_write = CONV_SEN2WALL(sen_l.is_wall);					   // ���ǂ̗L���𔻒f
		s_write = NOWALL;										   // ���͕K���ǂ��Ȃ�

		break;

	case east: // ���������Ă���Ƃ�

		e_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall); // �O�ǂ̗L���𔻒f
		s_write = CONV_SEN2WALL(sen_r.is_wall);					   // �E�ǂ̗L���𔻒f
		n_write = CONV_SEN2WALL(sen_l.is_wall);					   // ���ǂ̗L���𔻒f
		w_write = NOWALL;										   // ���͕K���ǂ��Ȃ�

		break;

	case south: // ��������Ă���Ƃ�

		s_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall); // �O�ǂ̗L���𔻒f
		w_write = CONV_SEN2WALL(sen_r.is_wall);					   // �E�ǂ̗L���𔻒f
		e_write = CONV_SEN2WALL(sen_l.is_wall);					   // ���ǂ̗L���𔻒f
		n_write = NOWALL;										   // ���͕K���ǂ��Ȃ�

		break;

	case west: // ���������Ă���Ƃ�

		w_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall); // �O�ǂ̗L���𔻒f
		n_write = CONV_SEN2WALL(sen_r.is_wall);					   // �E�ǂ̗L���𔻒f
		s_write = CONV_SEN2WALL(sen_l.is_wall);					   // ���ǂ̗L���𔻒f
		e_write = NOWALL;										   // ���͕K���ǂ��Ȃ�

		break;
	}

	wall[x][y].north = n_write; // ���ۂɕǏ�����������
	wall[x][y].south = s_write; // ���ۂɕǏ�����������
	wall[x][y].east = e_write;	// ���ۂɕǏ�����������
	wall[x][y].west = w_write;	// ���ۂɕǏ�����������

	if (y < MAZESIZE_Y - 1) // �͈̓`�F�b�N
	{
		wall[x][y + 1].south = n_write; // ���Α����猩���ǂ���������
	}

	if (x < MAZESIZE_X - 1) // �͈̓`�F�b�N
	{
		wall[x + 1][y].west = e_write; // ���Α����猩���ǂ���������
	}

	if (y > 0) // �͈̓`�F�b�N
	{
		wall[x][y - 1].north = s_write; // ���Α����猩���ǂ���������
	}

	if (x > 0) // �͈̓`�F�b�N
	{
		wall[x - 1][y].east = w_write; // ���Α����猩���ǂ���������
	}
}

int get_nextdir_D(int x, int y, int mask, t_direction *dir)
{
	// �S�[�����Wx,y�Ɍ������ꍇ�A���ǂ���ɍs���ׂ����𔻒f����B
	// �T���A�ŒZ�̐؂�ւ��̂��߂�mask���w��Adir�͕��p������
	int little, priority, tmp_priority; // �ŏ��̒l��T�����߂Ɏg�p����ϐ�

	make_map_D(x, y, mask); // ����Map����
	little = 255;			// �ŏ�������255��(map��unsigned char�^�Ȃ̂�)�ɐݒ�

	priority = 0; // �D��x�̏����l��0

	// mask�̈Ӗ���static_parameter.h���Q��
	if ((wall[mypos.x][mypos.y].north & mask) == NOWALL) // �k�ɕǂ��Ȃ����
	{
		tmp_priority = get_priority(mypos.x, mypos.y + 1, north); // �D��x���Z�o
		if (map[mypos.x][mypos.y + 1] < little)					  // ��ԕ�����������������������
		{
			little = map[mypos.x][mypos.y + 1]; // �ЂƂ܂��k�����������������ɂ���
			*dir = north;						// ������ۑ�
			priority = tmp_priority;			// �D��x��ۑ�
		}
		else if (map[mypos.x][mypos.y + 1] == little) // �����������ꍇ�͗D��x���画�f����
		{
			if (priority < tmp_priority) // �D��x��]��
			{
				*dir = north;			 // �������X�V
				priority = tmp_priority; // �D��x��ۑ�
			}
		}
	}

	if ((wall[mypos.x][mypos.y].east & mask) == NOWALL) // ���ɕǂ��Ȃ����
	{
		tmp_priority = get_priority(mypos.x + 1, mypos.y, east); // �D��x���Z�o
		if (map[mypos.x + 1][mypos.y] < little)					 // ��ԕ�����������������������
		{
			little = map[mypos.x + 1][mypos.y]; // �ЂƂ܂��������������������ɂ���
			*dir = east;						// ������ۑ�
			priority = tmp_priority;			// �D��x��ۑ�
		}
		else if (map[mypos.x + 1][mypos.y] == little) // �����������ꍇ�A�D��x���画�f
		{
			if (priority < tmp_priority) // �D��x��]��
			{
				*dir = east;			 // ������ۑ�
				priority = tmp_priority; // �D��x��ۑ�
			}
		}
	}

	if ((wall[mypos.x][mypos.y].south & mask) == NOWALL) // ��ɕǂ��Ȃ����
	{
		tmp_priority = get_priority(mypos.x, mypos.y - 1, south); // �D��x���Z�o
		if (map[mypos.x][mypos.y - 1] < little)					  // ��ԕ�����������������������
		{
			little = map[mypos.x][mypos.y - 1]; // �ЂƂ܂��삪���������������ɂ���
			*dir = south;						// ������ۑ�
			priority = tmp_priority;			// �D��x��ۑ�
		}
		else if (map[mypos.x][mypos.y - 1] == little) // �����������ꍇ�A�D��x�ŕ]��
		{
			if (priority < tmp_priority) // �D��x��]��
			{
				*dir = south;			 // ������ۑ�
				priority = tmp_priority; // �D��x��ۑ�
			}
		}
	}

	if ((wall[mypos.x][mypos.y].west & mask) == NOWALL) // ���ɕǂ��Ȃ����
	{
		tmp_priority = get_priority(mypos.x - 1, mypos.y, west); // �D��x���Z�o
		if (map[mypos.x - 1][mypos.y] < little)					 // ��ԕ�����������������������
		{
			little = map[mypos.x - 1][mypos.y]; // ����������������
			*dir = west;						// ������ۑ�
			priority = tmp_priority;			// �D��x��ۑ�
		}
		else if (map[mypos.x - 1][mypos.y] == little) // �����������ꍇ�A�D��x�ŕ]��
		{
			*dir = west;			 // ������ۑ�
			priority = tmp_priority; // �D��x��ۑ�
		}
	}

	return ((int)((4 + *dir - mypos.dir) % 4)); // �ǂ����Ɍ������ׂ�����Ԃ��B
												// ���Z�̈Ӗ���mytyedef.h����enum�錾����B
}

/* PUSH����֐� */
void push(STACK_T *stack, POS_T *input)
{

	/* �X�^�b�N�����t�Ȃ牽�������֐��I�� */
	if (stack->tail >= STACK_SIZE - 1)
	{
		// printf("�X�^�b�N�����t��PUSH�ł��܂���\n");
		return 0;
	}

	/* �f�[�^���f�[�^�̍Ō���̂P���Ɋi�[ */
	stack->data[stack->tail + 1] = *input;

	/* �f�[�^�̍Ō�����P���Ɉړ� */
	stack->tail = stack->tail + 1;
}

/* POP����֐� */
POS_T *pop(STACK_T *stack)
{
	POS_T *ret;

	/* �X�^�b�N����Ȃ牽�������Ɋ֐��I�� */
	if (stack->tail == -1)
	{
		// printf("�X�^�b�N����ł�\n");
		return 0;
	}

	/* �f�[�^�̍Ō������f�[�^���擾 */
	ret = &(stack->data[stack->tail]);

	/* �f�[�^�̍Ō�����P�O�ɂ��炷 */
	stack->tail = stack->tail - 1;

	/* �擾�����f�[�^��ԋp */
	return ret;
}

/* (x,y) ���ʉ߉\�ȃ}�X���ǂ������m�F����֐� */
t_bool check(int x, int y)
{
	if (x < 0 || x >= MAZESIZE_X || y < 0 || y >= MAZESIZE_Y)
	{
		/* (x,y) �͖��H�O�Ȃ̂Œʉߕs�� */
		return false;
	}
	else if (maze[x][y] == PASSED)
	{
		/* (x,y) �͒ʉߍς݂Ȃ̂Œʉߕs�� */
		return false;
	}
	else
	{
		return true;
	}
}

/* ���ݒl(x, y)����ڕW���w�肷��֐�*/
void search_unknown_pos(int x, int y, int mask)
{

	POS_T pos;
	stack_flag = false;

	/* ���ݒT�����̃}�X����H��鎟�̒T�������X�^�b�N�Ɋi�[ */
		if ((check(x, y + 1) == true) && ((wall[x][y].north & mask) == NOWALL)) // �����̌������l������
		{
			/* ������ɒH���ꍇ */

			pos.x = x;	   /* ������̃}�X��i���W */
			pos.y = y + 1; /* ������̃}�X��j���W */

			/* ������̃}�X��T�����Ƃ��ăX�^�b�N�Ɋi�[*/
			push(&stack, &pos);
			stack_flag = true;
		}

		if ((check(x + 1, y) == true) && ((wall[x][y].east & mask) == NOWALL))
		{
			/* �E�����ɒH���ꍇ */
			pos.x = x + 1; /* �E�����̃}�X��i���W */
			pos.y = y;	   /* �E�����̃}�X��j���W */

			/* �E�����̃}�X��T�����Ƃ��ăX�^�b�N�Ɋi�[*/
			push(&stack, &pos);
			stack_flag = true;
		}

		if ((check(x - 1, y) == true) && ((wall[x][y].west & mask) == NOWALL))
		{
			/* �������ɒH���ꍇ */

			pos.x = x - 1; /* �������̃}�X��i���W */
			pos.y = y;	   /* �������̃}�X��j���W */

			/* �������̃}�X��T�����Ƃ��ăX�^�b�N�Ɋi�[*/
			push(&stack, &pos);
			stack_flag = true;
		}

		if ((check(x, y - 1) == true) && ((wall[x][y].south & mask) == NOWALL))
		{
			/* �������ɒH���ꍇ */

			pos.x = x;	   /* �������̃}�X��i���W */
			pos.y = y - 1; /* �������̃}�X��j���W */

			/* �������̃}�X��T�����Ƃ��ăX�^�b�N�Ɋi�[*/
			push(&stack, &pos);
			stack_flag = true;
		}

}

// �X�^�b�N������o�����}�X���A���Ɍ������ڕW�Ƃ���֐�
void get_next_target()
{
	POS_T pos;
	unsigned int x, y;

	//if (stack_flag == true)  //�X�^�b�N�ɏ�񂪓�����Ȃ������Ƃ��i���͂ɖ��T���̃}�X���Ȃ��Ƃ��j�͍X�V����Ȃ��悤�ɂ���
	//{
	/* �X�^�b�N���玟�̒T�����̃}�X���擾*/
	POS_T *next = pop(&stack);

	/* �T������}�X�̍��W���擾 */
	x = next->x;
	y = next->y;

	// ��x����ۑ�
	bef_x = x;
	bef_y = y;
	//}

	// �T������}�X��ڕW�Ƃ��Đݒ�
	map[bef_x][bef_y] = 0;
}

void depth_first_search(int gx, int gy)
{
	// ����gx,gy�Ɍ������đ����@�Ŗ��H��T������
	t_direction glob_nextdir; // ���Ɍ������������L�^����ϐ�
	POS_T pos;

	accel = SEARCH_ACCEL;
	len_count = 0;
	slalom_count = 0;
	x_position = y_position = 0;
	len_mouse = 0;

	// �X�^�b�N�̏�����
	initStack(&stack);

	/* ���̒T�����Ƃ��ăX�^�[�g����̃}�X�̏����X�^�b�N�Ɋi�[ */
	search_unknown_pos(mypos.x, mypos.y, MASK_SEARCH);

	if (gx != 0 && gy != 0)
	{
		slalom_straight_2(10, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED);
	}

	switch (get_nextdir_D(gx, gy, MASK_SEARCH, &glob_nextdir)) // ���ɍs��������߂�l�Ƃ���֐����Ă�
	{
	case front:

		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��
		break;

	case right:

		turn(90, TURN_ACCEL, TURN_SPEED, RIGHT);						  // �E�ɋȂ�����
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��
		break;

	case left:

		turn(90, TURN_ACCEL, TURN_SPEED, LEFT);							  // ���ɋȂ�����
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��
		break;

	case rear:

		turn(180, TURN_ACCEL, TURN_SPEED, RIGHT);						  // 180�^�[��
		straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��
		break;
	}
	accel = SEARCH_ACCEL; // �����x��ݒ�
	// con_wall.enable = true;					//�ǐ����L���ɂ���
	// MOT_CWCCW_R = MOT_CWCCW_L = MOT_FORWARD;		//�O���ɐi��
	len_mouse = 0;							   // �i�񂾋����J�E���g�p�ϐ������Z�b�g
	MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // �J�E���g�X�^�[�g

	mypos.dir = glob_nextdir; // �������X�V

	// �����������ɂ���Ď����̍��W���X�V����
	switch (mypos.dir)
	{
	case north:
		mypos.y++; // �k������������Y���W�𑝂₷
		break;

	case east:
		mypos.x++; // ��������������X���W�𑝂₷
		break;

	case south:
		mypos.y--; // �������������Y���W�����炷
		break;

	case west:
		mypos.x--; // �����������Ƃ���X���W�����炷
		break;
	}

	maze[mypos.x][mypos.y] = PASSED;

	while ((mypos.x != gx) || (mypos.y != gy))
	{ // �S�[������܂ŌJ��Ԃ�

		set_wall_D(mypos.x, mypos.y);					   // �ǂ��Z�b�g
		search_unknown_pos(mypos.x, mypos.y, MASK_SEARCH); // �X�^�b�N�ɐV�����T������v�b�V��

		// get_next_target();   //�X�^�b�N�̍X�V���K�v�Ȏ������ĂԂ悤�ɂ����� + initmap�ɓ����

		switch (get_nextdir_D(gx, gy, MASK_SEARCH, &glob_nextdir)) // ���ɍs��������߂�l�Ƃ���֐����Ă�
		{
		case front:

			slalom_straight_2(SECTION, SEARCH_ACCEL, SEARCH_SPEED, SEARCH_SPEED); // �����i��
			break;

		case right:
			slalom_2(90, SLALOM_ACCEL_2, SLALOM_SPEED_2, RIGHT);
			break;

		case left:
			slalom_2(90, SLALOM_ACCEL_2, SLALOM_SPEED_2, LEFT);
			break;

		case rear: // �Ǔ��Ă͐攻�f�̕����悢

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

		// con_wall.enable = true;						//�ǐ����L���ɂ���
		len_mouse = 0;							   // �i�񂾋������J�E���g����ϐ������Z�b�g
		MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1; // �J�E���g�X�^�[�g

		mypos.dir = glob_nextdir; // �������X�V

		// �����������ɂ���Ď����̍��W���X�V����
		switch (mypos.dir)
		{
		case north:
			mypos.y++; // �k������������Y���W�𑝂₷
			break;

		case east:
			mypos.x++; // ��������������X���W�𑝂₷
			break;

		case south:
			mypos.y--; // �������������Y���W�����炷
			break;

		case west:
			mypos.x--; // �����������Ƃ���X���W�����炷
			break;
		}

		maze[mypos.x][mypos.y] = PASSED; // ���ݍ��W��ʉߍς݂Ƃ��ĕۑ�
	}
	set_wall_D(mypos.x, mypos.y); // �ǂ��Z�b�g
	straight(HALF_SECTION, SEARCH_ACCEL, SEARCH_SPEED, 0);
}
