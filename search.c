#include "mytypedef.h"
#include "parameters.h"
#include "iodefine.h"
#include "portdef.h"
#include "glob_var.h"
#include "macro.h"
#include "run.h"
#include "interface.h"

void search_lefthand(void);					//����@

void init_map(int x, int y);					//����Map�̏�����
void make_map(int x, int y, int mask);				//�����}�b�v�쐬
void set_wall(int x, int y);					//�Ǐ���ۑ�
t_bool is_unknown(int x, int y);				//���T����Ԃ��ۂ��𔻒�
int get_priority(int x, int y, t_direction dir);		//�D��x���擾(���T���A�O�������D�悳���)
int get_nextdir(int x, int y, int mask, t_direction *dir);	//���ɍs���ׂ��������擾����
void search_adachi(int gx, int gy);				//�����@

extern void wait_ms(int wtime);

void search_lefthand(void)
{
	
	max_speed = SEARCH_SPEED;				//�T���̑��x���w��
	accel = SEARCH_ACCEL;
	
	straight(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);			//�܂��A�����i��
		
	while(1)
	{
		if(sen_l.is_wall == false)			//���ɕǂ��Ȃ���΍��ɐi��
		{
			straight(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,0);		//�����i���
			turn(90,TURN_ACCEL,TURN_SPEED,LEFT);				//���ɋȂ�����
			straight(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);	//�����i��
		}
		else if( (sen_fl.is_wall == false) && (sen_fr.is_wall == false) )	//�O�ɕǂ��Ȃ���ΑO�ɐi��
		{
			straight(SECTION,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);		//����i��
		}
		else if(sen_r.is_wall == false)			//�E�ɕǂ��Ȃ���ΉE�ɐi��
		{
			straight(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,0);		//�����i��
			turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);			//�E�ɋȂ���
			straight(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);	//�����i��
		}
		else						//����ȊO�̏ꍇ�A���ɐi��
		{
			straight(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,0);		//�����i��
			turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);			//���Ɍ���
			straight(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);	//�����i��
		}	
	
	}
}


void init_map(int x, int y)
{
//���H�̕���Map������������B�S�̂�0xff�A�����̍��Wx,y��0�ŏ���������

	int i,j;
	
	for(i = 0; i < MAZESIZE_X; i++)		//���H�̑傫�������[�v(x���W)
	{
		for(j = 0; j < MAZESIZE_Y; j++)	//���H�̑傫�������[�v(y���W)
		{
			map[i][j] = 255;	//���ׂ�255�Ŗ��߂�
		}
	}
	
	map[x][y] = 0;				//�S�[�����W�̕������O�ɐݒ�
	
}


void make_map(int x, int y, int mask)	//�����}�b�v���쐬����
{
//���Wx,y���S�[���Ƃ�������Map���쐬����B
//mask�̒l(MASK_SEARCH or MASK_SECOND)�ɂ���āA
//�T���p�̕���Map����邩�A�ŒZ���s�̕���Map����邩���؂�ւ��
	int i,j;
	t_bool change_flag;			//Map�쐬�I�������ɂ߂邽�߂̃t���O

	init_map(x,y);				//Map������������

	do
	{
		change_flag = false;				//�ύX���Ȃ������ꍇ�ɂ̓��[�v�𔲂���
		for(i = 0; i < MAZESIZE_X; i++)			//���H�̑傫�������[�v(x���W)
		{
			for(j = 0; j < MAZESIZE_Y; j++)		//���H�̑傫�������[�v(y���W)
			{
				if(map[i][j] == 255)		//255�̏ꍇ�͎���
				{
					continue;
				}
				
				if(j < MAZESIZE_Y-1)					//�͈̓`�F�b�N
				{
					if( (wall[i][j].north & mask) == NOWALL)	//�ǂ��Ȃ����(mask�̈Ӗ���static_parameters���Q��)
					{
						if(map[i][j+1] == 255)			//�܂��l�������Ă��Ȃ����
						{
							map[i][j+1] = map[i][j] + 1;	//�l����
							change_flag = true;		//�l���X�V���ꂽ���Ƃ�����
						}
					}
				}
			
				if(i < MAZESIZE_X-1)					//�͈̓`�F�b�N
				{
					if( (wall[i][j].east & mask) == NOWALL)		//�ǂ��Ȃ����
					{
						if(map[i+1][j] == 255)			//�l�������Ă��Ȃ����
						{
							map[i+1][j] = map[i][j] + 1;	//�l����
							change_flag = true;		//�l���X�V���ꂽ���Ƃ�����
						}
					}
				}
			
				if(j > 0)						//�͈̓`�F�b�N
				{
					if( (wall[i][j].south & mask) == NOWALL)	//�ǂ��Ȃ����
					{
						if(map[i][j-1] == 255)			//�l�������Ă��Ȃ����
						{
							map[i][j-1] = map[i][j] + 1;	//�l����
							change_flag = true;		//�l���X�V���ꂽ���Ƃ�����
						}
					}
				}
			
				if(i > 0)						//�͈̓`�F�b�N
				{
					if( (wall[i][j].west & mask) == NOWALL)		//�ǂ��Ȃ����
					{
						if(map[i-1][j] == 255)			//�l�������Ă��Ȃ����
						{
							map[i-1][j] = map[i][j] + 1;	//�l����	
							change_flag = true;		//�l���X�V���ꂽ���Ƃ�����
						}
						
					}
					
				}
				
			}
			
		}
		
	}while(change_flag == true);	//�S�̂����I���܂ő҂�
	
}



void set_wall(int x, int y)	//�Ǐ����L�^
{
//�����̍��Wx,y�ɕǏ�����������
	int n_write,s_write,e_write,w_write;
	
	
	//�����̕����ɉ����ď������ރf�[�^�𐶐�
	//CONV_SEN2WALL()��macro.h���Q��
	switch(mypos.dir){
		case north:	//�k�������Ă��鎞
		
			n_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall);	//�O�ǂ̗L���𔻒f
			e_write = CONV_SEN2WALL(sen_r.is_wall);				//�E�ǂ̗L���𔻒f
			w_write = CONV_SEN2WALL(sen_l.is_wall);				//���ǂ̗L���𔻒f
			s_write = NOWALL;						//���͕K���ǂ��Ȃ�
			
			break;
			
		case east:	//���������Ă���Ƃ�
			
			e_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall);	//�O�ǂ̗L���𔻒f
			s_write = CONV_SEN2WALL(sen_r.is_wall);				//�E�ǂ̗L���𔻒f
			n_write = CONV_SEN2WALL(sen_l.is_wall);				//���ǂ̗L���𔻒f
			w_write = NOWALL;						//���͕K���ǂ��Ȃ�
			
			break;
			
		case south:	//��������Ă���Ƃ�
		
			s_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall);	//�O�ǂ̗L���𔻒f
			w_write = CONV_SEN2WALL(sen_r.is_wall);				//�E�ǂ̗L���𔻒f
			e_write = CONV_SEN2WALL(sen_l.is_wall);				//���ǂ̗L���𔻒f
			n_write = NOWALL;						//���͕K���ǂ��Ȃ�

			break;
			
		case west:	//���������Ă���Ƃ�
		
			w_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall);	//�O�ǂ̗L���𔻒f
			n_write = CONV_SEN2WALL(sen_r.is_wall);				//�E�ǂ̗L���𔻒f
			s_write = CONV_SEN2WALL(sen_l.is_wall);				//���ǂ̗L���𔻒f
			e_write = NOWALL;						//���͕K���ǂ��Ȃ�
			
			break;
			
	}
	
	wall[x][y].north = n_write;	//���ۂɕǏ�����������
	wall[x][y].south = s_write;	//���ۂɕǏ�����������
	wall[x][y].east  = e_write;	//���ۂɕǏ�����������
	wall[x][y].west  = w_write;	//���ۂɕǏ�����������
	
	if(y < MAZESIZE_Y-1)	//�͈̓`�F�b�N
	{
		wall[x][y+1].south = n_write;	//���Α����猩���ǂ���������
	}
	
	if(x < MAZESIZE_X-1)	//�͈̓`�F�b�N
	{
		wall[x+1][y].west = e_write;	//���Α����猩���ǂ���������
	}
	
	if(y > 0)	//�͈̓`�F�b�N
	{
		wall[x][y-1].north = s_write;	//���Α����猩���ǂ���������
	}
	
	if(x > 0)	//�͈̓`�F�b�N
	{
		wall[x-1][y].east = w_write;	//���Α����猩���ǂ���������
	}
	
}


t_bool is_unknown(int x, int y)	//�w�肳�ꂽ��悪���T�����ۂ��𔻒f����֐� ���T��:true�@�T����:false
{
	//���Wx,y�����T����Ԃ��ۂ��𒲂ׂ�
	
	if((wall[x][y].north == UNKNOWN) || (wall[x][y].east == UNKNOWN) || (wall[x][y].south == UNKNOWN) || (wall[x][y].west == UNKNOWN))
	{			//�ǂ����̕Ǐ�񂪕s���̂܂܂ł����
		return true;	//���T��
	}
	else
	{
		return false;	//�T����
	}
}



int get_priority(int x, int y, t_direction dir)	//���̃}�X�̏�񂩂�A�D��x���Z�o����
{
	//���Wx,y�ƁA�����Ă�����pdir����D��x���Z�o����
	
	//���T������ԗD��x������.(4)
	//����ɉ����A�����̌����ƁA�s��������������A
	//�O(2)��(1)��(0)�̗D��x��t������B

	int priority;	//�D��x���L�^����ϐ�
	
	priority = 0;
	
	if(mypos.dir == dir)				//�s���������������݂̐i�s�����Ɠ����ꍇ
	{
		priority = 2;
	}
	else if( ((4+mypos.dir-dir)%4) == 2)		//�s���������������݂̐i�s�����Ƌt�̏ꍇ
	{
		priority = 0;
	}
	else						//����ȊO(���E�ǂ��炩)�̏ꍇ
	{
		priority = 1;
	}
	
	
	if(is_unknown(x,y) == true)
	{
		priority += 4;				//���T���̏ꍇ�D��x������ɕt��
	}
	
	return priority;				//�D��x��Ԃ�
	
}


int get_nextdir(int x, int y, int mask, t_direction *dir)	
{
	//�S�[�����Wx,y�Ɍ������ꍇ�A���ǂ���ɍs���ׂ����𔻒f����B
	//�T���A�ŒZ�̐؂�ւ��̂��߂�mask���w��Adir�͕��p������
	int little,priority,tmp_priority;		//�ŏ��̒l��T�����߂Ɏg�p����ϐ�


	make_map(x,y,mask);				//����Map����
	little = 255;					//�ŏ�������255��(map��unsigned char�^�Ȃ̂�)�ɐݒ�	

	priority = 0;					//�D��x�̏����l��0
	
		//mask�̈Ӗ���static_parameter.h���Q��
	if( (wall[mypos.x][mypos.y].north & mask) == NOWALL)			//�k�ɕǂ��Ȃ����
	{
		tmp_priority = get_priority(mypos.x, mypos.y + 1, north);	//�D��x���Z�o
		if(map[mypos.x][mypos.y+1] < little)				//��ԕ�����������������������
		{
			little = map[mypos.x][mypos.y+1];			//�ЂƂ܂��k�����������������ɂ���
			*dir = north;						//������ۑ�
			priority = tmp_priority;				//�D��x��ۑ�
		}
		else if(map[mypos.x][mypos.y+1] == little)			//�����������ꍇ�͗D��x���画�f����
		{
			if(priority < tmp_priority )				//�D��x��]��
			{
				*dir = north;					//�������X�V
				priority = tmp_priority;			//�D��x��ۑ�
			}
		}
	}
	
	if( (wall[mypos.x][mypos.y].east & mask) == NOWALL)			//���ɕǂ��Ȃ����
	{
		tmp_priority = get_priority(mypos.x + 1, mypos.y, east);	//�D��x���Z�o
		if(map[mypos.x + 1][mypos.y] < little)				//��ԕ�����������������������
		{
			little = map[mypos.x+1][mypos.y];			//�ЂƂ܂��������������������ɂ���
			*dir = east;						//������ۑ�
			priority = tmp_priority;				//�D��x��ۑ�
		}
		else if(map[mypos.x + 1][mypos.y] == little)			//�����������ꍇ�A�D��x���画�f
		{
			if(priority < tmp_priority)				//�D��x��]��
			{
				*dir = east;					//������ۑ�
				priority = tmp_priority;			//�D��x��ۑ�
			}
		}
	}
	
	if( (wall[mypos.x][mypos.y].south & mask) == NOWALL)			//��ɕǂ��Ȃ����
	{
		tmp_priority = get_priority(mypos.x, mypos.y - 1, south);	//�D��x���Z�o
		if(map[mypos.x][mypos.y - 1] < little)				//��ԕ�����������������������
		{
			little = map[mypos.x][mypos.y-1];			//�ЂƂ܂��삪���������������ɂ���
			*dir = south;						//������ۑ�
			priority = tmp_priority;				//�D��x��ۑ�
		}
		else if(map[mypos.x][mypos.y - 1] == little)			//�����������ꍇ�A�D��x�ŕ]��
		{
			if(priority < tmp_priority)				//�D��x��]��
			{
				*dir = south;					//������ۑ�
				priority = tmp_priority;			//�D��x��ۑ�
			}
		}
	}
	
	if( (wall[mypos.x][mypos.y].west & mask) == NOWALL)			//���ɕǂ��Ȃ����
	{
		tmp_priority = get_priority(mypos.x - 1, mypos.y, west);	//�D��x���Z�o
		if(map[mypos.x-1][mypos.y] < little)				//��ԕ�����������������������
		{
			little = map[mypos.x-1][mypos.y];			//����������������
			*dir = west;						//������ۑ�
			priority = tmp_priority;				//�D��x��ۑ�
		}
		else if(map[mypos.x - 1][mypos.y] == little)			//�����������ꍇ�A�D��x�ŕ]��
		{
			*dir = west;						//������ۑ�
			priority = tmp_priority;				//�D��x��ۑ�
		}
	}


	return ( (int)( ( 4 + *dir - mypos.dir) % 4 ) );			//�ǂ����Ɍ������ׂ�����Ԃ��B
										//���Z�̈Ӗ���mytyedef.h����enum�錾����B
	
}



void search_adachi(int gx, int gy)
{
//����gx,gy�Ɍ������đ����@�Ŗ��H��T������
	t_direction glob_nextdir;					//���Ɍ������������L�^����ϐ�

	accel=SEARCH_ACCEL;

	switch(get_nextdir(gx,gy,MASK_SEARCH,&glob_nextdir))		//���ɍs��������߂�l�Ƃ���֐����Ă�
	{
		case front:
			straight(20,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);
			straight(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);		//�����i��
			break;
		
		case right:
			turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);				//�E�ɋȂ�����
			straight(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);		//�����i��
			break;
		
		case left:
			turn(90,TURN_ACCEL,TURN_SPEED,LEFT);				//���ɋȂ�����
			straight(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);		//�����i��
			break;
		
		case rear:
			turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);					//180�^�[��
			straight(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);		//�����i��
			break;
	}
		accel=SEARCH_ACCEL;				//�����x��ݒ�
		//con_wall.enable = true;					//�ǐ����L���ɂ���
		//MOT_CWCCW_R = MOT_CWCCW_L = MOT_FORWARD;		//�O���ɐi��
		len_mouse = 0;					//�i�񂾋����J�E���g�p�ϐ������Z�b�g
		len_count = 0;
		MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1;		//�J�E���g�X�^�[�g
	
		mypos.dir = glob_nextdir;				//�������X�V


	//�����������ɂ���Ď����̍��W���X�V����
	switch(mypos.dir)
	{
		case north:
			mypos.y++;	//�k������������Y���W�𑝂₷
			break;
			
		case east:
			mypos.x++;	//��������������X���W�𑝂₷
			break;
			
		case south:
			mypos.y--;	//�������������Y���W�����炷
			break;
		
		case west:
			mypos.x--;	//�����������Ƃ���X���W�����炷
			break;

	}

	
	while((mypos.x != gx) || (mypos.y != gy)){				//�S�[������܂ŌJ��Ԃ�

		set_wall(mypos.x,mypos.y);					//�ǂ��Z�b�g

		
	

		switch(get_nextdir(gx,gy,MASK_SEARCH,&glob_nextdir))		//���ɍs��������߂�l�Ƃ���֐����Ă�
		{
			case front:

				straight(SECTION,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);		//�����i��
				
				break;
			
			case right:
				straight(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,0);		//�����i��
				if(sen_l.value > TH_SEN_L_BE_T && sen_fr.is_wall == true && sen_fl.is_wall == true){
                    turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
                    check_straight(HALF_SECTION,-SEARCH_ACCEL,SEARCH_SPEED,0);
                    straight(20,SEARCH_ACCEL,SEARCH_SPEED,0);
                    turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
                    check_straight(HALF_SECTION,-SEARCH_ACCEL,SEARCH_SPEED,0);
                    straight(18,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);
                }
                else if(sen_l.value > TH_SEN_L_BE_T){
                   turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
                   check_straight(HALF_SECTION,-SEARCH_ACCEL,SEARCH_SPEED,0);
                   straight(18,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);
                }
				else if (sen_fr.is_wall == true && sen_fl.is_wall == true)
				{
				   turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
                   check_straight(HALF_SECTION,-SEARCH_ACCEL,SEARCH_SPEED,0);
                   straight(20,SEARCH_ACCEL,SEARCH_SPEED,0);
				   turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
				}
				
                else{
                    turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
                    }
				straight(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);	
				break;
			
			case left:
			    
				straight(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,0);		//�����i��
				if(sen_r.value > TH_SEN_R_BE_T && sen_fr.is_wall == true && sen_fl.is_wall == true){
                    turn(180,TURN_ACCEL,TURN_SPEED,LEFT);
                    check_straight(HALF_SECTION,-SEARCH_ACCEL,SEARCH_SPEED,0);
                    straight(20,SEARCH_ACCEL,SEARCH_SPEED,0);
                    turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
                    check_straight(HALF_SECTION,-SEARCH_ACCEL,SEARCH_SPEED,0);
                    straight(18,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);
                }
                else if(sen_r.value > TH_SEN_R_BE_T){
                   turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
                   check_straight(HALF_SECTION,-SEARCH_ACCEL,SEARCH_SPEED,0);
                   straight(18,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);
                }
				else if (sen_fr.is_wall == true && sen_fl.is_wall == true)
				{
				   turn(180,TURN_ACCEL,TURN_SPEED,LEFT);
                   check_straight(HALF_SECTION,-SEARCH_ACCEL,SEARCH_SPEED,0);
                   straight(20,SEARCH_ACCEL,SEARCH_SPEED,0);
				   turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
				}
				
                else{
                    turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
                    }
				straight(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);		
				break;
			
			case rear:
			    
				straight(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,0);		//�����i��
				if(sen_r.is_wall == true && sen_l.is_wall == true && sen_fr.is_wall == true && sen_fl.is_wall == true){
                    turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
                    check_straight(HALF_SECTION,-SEARCH_ACCEL,SEARCH_SPEED,0);
                    straight(20,SEARCH_ACCEL,SEARCH_SPEED,0);
                    turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
                    check_straight(HALF_SECTION,-SEARCH_ACCEL,SEARCH_SPEED,0);
                    straight(18,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);
                }
                else if(sen_fr.is_wall == true && sen_fl.is_wall ==true){
                   turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
                   check_straight(HALF_SECTION,-SEARCH_ACCEL,SEARCH_SPEED,0);
                   straight(18,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);
                }
                else{
                    turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
                    }
				straight(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);		
				break;
		}

        
		//con_wall.enable = true;						//�ǐ����L���ɂ���
		len_mouse = 0;						//�i�񂾋������J�E���g����ϐ������Z�b�g
		MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1;			//�J�E���g�X�^�[�g
	
		mypos.dir = glob_nextdir;					//�������X�V
		
		//�����������ɂ���Ď����̍��W���X�V����
		switch(mypos.dir)
		{
			case north:
				mypos.y++;	//�k������������Y���W�𑝂₷
				break;
				
			case east:
				mypos.x++;	//��������������X���W�𑝂₷
				break;
				
			case south:
				mypos.y--;	//�������������Y���W�����炷
				break;
			
			case west:
				mypos.x--;	//�����������Ƃ���X���W�����炷
				break;

		}
		
	}
	set_wall(mypos.x,mypos.y);		//�ǂ��Z�b�g
	straight(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,0);	

}
