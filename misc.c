
extern volatile unsigned int timer;


void wait_ms(int wtime)		//mS�P�ʂő҂����Ԃ𐶐�����
{
	unsigned int start_time;
	
	start_time = timer;
	
	while( (timer - start_time) < wtime)	;

}