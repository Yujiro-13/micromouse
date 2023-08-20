/*#include "iodefine.h"
#include "mathf.h"
#include "sci.h"
#include "init.h"
#include "spi.h"
#include "parameters.h"
#include "glob_var.h"
#include "mytypedef.h"
#include "portdef.h"
#include "interface.h"
#include "get_run_log.h"


void Get_run_log(){    //ÉçÉOéÊóp
	if (log_timer % 4 == 0 && log_flag == 1)
	{
		if (log_timer < (LOG_CNT2 * 4))
		{
			loga[0][log_timer / 4] = (long)(x_position);
			loga[1][log_timer / 4] = (long)(y_position);
		}
	}

	log_timer++;
}*/
