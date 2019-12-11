#include "system.h"

extern int SystemMonitor;
int main(void)
{
	System_Init();
	while(1)
	{
		Loop();
	}
}
	




