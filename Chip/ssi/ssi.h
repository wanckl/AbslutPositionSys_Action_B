#include "sys.h"

#define SSI_CSn PAout(4)
#define SSI_CLK PAout(5)
#define SSI_DI1 PAin(6)
#define SSI_DI2 PAin(7)

#define TclkFE  1	//us
#define Hclk	1
#define Tcsn	1

void SSI_GPIO_Init(void);
uint32_t SSI_ReadPKG(void);

