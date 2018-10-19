#include "lora_test.h"
#include "sx1276.h"
#include "sx1276-board.h"
#include "gpio.h"
#include "delay.h"
//#include "stdbool.h"

static RadioEvents_t RadioEvents_Test;

void Lightning(void)
{
	GpioWrite(&LED,RESET);
	DelayMs(100);
	GpioWrite(&LED,SET);
}
void Lightning2(void)
{
	GpioWrite(&LED,RESET);
	DelayMs(100);
	GpioWrite(&LED,SET);
	DelayMs(100);
	GpioWrite(&LED,RESET);
	DelayMs(100);
	GpioWrite(&LED,SET);
}

void CheckReversion(void)
{
	uint8_t temp = 0;
	temp = SX1276Read(REG_LR_VERSION);
	if(temp == 0x12)
		Lightning();
	else
		Lightning2();
}

void sx1276init(void)
{
//	SX1276Init(&RadioEvents_Test);
	SX1276Reset( );
	SX1276.Settings.State = RF_IDLE;
	SX1276SetChannel(470000000);
	SX1276SetTxConfig(MODEM_LORA,18,0,1,7,1,127,false,true,false,0,false,0);
//	SX1276SetRxConfig(MODEM_LORA,1,7,1,0,127,100,false,0,true,false,0,false,false);
//	SX1276SetTxConfig(MODEM_LORA,18,0,1,7,1,127,false,true,false,0,false,0);
//	SX1276Send((uint8_t*)"123",4);
// 	SX1276SetRx(0);
}

void test_tx(void)
{
	uint8_t irqFlags = 0;
	uint8_t t=0;	
//	SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE );
	if(t==0)
	{
		t=100;
		SX1276Send((uint8_t*)"123",4);
	}
	if(t>0)
	{
		irqFlags = SX1276Read( REG_LR_IRQFLAGS );
		if(( irqFlags & RFLR_IRQFLAGS_TXDONE_MASK ) == RFLR_IRQFLAGS_TXDONE )
		{
//			SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE );
			Lightning();
		}
		else
		{
			t--;
			DelayMs(1);
		}
	}
	else
		Lightning2();
	
}
