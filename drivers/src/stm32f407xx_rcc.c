
#include <stm32f407xx_rcc.h>



uint32_t RCC_GetPLLOutputClock(void)
{
	// TODO
	return 0;
}

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t system_clk;
	uint32_t pclk1;
	uint8_t clksrc;

	clksrc = ((RCC->CFGR >> 2) & 0x03);

	/* System clock */
	if(clksrc == 0){
		// HSI
		system_clk = HSI_FREQ;
	}else if(clksrc == 1){
		// HSE
		system_clk = HSE_FREQ;
	}else if(clksrc == 2){
		// PLL
		system_clk = RCC_GetPLLOutputClock();
	}else{
		while(1); // Shouldn't get here
	}

	/* Buses' clocks */
	// AHB
	uint32_t AHB_preescaler_bits = ((RCC->CFGR >> 4) & 0x0f);
	uint8_t AHB_preescaler;

	if(AHB_preescaler_bits < 8){
		AHB_preescaler = 1;
	}else{
		AHB_preescaler = 2^(AHB_preescaler_bits - 8 + 1);
	}

	// APB1
	uint32_t APB1_preescaler_bits = ((RCC->CFGR >> 10) & 0x07);
	uint8_t APB1_preescaler;
	if(APB1_preescaler_bits < 4){
		APB1_preescaler = 1;
	}else{
		APB1_preescaler = 2^(APB1_preescaler_bits - 4 + 1);
	}

	pclk1 = (system_clk / AHB_preescaler) / APB1_preescaler;

	return pclk1;
}

uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t system_clk;
	uint32_t pclk2;
	uint8_t clksrc;

	clksrc = ((RCC->CFGR >> 2) & 0x03);

	/* System clock */
	if(clksrc == 0){
		// HSI
		system_clk = HSI_FREQ;
	}else if(clksrc == 1){
		// HSE
		system_clk = HSE_FREQ;
	}else if(clksrc == 2){
		// PLL
		system_clk = RCC_GetPLLOutputClock();
	}else{
		while(1); // Shouldn't get here
	}

	/* Buses' clocks */
	// AHB
	uint32_t AHB_preescaler_bits = ((RCC->CFGR >> 4) & 0x0f);
	uint8_t AHB_preescaler;

	if(AHB_preescaler_bits < 8){
		AHB_preescaler = 1;
	}else{
		AHB_preescaler = 2^(AHB_preescaler_bits - 8 + 1);
	}

	// APB1
	uint32_t APB2_preescaler_bits = ((RCC->CFGR >> 13) & 0x07);
	uint8_t APB2_preescaler;
	if(APB2_preescaler_bits < 4){
		APB2_preescaler = 1;
	}else{
		APB2_preescaler = 2^(APB2_preescaler_bits - 4 + 1);
	}

	pclk2 = (system_clk / AHB_preescaler) / APB2_preescaler;

	return pclk2;
}
