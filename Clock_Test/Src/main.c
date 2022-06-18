#include <stdint.h>

#define ADC_BASE_ADDR 	0x40012000UL

#define ADC_CR1_REG_OFFSET	0x04UL

#define ADC_CR1_REG_ADDR (ADC_BASE_ADDR + ADC_CR1_REG_OFFSET)

int main(void)
{
	uint32_t *pAdcCr1Reg = (uint32_t*) ADC_CR1_REG_ADDR;

	*pAdcCr1Reg |= (1 << 8);

	for (;;);
}
