#include "button.h"
#include "main.h"
#include "stm32f1xx_it.h"

int iButtonCount; // Button pressed time counter
int iButtonFlag; // Button reset mark, 1 represents reset
int g_iButtonState; // Button state, global variable, 1 pressed, 0 released

void ButtonScan(void) {
	if (HAL_GPIO_ReadPin(GPIOA, Rb_Pin) == GPIO_PIN_RESET) {
		iButtonCount++;
		
		// If button pressed over 30ms
		if (iButtonCount >= 30) {
			// Havn't reset
			if (iButtonFlag == 0) {
				g_iButtonState = 1;
				iButtonCount = 0;
				iButtonFlag = 1;
			}
			else iButtonCount = 0;
		}
		else g_iButtonState = 0;
	}
	else {
		iButtonCount = 0;
		g_iButtonState = 0;
		iButtonFlag = 0;
	}
}