/**
 * 5 semestre - Eng. da Computação - Insper
 * Rafael Corsi - rafael.corsi@insper.edu.br
 *
 * Projeto 0 para a placa SAME70-XPLD
 *
 * Objetivo :
 *  - Introduzir ASF e HAL
 *  - Configuracao de clock
 *  - Configuracao pino In/Out
 *
 * Material :
 *  - Kit: ATMEL SAME70-XPLD - ARM CORTEX M7
 */

/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include "asf.h"

/************************************************************************/
/* defines                                                              */
/************************************************************************/

// LEDs

// LED 0
#define LED0_PIO           PIOC                 // periferico que controla o LED
#define LED0_PIO_ID        ID_PIOC              // ID do periférico PIOC (controla LED)
#define LED0_PIO_IDX       8                    // ID do LED no PIO
#define LED0_PIO_IDX_MASK  (1 << LED0_PIO_IDX)   // Mascara para CONTROLARMOS o LED

// LED 1
#define LED1_PIO           PIOA                 // periferico que controla o LED 1
#define LED1_PIO_ID        ID_PIOA              // ID do periférico PIOC (controla LED 1)
#define LED1_PIO_IDX       0                    // ID do LED 1 no PIO
#define LED1_PIO_IDX_MASK  (1 << LED1_PIO_IDX)   // Mascara para CONTROLARMOS o LED 1

// LED 2
#define LED2_PIO           PIOC                 // periferico que controla o LED 1
#define LED2_PIO_ID        ID_PIOC              // ID do periférico PIOC (controla LED 1)
#define LED2_PIO_IDX       30                    // ID do LED 1 no PIO
#define LED2_PIO_IDX_MASK  (1 << LED2_PIO_IDX)   // Mascara para CONTROLARMOS o LED 1

// LED 3
#define LED3_PIO           PIOB                 // periferico que controla o LED 1
#define LED3_PIO_ID        ID_PIOB              // ID do periférico PIOC (controla LED 1)
#define LED3_PIO_IDX       2                    // ID do LED 1 no PIO
#define LED3_PIO_IDX_MASK  (1 << LED3_PIO_IDX)   // Mascara para CONTROLARMOS o LED 1

// Botões
// Botão 0
#define BUT0_PIO      PIOA
#define BUT0_PIO_ID   ID_PIOA
#define BUT0_IDX      11
#define BUT0_IDX_MASK (1 << BUT0_IDX)

// Botão 1
#define BUT1_PIO      PIOD
#define BUT1_PIO_ID   ID_PIOD
#define BUT1_IDX      28
#define BUT1_IDX_MASK (1 << BUT1_IDX)

// Botão 2
#define BUT2_PIO      PIOC
#define BUT2_PIO_ID   ID_PIOC
#define BUT2_IDX      31
#define BUT2_IDX_MASK (1 << BUT2_IDX)

// Botão 3
#define BUT3_PIO      PIOA
#define BUT3_PIO_ID   ID_PIOA
#define BUT3_IDX      19
#define BUT3_IDX_MASK (1 << BUT3_IDX)

/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

void init(void);
void pisca(Pio*, const uint32_t);

/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/
void pisca(Pio *p_pio, const uint32_t mask){
	for (int i=0; i<5; i++) {
		pio_clear(p_pio, mask);
		delay_ms(100);
		pio_set(p_pio, mask);
		delay_ms(100);
	}
}
// Função de inicialização do uC
// Função de inicialização do uC
void init(void){
	// Initialize the board clock
	sysclk_init();

	// Desativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	// Ativa o PIO na qual o LED foi conectado
	// para que possamos controlar o LED.
 	pmc_enable_periph_clk(LED0_PIO_ID);
 	pmc_enable_periph_clk(LED1_PIO_ID);
	pmc_enable_periph_clk(LED2_PIO_ID);
	pmc_enable_periph_clk(LED3_PIO_ID);
	
	//Inicializa PC8 como saída
	pio_set_output(LED0_PIO, LED0_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, 0, 0, 0);
	
	// Inicializa PIO do botao
	pmc_enable_periph_clk(BUT0_PIO_ID);
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);
	
	// configura pino ligado ao botão como entrada com um pull-up.
	pio_set_input(BUT0_PIO, BUT0_IDX_MASK, PIO_DEFAULT);
	pio_set_input(BUT1_PIO, BUT1_IDX_MASK, PIO_DEFAULT);
	pio_set_input(BUT2_PIO, BUT2_IDX_MASK, PIO_DEFAULT);
	pio_set_input(BUT3_PIO, BUT3_IDX_MASK, PIO_DEFAULT);
	
	pio_pull_up(BUT0_PIO, BUT0_IDX_MASK, 1);
	pio_pull_up(BUT1_PIO, BUT1_IDX_MASK, 1);
	pio_pull_up(BUT2_PIO, BUT2_IDX_MASK, 1);
	pio_pull_up(BUT3_PIO, BUT3_IDX_MASK, 1);

}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/

// Funcao principal chamada na inicalizacao do uC.
int main(void) {
	// inicializa sistema e IOs
	init();

	// super loop
	// aplicacoes embarcadas não devem sair do while(1).
	while (1)
	{
		if (!pio_get(BUT0_PIO,PIO_INPUT,BUT0_IDX_MASK))
			pisca(LED0_PIO,LED0_PIO_IDX_MASK);
		else if (!pio_get(BUT1_PIO,PIO_INPUT,BUT1_IDX_MASK))
			pisca(LED1_PIO,LED1_PIO_IDX_MASK);
		else if (!pio_get(BUT2_PIO,PIO_INPUT,BUT2_IDX_MASK))
			pisca(LED2_PIO,LED2_PIO_IDX_MASK);
		else if (!pio_get(BUT3_PIO,PIO_INPUT,BUT3_IDX_MASK))
			pisca(LED3_PIO,LED3_PIO_IDX_MASK);
		else {
			pio_set(LED0_PIO, LED0_PIO_IDX_MASK);
			pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
			pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
			pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
		}            
	}
	return 0;
}

