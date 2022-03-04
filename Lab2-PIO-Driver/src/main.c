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

/*  Default pin configuration (no attribute). */
#define _PIO_DEFAULT             (0u << 0)
/*  The internal pin pull-up is active. */
#define _PIO_PULLUP              (1u << 0)
/*  The internal glitch filter is active. */
#define _PIO_DEGLITCH            (1u << 1)
/*  The internal debouncing filter is active. */
#define _PIO_DEBOUNCE            (1u << 3)


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
/**
 * \brief Set a high output level on all the PIOs defined in ul_mask.
 * This has no immediate effects on PIOs that are not output, but the PIO
 * controller will save the value if they are changed to outputs.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 */
void _pio_set(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_SODR = ul_mask;	
}

/**
 * \brief Set a low output level on all the PIOs defined in ul_mask.
 * This has no immediate effects on PIOs that are not output, but the PIO
 * controller will save the value if they are changed to outputs.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 */
void _pio_clear(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_CODR = ul_mask;
}

/**
 * \brief Configure PIO internal pull-up.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 * \param ul_pull_up_enable Indicates if the pin(s) internal pull-up shall be
 * configured.
 */
void _pio_pull_up(Pio *p_pio, const uint32_t ul_mask,
        const uint32_t ul_pull_up_enable){
	if (ul_pull_up_enable)
		p_pio->PIO_PUER = ul_mask;
	else
		p_pio->PIO_PUDR = ul_mask;
 }

/**
 * \brief Configure one or more pin(s) or a PIO controller as inputs.
 * Optionally, the corresponding internal pull-up(s) and glitch filter(s) can
 * be enabled.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask indicating which pin(s) to configure as input(s).
 * \param ul_attribute PIO attribute(s).
 */
void _pio_set_input(Pio *p_pio, const uint32_t ul_mask,
        const uint32_t ul_attribute)
{
	_pio_pull_up(p_pio, ul_mask, ul_attribute & _PIO_PULLUP);

	if (ul_attribute & (PIO_DEGLITCH | PIO_DEBOUNCE))
		p_pio->PIO_IFER = ul_mask;
	else
		p_pio->PIO_IFDR = ul_mask;
	
}

/**
 * \brief Configure one or more pin(s) of a PIO controller as outputs, with
 * the given default value. Optionally, the multi-drive feature can be enabled
 * on the pin(s).
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask indicating which pin(s) to configure.
 * \param ul_default_level Default level on the pin(s).
 * \param ul_multidrive_enable Indicates if the pin(s) shall be configured as
 * open-drain.
 * \param ul_pull_up_enable Indicates if the pin shall have its pull-up
 * activated.
 */
void _pio_set_output(Pio *p_pio, const uint32_t ul_mask,
        const uint32_t ul_default_level,
        const uint32_t ul_multidrive_enable,
        const uint32_t ul_pull_up_enable)
{
	p_pio->PIO_PER = ul_mask;
	p_pio->PIO_OER = ul_mask;

	if(ul_default_level)
		_pio_set(p_pio, ul_mask);
	else
		_pio_clear(p_pio, ul_mask);
	
	if(ul_multidrive_enable)
		p_pio->PIO_MDER = ul_mask;
	else
		p_pio->PIO_MDDR = ul_mask;

	_pio_pull_up(p_pio, ul_mask, ul_pull_up_enable);
}

/**
 * \brief Return 1 if one or more PIOs of the given Pin instance currently have
 * a high level; otherwise returns 0. This method returns the actual value that
 * is being read on the pin. To return the supposed output value of a pin, use
 * pio_get_output_data_status() instead.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_type PIO type.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 *
 * \retval 1 at least one PIO currently has a high level.
 * \retval 0 all PIOs have a low level.
 */
uint32_t _pio_get(Pio *p_pio, const pio_type_t ul_type,
        const uint32_t ul_mask)
{
	if(ul_type == PIO_INPUT)
		return p_pio->PIO_PDSR & ul_mask;
	else 
		return p_pio->PIO_ODSR & ul_mask;
}

void _delay_ms(uint32_t ms)
{
	for(int i = 0; i < (ms*150000); i++) {
		asm("nop");
	}
}

void pisca(Pio *p_pio, const uint32_t mask){
	for (int i=0; i<5; i++) {
		_pio_clear(p_pio, mask);
		_delay_ms(100);
		_pio_set(p_pio, mask);
		_delay_ms(100);
	}
}

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
	_pio_set_output(LED0_PIO, LED0_PIO_IDX_MASK, 0, 0, 0);
	_pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, 0, 0, 0);
	_pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, 0, 0, 0);
	_pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, 0, 0, 0);
	
	// Inicializa PIO do botao
	pmc_enable_periph_clk(BUT0_PIO_ID);
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);
	
	// configura pino ligado ao botão como entrada com um pull-up.
	_pio_set_input(BUT0_PIO, BUT0_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	_pio_set_input(BUT1_PIO, BUT1_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	_pio_set_input(BUT2_PIO, BUT2_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	_pio_set_input(BUT3_PIO, BUT3_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);


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
		if (!_pio_get(BUT0_PIO,PIO_INPUT,BUT0_IDX_MASK))
			pisca(LED0_PIO,LED0_PIO_IDX_MASK);
		else if (!_pio_get(BUT1_PIO,PIO_INPUT,BUT1_IDX_MASK))
			pisca(LED1_PIO,LED1_PIO_IDX_MASK);
		else if (!_pio_get(BUT2_PIO,PIO_INPUT,BUT2_IDX_MASK))
			pisca(LED2_PIO,LED2_PIO_IDX_MASK);
		else if (!_pio_get(BUT3_PIO,PIO_INPUT,BUT3_IDX_MASK))
			pisca(LED3_PIO,LED3_PIO_IDX_MASK);
		else {
			_pio_set(LED0_PIO, LED0_PIO_IDX_MASK);
			_pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
			_pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
			_pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
		}            
	}
	return 0;
}

