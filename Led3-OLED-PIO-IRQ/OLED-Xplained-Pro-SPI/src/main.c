#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

// LEDs

// LED 0
#define LED0_PIO           PIOC
#define LED0_PIO_ID        ID_PIOC              
#define LED0_PIO_IDX       8                    
#define LED0_PIO_IDX_MASK  (1 << LED0_PIO_IDX)

// LED 1
#define LED1_PIO           PIOA                 
#define LED1_PIO_ID        ID_PIOA              
#define LED1_PIO_IDX       0                    
#define LED1_PIO_IDX_MASK  (1 << LED1_PIO_IDX)   

// LED 2
#define LED2_PIO           PIOC                 
#define LED2_PIO_ID        ID_PIOC              
#define LED2_PIO_IDX       30                    
#define LED2_PIO_IDX_MASK  (1 << LED2_PIO_IDX)   

// LED 3
#define LED3_PIO           PIOB                 
#define LED3_PIO_ID        ID_PIOB              
#define LED3_PIO_IDX       2                    
#define LED3_PIO_IDX_MASK  (1 << LED3_PIO_IDX)   

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

volatile char change_freq_flag;
volatile char start_stop_flag;
volatile char decrease_freq_flag;

void init(void);
void pisca_led(Pio*, const uint32_t, int n, int t);

void oled_update(int freq) {
	char freq_str[128];
	gfx_mono_draw_string("          ", 5, 5, &sysfont);
	sprintf(freq_str, "%d ms", freq);
	gfx_mono_draw_string(freq_str, 5, 5, &sysfont);
}

// pisca led N vez no periodo T
void pisca_led(Pio *p_pio, const uint32_t mask, int n, int t){
	int counter = 0;

	gfx_mono_generic_draw_horizontal_line(90, 20, 30, GFX_PIXEL_SET);

	for (int i=0; i<n; i++) {

		if(start_stop_flag) {
			pio_set(p_pio, mask);
			start_stop_flag = 0;
			break;
		}

		gfx_mono_generic_draw_vertical_line(90 + counter, 10, 10, GFX_PIXEL_SET);
		pio_clear(p_pio, mask);
		delay_ms(t/2);
		pio_set(p_pio, mask);
		delay_ms(t/2);
		counter++;
	}

	gfx_mono_generic_draw_filled_rect(90, 10, 30, 11, GFX_PIXEL_CLR);
}

int change_frequency(int freq){
	if(decrease_freq_flag){
		freq -= 100;
		oled_update(freq);
		decrease_freq_flag = 0;
		
		return freq;
	}

	for(double i = 0; i<2500000; i++){
		if(!change_freq_flag){
			freq += 100;
			oled_update(freq);
			
			return freq ;
		}
	}

	change_freq_flag = 0;
	freq -= 100;
	oled_update(freq);
	
	return freq;
}

void but_callback(void) {
  if (pio_get(BUT1_PIO, PIO_INPUT, BUT1_IDX_MASK)) {
        // PINO == 1 --> Borda de subida
      change_freq_flag = 0;
    } else {
        // PINO == 0 --> Borda de descida
      change_freq_flag = 1;
    }  
}

void start_stop_callback(void) {
	start_stop_flag = 1;
}

void decrease_freq_callback(void) {
	decrease_freq_flag = 1;
}


void init(void){
	// Initialize the board clock
	sysclk_init();

	// Desativa WatchDog Timer
	board_init();
	
	delay_init();
	
	// Ativa o PIO na qual o LED foi conectado
	// para que possamos controlar o LED.
 	pmc_enable_periph_clk(LED0_PIO_ID);
 	pmc_enable_periph_clk(LED1_PIO_ID);
	pmc_enable_periph_clk(LED2_PIO_ID);
	pmc_enable_periph_clk(LED3_PIO_ID);
	
	//Inicializa PC8 como saída
	pio_configure(LED0_PIO, PIO_OUTPUT_0, LED0_PIO_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_PIO_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_PIO_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED3_PIO, PIO_OUTPUT_0, LED3_PIO_IDX_MASK, PIO_DEFAULT);
	
	// Inicializa PIO do botao
	pmc_enable_periph_clk(BUT0_PIO_ID);
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);
	
	// configura pino ligado ao botão como entrada com um pull-up.
	pio_configure(BUT0_PIO, PIO_INPUT, BUT0_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
  	pio_set_debounce_filter(BUT0_PIO, BUT0_IDX_MASK, 60);
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
  	pio_set_debounce_filter(BUT1_PIO, BUT1_IDX_MASK, 60);
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
  	pio_set_debounce_filter(BUT2_PIO, BUT2_IDX_MASK, 60);
	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
  	pio_set_debounce_filter(BUT3_PIO, BUT3_IDX_MASK, 60);

	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BUT0_PIO,
					BUT0_PIO_ID,
					BUT0_IDX_MASK,
					PIO_IT_EDGE,
					but_callback);
	pio_handler_set(BUT1_PIO,
					BUT1_PIO_ID,
					BUT1_IDX_MASK,
					PIO_IT_EDGE,
					but_callback);
	pio_handler_set(BUT2_PIO,
					BUT2_PIO_ID,
					BUT2_IDX_MASK,
					PIO_IT_FALL_EDGE,
					start_stop_callback);	
	pio_handler_set(BUT3_PIO,
					BUT3_PIO_ID,
					BUT3_IDX_MASK,
					PIO_IT_FALL_EDGE,
					decrease_freq_callback);

	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT0_PIO, BUT0_IDX_MASK);
	pio_get_interrupt_status(BUT0_PIO);
	pio_enable_interrupt(BUT1_PIO, BUT1_IDX_MASK);
	pio_get_interrupt_status(BUT1_PIO);
	pio_enable_interrupt(BUT2_PIO, BUT2_IDX_MASK);
	pio_get_interrupt_status(BUT2_PIO);
	pio_enable_interrupt(BUT3_PIO, BUT3_IDX_MASK);
	pio_get_interrupt_status(BUT3_PIO);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT0_PIO_ID);
	NVIC_SetPriority(BUT0_PIO_ID, 4);
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4);
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 5);
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 4);	
}

int main (void)
{
	init();

  	// Init OLED
	gfx_mono_ssd1306_init();

	// Frequência inicial
	int frequency = 300;

	// Desliga LED2
    pio_set(LED2_PIO, LED2_PIO_IDX_MASK);

	// Frequência inicial na tela
	char freq_str[128];
	sprintf(freq_str, "%d ms", frequency);
	gfx_mono_draw_string(freq_str, 5, 5, &sysfont);
  
  

  /* Insert application code here, after the board has been initialized. */
	while(1) {

			if(change_freq_flag || start_stop_flag || decrease_freq_flag) {
				if(change_freq_flag) {
					frequency = change_frequency(frequency);
				}
				else if(decrease_freq_flag) {
					frequency = change_frequency(frequency);
				}
				else if(start_stop_flag) {
					start_stop_flag = 0;
					pisca_led(LED2_PIO, LED2_PIO_IDX_MASK, 30, frequency);
			}

			change_freq_flag = 0;
			start_stop_flag = 0;
			decrease_freq_flag = 0;
		}
   		// Entra em sleep mode    
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI); // SLEEP
			
			
	}
}
