#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

typedef struct
{
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
} calendar;

// LEDs

// LED 0
#define LED0_PIO PIOC
#define LED0_PIO_ID ID_PIOC
#define LED0_PIO_IDX 8
#define LED0_PIO_IDX_MASK (1 << LED0_PIO_IDX)

// LED 1
#define LED1_PIO PIOA
#define LED1_PIO_ID ID_PIOA
#define LED1_PIO_IDX 0
#define LED1_PIO_IDX_MASK (1 << LED1_PIO_IDX)

// LED 2
#define LED2_PIO PIOC
#define LED2_PIO_ID ID_PIOC
#define LED2_PIO_IDX 30
#define LED2_PIO_IDX_MASK (1 << LED2_PIO_IDX)

// LED 3
#define LED3_PIO PIOB
#define LED3_PIO_ID ID_PIOB
#define LED3_PIO_IDX 2
#define LED3_PIO_IDX_MASK (1 << LED3_PIO_IDX)

// Botões
// Botão 0
#define BUT0_PIO PIOA
#define BUT0_PIO_ID ID_PIOA
#define BUT0_IDX 11
#define BUT0_IDX_MASK (1 << BUT0_IDX)

// Botão 1
#define BUT1_PIO PIOD
#define BUT1_PIO_ID ID_PIOD
#define BUT1_IDX 28
#define BUT1_IDX_MASK (1 << BUT1_IDX)

// Botão 2
#define BUT2_PIO PIOC
#define BUT2_PIO_ID ID_PIOC
#define BUT2_IDX 31
#define BUT2_IDX_MASK (1 << BUT2_IDX)

// Botão 3
#define BUT3_PIO PIOA
#define BUT3_PIO_ID ID_PIOA
#define BUT3_IDX 19
#define BUT3_IDX_MASK (1 << BUT3_IDX)

void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);
void pin_toggle(Pio *pio, uint32_t mask);

volatile char flag_rtc_alarm = 0;
volatile char flag_rtc_second = 0;
volatile char but1_flag;

void but1_callback(void)
{
	but1_flag = 1;
}

void oled_update(uint32_t hour, uint32_t min, uint32_t sec)
{
	char date_str[128];
	gfx_mono_draw_string("            ", 0, 0, &sysfont);
	// gfx_mono_draw_filled_rect(0, 0, 128, 64, GFX_PIXEL_CLR);
	sprintf(date_str, "%02d:%02d:%02d", hour, min, sec);
	gfx_mono_draw_string(date_str, 5, 5, &sysfont);
}

void LED_init(Pio *p_pio, uint32_t ul_id, const uint32_t mask, int estado)
{
	pmc_enable_periph_clk(ul_id);
	pio_set_output(p_pio, mask, estado, 0, 0);
};

void BUT_init(Pio *p_pio, uint32_t ul_id, const uint32_t mask)
{
	pmc_enable_periph_clk(ul_id);
	pio_configure(p_pio, PIO_INPUT, mask, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(p_pio, mask, 60);
};

void pisca_led(Pio *p_pio, const uint32_t mask, int n, int t)
{
	for (int i = 0; i < n; i++)
	{
		pio_clear(p_pio, mask);
		delay_ms(t);
		pio_set(p_pio, mask);
		delay_ms(t);
	}
}

void pin_toggle(Pio *pio, uint32_t mask)
{
	if (pio_get_output_data_status(pio, mask))
		pio_clear(pio, mask);
	else
		pio_set(pio, mask);
}

void TC0_Handler(void)
{

	volatile uint32_t status = tc_get_status(TC0, 0);

	pin_toggle(LED3_PIO, LED3_PIO_IDX_MASK);
}

void TC1_Handler(void)
{

	volatile uint32_t status = tc_get_status(TC0, 1);

	pin_toggle(LED0_PIO, LED0_PIO_IDX_MASK);
}

void TC2_Handler(void)
{

	volatile uint32_t status = tc_get_status(TC0, 2);

	pin_toggle(LED1_PIO, LED1_PIO_IDX_MASK);
}

void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS)
	{
		RTT_init(4, 16, RTT_MR_RTTINCIEN);
		pin_toggle(LED2_PIO, LED2_PIO_IDX_MASK); // BLINK Led
	}

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC)
	{
	}
}

void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/* seccond tick */
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC)
	{
		// o código para irq de segundo vem aqui
		flag_rtc_second = 1;
	}

	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM)
	{
		// o código para irq de alame vem aqui
		flag_rtc_alarm = 1;
	}

	rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}

void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq)
{
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura NVIC*/
	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type)ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource)
{

	uint16_t pllPreScale = (int)(((float)32768) / freqPrescale);

	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);

	if (rttIRQSource & RTT_MR_ALMIEN)
	{
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT))
			;
		rtt_write_alarm_time(RTT, IrqNPulses + ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
		rtt_enable_interrupt(RTT, rttIRQSource);
	else
		rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
}

void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type)
{
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.second);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc, irq_type);
}

int main(void)
{
	sysclk_init();
	board_init();
	delay_init();

	// Init OLED
	gfx_mono_ssd1306_init();

	LED_init(LED0_PIO, LED0_PIO_ID, LED0_PIO_IDX_MASK, 0);
	LED_init(LED1_PIO, LED1_PIO_ID, LED1_PIO_IDX_MASK, 0);
	LED_init(LED2_PIO, LED2_PIO_ID, LED2_PIO_IDX_MASK, 0);
	LED_init(LED3_PIO, LED3_PIO_ID, LED3_PIO_IDX_MASK, 1);
	BUT_init(BUT1_PIO, BUT1_PIO_ID, BUT1_IDX_MASK);

	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BUT1_PIO,
					BUT1_PIO_ID,
					BUT1_IDX_MASK,
					PIO_IT_FALL_EDGE,
					but1_callback);

	// PIO_IT_RISE_EDGE, PIO_IT_FALL_EDGE
	//  Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT1_PIO, BUT1_IDX_MASK);
	pio_get_interrupt_status(BUT1_PIO);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4); // Prioridade 4

	TC_init(TC0, ID_TC1, 1, 5);
	tc_start(TC0, 1);

	TC_init(TC0, ID_TC2, 2, 4);
	tc_start(TC0, 2);

	RTT_init(4, 16, RTT_MR_ALMIEN);

	calendar rtc_initial = {2018, 3, 19, 12, 15, 45, 1};
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_SR_SEC | RTC_SR_ALARM);

	// char date_str[128];
	// sprintf(date_str, "%02d:%02d:%02d", 12, 15, 45);
	// gfx_mono_draw_string(date_str, 5, 5, &sysfont);

	/* Leitura do valor atual do RTC */
	uint32_t current_hour, current_min, current_sec;
	uint32_t current_year, current_month, current_day, current_week;

	/* Insert application code here, after the board has been initialized. */
	while (1)
	{

		if (flag_rtc_second)
		{
			rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
			rtc_get_time(RTC, &current_hour, &current_min, &current_sec);

			oled_update(current_hour, current_min, current_sec);
		}

		if (but1_flag)
		{
			uint32_t next_min, next_sec;

			next_sec = current_sec + 20;

			if (next_sec >= 60)
			{
				next_min = current_min + 1;
				next_sec = next_sec - 60;
			}
			else
			{
				next_min = current_min;
			}

			rtc_set_time_alarm(RTC, 1, current_hour, 1, next_min, 1, next_sec);

			but1_flag = 0;
		}

		if (flag_rtc_alarm)
		{
			TC_init(TC0, ID_TC0, 0, 4);
			tc_start(TC0, 0);
			// pisca_led(LED3_PIO, LED3_PIO_IDX_MASK, 5, 20);

			flag_rtc_alarm = 0;
		}

		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}
