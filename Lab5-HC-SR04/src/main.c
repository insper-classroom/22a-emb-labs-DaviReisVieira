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

#define TRIG_PIO PIOD
#define TRIG_PIO_ID ID_PIOD
#define TRIG_PIO_IDX 30
#define TRIG_PIO_IDX_MASK (1 << TRIG_PIO_IDX)

#define ECHO_PIO PIOA
#define ECHO_PIO_ID ID_PIOA
#define ECHO_PIO_IDX 6
#define ECHO_PIO_IDX_MASK (1 << ECHO_PIO_IDX)

#define VELOCIDADE_SOM 340

void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);
void pin_toggle(Pio *pio, uint32_t mask);

volatile char but1_flag = 0;
volatile char echo_flag = 0;
volatile char rtt_alarm = 0;

volatile int number_of_pulses = 0;
float frequency = (float)1 / (0.000058 * 2);

void but1_callback(void)
{
	but1_flag = 1;
}

void draw_contact_error()
{
	gfx_mono_draw_string("     ", 0, 0, &sysfont);
	gfx_mono_draw_string("ERRO!", 0, 0, &sysfont);
}

void callback_echo(void)
{
	float time_alarme = 4 / VELOCIDADE_SOM;
	if (!pio_get(ECHO_PIO, PIO_INPUT, ECHO_PIO_IDX_MASK))
	{
		echo_flag = 0;
		number_of_pulses = rtt_read_timer_value(RTT);
	}
	else
	{

		if (but1_flag)
		{
			// RTT_init(frequency, (int)time_alarme * frequency, RTT_MR_ALMIEN);
			RTT_init(frequency, 0, 0);
			echo_flag = 1;
		}
		else
		{
			rtt_alarm = 1;
		}
		but1_flag = 0;
	}
}

float distance_read()
{
	while (echo_flag)
	{
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI); // SLEEP
	}

	float time = (float)number_of_pulses / frequency;
	float distancia = (VELOCIDADE_SOM * time * 100) / 2.0;

	return distancia;
}

int oled_update(float distancia, int x_position)
{
	if ((distancia >= 400))
	{
		gfx_mono_draw_string("                    ", 0, 0, &sysfont);
		gfx_mono_draw_string("Espaco aberto.", 0, 0, &sysfont);
		rtt_alarm = 0;
		return x_position;
	}

	char string[20];
	sprintf(string, "%2.1fcm", distancia);
	gfx_mono_draw_string("                    ", 0, 0, &sysfont);
	gfx_mono_draw_string(string, 0, 0, &sysfont);

	if (x_position == 0)
	{
		gfx_mono_generic_draw_filled_rect(0, 0, 128, 31, GFX_PIXEL_CLR);
	}

	volatile int posicao = 16 + (31 - 16) * (400 - (distancia)) / 398;
	gfx_mono_generic_draw_horizontal_line(0 + x_position, posicao, 1, GFX_PIXEL_SET);
	x_position = x_position >= 128 ? 0 : x_position + 1;
	return x_position;
}

void configure_interruption(Pio *pio, uint32_t ul_id, const uint32_t ul_mask, uint32_t ul_attr, void (*p_handler)(uint32_t, uint32_t), uint32_t priority)
{
	pio_handler_set(pio, ul_id, ul_mask, ul_attr, p_handler);
	pio_enable_interrupt(pio, ul_mask);
	pio_get_interrupt_status(pio);
	NVIC_EnableIRQ(ul_id);
	NVIC_SetPriority(ul_id, priority);
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

void SR04_init(void)
{
	pmc_enable_periph_clk(TRIG_PIO_ID);
	pmc_enable_periph_clk(ECHO_PIO_ID);
	pio_set_output(TRIG_PIO, TRIG_PIO_IDX_MASK, 0, 0, 0);
	pio_configure(ECHO_PIO, PIO_INPUT, ECHO_PIO_IDX_MASK, PIO_DEBOUNCE);
	pio_set_debounce_filter(ECHO_PIO_IDX, ECHO_PIO_IDX_MASK, 60);

	configure_interruption(ECHO_PIO, ECHO_PIO_ID, ECHO_PIO_IDX_MASK, PIO_IT_EDGE, callback_echo, 1);
}

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

void trig_pulse()
{
	pio_set(TRIG_PIO, TRIG_PIO_IDX_MASK);
	delay_us(10);
	pio_clear(TRIG_PIO, TRIG_PIO_IDX_MASK);
}

void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS)
	{
		rtt_alarm = 1;
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
	}

	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM)
	{
		// o código para irq de alame vem aqui
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

	gfx_mono_ssd1306_init();

	LED_init(LED1_PIO, LED1_PIO_ID, LED1_PIO_IDX_MASK, 0);
	BUT_init(BUT1_PIO, BUT1_PIO_ID, BUT1_IDX_MASK);
	configure_interruption(BUT1_PIO, BUT1_PIO_ID, BUT1_IDX_MASK, PIO_IT_FALL_EDGE, but1_callback, 4);

	SR04_init();

	int x_position = 0;

	// RTT_init(4, 16, RTT_MR_ALMIEN);

	/* Insert application code here, after the board has been initialized. */
	while (1)
	{
		if (rtt_alarm)
		{
			draw_contact_error();
			rtt_alarm = 0;
		}
		else if (echo_flag)
		{
			float distancia = distance_read();

			x_position = oled_update(distancia, x_position);

			but1_flag = 0;
		}
		else if (but1_flag)
		{
			trig_pulse();
		}

		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}
