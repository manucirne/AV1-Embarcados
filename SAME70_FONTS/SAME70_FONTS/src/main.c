/*
 * main.c
 *
 * Created: 05/03/2019 18:00:58
 *  Author: eduardo
 */ 

#include "rtt.h"
#include "tc.h"
#include <asf.h>
#include "tfont.h"
#include "sourcecodepro_28.h"
#include "calibri_36.h"
#include "arial_72.h"
#include "stdio.h"

#define YEAR        2019
#define MOUNTH      4
#define DAY         0
#define WEEK        0
#define HOUR        0
#define MINUTE      0
#define SECOND      0
#define MINUTEA     0

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
	#endif
	
#define TC_0 TC0
#define TC_CH 0
#define TC_ID ID_TC0

#define LED_PIO       PIOC
#define LED_PIO_ID    ID_PIOC
#define LED_IDX       8u
#define LED_IDX_MASK  (1u << LED_IDX)


#define BUT_PIO           PIOA // periferico que controla o LED
#define BUT_PIO_ID        ID_PIOA // ID do periférico PIOC (controla LED)
#define BUT_PIO_IDX       19 // ID do LED no PIO
#define BUT_PIN_MASK  (1 << BUT_PIO_IDX)

#define BUT_PIO_ID2			  ID_PIOC
#define BUT_PIO2				  PIOC
#define BUT_PIN2				  31
#define BUT_PIN_MASK2			  (1 << BUT_PIN2)

volatile int count = 0;
volatile int countvel = 0;
volatile int dist = 0;
volatile Bool f_rtt_alarme = false;
uint32_t vel = 0;
char Buffer_Vel[32];
char Buffer_Time[32];
char Buffer_Dist[32];
volatile int timeN;
volatile Bool f_time = false;
volatile Bool pause = false;

/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void BUT_init(void);
void RTC_init(void);
void pin_toggle(Pio *pio, uint32_t mask);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void io_init(void);
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);
void font_draw_text(tFont *font, const char *text, int x, int y, int spacing);

/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/
void Button_Handler(){
	count += 1;
	countvel += 1;
}
void Button2_Handler(){
	pause = !pause;
}

void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/*
	*  Verifica por qual motivo entrou
	*  na interrupcao, se foi por segundo
	*  ou Alarm
	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
		f_time = true;
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
			rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
			
			
			
	}
	
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
	
}

void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
		
	  }

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		
		dist = count*3.14*2*0.5;
		vel = countvel*2*3.14*0.5/4;
		countvel = 0;
		pin_toggle(LED_PIO, LED_IDX_MASK);    // BLINK Led
		f_rtt_alarme = true;                  // flag RTT alarme
	}
}
	
void TC0_Handler(void){
		//countvel = 0;
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

void io_init(void){
	/* led */
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);
}

static float get_time_rtt(){
	uint ul_previous_time = rtt_read_timer_value(RTT);
}

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));
	
	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	dist += 2*3.14*count;
	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
}



struct ili9488_opt_t g_ili9488_display_opt;

void configure_lcd(void){
	/* Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

	/* Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
	
}


void font_draw_text(tFont *font, const char *text, int x, int y, int spacing) {
	char *p = text;
	while(*p != NULL) {
		char letter = *p;
		int letter_offset = letter - font->start_char;
		if(letter <= font->end_char) {
			tChar *current_char = font->chars + letter_offset;
			ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
			x += current_char->image->width + spacing;
		}
		p++;
	}	
}

void BUT_init(void){
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(BUT_PIO_ID);
	
	pio_set_input(BUT_PIO, BUT_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	/* config. interrupcao em borda de descida no botao do kit */
	/* indica funcao (but_Handler) a ser chamada quando houver uma interrupção */
	pio_enable_interrupt(BUT_PIO, BUT_PIN_MASK);
	
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIN_MASK, PIO_IT_FALL_EDGE, Button_Handler);

	/* habilita interrupçcão do PIO que controla o botao */
	/* e configura sua prioridade                        */
	NVIC_EnableIRQ(BUT_PIO_ID);
	
	NVIC_SetPriority(BUT_PIO_ID, 1);
};
void BUT2_init(void){
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(BUT_PIO_ID2);
	
	pio_set_input(BUT_PIO2, BUT_PIN_MASK2, PIO_PULLUP | PIO_DEBOUNCE);

	/* config. interrupcao em borda de descida no botao do kit */
	/* indica funcao (but_Handler) a ser chamada quando houver uma interrupção */
	pio_enable_interrupt(BUT_PIO2, BUT_PIN_MASK2);
	
	pio_handler_set(BUT_PIO2, BUT_PIO_ID2, BUT_PIN_MASK2, PIO_IT_FALL_EDGE, Button2_Handler);

	/* habilita interrupçcão do PIO que controla o botao */
	/* e configura sua prioridade                        */
	NVIC_EnableIRQ(BUT_PIO_ID2);
	
	NVIC_SetPriority(BUT_PIO_ID2, 0);
};

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	uint32_t channel = 1;

	/* Configura o PMC */
	/* O TimerCounter é meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrupçcão no TC canal 0 */
	/* Interrupção no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}
void FuncRTC(){
	if (f_rtt_alarme){
			//interrupção a cada segundo
			uint16_t pllPreScale = (int) (((float) 32768) / 2.0);
			uint32_t irqRTTvalue  = 4;
			RTT_init(pllPreScale, irqRTTvalue); 
			
			sprintf(Buffer_Vel,"Vel: %02d m/s", vel);
			font_draw_text(&calibri_36, Buffer_Vel , 50, 200, 2);
			
		
			sprintf(Buffer_Dist,"Dist: %02d m",dist);
			font_draw_text(&calibri_36, Buffer_Dist , 50,250, 4);
			
			 /*
			* caso queira ler o valor atual do RTT, basta usar a funcao
			*   rtt_read_timer_value()
			 */	
			 
			 f_rtt_alarme = false;
			
		}
}

void countTime(){
	if(f_time){
		uint h, m, s;
		rtc_get_time(RTC, &h, &m ,&s );
		sprintf(Buffer_Time,"Time:%02dh %02dm  %02ds",h, m, s); // não tava cabendo tirei a hora
		font_draw_text(&calibri_36, Buffer_Time , 10, 100, 3);
		f_time = false;
		
	}
}


void RTC_init(){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(RTC, YEAR, MOUNTH, DAY, WEEK);
	rtc_set_time(RTC, HOUR, MINUTE, SECOND);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 0);
	NVIC_EnableIRQ(RTC_IRQn);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(RTC, RTC_IER_SECEN);

}



int main(void) {
	
	// Desliga watchdog
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	
	board_init();
	sysclk_init();	
	configure_lcd();
	io_init();
	BUT_init();
	BUT2_init();
	RTC_init();
	
	//TC_init(TC_0, TC_ID, TC_CH, 1);

	// Inicializa RTT com IRQ no alarme.
	f_rtt_alarme = true;
	
	//font_draw_text(&sourcecodepro_28, "OIMUNDO", 50, 50, 1);
	
	
	while(1) {
		while(pause){}
		countTime();
		FuncRTC();
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		
	}
}