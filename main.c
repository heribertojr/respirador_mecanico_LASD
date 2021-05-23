/*
 * Sprint9.c
 *
 * Created: 22/05/2021 17:04:26
 * Author : Pichau
 */ 

#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#define FREQ_INICIAL 5
#define TAM_VETOR 3

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#include "nokia5110.h"

uint8_t flag_200ms = 0;
uint8_t flag_20ms = 0;
uint8_t flag_freq = 0;
uint8_t flag_adc = 0;
uint8_t flag_erro = 0;
uint8_t flag_o2 = 0;
uint8_t flag_temp = 0;
uint8_t sel = 0;
uint8_t valve = 0;
uint8_t ctrl = 1;
uint8_t FreqRespiracao = FREQ_INICIAL;
uint32_t bpm;
uint8_t cont = 0;
char recebido[10] = {';','H','H','H','x','L','L','L',':','\0'};
float conv_adc0 = 0, conv_adc1 = 0, media_conv_adc0 = 0, media_conv_adc1 = 0;
long int tempo_ms = 0;

ISR (INT0_vect){ //Controle da interrupção PD2
	if(sel == 2)
		if(FreqRespiracao<30)
			FreqRespiracao++;
	if(sel == 4)
		if(valve < 100)
			valve+=10;
	if(sel == 6)
		if(ctrl < 8)
			ctrl++;

	
	OCR1B = ((valve/10)*200) + 2000;
}

ISR (INT1_vect){ //Controle da interrupção PD3
	if(sel == 2)
		if(FreqRespiracao>5)
			FreqRespiracao--;
	if(sel == 4)
		if(valve > 0)
			valve-=10;
	if(sel == 6)
		if(ctrl > 1)
			ctrl--;
	
	
	OCR1B = ((valve/10)*200) + 2000;
}

ISR (TIMER0_COMPA_vect){
	tempo_ms+=1;
	
	if((tempo_ms % 200) == 0)
		flag_200ms = 1;
	if((tempo_ms % (3750/FreqRespiracao)) == 0)
		flag_freq = 1;
	if((tempo_ms % 20) == 0)
		flag_20ms = 1;
}

ISR (PCINT0_vect){
	sel+=1;
	if(sel == 8){
		sel = 0;
	}
}

ISR (PCINT2_vect){
	static long int tempo_inicial = 0;
	bpm = 60000/((tempo_ms-tempo_inicial)*2);
	tempo_inicial = tempo_ms;
}

ISR (ADC_vect)
{
	float aux_adc0 = 0, aux_adc1 = 0;
	if (flag_20ms)
	{
		if(!flag_adc)
		{
			ADMUX = 0b01000000;
			DIDR0 = 0b00111110;
			conv_adc0 = ((25*((5*(ADC))/1024.0)) - (45));//Temperatura
			
			flag_temp++;
			
			aux_adc0 += conv_adc0;
			
			if(flag_temp == 10){
				media_conv_adc0 = (aux_adc0/10.0);
				flag_temp = 0;
				aux_adc0 = 0;
			}
		}
		
		else
		{
			ADMUX = 0b01000001;
			DIDR0 = 0b00111101;
			conv_adc1 = (((5*(ADC))/1024.0)*25); //O2
			
			flag_o2++;
			
			aux_adc1 += conv_adc1;
			
			if(flag_o2 == 10){
				media_conv_adc1 = (aux_adc1/10.0);
				flag_o2 = 0;
				aux_adc1 = 0;
			}
		}
		
		flag_adc = (!flag_adc);
		flag_20ms = 0;
	}
}

ISR (USART_RX_vect){
	static uint8_t i = 0;
	char comp[10] = {'0','1','2','3','4','5','6','7','8','9'};
	uint8_t cont_i = 0;
	if(i>8){
		flag_erro = 1;
		i=0;
	}
	recebido[i] = UDR0;
	recebido[9] = '\0';
	
	if(i == 8){
		if((recebido[0] != ';') | (recebido[8] != ':') | (recebido[4] != 'x'))
		flag_erro = 1;
		else{
			flag_erro = 0;
			for(int k = 0; k < 10; k++){
				if(recebido[1] == comp[k])
				cont_i++;
				if(recebido[2] == comp[k])
				cont_i++;
				if(recebido[3] == comp[k])
				cont_i++;
				if(recebido[5] == comp[k])
				cont_i++;
				if(recebido[6] == comp[k])
				cont_i++;
				if(recebido[7] == comp[k])
				cont_i++;
			}
			if(cont_i != 6)
			flag_erro = 1;
		}
	}
	else{
		flag_erro = 1;
	}
	
	i++;
}

void controle_frequencia (uint8_t FreqRespiracao);
void controle_display (uint8_t FreqRespiracao, uint8_t valve, uint32_t bpm, uint8_t *flag_200ms, float media_conv_adc0, float media_conv_adc1, char pres[]);
void controla_buzzer (float conv_adc0, float conv_adc1);
void USART_init (unsigned int ubrr);

int main(void)
{
	//GPIO
	DDRB = 0b00000110; //Habilita os pinos PB0...PB7 como saídas.
	DDRC = 0b11111100; //Habilita os pinos PC0 e PC1 como entradas.
	DDRD = 0b11100011;   // Habilita os pino PD2, PD3 e PD4 como entrada.
	PORTB = 0b01000000;
	PORTC = 0b00000011;
	PORTD = 0b00011100; // Habilita os resistores de pull-up dos pinos PD2 e PD3.
	
	//Timers
	TCCR0A = 0b00000010; //Habilita o modo CTC do TC0.
	TCCR0B = 0b00000011; //Habilita o prescaler do TC0 como 64.
	OCR0A = 249; //Ajusta o comparador para o TC0 contar até 249.
	TIMSK0 = 0b00000010; //Habilita a interrupção na igualdade de comparação com o TC0.
	ICR1 = 39999;
	TCCR1A = 0b10100010;
	TCCR1B = 0b00011010;
	
	//ADC
	ADCSRA = 0b11101111;
	ADCSRB = 0x00;
	
	//Interrupções
	EICRA = 0b00001010;
	EIMSK = 0b00000011;
	PCICR = 0b00000101;
	PCMSK0 = 0b01000000;
	PCMSK2 = 0b00010000;
	sei();
	
	USART_init (MYUBRR);
	nokia_lcd_init();
	
	while (1)
	{
		controle_frequencia(FreqRespiracao);
		controle_display(FreqRespiracao,valve, bpm, &flag_200ms, media_conv_adc0, media_conv_adc1, recebido);
		controla_buzzer(media_conv_adc0,media_conv_adc1);
	}
}


void controle_frequencia (uint8_t FreqRespiracao){
	static uint8_t down = 1;
	static uint16_t servo = 2000;
	
	OCR1A = servo;
	
	if(flag_freq){
		if (down){
			if(servo == (2000 + (ctrl*250))){
				servo -= (250);
				down = 0;
			}
			else{
				servo += (250);
			}
		}
		else{
			if(servo == 2000){
				servo += (250);
				down = 1;
			}
			
			else{
				servo -= (250);
			}
		}
		flag_freq = 0;
	}
};

void controla_buzzer (float conv_adc0, float conv_adc1)
{
	if (conv_adc0 > 41 || conv_adc0 < 35 || conv_adc1 <60 || OCR1A == 2000)
		PORTD |= 0b10000000;
	else
		PORTD &= 0b01111111;
}

void USART_init (unsigned int ubrr){
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	UCSR0B |= (1<<RXCIE0);
	UCSR0C = (0<<USBS0)|(3<<UCSZ00);
}


void controle_display (uint8_t FreqRespiracao, uint8_t valve, uint32_t bpm, uint8_t *flag_200ms, float media_conv_adc0, float media_conv_adc1, char pres[]){
	unsigned char freq[3];
	unsigned char val [4];
	unsigned char ctr [3];
	unsigned char bat[4];
	unsigned char adc0_inteiro[3];
	unsigned char adc0_decimal [3];
	unsigned char adc1_inteiro [3];
	unsigned char preasure[8];
	unsigned char string_erro;
	for (int l = 0; l < 7; l++){
		preasure[l] = pres[l+1];
	}
	preasure[7] = '\0';
	uint8_t parte_inteira_0 = media_conv_adc0;
	float decimal_0 = (media_conv_adc0 - parte_inteira_0)*10;
	uint8_t parte_decimal_0 = decimal_0;
	uint8_t parte_inteira_1 = media_conv_adc1;
	
	if(*flag_200ms){
		
		nokia_lcd_clear();
		nokia_lcd_set_cursor(0,10);
		
		sprintf(freq, "%u", FreqRespiracao);
		sprintf(val, "%u", valve);
		sprintf(ctr," %u", ctrl);
		sprintf(bat, "%u", bpm);
		sprintf(adc0_inteiro, "%u", parte_inteira_0);
		sprintf(adc0_decimal, "%u", parte_decimal_0);
		sprintf(adc1_inteiro, "%u", parte_inteira_1);
		
		if (sel == 0)
		{
			nokia_lcd_set_cursor(5,2);
			nokia_lcd_write_string("Sinais Vitais", 1);
			
			nokia_lcd_set_cursor(5,12);
			nokia_lcd_write_string(bat,1);
			nokia_lcd_set_cursor(50,11);
			nokia_lcd_write_string("bpm",1);
			
			
			nokia_lcd_set_cursor(5,22);
			nokia_lcd_write_string(adc0_inteiro,1);
			nokia_lcd_set_cursor(17,22);
			nokia_lcd_write_string(",",1);
			nokia_lcd_set_cursor(22,22);
			nokia_lcd_write_string(adc0_decimal,1);
			nokia_lcd_set_cursor(50,21);
			nokia_lcd_write_string("wC",1);
			
			nokia_lcd_set_cursor(5,32);
			nokia_lcd_write_string(adc1_inteiro,1);
			nokia_lcd_set_cursor(45,31);
			nokia_lcd_write_string("%SpO2",1);
			
			nokia_lcd_set_cursor(2,41);
			
			if(flag_erro)
			nokia_lcd_write_string("ERRO!",1);
			else
			nokia_lcd_write_string(preasure,1);
			nokia_lcd_set_cursor(50,40);
			nokia_lcd_write_string("mmHg",1);
			nokia_lcd_render();
		}
		if (sel == 2){
			nokia_lcd_set_cursor(5,2);
			nokia_lcd_write_string("Parametros", 1);
			nokia_lcd_set_cursor(5,15);
			nokia_lcd_write_string(freq, 1);
			nokia_lcd_set_cursor(25,14);
			nokia_lcd_write_string("* resp/min",1);
			nokia_lcd_set_cursor(5,25);
			nokia_lcd_write_string(val, 1);
			nokia_lcd_set_cursor(25,24);
			nokia_lcd_write_string("  %O2",1);
			nokia_lcd_set_cursor(0,35);
			nokia_lcd_write_string(ctr, 1);
			nokia_lcd_set_cursor(25,34);
			nokia_lcd_write_string("   vol",1);
			nokia_lcd_render();
		}
		if(sel == 4){
			nokia_lcd_set_cursor(5,2);
			nokia_lcd_write_string("Parametros", 1);
			nokia_lcd_set_cursor(5,15);
			nokia_lcd_write_string(freq, 1);
			nokia_lcd_set_cursor(25,14);
			nokia_lcd_write_string("  resp/min",1);
			nokia_lcd_set_cursor(5,25);
			nokia_lcd_write_string(val, 1);
			nokia_lcd_set_cursor(25,24);
			nokia_lcd_write_string("*  %O2",1);
			nokia_lcd_set_cursor(0,35);
			nokia_lcd_write_string(ctr, 1);
			nokia_lcd_set_cursor(25,34);
			nokia_lcd_write_string("   vol",1);
			nokia_lcd_render();
		}
		if(sel == 6){
			nokia_lcd_set_cursor(5,2);
			nokia_lcd_write_string("Parametros", 1);
			nokia_lcd_set_cursor(5,15);
			nokia_lcd_write_string(freq, 1);
			nokia_lcd_set_cursor(25,14);
			nokia_lcd_write_string("  resp/min",1);
			nokia_lcd_set_cursor(5,25);
			nokia_lcd_write_string(val, 1);
			nokia_lcd_set_cursor(25,24);
			nokia_lcd_write_string("   %O2",1);
			nokia_lcd_set_cursor(0,35);
			nokia_lcd_write_string(ctr, 1);
			nokia_lcd_set_cursor(25,34);
			nokia_lcd_write_string("*  vol",1);
			nokia_lcd_render();
		}
		flag_200ms = 0;
	}
};

