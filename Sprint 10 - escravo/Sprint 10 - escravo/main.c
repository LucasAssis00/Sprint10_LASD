/*
 * Sprint 10 - escravo.c
 *
 * Created: 10/10/2021 10:08:01 PM
 * Author : justi
 */ 


#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1


uint32_t tempo[3] = {1, 1, 4}; //pras 3 cores, vermelho, amarelo e verde, usadas durante o código e no display em inglês para ter iniciais diferentes
uint32_t g_mestre, y_mestre, r_mestre;

int aux = 0; // variavel auxiliar para função exe4cuta tudo, pra não correr o risco ede executar mais de uma das configurações der cores nas leds

uint32_t tempo_100us = 0; // variavel que cnsidera a passagem do tempo

char recebido;


ISR(TIMER0_COMPA_vect) //interrupcao do TC0 a cada 100us = (8*(199+1))/16MHz
{
	executa_tudo(tempo_100us);	//acende LEDs antes de adicionar o termpo
	tempo_100us++;
}


ISR(USART_RX_vect)
{
	static uint8_t flag_inicio = 0;
	
	recebido = UDR0;

	switch(recebido)
	{
		case 'g':
			flag_inicio = 1;
			break;
		case 'y':
			flag_inicio = 2;
			break;
		case 'r':
			flag_inicio = 3;
			break;
		default:
			if(flag_inicio==1)	g_mestre = recebido - '0';
			if(flag_inicio==2)	y_mestre = recebido - '0';
			if(flag_inicio==3)	r_mestre = recebido - '0';
			
			flag_inicio = 0;
			
			break;
	}
		tempo[0] = r_mestre - y_mestre;
		tempo[1] = y_mestre;
		tempo[2] = g_mestre + y_mestre;			//a equacao ta certa, mas o tempo nao bate (ta menor), nao sei o motivo
}
//recebe char no formato ['r'][r_time]['y'][y_time]['g'][g_time]



int main(void)
{
	DDRB |= 0b11111111; //habilita leds verdes e vermelhas como saídas
	DDRD |= 0b10000000; //habilita led amarela como saída d7


	DDRD &= 0b111100010; //habilitar botões como entrada d2 e d3 e d4		e d0
	PORTD |= 0b00011101; //habilitar resistores de pull-up d2 e d3 e d4		e d0


	EICRA = 0b00001010; //interrupcao externa int0 e int1 na borda de descida
	EIMSK = 0b00000011; //habilita interrupcao externa int0 e int1

	PCICR = 0b00000100; //habilita interrupção pcint2 port D ???
	PCIFR = 0b00000100; //sinalizadores que indicam se ocorreu interrupcao em pcint2
	PCMSK2 = 0b00010000; //habilita interrupção pcint2 port D4 ???

	TCCR0A = 0b00000010; //habilita modo CTC do TC0
	TCCR0B = 0b00000010; //liga TC0 com prescaler = 8
	OCR0A = 199; //ajusta o comparador para o TC0 contar ate 199
	TIMSK0 = 0b00000010; //habilita a interrupcao na igualdade de comparacao com OCR0A
	
	//CONFIG USART
	UBRR0H = (unsigned char)(MYUBRR>>8); //Ajusta a taxa de transmissão, PARTE ALTA
	UBRR0L = (unsigned char)MYUBRR;		//Ajusta a taxa de transmissão, PARTE BAIXA
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0); //HABILITA A INTERRUP. DO RECEPTOR, Habilita o transmissor e o receptor
	UCSR0C = (3<<UCSZ00); //Ajusta o formato do frame: 8 bits de dados e 1 de parada, PARIDADE NONE
	
	sei(); //habilita a chave de interrupcao global

	while (1)
	{
	}
}


executa_tudo(uint32_t TEMPO_100us)
{
	static uint32_t TEMPO_100us_anterior = 0;	//compara com o tempo_100us pra acender as combinações dos LEDs
		if(recebido =='Z'){
			if((tempo_100us - TEMPO_100us_anterior) >= 0)
			{
				if(aux==0){
					PORTD &= 0b01111111;//apaga amarelo
					PORTB = 0b11110000;
					aux++;
				}
			}
			if((tempo_100us - TEMPO_100us_anterior) >= 2500*tempo[2])
			{
				if(aux==1){
					PORTB = 0b01110000;
					aux++;
				}
			}
			if((tempo_100us - TEMPO_100us_anterior) >= 2*2500*tempo[2])
			{
				if(aux==2){
					PORTB = 0b00110000;
					aux++;
				}
			}
			if((tempo_100us - TEMPO_100us_anterior) >= 3*2500*tempo[2])
			{
				if(aux==3){
					PORTB = 0b00010000;
					aux++;
				}
			}
			if((tempo_100us - TEMPO_100us_anterior) >= 4*2500*tempo[2])
			{
				if(aux==4)  {
					PORTB = 0b00001111; 		//portd usado em outras aplicações, por isso precisa de máscara
					aux++;
				}
			}
			if((tempo_100us - TEMPO_100us_anterior) >= (4*2500*tempo[2]+2500*tempo[0]))
			{
				if(aux==5)  {
					PORTB = 0b00000111;
					aux++;
				}
			}
			if((tempo_100us - TEMPO_100us_anterior) >= (4*2500*tempo[2]+2*2500*tempo[0]))
			{
				if(aux==6){
					PORTB = 0b00000011;
			
					aux++;
				}
			}
			if((tempo_100us - TEMPO_100us_anterior) >= (4*2500*tempo[2]+3*2500*tempo[0]))
			{
				if(aux==7){
					PORTB = 0b00000001;
					aux++;
				}
			}
			if((tempo_100us - TEMPO_100us_anterior) >= (4*2500*tempo[2]+4*2500*tempo[0]))
			{
				if(aux==8){
					PORTB = 0b00000000;
					PORTD |= 0b10000000;		//portd usado em outras aplicações, por isso precisa de máscara
					aux++;
				}
			}
			if(recebido == 'i')
			{
				recebido = ' ';
				TEMPO_100us_anterior = tempo_100us;
				aux = 0;
			}
		}
		
		if(recebido =='X'){
			PORTB = 0b00001111;
		}
		
		if(recebido =='X'){
			PORTB = 0b11110000;
		}
}
//	if(recebido=='i'){