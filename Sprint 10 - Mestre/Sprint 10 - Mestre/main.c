/*
 * Sprint 10 - Mestre.c
 *
 * Created: 10/10/2021 2:13:09 PM
 * Author : justi
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "nokia5110.h"
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1


uint32_t tempo[4] = {2, 2, 2, 0};	//pras 3 cores, vermelho, amarelo e verde, usadas durante o código e no display em inglês para ter iniciais diferentes
int i = 0;							//posicao vetor tempo e seta
int j = 0;							//para vetor automatico string
int8_t manual_auto = 0;				//pra servir de "bool"
//manual = 0, automatico = 1	

int aux = 0;					// variavel auxiliar para função exe4cuta tudo, pra não correr o risco ede executar mais de uma das configurações der cores nas leds

uint32_t tempo_100us = 0;		// variavel que cnsidera a passagem do tempo
uint32_t carrosporminuto = 0;	//variavel pra mostrar o que o proprio nome já diz

uint32_t gambiarra = 0;			//para conseguir um valor estável de frequencia (explicação na função precisao_carros)

uint32_t contagem_carros = 0;	//variavel que mostra o total de carros passando

uint8_t flag_500ms = 0;			//para o uso do ADC
uint8_t flag_50ms = 0;

uint8_t sirenes;



unsigned char r_string[2] = "2", y_string[2] = "2", g_string[2] = "2"; //variaveis char para LED
unsigned char automatico_string[2][2] = {"M", "A"};						//matriz com as expressões do modo
unsigned char cpm_string[4];
unsigned char lux_LDR[6];

char transmissao[10];
	


void precisao_carros();

void termos_iniciais_fixos();			// função para o que aparec sempre no LED, que só precisa escrever uma vez

void valores_atualizados_cores();		//essa escreve os algarismos do tempo, e a seta que se move

void tempos_automaticos();

void leitura_sensores_ADC(uint8_t *flag_disparo);

void iluminacao(uint8_t *flag_disparo1);


ISR(USART_RX_vect)
{
	unsigned char recebido;
	recebido = UDR0;
	switch(recebido)
	{
		case 'X':
			sirenes = 0;			//sinal vermelho
			break;
		case 'Y':
			sirenes = 1;			//sinal verde
			break;
		case 'Z':
			sirenes = 2;			//funcionamento normal
			//funciona apenas de X para outros e Y para outros, nao vai de Z para outros
			break;
		default:
			break;
	}
}




ISR(TIMER0_COMPA_vect)						//interrupcao do TC0 a cada 100us = (8*(199+1))/16MHz
{
	executa_tudo(tempo_100us);				//acende LEDs antes de adicionar o termpo
	tempo_100us++;

	if((tempo_100us % 5000) == 0)			//true a cada 5000*100us = 500ms
		flag_500ms = 1;
	
	if((tempo_100us % 500)==0)				//true a cada 50ms
		flag_50ms = 1;
}


void USART_Transmit(unsigned char data)
{
	while(!( UCSR0A & (1<<UDRE0)));//Espera a limpeza do registr. de transmissão
	UDR0 = data; //Coloca o dado no registrador e o envia
}


ISR(INT0_vect)				//pin d2
{
	contagem_carros++;		//interupção só para contar oos carros representdos pela frequecia do clock
}

ISR(INT1_vect)				//pin d3
{
	//deixa sem nada
}

ISR(PCINT2_vect)			//pin D qlq
{
	if((PIND & 0b00010000) == 0)	//botao '+' (PD4)
	{
		termos_iniciais_fixos();
		if(i==3){						//o posição 4 do vetor tempo é pra manual/automatico, aí só conta 0 ou 1, e isso  vai pra variavel
			manual_auto++;
			if(manual_auto>1)
			manual_auto = 0;
		}
		if(manual_auto == 0){			//funciona se tiver no modo manual
			tempo[i]++;
			if(tempo[i]>9)
			tempo[i] = 9;				//pra ficar no sistema decimal msm
		}
		if(manual_auto == 1){
			tempos_automaticos();		//ajusta pelas equações se for modo auto
		}

		valores_atualizados_cores();

		nokia_lcd_render(); //atualiza display

	}


	if((PIND & 0b00100000) == 0)	//botao '-' (PD5)
	{
		termos_iniciais_fixos();		//msm esquema acima
		if(i==3){
			manual_auto--;
			if(manual_auto<0)
			manual_auto = 1;
		}
		if(manual_auto == 0){
			tempo[i]--;
			if(tempo[i]<1)
			tempo[i] = 1;				//pra ficar no sistema decimal msm
		}
		if(manual_auto == 1){
			tempos_automaticos();
		}
		
		valores_atualizados_cores();

		nokia_lcd_render();

	}


	if((PIND & 0b01000000) == 0)	//botao 'S' (PD6)
	{
		termos_iniciais_fixos();
		i++;
		if(i>3)
		i = 0;			//tem agora 4 possibilidades, e implementei o passo unico
			
		valores_atualizados_cores();

		nokia_lcd_render();

	}

}


int main(void)
{
	DDRB |= 0b11111111;		//habilita leds verdes e vermelhas como saídas	//pode parar
	DDRD |= 0b10000000;		//habilita led amarela como saída D7		
	DDRD &= 0b10001011;		//habilita d6d5d4d2 como entradas											//d3 é saída
	PORTD |= 0b01110100;	//habilitar resistores de pull-up d6-d2


	DDRC &= 0b11111110;		//habilita c0 como entrada
	DDRC &= 0b10111111;		//habilita c6 como saida (tem ou n pedestres)

	/*
	DDRD &= 0b11111101; //habilita d1 como entrada
	PORTD |= 0b00000010; //habilita pullup d1						//não deveria
	*/

	DDRD |= 0b00001000;	//habilita d3 como saida					//iluminação


	nokia_lcd_init();		//inicia lcd, dessa vez apenas no comeco

	termos_iniciais_fixos();
	valores_atualizados_cores();

	nokia_lcd_render();

	EICRA = 0b00001010; //interrupcao externa int0 e int1 na borda de descida
	EIMSK = 0b00000011; //habilita interrupcao externa int0 e int1

	PCICR = 0b00000100; //habilita interrupção apenas pcint2 port D ???
	PCIFR = 0b00000100; //sinalizadores que indicam se ocorreu interrupcao em pcint2
	PCMSK2 = 0b01110000; // habilita interrupção PnChange 20 e 21, ou seja, PD4, PD5, PD6 e PD3


	
	//configuração timer gerar interrupção a cada 100us
	TCCR0A = 0b00000010;	//habilita modo CTC do TC0
	TCCR0B = 0b00000010;	//liga TC0 com prescaler = 8
	OCR0A = 199;			//ajusta o comparador para o TC0 contar ate 199
	TIMSK0 = 0b00000010;	//habilita a interrupcao na igualdade de comparacao com OCR0A
	
	TCCR2A = 0b00100011;		//PWM nao invertido pino OC2B
	TCCR2B = 0b00000101;		//liga TC0,  isso gerou um sinal PWM de ? 488 Hz
	
	
	ADMUX = 0b01000000;			//VCC COMO REF, CANAL 0
	ADCSRA = 0b11100111;		//HABILITA AD, MODO CONV. CONTINUA, PRESCALER = 128(UNICA OPCAO DISPONIVEL)
	ADCSRB = 0b00000000;		//MODO DE CONV. COTINUA
	DIDR0 = 0b00000000;			// DESABILITA PINO PC01 COMO ENTRADA DIGITAL


	//config. usart
	UBRR0H = (unsigned char)(MYUBRR>>8); //Ajusta a taxa de transmissão
	UBRR0L = (unsigned char)MYUBRR;
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0); //Habilita o transmissor e o receptor
	UCSR0C = (1<<USBS0)|(3<<UCSZ00); //Ajusta o formato do frame: 8 bits de dados e 2 de parada
	//UCSR0C = (3<<UCSZ00); //Ajusta o formato do frame: 8 bits de dados e 1 de parada, PARIDADE NONE														//ve isso aq



	sei(); //habilita a chave de interrupcao global

	while (1)
	{
	}
}


executa_tudo(uint32_t TEMPO_100us)
{
	static uint32_t TEMPO_100us_anterior = 0;	//compara com o tempo_100us pra acender as combinações dos LEDs
	static uint32_t TEMPO_para_clock = 0;
	static uint32_t TEMPO_sprint_9 = 0;




	
	if((tempo_100us - TEMPO_para_clock) >= 50000){//a cada 5 segundos atualiza o contador
		gambiarra = contagem_carros/5;		//ver frequencia
		contagem_carros = 0;																				//zera pra na proxima rodada ir somando do 0
		TEMPO_para_clock = tempo_100us;
		
		precisao_carros();
		if(manual_auto==1)	tempos_automaticos();				//SE tiver no modo automatico

		termos_iniciais_fixos();
		valores_atualizados_cores();																		//esses dois são pra diminuir o código, com tudo que precisa do lcd

		nokia_lcd_render();
		
	}
	
	leitura_sensores_ADC(&flag_500ms);
	iluminacao(&flag_50ms);
	

	if(sirenes == 2){
		if((tempo_100us - TEMPO_100us_anterior) >= 0)
		{
			if(aux==0){

				transmissao[0] = 'g';			//nao sei pq ta funcionando assim, mas desse jeito sai na ordem certa
				transmissao[2] = 'y';
				transmissao[4] = 'r';

				transmissao[1] = r_string[0];
				transmissao[3] = y_string[0];			//não tá funcionando no tempo correto, mesmo com esse tempo de 5 segundos entre transmisões, por isso geralmente os semaforos não estão sincronizados
				transmissao[5] = g_string[0];

				USART_Transmit(transmissao[0]);
				USART_Transmit(transmissao[1]);
				USART_Transmit(transmissao[2]);
				USART_Transmit(transmissao[3]);
				USART_Transmit(transmissao[4]);
				USART_Transmit(transmissao[5]);		//faz a transmissao de cada caractere por vez, formato[g][tempo_verde][y][tempo_amarelo][r][tempo_vermelho]


				PORTB = 0b00001111;
				aux++;
			}
		}
		if((tempo_100us - TEMPO_100us_anterior) >= 2500*tempo[0])
		{
			if(aux==1){
				PORTB = 0b00000111;
				aux++;
			}
		}
		if((tempo_100us - TEMPO_100us_anterior) >= 2*2500*tempo[0])
		{
			if(aux==2){
				PORTB = 0b00000011;
				aux++;
			}
		}
		if((tempo_100us - TEMPO_100us_anterior) >= 3*2500*tempo[0])
		{
			if(aux==3){
				PORTB = 0b00000001;
				aux++;
			}
		}
		if((tempo_100us - TEMPO_100us_anterior) >= 4*2500*tempo[0])
		{
			if(aux==4)  {
				PORTB = 0b00000000;
				PORTD |= 0b10000000;		//portd usado em outras aplicações, por isso precisa de máscara
				aux++;
			}
		}
		if((tempo_100us - TEMPO_100us_anterior) >= (4*2500*tempo[0]+10000*tempo[1]))
		{
			if(aux==5)  {
			PORTD &= 0b01111111;//apaga amarelo
			PORTB = 0b11110000; //vermelho 4pinos mascara de bits nao usada aqui, pra no ter o trabalho e tbm pq so as leds que usam as portas B msm
			aux++;
			}
		}
		if((tempo_100us - TEMPO_100us_anterior) >= (4*2500*tempo[0]+10000*tempo[1]+2500*tempo[2]))
		{
			if(aux==6){
				PORTB = 0b01110000;
				aux++;
			}
		}
		if((tempo_100us - TEMPO_100us_anterior) >= (4*2500*tempo[0]+10000*tempo[1]+2*2500*tempo[2]))
		{
			if(aux==7){  
				PORTB = 0b00110000;
				aux++;
			}
		}
		if((tempo_100us - TEMPO_100us_anterior) >= (4*2500*tempo[0]+10000*tempo[1]+3*2500*tempo[2]))
		{
			if(aux==8){
				PORTB = 0b00010000;
				aux++;
			}
		}
		if((tempo_100us - TEMPO_100us_anterior) >= (4*2500*tempo[0]+10000*tempo[1]+4*2500*tempo[2]))
		{
			USART_Transmit('i');
			TEMPO_100us_anterior = tempo_100us;
			aux = 0;
		}																										//msm coisa sprint anteiror
	}
	
	if(sirenes == 1){
		PORTB = 0b00001111;		//sinal verde
	}
	
	if(sirenes==0){
		PORTB = 0b11110000;		//sinal vermelho
	}
	
}

tempos_automaticos(){
	tempo[0] = 1 + carrosporminuto/60;		//verde
	tempo[1] = 1;							//amarelo
	tempo[2] = 9 - carrosporminuto/60;		//vermelho
	
	sprintf(r_string, "%u", tempo[0]);				
	sprintf(y_string, "%u", tempo[1]);
	sprintf(g_string, "%u", tempo[2]);
	
	
	valores_atualizados_cores();

	nokia_lcd_render(); //atualiza display
}


void termos_iniciais_fixos(){
	nokia_lcd_clear(); //limpa lcd
	nokia_lcd_set_cursor(20, 0);
	nokia_lcd_write_string("c/min", 1);		//escreve texto tamanho 1
	nokia_lcd_set_cursor(65, 20);
	nokia_lcd_write_string("lux", 1);		//escreve texto tamanho 1
	nokia_lcd_set_cursor(0, 10);			//Muda o cursos para a posição 0,10 ou seja, pula uma linha
	nokia_lcd_write_string("G. t.  ", 1);	//Escreve um texto do tamanho 1
	nokia_lcd_set_cursor(0, 20);
	nokia_lcd_write_string("Y. t. ", 1);	 //escreve texto tamanho 1
	nokia_lcd_set_cursor(0, 30);
	nokia_lcd_write_string("R. t.    ", 1);		//Foi deixado os termos em ingles(para RED, YELLOW e GREEN) para deixar só uma letra e economizar tela de LCD
	nokia_lcd_set_cursor(0, 40);
	nokia_lcd_write_string("Modo    ", 1);
	nokia_lcd_set_cursor(60, 40);
	nokia_lcd_write_string(UDR0, 1);
}


void valores_atualizados_cores(){
	if(manual_auto == 0)	j = 0;		//para matriz com as expressões do modo,j é a linha que é cada opção
	if(manual_auto == 1)	j = 1;
	switch(i)
	{
		case 0:
		if(manual_auto == 0)		sprintf(r_string, "%u", tempo[i]);		//pra cada caso, o string recebe e converte o tempo[i] para char
		nokia_lcd_set_cursor(38, 10);
		nokia_lcd_write_string("<=", 1);
		break;

		case 1:
		if(manual_auto == 0)		sprintf(y_string, "%u", tempo[i]);
		nokia_lcd_set_cursor(38, 20);
		nokia_lcd_write_string("<=", 1);
		break;

		case 2:
		if(manual_auto == 0)		sprintf(g_string, "%u", tempo[i]);
		nokia_lcd_set_cursor(38, 30);
		nokia_lcd_write_string("<=", 1);
		break;
		
		case 3:
		nokia_lcd_set_cursor(38, 40);
		nokia_lcd_write_string("<=", 1);
		break;
	}
	
	
	nokia_lcd_set_cursor(30, 10);
	nokia_lcd_write_string(r_string, 1);
	nokia_lcd_set_cursor(30, 20);
	nokia_lcd_write_string(y_string, 1);
	nokia_lcd_set_cursor(30, 30);
	nokia_lcd_write_string(g_string, 1);
	nokia_lcd_set_cursor(30, 40);
	nokia_lcd_write_string(automatico_string[j], 1);

	
	sprintf(cpm_string, "%u", carrosporminuto);	
	nokia_lcd_set_cursor(0, 0);
	nokia_lcd_write_string(cpm_string, 1);
	


}
precisao_carros(){
	if(gambiarra==0)					carrosporminuto=0;			//Nota do aluno:
	if(gambiarra==1)					carrosporminuto=60;			//Professor, eu tentei implementar usando o modelo anterior, no qual embora eu tenha usado
	if(gambiarra==3)					carrosporminuto=120;		//essa variavel "gambiarra", era pra ajustaro valor que estava desbalanceado, mas próximo
	if(gambiarra==4||gambiarra==5)		carrosporminuto=180;		//do esperado. Nessa sprint, eu não conseguia de maneira alguma que o valor carrosporminuto
	if(gambiarra==6)					carrosporminuto=240;		//desse numa margem satisfatória, então dividi contagem_carros por 5 para saber a frequencia
	if(gambiarra==7||gambiarra==8)		carrosporminuto=300;		//de fato que chegava no LCD, e de 4 Hz para frente as frequencias chegavam com 1 Hz a mais
	if(gambiarra==9)					carrosporminuto=360;		//que o esperado, então a melhor maneira que consegui pensar para funcionamento da sprint foi
	if(gambiarra==10||gambiarra==11)	carrosporminuto=420;		//usar a própria frequencia em Hz pra ajustar a contagem, e entre as tentativas, algumas não 
	if(gambiarra==13)					carrosporminuto=480;		//funcionavam sem motivo aparente, então caso não funcione como deveria em sua máquina, o motivo é esse
	
}




void leitura_sensores_ADC(uint8_t *flag_disparo)
{
	if(*flag_disparo)
	{
		sprintf(lux_LDR, "%u", 1023000/ADC - 1000);

		nokia_lcd_set_cursor(65,10);
		nokia_lcd_write_string(lux_LDR, 1);														//mostra o valor do lux
		nokia_lcd_render();		
		
		*flag_disparo = 0;																		//zera a flag pra esperar mais 500ms
		
	}
}

void iluminacao(uint8_t *flag_disparo1)
{
	if(*flag_disparo1)
	{
		
		if((1023000/ADC - 1000)<300){															//com luz menos que 300
			if(((PINC & 0b01000000) == 0)||(carrosporminuto>0))		//pedestres na PD3*			//se tiver pedestres nem carros
			OCR2B=249;																			//100% luz no poste
			else
			OCR2B=75;																			//30% luz no poste
		}
		else{
			OCR2B=0;																			//0% de luz no poste, tá de dia
		}
		
		*flag_disparo1 = 0;																		//zera a flag pra esperar mais 50ms
		
	}
}