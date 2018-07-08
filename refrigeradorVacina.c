/*******************************************************
This program was created by the
CodeWizardAVR V3.12 Advanced
Automatic Program Generator
© Copyright 1998-2014 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : 
Version : 
Date    : 30/06/2018
Author  : 
Company : 
Comments: 


Chip type               : ATmega16
Program type            : Application
AVR Core Clock frequency: 14.745600 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 256
*******************************************************/

#include <mega16.h>

#include <delay.h>
#include <stdio.h>

// Alphanumeric LCD functions
#include <alcd.h>

// 1 Wire Bus interface functions
#include <1wire.h>

// DS1820 Temperature Sensor functions
#include <ds1820.h>

// Declare your global variables here

#define DATA_REGISTER_EMPTY (1<<UDRE)
#define RX_COMPLETE (1<<RXC)
#define FRAMING_ERROR (1<<FE)
#define PARITY_ERROR (1<<UPE)
#define DATA_OVERRUN (1<<DOR)

//Variáveis de Controle

#define BUTTONL PIND.4
#define BUTTONR PIND.2
#define BUTTONC PIND.3
#define BACKLIGHT PORTB.3


int menuOption = 'i';
float systemTemperature;
float maximumTemp = 18.3;
unsigned char x = 0;
unsigned char y = 0;
char strbuff[16];
int phoneNumber[12] = {0,3,2,9,9,9,6,7,4,7,3,2};
int cursorPosition = 0;
int sentMessage = 0;
//char menuOrder[5] = {'i','t','n','m','r','e'};

eeprom char stateChange = 0;

eeprom char triggered;


int temp;

// USART Receiver buffer
#define RX_BUFFER_SIZE 8
char rx_buffer[RX_BUFFER_SIZE];

#if RX_BUFFER_SIZE <= 256
unsigned char rx_wr_index=0,rx_rd_index=0;
#else
unsigned int rx_wr_index=0,rx_rd_index=0;
#endif

#if RX_BUFFER_SIZE < 256
unsigned char rx_counter=0;
#else
unsigned int rx_counter=0;
#endif

// This flag is set on USART Receiver buffer overflow
bit rx_buffer_overflow;

// USART Receiver interrupt service routine
interrupt [USART_RXC] void usart_rx_isr(void)
{
	char status,data;
	status=UCSRA;
	data=UDR;
	if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
	{
		rx_buffer[rx_wr_index++]=data;
#if RX_BUFFER_SIZE == 256
		// special case for receiver buffer size=256
		if (++rx_counter == 0) rx_buffer_overflow=1;
#else
		if (rx_wr_index == RX_BUFFER_SIZE) rx_wr_index=0;
		if (++rx_counter == RX_BUFFER_SIZE)
		{
			rx_counter=0;
			rx_buffer_overflow=1;
		}
#endif
	}
}

// Timer 0 overflow interrupt service routine
interrupt [TIM0_OVF] void timer0_ovf_isr(void)
{
	// Place your code here

}

#ifndef _DEBUG_TERMINAL_IO_
// Get a character from the USART Receiver buffer
#define _ALTERNATE_GETCHAR_
#pragma used+
char getchar(void)
{
	char data;
	while (rx_counter==0);
	data=rx_buffer[rx_rd_index++];
#if RX_BUFFER_SIZE != 256
	if (rx_rd_index == RX_BUFFER_SIZE) rx_rd_index=0;
#endif
#asm("cli")
	--rx_counter;
#asm("sei")
	return data;
}
#pragma used-
#endif

// Standard Input/Output functions
#include <stdio.h>

// Voltage Reference: AVCC pin
#define ADC_VREF_TYPE ((0<<REFS1) | (1<<REFS0) | (0<<ADLAR))

// Read the AD conversion result
unsigned int read_adc(unsigned char adc_input)
{
	ADMUX=adc_input | ADC_VREF_TYPE;
	// Delay needed for the stabilization of the ADC input voltage
	delay_us(10);
	// Start the AD conversion
	ADCSRA|=(1<<ADSC);
	// Wait for the AD conversion to complete
	while ((ADCSRA & (1<<ADIF))==0);
	ADCSRA|=(1<<ADIF);
	return ADCW;
}




float readTemp();

int readButtons();

void sendMessage();

void showMenu();

void showControls(int type);

void menuHome();

void menuSetTemp();

void menuSetNumber();

void menuTestMessage();

void menuExit();

void menuSetTempChange();

void menuSetNumberChange();

void menuSendTxtMsg();

void menuAcknowledgeTrigger();

void menuAcknowledgeTriggerChange();

void sendSMS (char type);



void main(void)
{
	// Declare your local variables here

	// Input/Output Ports initialization
	// Port A initialization
	// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
	DDRA=(0<<DDA7) | (0<<DDA6) | (0<<DDA5) | (0<<DDA4) | (0<<DDA3) | (0<<DDA2) | (0<<DDA1) | (0<<DDA0);
	// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
	PORTA=(0<<PORTA7) | (0<<PORTA6) | (0<<PORTA5) | (0<<PORTA4) | (0<<PORTA3) | (0<<PORTA2) | (0<<PORTA1) | (0<<PORTA0);

	// Port B initialization
	// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
	DDRB=(0<<DDB7) | (0<<DDB6) | (0<<DDB5) | (0<<DDB4) | (0<<DDB3) | (0<<DDB2) | (0<<DDB1) | (0<<DDB0);
	// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
	PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (1<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

	// Port C initialization
	// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
	DDRC=(0<<DDC7) | (0<<DDC6) | (0<<DDC5) | (0<<DDC4) | (0<<DDC3) | (0<<DDC2) | (0<<DDC1) | (0<<DDC0);
	// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
	PORTC=(0<<PORTC7) | (0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

	// Port D initialization
	// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
	DDRD=(0<<DDD7) | (0<<DDD6) | (0<<DDD5) | (0<<DDD4) | (0<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0);
	// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
	PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);

	// Timer/Counter 0 initialization
	// Clock source: System Clock
	// Clock value: Timer 0 Stopped
	// Mode: Normal top=0xFF
	// OC0 output: Disconnected
	TCCR0=(0<<WGM00) | (0<<COM01) | (0<<COM00) | (0<<WGM01) | (0<<CS02) | (0<<CS01) | (0<<CS00);
	TCNT0=0x00;
	OCR0=0x00;

	// Timer/Counter 1 initialization
	// Clock source: System Clock
	// Clock value: Timer1 Stopped
	// Mode: Normal top=0xFFFF
	// OC1A output: Disconnected
	// OC1B output: Disconnected
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// Timer1 Overflow Interrupt: Off
	// Input Capture Interrupt: Off
	// Compare A Match Interrupt: Off
	// Compare B Match Interrupt: Off
	TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
	TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (0<<CS10);
	TCNT1H=0x00;
	TCNT1L=0x00;
	ICR1H=0x00;
	ICR1L=0x00;
	OCR1AH=0x00;
	OCR1AL=0x00;
	OCR1BH=0x00;
	OCR1BL=0x00;

	// Timer/Counter 2 initialization
	// Clock source: System Clock
	// Clock value: Timer2 Stopped
	// Mode: Normal top=0xFF
	// OC2 output: Disconnected
	ASSR=0<<AS2;
	TCCR2=(0<<PWM2) | (0<<COM21) | (0<<COM20) | (0<<CTC2) | (0<<CS22) | (0<<CS21) | (0<<CS20);
	TCNT2=0x00;
	OCR2=0x00;

	// Timer(s)/Counter(s) Interrupt(s) initialization
	TIMSK=(0<<OCIE2) | (0<<TOIE2) | (0<<TICIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (0<<TOIE1) | (0<<OCIE0) | (0<<TOIE0);

	// External Interrupt(s) initialization
	// INT0: Off
	// INT1: Off
	// INT2: Off
	MCUCR=(0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
	MCUCSR=(0<<ISC2);

	// USART initialization
	// Communication Parameters: 8 Data, 1 Stop, No Parity
	// USART Receiver: On
	// USART Transmitter: On
	// USART Mode: Asynchronous
	// USART Baud Rate: 19200
	UCSRA=(0<<RXC) | (0<<TXC) | (0<<UDRE) | (0<<FE) | (0<<DOR) | (0<<UPE) | (0<<U2X) | (0<<MPCM);
	UCSRB=(1<<RXCIE) | (0<<TXCIE) | (0<<UDRIE) | (1<<RXEN) | (1<<TXEN) | (0<<UCSZ2) | (0<<RXB8) | (0<<TXB8);
	UCSRC=(1<<URSEL) | (0<<UMSEL) | (0<<UPM1) | (0<<UPM0) | (0<<USBS) | (1<<UCSZ1) | (1<<UCSZ0) | (0<<UCPOL);
	UBRRH=0x00;
	UBRRL=0x2F;

	// Analog Comparator initialization
	// Analog Comparator: Off
	// The Analog Comparator's positive input is
	// connected to the AIN0 pin
	// The Analog Comparator's negative input is
	// connected to the AIN1 pin
	ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);

	// ADC initialization
	// ADC Clock frequency: 921.600 kHz
	// ADC Voltage Reference: AVCC pin
	// ADC Auto Trigger Source: ADC Stopped
	ADMUX=ADC_VREF_TYPE;
	ADCSRA=(1<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);
	SFIOR=(0<<ADTS2) | (0<<ADTS1) | (0<<ADTS0);

	// SPI initialization
	// SPI disabled
	SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);

	// TWI initialization
	// TWI disabled
	TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);

	// Alphanumeric LCD initialization
	// Connections are specified in the
	// Project|Configure|C Compiler|Libraries|Alphanumeric LCD menu:
	// RS - PORTB Bit 0
	// RD - PORTB Bit 1
	// EN - PORTB Bit 2
	// D4 - PORTB Bit 4
	// D5 - PORTB Bit 5
	// D6 - PORTB Bit 6
	// D7 - PORTB Bit 7
	// Characters/line: 16    
    
	lcd_init(16);
	BACKLIGHT = 1;  
    
    //
//// Timer/Counter 0 initialization
//// Clock source: System Clock
//// Clock value: 1843.200 kHz
//// Mode: Normal top=0xFF
//// OC0 output: Disconnected
//// Timer Period: 0.13889 ms
//TCCR0=(0<<WGM00) | (0<<COM01) | (0<<COM00) | (0<<WGM01) | (0<<CS02) | (1<<CS01) | (0<<CS00);
//TCNT0=0x00;
//OCR0=0x00;

// 1 Wire Bus initialization
// 1 Wire Data port: PORTD
// 1 Wire Data bit: 7
// Note: 1 Wire port settings are specified in the
// Project|Configure|C Compiler|Libraries|1 Wire menu.
    w1_init();
    

	// Global enable interrupts
#asm("sei")

	while (1)
	{
		
		        
		showMenu();
		delay_ms(200);
		
		if(menuOption == 'i')
		systemTemperature = readTemp();
	
        if(stateChange){
            lcd_clear();
            stateChange = 0;
            
            
        }
		
		if((systemTemperature >= maximumTemp) && (triggered == 0)){
			triggered = 1;
			if(!sentMessage){
				sendSMS('e');
				sentMessage = 1;
			}
			
			
		}
		
		
		

    }
}


float readTemp(){
    
    // te vira depois pra fazer essa merda ler a temperatura e vá fazendo o resto do código
    float temperature;
    
    temp=ds1820_temperature_10(0);
    
    temperature = (float) temp/100;
    
    
    
    return temperature;
}

int readButtons(){
    
    if(~BUTTONL){
        stateChange=1;
        return 'l';
    }
    
    if(~BUTTONR){
        stateChange=1;
        return 'r';
    }
    
    if(~BUTTONC){
        stateChange=1;
        return 'c';
    }
    
}

void sendMessage(){
    
}

void showMenu(){

    switch (menuOption){

        //aqui começa o primeir nível do menu
    case 'i':
        
        menuHome();
        showControls(4);
        
        if(readButtons() == 'c')
        menuOption = 't';
        
        break;
        
    case 't':
        menuSetTemp();
        showControls(0);
        
        if(readButtons() == 'c')
        menuOption = 'u';
        if(readButtons() == 'l')
        menuOption = 'e';
        if(readButtons() == 'r')
        menuOption = 'n';
        
        break;
        
    case 'n':
        menuSetNumber();
        showControls(0);
        
        if(readButtons() == 'c')
        menuOption = 'o';
        if(readButtons() == 'l')
        menuOption = 't';
        if(readButtons() == 'r')
        menuOption = 'm';
        
        break;
        
    case 'm':
        menuTestMessage();
        showControls(0);
        
        if(readButtons() == 'c')
        menuOption = 'p';
        if(readButtons() == 'l')
        menuOption = 'n';
        if(readButtons() == 'r')
        menuOption = 'r';
        
        break;
		
	case 'r':
		menuAcknowledgeTrigger();
        showControls(0);
        
        if(readButtons() == 'c')
        menuOption = 's';
        if(readButtons() == 'l')
        menuOption = 'm';
        if(readButtons() == 'r')
        menuOption = 'e';
        
        break;	
    
    case 'e':
        menuExit();
        showControls(0);
        
        if(readButtons() == 'c')
        menuOption = 'i';
        if(readButtons() == 'l')
        menuOption = 'r';
        if(readButtons() == 'r')
        menuOption = 't';
        
        break;
        
        //aqui começa o segundo nível dos menus
        
    case 'u':
        menuSetTempChange();
        showControls(1);
        break;
        
    case 'o':
        menuSetNumberChange();
        showControls(5);
        break;
        
    case 'p':
        menuSendTxtMsg();
        showControls(3);
		break;
	
	case 's':
		menuAcknowledgeTriggerChange();
		if(triggered)
        showControls(3);
		else
		showControls(2);
	
		break;
    }
}

void showControls(int type){
    
    switch(type){
        
    case 0:
        sprintf(strbuff, "  <<   ==   >>");
        x=0;
        y=1;
        lcd_gotoxy(x,y);
        lcd_puts(strbuff);
        break;
        
    case 1:
        sprintf(strbuff, "  ^^   ==   vv");
        x=0;
        y=1;
        lcd_gotoxy(x,y);
        lcd_puts(strbuff);
        break;
        
    case 2:
        sprintf(strbuff, "       ==");
        x=0;
        y=1;
        lcd_gotoxy(x,y);
        lcd_puts(strbuff);
        break;
        
    case 3:
        sprintf(strbuff, " Yes        No");
        x=0;
        y=1;
        lcd_gotoxy(x,y);
        lcd_puts(strbuff);
        break;
		
	case 4:
        sprintf(strbuff, "      menu");
        x=0;
        y=1;
        lcd_gotoxy(x,y);
        lcd_puts(strbuff);
        break;
		
	case 5:
        sprintf(strbuff, "  ^^   ==   next");
        x=0;
        y=1;
        lcd_gotoxy(x,y);
        lcd_puts(strbuff);
        break;
    }
    
    
}

void menuHome(){
    sprintf(strbuff, "%.1fC", systemTemperature);
    x=1;
    y=0;
    lcd_gotoxy(x,y);
    lcd_puts(strbuff);
    
    if(!triggered){
        sprintf(strbuff, "NORMAL");
        x=10;
        y=0;
        lcd_gotoxy(x,y);
        lcd_puts(strbuff);
    }
    else{
        sprintf(strbuff, "*VOID* ");
        x=9;
        y=0;
        lcd_gotoxy(x,y);
        lcd_puts(strbuff);
    }

}

void menuSetTemp(){
    sprintf(strbuff, "    MAX TEMP");
    x=0;
    y=0;
    lcd_gotoxy(x,y);
    lcd_puts(strbuff);
}

void menuSetNumber(){
    sprintf(strbuff, "   SET NUMBER");
    x=0;
    y=0;
    lcd_gotoxy(x,y);
    lcd_puts(strbuff);
    
}

void menuTestMessage(){
    sprintf(strbuff, "  TEST MESSAGE");
    x=0;
    y=0;
    lcd_gotoxy(x,y);
    lcd_puts(strbuff);
    
}

void menuExit(){
    sprintf(strbuff, "      EXIT");
    x=0;
    y=0;
    lcd_gotoxy(x,y);
    lcd_puts(strbuff);
    
}

void menuSetTempChange(){
	
	float localMaxTemp;
	
	localMaxTemp = maximumTemp;
    sprintf(strbuff, " Maximum: %.1fC", localMaxTemp);
    x=0;
    y=0;
    lcd_gotoxy(x,y);
    lcd_puts(strbuff);
    
    if(readButtons() == 'c')
    menuOption = 't';
    if(readButtons() == 'l')
    maximumTemp-=0.2;
    if(readButtons() == 'r')
    maximumTemp+=0.2;
    
    
}

void menuSetNumberChange(){
    
    lcd_gotoxy(0,0);
    lcd_putchar(phoneNumber[0] + 48);
    lcd_gotoxy(1,0);
    lcd_putchar(phoneNumber[1] + 48);
    lcd_gotoxy(2,0);
    lcd_putchar(phoneNumber[2] + 48);
    lcd_gotoxy(3,0);
    lcd_putchar(phoneNumber[3] + 48);
    lcd_gotoxy(4,0);
    lcd_putchar(phoneNumber[4] + 48);
    lcd_gotoxy(5,0);
    lcd_putchar(phoneNumber[5] + 48);
    lcd_gotoxy(6,0);
    lcd_putchar(phoneNumber[6] + 48);
    lcd_gotoxy(7,0);
    lcd_putchar(phoneNumber[7] + 48);
    lcd_gotoxy(8,0);
    lcd_putchar(phoneNumber[8] + 48);
    lcd_gotoxy(9,0);
    lcd_putchar(phoneNumber[9] + 48);
    lcd_gotoxy(10,0);
    lcd_putchar(phoneNumber[10] + 48);
    lcd_gotoxy(11,0);
    lcd_putchar(phoneNumber[11] + 48);
	
	if(readButtons() == 'c')
    menuOption = 'n';
    if(readButtons() == 'l')
    phoneNumber[cursorPosition]++;;
    if(readButtons() == 'r')
    cursorPosition++;

	if (cursorPosition == 12)
	cursorPosition = 0;
	
	if (phoneNumber[cursorPosition]>9)
    phoneNumber[cursorPosition] = 0;
}

void menuSendTxtMsg(){
    sprintf(strbuff, "SEND A TEST MSG?");
    x=0;
    y=0;
    lcd_gotoxy(x,y);
    lcd_puts(strbuff);
    
    if(readButtons() == 'l'){
        sendSMS('t');
        menuOption = 'm';
		
        
    }
    if(readButtons() == 'r')
		menuOption = 'm';
    
}

void menuAcknowledgeTrigger(){
	
	sprintf(strbuff, "   RESET VOID   ");
    x=0;
    y=0;
    lcd_gotoxy(x,y);
    lcd_puts(strbuff);
	
	
}

void menuAcknowledgeTriggerChange(){
	
	if(triggered){
	sprintf(strbuff, "RESET VOID");
	
	if(readButtons() == 'l'){
		triggered = 0;
		sentMessage = 0;
        menuOption = 'm';
	}
        
    
    if(readButtons() == 'r')
		menuOption = 'm';
	
    }
	else{
	sprintf(strbuff, "   NOT VOIDED   ");
	if(readButtons() == 'c'){
        menuOption = 'r';	
	}
	}
	x=0;
    y=0;
    lcd_gotoxy(x,y);
    lcd_puts(strbuff);
	
    if(readButtons() == 'l')
        menuOption = 'r';
        
    
    if(readButtons() == 'r')
		menuOption = 'r';
	
}

void menuTextConfirmation(){
	
	sprintf(strbuff, "RECEIVED MY MSG?");
    x=0;
    y=0;
    lcd_gotoxy(x,y);
    lcd_puts(strbuff);
	
	
    if(readButtons() == 'l')
    menuOption = 'n';

    if(readButtons() == 'r')
    menuOption = 'o';
	
}

void sendSMS (char type){
    char phoneNumberASCII [11];
    int i =0;
    printf("%c", type);
//    for(i=0;i<12;i++){
//    phoneNumberASCII[i] = phoneNumber[i] + 48;
//    }
    
    
	
	printf("AT+CMGF=1");
	printf("AT+CMGS=\"");
    
    putchar(phoneNumber[0] +48);
    putchar(phoneNumber[1] +48);
    putchar(phoneNumber[2] +48);
    putchar(phoneNumber[3] +48);
    putchar(phoneNumber[4] +48);
    putchar(phoneNumber[5] +48);
    putchar(phoneNumber[6] +48);
    putchar(phoneNumber[7] +48);
    putchar(phoneNumber[8] +48);
    putchar(phoneNumber[9] +48);
    putchar(phoneNumber[10] +48);
    putchar(phoneNumber[11] +48);
	printf("\"");
    
    
    if(type == 't'){
	printf("Hey, let me know you received this!");
	printf("%c", 26);
    }
    
    if(type == 'e'){
	printf("Your microfreezer temperature is above the max level set");
	printf("%c", 26);
    }
    
}