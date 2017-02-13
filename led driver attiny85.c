#pragma disablecontexsaving
#define SYS_CLK 8000000L

#define BAT_VOLTAGE_LOW 112530L/310L
#define BAT_VOLTAGE_OFF 112530L/290L

#define CURRENT_HI 930       // 200ma
#define CURRENT_MID 300       // 100ma
#define CURRENT_LOW 80        // 20ma

#define STATUS_WORK 0
#define STATUS_WORK_PRESS_KEY 1
#define STATUS_PW_OFF 2
#define STATUS_PW_ON 3

#define ADC_VOLTAGE 0x0C
#define ADC_CURENT 0xA7
#define ADC_TEMP 0x8F
#define ADC_OFFSET 0x85

#define CUR_MODE_LOW 0
#define CUR_MODE_MID 1
#define CUR_MODE_HI  2
#define MAX_MODE 2

//sigma delta 16bit pwm
   rx sfr unsigned char zero = 0          absolute 0x08;
   rx sfr unsigned char pwmError = 0      absolute 0x09;
   rx sfr unsigned char pwmLo = 0         absolute 0x0A;
   rx sfr unsigned char pwmHi = 0         absolute 0x0B;
   rx sfr unsigned int  pwm = 0           absolute 0x0A;
  
   volatile int  irefDiff = 0;
   volatile char irefDiffMod = 0;
   volatile char irefMod = 0;

// led class
   volatile unsigned char ledMode;
   volatile unsigned char moonlightOn = 0;
   volatile unsigned char enableCurrentControl = 0;
   volatile unsigned int ledCurrentSet = 0;
   volatile unsigned int current = 0;     // Curent      0xxx xxxx xxx0 0000
   volatile unsigned int iOffSet = 0;    // IOffSet     0xxx xxxx xxx0 0000

// timer0
   volatile unsigned int timer = 0;
   volatile unsigned int timerVoltageLow = 0;
   volatile unsigned int timerVoltageOff = 0;
   volatile unsigned char period = 0;
   volatile unsigned int countSamples = 501;
   volatile unsigned int  batVoltage = 0;
   volatile unsigned char batVoltageReady = 0;
   
// main
   unsigned char status;
   unsigned int voltage = 0;
   bit block;
 
// button 
    unsigned char IsKeyDown(void){
          if(PINB&0x01) return 0;
          return 1;
    }

    void ClearTimer(void){
          asm{CLI}
          timer = 0;
          asm{SEI}
    }

    unsigned int GetTimer(void){
        unsigned int t;
          asm{CLI}
          t = timer;
          asm{SEI}
        return t;
    }
    
    void ClearTimerVoltageLow(void){
        asm{CLI}
        timerVoltageLow = 0;
        asm{SEI}
    }

    unsigned int GetTimerVoltageLow(void){
        unsigned int t;
          asm{CLI}
          t = timerVoltageLow;
          asm{SEI}
        return t;
    }
    
    void ClearTimerVoltageOff(void){
        asm{CLI}
        timerVoltageOff = 0;
        asm{SEI}
    }

    unsigned int GetTimerVoltageOff(void){
        unsigned int t;
          asm{CLI}
          t = timerVoltageOff;
          asm{SEI}
        return t;
    }

// Setup
     void Timer0Start(void){
           TCCR0A = 0x00; //stop
           TCNT0  = 0x00; //set count
           OCR0A  = SYS_CLK/64/1000;   //1khz
           TCCR0B = 0x03; // presacaler 64
           TIMSK = 0x14;
           TCCR0A = 0x02;
     }

    void Timer1Start(void){       // SYS_CLK/256
          PLLCSR=0x00;
          OCR1C = 255;
          OCR1A = 0x00;
          TIMSK = 0x14;
          TCCR1 = 0x61;
          GTCCR = 0x00;
    }

    void AdcFirstStart(void){
          ACSR  = 0x80;   // disable comparator
          ADCSRB = 0x00;
          
          ADCSRA = 0x00;
          Delay_ms(1);
          ADMUX = ADC_OFFSET;
          ADCSRA = 0x87; // prescaler 128
          Delay_ms(10);

          IOffSet = 0;
          ADCSRA |= (1<<ADSC);
          while(ADCSRA & (1<<ADSC));
          IOffSet = ADCL;
          IOffSet = IOffSet  + (ADCH<<8);

          Delay_ms(1);

          IOffSet = 0;
          ADCSRA |= (1<<ADSC);
          while(ADCSRA & (1<<ADSC));
          IOffSet = ADCL;
          IOffSet = IOffSet  + (ADCH<<8);

          IOffSet = (IOffSet << 5) & 0x7FE0;   // IOffSet 0xxx xxxx xxx0 0000


          ADMUX = ADC_CURENT;
          ADCSRA = 0x87; // prescaler 128
          Delay_ms(10);
          ADCSRA |= (1<<ADSC);
    }

    void InitHard(void){
          CLKPR=0x80;
          CLKPR=0x00;  // CPU prescaler 1
          MCUCR=0x00;
          PRR=1<<PRUSI;  //disable USI
          TIMSK=0x00; //timer interrupt sources
          GIMSK=0x20; //interrupt sources
          DIDR0  |= (1<<ADC2D)| (1<<ADC3D);   // disable digital input
          PORTB = 0x01;
          DDRB  = 0x06;
    }
// end Setup


// Led class

    void LedOn(void){
         asm CLI
             enableCurrentControl = 1;
             pwm = 0;
             OCR1A = 0;
         asm SEI
    }
    
    void LedOffFast(void){
         asm CLI
             enableCurrentControl = 0;
             pwm = 0;
             OCR1A = 0;
         asm SEI
    }
    
    void LedOffSlow(void){    // slow time!
         asm CLI
             enableCurrentControl = 0;
         asm SEI
         pwmLo = 0;
         while (pwmHi > 0){
             if (pwmHi < 70) {
                  pwmHi = 0;
             }else{
                  pwmHi = pwmHi - pwmHi/32;
             }
             delay_ms(12);
         }
    }
    
    void LedSetMode(unsigned char mode){
         asm CLI
             ledCurrentSet = CURRENT_LOW;
             if(mode == CUR_MODE_MID) ledCurrentSet = CURRENT_MID;
             else if(mode == CUR_MODE_HI) ledCurrentSet = CURRENT_HI;
              //ledCurrentSet = ((ledCurrentSet << 6) + IOffSet);
             ledCurrentSet = ledCurrentSet * 64;
             
             irefDiff = 0;
             irefDiffMod = 0;
             irefMod = 0;
         asm SEI
    }
    
    
// end Led class

// Interrupts
   void timer1_ovf(void) iv IVT_ADDR_TIMER1_OVF {
        asm{
             in         R7,     SREG
             clr        R8
             add        R9,     R10
             adc        R8,     R11
             out        OCR1A,  R8
             out        SREG,        R7
        }
    }

    void pin_change_int0(void) iv IVT_ADDR_PCINT0{
       asm  PUSH  R27
       asm  in R7, SREG
       PCMSK=0x00;   //disable interrupt on INT0
       asm out SREG,  R7
       asm  POP  R27
    }

    void timer0_compa_isr(void) iv IVT_ADDR_TIMER0_COMPA{    //1kHz
       
       asm {
        in R7, SREG
        
        PUSH       R16
        PUSH       R17
        PUSH       R18
        PUSH       R19
        PUSH       R20
        PUSH       R21
        PUSH       R22
        PUSH       R23
        PUSH       R24
        PUSH       R25
        PUSH       R26
        PUSH       R27

        PUSH       R30
        PUSH       R31
       }
        
       timer++;
       timerVoltageLow++;
       timerVoltageOff++;
       
        if (period == 0) {
        
          asm{
              in   R22,   ADCL
              in   R23,   ADCH
              sts  _current+0,  R22
              sts  _current+1,  R23
          }
           
          if (ADMUX == ADC_CURENT) {
             if (enableCurrentControl == 1) {
                
                asm{
                lds     R22,    _current+0
                lds     R23,    _current+1
                lds     R16,    _ledCurrentSet+0
                lds     R17,    _ledCurrentSet+1
                lds     R18,    _irefDiff+0
                lds     R19,    _irefDiff+1
                lds     R26,    _irefMod+0
                lds     R27,    _irefDiffMod+0
                
                mov        R24,        R10
                mov        R25,        R11

                subi       R17,        128
                subi       R23,        128
                subi       R25,        128

                sub        R16,        R22     // E = ledCurrentSet - current
                sbc        R17,        R23
                brvc       errOk0
                brmi       errPos0
                ldi        R16,        0
                ldi        R17,        128
                rjmp       errOk0
errPos0:
                ldi        R16,        255
                ldi        R17,        127

errOk0:
                mov        R20,        R16
                mov        R21,        R17
                
                mov        R28,        R17
                
                sub        R20,        R18     // X = (E - IrefDiff)/256
                sbc        R21,        R19
                brvc       errOk1
                brmi       errPos1
                ldi        R20,        0
                ldi        R21,        128
                rjmp       difPos
errPos1:
                ldi        R20,        255
                ldi        R21,        127
                rjmp       difPos

errOk1:
                clr        R8
                cpi        R21,        127
                breq       difPos
                cpi        R21,        128
                breq       difPos
                tst        R21
                brmi       difNeg
                add        R27,        R20
                adc        R21,        R0
                rjmp       difPos
difNeg:
                mov        R8,         R20
                ldi        R20,        255
                sub        R20,        R8
                clr        R8
                sub        R27,        R20
                sbc        R21,        R8
difPos:
                clr        R20
                sbrc       R21,        7
                ldi        R20,        255

                add        R18,        R21     // IrefDiff += X (mod)
                adc        R19,        R20
                brvc       errOk2
                brbc       4,          3
                ldi        R18,        0
                ldi        R19,        128
                rjmp       errOk2
errPos2:
                ldi        R18,        255
                ldi        R19,        127

errOk2:
                add        R24,        R21      // pwm += X
                adc        R25,        R20
                brvc       errOk3
                brmi       errPos3
                ldi        R24,        0
                ldi        R25,        128
                rjmp       errOk3
errPos3:
                ldi        R24,        255
                ldi        R25,        127

errOk3:
                clr        R8                   // pwm += E/256 (mod)
                cpi        R17,        127
                breq       errPos
                cpi        R17,        128
                breq       errPos
                tst        R17
                brmi       errNeg
                add        R26,        R16
                adc        R17,        R8
                rjmp       errPos
errNeg:
                mov        R8,         R16
                ldi        R16,        255
                sub        R16,        R8
                clr        R8
                sub        R26,        R16
                sbc        R17,        R8
errPos:
                clr        R16
                sbrc       R17,        7
                ldi        R16,        255

                add        R24,        R17
                adc        R25,        R16
                brvc       outOk
                brmi       outPos
                ldi        R24,        0
                ldi        R25,        128
                rjmp       outOk
outPos:
                ldi        R24,        255
                ldi        R25,        127

outOk:
                tst        R28           // proportional P = [E/32]  -> [-2, 2]
                brmi       point1              // if |E| < 32 -> E = 0;
                lsr        R28
                lsr        R28
                lsr        R28
                
                cpi        R28,  2
                brcs       point2
                ldi        R28,  2
                rjmp       point2
point1:
                lsr        R28
                lsr        R28
                lsr        R28
                
                ori        R28,       0xE0
                cpi        R28,       255
                brne       point3
                clr        R28
                rjmp       point2
point3:
                cpi        R28,  254
                brcc       point2
                ldi        R28,  254
point2:
                add        R25,        R28     // pwm += P   (-2 .. +2)
                brvc       out
                brmi       outPos2
                ldi        R24,        0
                ldi        R25,        128
                rjmp       out
outPos2:
                ldi        R24,        255
                ldi        R25,        127

out:
                subi       R25,        128
                ldi        R16,        0           // max pwm = 255 (100%)
                ldi        R17,        255
                cp         R24,        R16
                cpc        R25,        R17
                brcs       outOk1
                mov        R24,        R16
                mov        R25,        R17
outOk1:
                mov        R10,        R24
                mov        R11,        R25

                sts     _irefDiff+0,     R18
                sts     _irefDiff+1,     R19
                sts     _irefMod+0,      R26
                sts     _irefDiffMod+0,  R27
                }
                
             }
             countSamples++;
             if (countSamples > 500)  ADMUX = ADC_VOLTAGE;
          }
          else{
             batVoltage = current;
             batVoltageReady = 1;
             ADMUX = ADC_CURENT;
             countSamples = 0;
          }
        }

        if (period == 1) ADCSRA |= (1<<ADSC);  

        period = period + 1;
        if (period > 2) period = 0;
        
        asm {
        out SREG,  R7
        
            POP        R31
            POP        R30
            
            POP        R27
            POP        R26
            POP        R25
            POP        R24
            POP        R23
            POP        R22
            POP        R21
            POP        R20
            POP        R19
            POP        R18
            POP        R17
            POP        R16
        }
    }
// end Interrupts


    void PowerOff(void){
         asm CLI
         TCCR0B = 0x00;        // timer0 stop
         TCCR1  = 0x00;        // timer1 stop
         ADCSRA = 0x00;        // adc stop
         LedOffFast();
         PCMSK=0x01;   // interrupt on INT0
         GIFR|=0x60;
         MCUCR=0xB4;   //
         MCUCR=0xB0;
         asm SEI
         //asm sleep
    }

    void PowerOn(void){
         asm CLI
         AdcFirstStart();
         Delay_ms(10);
         Timer0Start();
         Timer1Start();
         Delay_ms(1);

         pwm = 0;
         OCR1A = 0;
         
         irefDiff = 0;
         irefDiffMod = 0;
         irefMod = 0;
         
         timer = 0;
         timerVoltageLow = 0;
         timerVoltageOff = 0;

         countSamples = 0;
         asm SEI
    }



// clear cache
   void ClearRx(void){
        asm{ 
             CLR        R0
             CLR        R1
             CLR        R2
             CLR        R3
             CLR        R4
             CLR        R5
             CLR        R6
             CLR        R7
             CLR        R8
             CLR        R9
             CLR        R10
             CLR        R11
             CLR        R12
             CLR        R13
             CLR        R14
             CLR        R15
             CLR        R16
             CLR        R17
             CLR        R18
             CLR        R19
             CLR        R20
             CLR        R21
             CLR        R22
             CLR        R23
             CLR        R24
             CLR        R25
             CLR        R26
             CLR        R27
             CLR        R28
             CLR        R29
             CLR        R30
             CLR        R31
        }
   }


// main
    void main() {

            asm CLI 
                InitHard();
                delay_ms(10);
                AdcFirstStart();
                delay_ms(10);
                Timer0Start();
                Timer1Start();
                delay_ms(1);
                ledMode = CUR_MODE_LOW;
                LedSetMode(ledMode);
            asm SEI

            block = 0;
            status = STATUS_PW_OFF;
            
            // Button
            while (1) {
                 switch(status) {
                    case STATUS_WORK:
                            if(IsKeyDown()) {
                                    status = STATUS_WORK_PRESS_KEY;
                                    ClearTimer();
                            }
                            else {
                                   if (batVoltageReady == 1) {     // BatMonitor

                                        batVoltageReady = 0;
                                        voltage = batVoltage;
                                        
                                        if (voltage > BAT_VOLTAGE_LOW){
                                             if (getTimerVoltageLow() > 5000)  {
                                                 ClearTimerVoltageLow();
                                                 LedOffFast();
                                                 delay_ms(100);
                                                 LedOn();
                                             }
                                        } else {
                                            ClearTimerVoltageLow();
                                        }
                                        
                                        if (voltage > BAT_VOLTAGE_OFF){
                                             if (getTimerVoltageOff() > 5000)  {
                                                 ClearTimerVoltageOff();
                                                 status = STATUS_PW_OFF;
                                                 LedOffSlow();
                                             }
                                        } else {
                                             ClearTimerVoltageOff();
                                        }
                                        
                                    }
                            }
                            break;
                            
                    case STATUS_WORK_PRESS_KEY:
                            if(IsKeyDown()){
                                    if(GetTimer() > 500) {
                                            status = STATUS_PW_OFF;
                                            LedOffSlow();
                                    }
                            }
                            else {
                                    if(GetTimer() > 30) {
                                            status = STATUS_PW_OFF;
                                            LedOffSlow();
                                    }
                                    else {
                                            status = STATUS_WORK;
                                    }
                            }
                            break;

                    case STATUS_PW_OFF:
                                 while(IsKeyDown()) delay_ms(10);
                                 PowerOff();
                                 
                                 asm sleep
                                 
                                 ClearRx();
                                 PowerOn();
                                 status = STATUS_PW_ON;
                            break;

                    case STATUS_PW_ON:
                            if(IsKeyDown()) {
                                 if(GetTimer() > 500) {
                                      ClearTimer();
                                      if (block) {
                                         block = 0;
                                         delay_ms(50);
                                         LedSetMode(CUR_MODE_MID);
                                         LedOn();
                                         delay_ms(250);
                                         LedOffFast();

                                         delay_ms(50);
                                         LedSetMode(CUR_MODE_MID);
                                         LedOn();
                                         delay_ms(250);
                                         LedOffFast();
                                      }
                                      else {
                                         block = 1;
                                         delay_ms(50);
                                         LedSetMode(CUR_MODE_MID);
                                         LedOn();
                                         delay_ms(250);
                                         LedOffFast();
                                      }
                                      status = STATUS_PW_OFF;
                                 }
                            }
                            else {
                                 if(GetTimer() > 30){
                                     if (block){
                                           Status = STATUS_PW_OFF;
                                     }else{
                                           ledMode = CUR_MODE_HI;
                                           LedSetMode(ledMode);
                                           LedOn();
                                           ClearTimer();
                                           status = STATUS_WORK;
                                     }
                                 }
                                 else{
                                     status = STATUS_PW_OFF;
                                 }
                            }
                            break;
                  }

                   delay_ms(15);
            }
    }