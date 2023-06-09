#include <LiquidCrystal.h>
#include <string.h>
#include <stdio.h>

// TIMER ASSIGNMENT
// 8BIT  TIMER0 - PWM output at 7.8 kHz to pin 6 (OCOA)
// 16BIT TIMER1 - input time measurement (1 reading = 8 full periods of 3150 Hz test tone)
// 8BIT  TIMER2 - 3150 Hz test tone (50% duty cycle pulses to R-C-diode limiter) to pin 11 (OC2A)

// ARDUINO NANO PIN ASSIGNMENT
// TOP SIDE ===========================================================
// 12 PB4     *** LCD DB4
// 11 PB3     *** OC2A = output 3150Hz to analogue output
// 10 PB2     *** LCD DB5
// 09 PB1     ***  
// 08 PB0     *** ICP1 = audio pulse input
// 07 PD7     *** 
// 06 PD6     *** OC0A = output 8-bit PWM to meter (nano: pin9)
// 05 PD5     *** LCD RS 
// 04 PD4     *** LCD EN
// 03 PD3     *** LCD DB7
// 02 PD2     *** LCD DB6
// 01 PD1 TX  *** 
// 00 PD0 RX  *** 
// BOTTOM SIDE ========================================================
// 13 PB5     *** 
// A0 PC0     *** debug pin
// A1 PC1     *** 
// A2 PC2     *** 
// A3 PC3     *** 
// A4 PC4     *** Switch3-x3
// A5 PC5     *** Switch2-x10
// A6 PC4     *** Battery sense
// A7 PC5     *** Switch1-DIN-RMS

// Timing constants. The sampling period equals eight input audio periods, thus the sampling rate is merely 393.75 Hz.
// Measuring each input period (around 0.3 ms or 5K clock cycles) is technically possible, but the resolution will be to coarse to be useful

const long     clock_frequency=16000000L;   
const uint8_t  pulses_per_pack=8;
const float    fr_dividend=128000000.0; /* clock_frequency*pulses_per_pack adjusted for actual clock speed*/
const float    std_signal_frequency_float=3150.0;
const uint16_t std_signal_frequency_int=3150;
const uint16_t std_pack_clocks=40635; /* pulses_per_pack*clock_frequency/std_signal_frequency) */ 
const float    std_sampling_frequency=393.75;
const float    std_pack_period=0.0025397; /* seconds per pack */
const uint16_t toofast_pack_clocks=27090; /* -1/2 std speed */
const uint16_t tooslow_pack_clocks=60952; /* +1/2 std speed */
const uint16_t nosignal_indicator_delay=200;
const uint16_t wrongspeed_indicator_delay=200;

// IIR DSP filter settings
// Alphas are variable, so they can be adjusted if input frequency (and thus the sampling rate) differs considerably from the standard.
// Narrowband channel filters trimmed in a spreadsheet for a center maximum at 4 Hz per DIN 45507

const float    instant_deviation_limit_pos=0.0999;
const float    instant_deviation_limit_neg=-0.0999;
const float    dispersion_limit=0.0499; 
const float    overflow_value=0.099;
      
const float    LPF_input_freq=200.0; /* Hz */  
const float    LPF_input_alpha=0.7; 
const float    LPF_input_beta=0.3; /* =1-? */      
const float    LPF_avg1_freq=0.2;  /* Hz */    
const float    LPF_avg1_alpha=0.0032;
const float    LPF_avg1_beta=0.9968; /* =1-? */      
const float    LPF_avg2_freq=0.2; /* Hz */     
const float    LPF_avg2_alpha=0.0032;
const float    LPF_avg2_beta=0.9968; /* =1-? */    
const float    HPF_wide_freq=0.2; /* Hz */     
const float    HPF_wide_beta=0.9968; /* =1-? */
const float    HPF_narrow_freq=1; /* Hz */
const float    HPF_narrow_beta=0.9842; /* =1-? */
const float    LPF_narrow_freq=10; /* Hz */
const float    LPF_narrow_alpha=0.15;
const float    LPF_narrow_beta=0.85; /* =1-? */     

const float    adjust_input=1.0;     /* placeholder */
const float    adjust_wide=1.0;      /* placeholder */ 
const float    adjust_narrow=1.0;    /* 1.0606 from spreadsheet */

const float    RMS_alpha=0.005;
const float    RMS_beta=0.995;
const float    decay_output_tau=0.5;  /* tau,seconds */
const float    decay_output_alpha=0.001; 
const float    decay_output_beta=0.999; /* =1-? */

// Reading input stream of pulses - IIR DSP filter variables
volatile float raw_input_frequency;
float          LPF_input_filter;
float          LPF_avg1_filter;
float          average_frequency; /* output of LPF_avg2 */
float          instant_deviation;
float          HPF_wide_previous_in;
float          HPF_wide_filter;
float          HPF_narrow_previous_in;
float          HPF_narrow_filter;
float          LPF_narrow_filter;
float          wide_dispersion;   /* unsigned abs value */
float          narrow_dispersion; /* unsigned abs value */
volatile float narrow_RMS_sumofsquares;

// buffering frequency, wideband and narrowband dispersion so that frequent updates won't interfere with slow LCD regen
volatile float buf_FR;
volatile float buf_WB;
volatile float buf_NB; 
volatile float buf_NB_RMS; 

// Processing input stream of pulses - flags
volatile uint8_t  loop_lockout; /* prohibits LCD redraw until a complete new capture/calculation is done */
uint8_t           edge_count; /* count rising fronts in a pack */
uint8_t           timer_overflow; /* no-input-signal flag */
uint16_t          timer_overflow_countdown; /* delayed copy of no-signal flag for blinker */
uint8_t           timer_wrongspeed; /* signal present but wrong speed flag */
uint16_t          timer_wrongspeed_countdown; /* too-fast-signal flag */
volatile uint16_t buffer_reading; /* state of timer0 at the time of n-th capture interrupt */
uint8_t           dispersion_overflow_flag;


// LCD in 4-bit mode 
const uint8_t  lcd_RS=7;
const uint8_t  lcd_EN=4;
const uint8_t  lcd_DB4=12;
const uint8_t  lcd_DB5=10;
const uint8_t  lcd_DB6=2;
const uint8_t  lcd_DB7=3;
LiquidCrystal  lcd(lcd_RS, lcd_EN, lcd_DB4, lcd_DB5, lcd_DB6, lcd_DB7); 
uint8_t        show_RMS; /* false: show DIN 45507 */
const uint8_t  lcd_chars=20;

// Display scale bands for 20x4 alphanumeric LCD (line1)
const size_t   x10_pin=A5;
const size_t   x3_pin=A4;
const size_t   RMS_pin=A7;
const size_t   debug_pin=A0;
const uint8_t  meter_bands=4;
const float    meter_band_limit[meter_bands]={ 0.00100, 0.00300, 0.0100, 0.0300 };
const char     meter_band_texts[meter_bands][lcd_chars+1]={
               "0   .03%  .06%   .1%",
               "0    .1%   .2%   .3%",
               "0    .3%   .6%    1%",
               "0     1%    2%    3%" };
const uint8_t  error_message_position=4;
const char     meter_band_nosignal_message[]  ="  NO SIGNAL";
const char     meter_band_wrongspeed_message[]="  FREQUENCY?";
const char     meter_band_overflow_message[]  ="  OVERFLOW ";

// Display bargraph animation
const char     bargraph_off=0x80;
const char     bargraph_on=0xFF;

// Battery voltage monitor
const uint8_t  battery_adc_pin=A6;
const uint8_t  battery_states=5;
uint8_t        battery_state; /* 0=low ... 4=high */
const char     battery_state_icon[battery_states]={ 0x9F, 0x9E, 0x9D, 0x9C, 0x9B };
const uint16_t battery_state_lowest_reading[battery_states]={ 0, 680, 720, 760, 820 };

void process_reading()
{
volatile float old, val;
      dispersion_overflow_flag=0;
 /* extract and denoise instant frequency from buffered timer reading ======================================= */           
      raw_input_frequency=fr_dividend/(float(buffer_reading));   
      old=LPF_input_filter; 
      LPF_input_filter=raw_input_frequency*LPF_input_alpha+old*LPF_input_beta;
 /* two LPFs to extract average frequency =================================================================== */      
      old=LPF_avg1_filter;            
      LPF_avg1_filter=LPF_input_filter*LPF_avg1_alpha+old*LPF_avg1_beta;
      old=average_frequency;                     
      average_frequency=LPF_avg1_filter*LPF_avg2_alpha+old*LPF_avg2_beta;
 /* extract deviation (relative to average frequency ========================================================= */     
      HPF_wide_previous_in=instant_deviation;  /* saving former value for HPF-wide first */
      instant_deviation=(LPF_input_filter/average_frequency)-1;
      if (instant_deviation>instant_deviation_limit_pos) { instant_deviation=instant_deviation_limit_pos; }
        else if (instant_deviation<instant_deviation_limit_neg) { instant_deviation=instant_deviation_limit_neg; }                            
 /* wideband channel HPF ===================================================================================== */            
      old=HPF_wide_filter;            
      HPF_wide_filter=HPF_wide_beta*old+(instant_deviation-HPF_wide_previous_in); /* y[n]=?y[n?1]+x[n]?x[n?1] */
  /* narrowband channel HPF+LPF ============================================================================== */                   
      HPF_narrow_previous_in=old;
      old=HPF_narrow_filter;          
      HPF_narrow_filter=HPF_narrow_beta*old+(HPF_wide_filter-HPF_narrow_previous_in); /* y[n]=?y[n?1]+x[n]?x[n?1] */      
      old=LPF_narrow_filter;            
      LPF_narrow_filter=HPF_narrow_filter*LPF_narrow_alpha+old*LPF_narrow_beta;
 /* wideband channel "precision rectifier" =================================================================== */ 
      val=(abs(HPF_wide_filter))*adjust_wide;                        
      if (val>dispersion_limit) { val=dispersion_limit; dispersion_overflow_flag=1; }  
      else { if (val>=wide_dispersion) { wide_dispersion=val; } else { wide_dispersion=wide_dispersion*decay_output_beta; } }     
 /* narrowband channel "precision rectifier" ================================================================= */  
      val=adjust_narrow*abs(LPF_narrow_filter);
      if (val>dispersion_limit) { val=dispersion_limit; dispersion_overflow_flag=1; } 
      if (val>wide_dispersion) { val=wide_dispersion; } 
      else { if (val>=narrow_dispersion) { narrow_dispersion=val; } else { narrow_dispersion=narrow_dispersion*decay_output_beta; } }                                          
/* narrowband RMS uses val calculated just above! */
      old=narrow_RMS_sumofsquares;
      narrow_RMS_sumofsquares=val*val*RMS_alpha+old*RMS_beta; 
      buf_NB_RMS=sqrt(narrow_RMS_sumofsquares);     
 /* buffering and other housekeeping ========================================================================= */          
      buf_FR=average_frequency;
      buf_WB=wide_dispersion;  
      buf_NB=narrow_dispersion;   
      loop_lockout=0;                                                      
}

ISR(TIMER1_CAPT_vect)
{ 
  digitalWrite(debug_pin,1);
  edge_count++; 
  /* upon the 8-th upward pulse front record the timing */
  if (edge_count>=pulses_per_pack)
  { 
    TCNT1=0; 
    edge_count=0; 
    timer_overflow=0; 
    if (timer_overflow_countdown) timer_overflow_countdown--; 
    buffer_reading=ICR1; 
    if((buffer_reading>toofast_pack_clocks)&&(buffer_reading<tooslow_pack_clocks)) 
    /* normal speed */
    { timer_wrongspeed=0;  if (timer_wrongspeed_countdown) timer_wrongspeed_countdown--;  }
    else 
    /* wrong speed */
    { buffer_reading=toofast_pack_clocks; timer_wrongspeed=1;  timer_wrongspeed_countdown=wrongspeed_indicator_delay; }
    process_reading();  
  }   
  digitalWrite(debug_pin,0);  
}

ISR(TIMER1_OVF_vect)
{ 
  /* overflow = no signal in = display exception */  
  edge_count=0;
  timer_overflow=1;
  timer_overflow_countdown=nosignal_indicator_delay;
  buffer_reading=std_pack_clocks;
  process_reading();
}

void setup() 
{
/* TIMER0 8-bit fast pwm at around 8 kHz */
   noInterrupts();
   TCCR0A=(1<<COM0A1) | (1<<WGM01) | (1<<WGM00);  
   TCCR0B=(1<<CS01) ;
   OCR0A =22;
   pinMode(6, OUTPUT);
/* TIMER2 8-bit 3150 Hz oscillator (actually 3.13 kHz) */   
   TCCR2A=(1<<COM2A0) | (1<<WGM21);  
   TCCR2B=(1<<WGM22) | (1<<CS21) | (1<<CS20);
   OCR2A =79;
   pinMode(11, OUTPUT);
/* TIMER 1 16-bit reading on input pulses */
   TCCR1A=0;
   TCCR1B=(1<<ICNC1) | (1<<ICES1) | (1<<CS10);
   TIMSK1=(1<<ICIE1) | (1<<TOIE1); 
   pinMode(8, INPUT);
/* reset data */ 
   raw_input_frequency=std_signal_frequency_float;
   LPF_input_filter=std_signal_frequency_float;
   LPF_avg1_filter=std_signal_frequency_float;
   average_frequency=std_signal_frequency_float; 
   instant_deviation=0.0;
   HPF_wide_previous_in=0.0;
   HPF_wide_filter=0.0;
   HPF_narrow_previous_in=0.0;
   HPF_narrow_filter=0.0;
   LPF_narrow_filter=0.0;
   wide_dispersion=0.0;   
   narrow_dispersion=0.0; 
   buf_FR=std_signal_frequency_float;
   buf_NB=0.0;
   buf_WB=0.0;  
   buf_NB_RMS=0.0;
/* reset flags */   
   loop_lockout=1;  
   edge_count=0;
   dispersion_overflow_flag=0;
   timer_overflow=1; 
   timer_overflow_countdown=nosignal_indicator_delay;
   timer_wrongspeed=1; 
   timer_wrongspeed_countdown=wrongspeed_indicator_delay;     
   show_RMS=0;
/* housekeeping */
   pinMode(battery_adc_pin, INPUT);
   pinMode(debug_pin, OUTPUT);
   pinMode(RMS_pin, INPUT_PULLUP);
   pinMode(x10_pin, INPUT_PULLUP);
   pinMode(x3_pin, INPUT_PULLUP);
   lcd.begin(20, 4);    
   interrupts();
}

void loop() 
{ 
   char lcd_line[lcd_chars+1];
   show_RMS=digitalRead(RMS_pin);
   while (loop_lockout) delayMicroseconds(1);
/* LINE 1 ============================================================================================ */
   /* get scale setting */
   uint8_t current_meter_band=0;
   if (!digitalRead(x10_pin)) { current_meter_band=current_meter_band+2; }
   if (!digitalRead(x3_pin))  { current_meter_band++; }
   /* fill string buffer */
   strncpy(lcd_line, &(meter_band_texts[current_meter_band][0]), lcd_chars);
   char *error_message=0;
   if (timer_overflow_countdown) { error_message=meter_band_nosignal_message; } 
      else { if (timer_wrongspeed_countdown) { error_message=meter_band_wrongspeed_message; }
           else { if (dispersion_overflow_flag) { error_message=meter_band_overflow_message; } } }
   if (error_message) strncpy(lcd_line+error_message_position, error_message, strlen(error_message));
   lcd_line[lcd_chars]=0;
   lcd.setCursor(0, 0); 
   lcd.print(lcd_line);    
/* LINE 2 ============================================================================================ */
   float block_size=meter_band_limit[current_meter_band]/lcd_chars;
   volatile float val;
   if (show_RMS) { val=buf_NB_RMS; } else { val=buf_WB; }
   uint16_t blocks=uint16_t(val/block_size+.5);
   if (blocks>lcd_chars) blocks=lcd_chars;
   for (byte i=0; i<lcd_chars; i++) lcd_line[i]=bargraph_off;
   for (byte i=0; i<blocks; i++) lcd_line[i]=bargraph_on; 
   lcd_line[lcd_chars]=0;
   lcd.setCursor(0, 1); 
   lcd.print(lcd_line);   
/* LINE 3 ============================================================================================ */
   blocks=uint16_t(buf_NB/block_size+.5);
   if (blocks>lcd_chars) blocks=lcd_chars;
   for (byte i=0; i<lcd_chars; i++) lcd_line[i]=bargraph_off;
   for (byte i=0; i<blocks; i++) lcd_line[i]=bargraph_on; 
   lcd_line[lcd_chars]=0;
   lcd.setCursor(0, 2); 
   lcd.print(lcd_line);       
/* LINE 4 ============================================================================================ */
   uint16_t adc=analogRead(battery_adc_pin);
   for (uint8_t i=0; i<battery_states ;i++) { if (adc>=battery_state_lowest_reading[i]) { battery_state=i; } }
   dtostrf(val*100, 4, 2, &(lcd_line[1]));
   dtostrf(buf_NB*100,      4, 2, &(lcd_line[8]));   
   dtostrf((buf_FR+.25),          4, 2, &(lcd_line[15]));
   if (show_RMS) { lcd_line[0]='R'; } else { lcd_line[0]='W'; };
   lcd_line[ 5]='%';
   lcd_line[ 6]=' ';
   lcd_line[ 7]='N';
   lcd_line[12]='%';
   lcd_line[13]=' ';
   lcd_line[14]='F';
   lcd_line[19]=battery_state_icon[battery_state];
   lcd_line[20]=0;
   lcd.setCursor(0, 3); 
   lcd.print(lcd_line); 
/* set wait-for-buffer flag =========================================================================== */    
   loop_lockout=1;  
}