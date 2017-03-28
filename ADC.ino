// We are using an oversampling and averaging method to increase the ADC resolution
// Now we store the ADC readings in float format
//#define ADC_OFFSET 0x07
#define ADC_CHN_OFFSET 0

void Read_adc_raw(void)
{
  static int i;
  static uint32_t temp1;    
  static uint16_t temp2;    
  static float tval;
  
  // ADC readings...
  for (i=0+ADC_CHN_OFFSET;i<ADC_HW_CHANNELS;i++)
    {
      do{
        temp1= analog_buffer[i];
        temp2= analog_count[i];
        } while(temp1 != analog_buffer[i]);  // Check if there was an ADC interrupt during readings...
      
      float tval = (float)temp1/(float)temp2;
      if (temp2>0) AN[i-ADC_CHN_OFFSET] = tval*4; //+2bit oversampling...     // Check for divide by zero 
            
    }
  // Initialization for the next readings...
  for (int i=0+ADC_CHN_OFFSET;i<ADC_HW_CHANNELS;i++){
    do{
      analog_buffer[i]=0;
      analog_count[i]=0;
      } while(analog_buffer[i]!=0); // Check if there was an ADC interrupt during initialization...
  }
}



float read_adc(int select)
{
  if (select < ADC_CHANNELS){
    return(AN[select]-AN_OFFSET[select]); 
  }else{
    return 0;
  }
}


//Activating the ADC interrupts. 
void Analog_Init(void)
{
 ADCSRA|=(1<<ADIE)|(1<<ADEN);
 ADCSRA|= (1<<ADSC);
}

//
void Analog_Reference(uint8_t mode)
{
  analog_reference = mode;
}


//ADC interrupt vector, this piece of code
//is executed everytime a convertion is done. 
ISR(ADC_vect)
{
  volatile uint8_t low, high;
  low = ADCL;
  high = ADCH;
  
  volatile uint16_t val = (high << 8) | low;

  if(analog_count[MuxSel]<ADC_MAXCOUNT) {
        analog_buffer[MuxSel] += val;   // cumulate analog values
        analog_count[MuxSel]++;
  }
  MuxSel++;
  //MuxSel &=0x03; //if(MuxSel >=4) MuxSel=0;
  MuxSel &=0x07; //if(MuxSel >=8) MuxSel=0;
  //MuxSel |= 0x08;   //if(MuxSel <=8) MuxSel=8;  
  //Serial.println(MuxSel,DEC);
  //MuxSel Ã¨ massimo 4 bit (0x0F) e sfrutto tutto.
  ADMUX = (analog_reference << 6) | MuxSel;
  // start the conversion
  ADCSRA|= (1<<ADSC);
}

