#include <Arduino.h>

//Parametros do projeto
#define q 0.0048875855327
#define F_sampling 840.0 //Frequencia de amostragem de cada canal
// Parâmetros do sinal de corrente
#define R_shunt 1.8         // Resitência do resistor shunt
#define G_Iso 8             // Ganho do AMPISO
#define G_FiltroC 2.2       // Ganho do amp. de diferenças

// Parâmetros do sinal de tensão
#define Rel_T 18.14        // Relação do transformador  anterior(13.79)
#define Div_Tensao 0.212    // Divisor de tensão

// Parâmetros do sinal de temperatura
#define P_Temp 100          // Proporção temperatura/tensão do LM35 (0,01mV/°C)
#define G_Temp 8.81         // Ganho do ampop

// Parâmetros do sinal de luminância
#define R_Ilum 2200        // Resistência utilizada
#define P_Ilum 0.0000008   // Proporção iluminância/corrente


const int N = 4;          //Number of channels
const int tam = 150;      //Size of data buffers

bool sendStatus = false;  //Flag to start data processing
int dataVector[N][tam];   //Data vectors for each channel

//----------------------------
//Initialization
void setup()
{
    //Set serial port configuration and establish communication
  Serial.begin(19200);//115200

    //Configure the ADC pins
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

  //---------- Configure the ADC --------------------------------
    //Configure the ADC mux
  ADMUX = 0x40;    //MUX[3:0]=0000 -> select analog channel 0, 
                   //ADLAR=0 -> the ADC results are right adjusted
                   //REFS[1:0]=01, set voltage reference do AVcc

    //Configure the ADC prescaler and enable interrupts
  ADCSRA = 0x08 | 0x20 | 0x06;    //ADIE=1, enable ADC interrupts
                                  //ADATE=1, enable auto-trigger
                                  //ADPS[2:0]=110, set prescaler to 64 -> ADC_clk = 16e6/64 = 250kHz  

    //Configure the ADC trigger source
  ADCSRB = 0x03;    //ACME=0, disable analog comparator multiplexer
                    //ADTS[2:0]=011 -> trigger source = Timer/Counter0 compare match A
        
    //Disable the ADC digital input buffers
  DIDR0 = 0xFF;

  //-------- Configure the timer/counter 0 --------------------------
    //ATENÇÃO: a ordem dos comandos altera o comportamento do sistema, 
    //portanto procure manter a definida abaixo.
  TCCR0A = 0x02;    //COM0A[1:0] = 00, COM0B[1:0] = 00 -> normal port operation, OC0A, OC0B disconnected.
                    //WGM0[2:0] = 010 -> CTC mode (clear timer/counter on compare)  
  TCCR0B = 0x00;    //FOC0A, FOC0B = 0 -> force output compare A, B = 0
                    //CS0[2:0] = 000 -> no clock source (timer/counter stopped)
  TCNT0 = 0;        //Reset the counter 0     
  OCR0A = 18;       //Set the compare register A
  OCR0B = 0;        //Reset the compare register B      
  TIMSK0 = 0x02;    //OCIE0A = 1 -> timer/counter 0 output compare A match interrupt enable.        

  //--------- Start acquisition ------------------------------------
    //Enable global interrupts
  SREG |= 0x80;     //Enable global interrupts
    //Enable the AD converter
  ADCSRA |= 0x80;   //ADEN=1, enable AD converter
    //Start the timer
  TCCR0B = 0x04;    //CS0[2:0] = 011 -> clkIO/256 = 62.5kHz
}

//-----------------------------
//ADC interrupt service routine
ISR(ADC_vect)
{
  int sample, CH;
  static int counter = 0;          //Controls the number of samples
  
      //Read the latest sample
  sample = ADCL;       //Read the lower byte
  sample += ADCH<<8;   //Read the upper byte
  
      //Store data from the specific channel
      //Halt acquisition after 'tam' samples and start transmission
  if (sendStatus == false){
    CH = ADMUX & 0x0F;
    dataVector[CH][counter] = sample;   //Store the data
          //Verify if all channels were acquired
    if (++CH < N)
      ADMUX += 1;    //If not, go to the next channel
    else{
      ADMUX &= 0xF0; //If so, turn to channel 0 and
      counter++;     //update the number of samples
    }
    
      //Verify if it is time to transmit
    if (counter == tam){
      counter = 0;
      sendStatus = true;
    }  
  }
}

ISR(TIMER0_COMPA_vect) {
  // just clears the interrupt flag
}
//--------------------------------------
double energia = 0;
double corrente_rms = 0;
double tensao_rms = 0;
//Loop to send data to the main computer
void loop()
{
  int i = 0;
  double cRms = 0;
  double vRms = 0;
  
    //Verify if it is time to transmit data
  if (sendStatus == true){
    for(i = 0; i < tam; i++){

      // Exibe um ponto da corrente instantânea e calcula seus valores máximos e mínimos
      double corrente = ((dataVector[0][i] * q) - 2.5) / (R_shunt * G_Iso * G_FiltroC);
      cRms += corrente*corrente;
      Serial.print("Corrente_mA:"); Serial.print(corrente*1000); Serial.print(",");
      Serial.print("CorrenteRMS_mA:"); Serial.print(corrente_rms*1000); Serial.print(",");
      
      // Exibe um ponto da tensão instantânea e calcula seus valores máximos e mínimos
      double tensao = ((dataVector[1][i] * q) - 2.5) * Rel_T / Div_Tensao;
      vRms += tensao*tensao;
      Serial.print("Tensao_V:"); Serial.print(tensao); Serial.print(","); 
      Serial.print("TensãoRMS_V:"); Serial.print(tensao_rms); Serial.print(",");
     
      // Exibe um ponto da potência instantânea
      double potencia = corrente * tensao;
      Serial.print("Potência_W:"); Serial.print(potencia); Serial.print(",");
      
      // Exibe um ponto de energia consumida
      energia += potencia/(F_sampling*3600.0);
      Serial.print("Energia_Wh:"); Serial.print(energia); Serial.print(",");

      // itera pelos valores amostrados de temp. e ilum.
      // Exibe um ponto da temperatura
      uint16_t temperatura = (dataVector[2][i] * q * P_Temp);
      Serial.print("Temperatura_C:"); Serial.print(temperatura); Serial.print(",");
      
      // Exibe um ponto da iluminância
      double iluminancia = (dataVector[3][i] * q) / (R_Ilum * P_Ilum);
      Serial.print("Iluminancia_Lumens:"); Serial.print(iluminancia); Serial.print(",");           

      
      Serial.println();
    }
    tensao_rms = sqrt(vRms/tam);
    corrente_rms = sqrt(cRms/tam);
    
    //Restart acquisition
    noInterrupts();
    sendStatus = false;
    interrupts();
  }
}