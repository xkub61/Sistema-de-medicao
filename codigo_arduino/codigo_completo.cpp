#include<Arduino.h>
//Variáveis de equipamento
#define q 0.0048875855327   // Passo de quantização = 5/(2^10-1) = 5/1023
// Parâmetros do sinal de corrente
#define R_shunt 3         // Resitência do resistor shunt
#define G_Iso 8             // Ganho do AMPISO
#define G_FiltroC 1.5       // Ganho do amp. de diferenças
// Parâmetros do sinal de tensão
#define Rel_T 13.79         // Relação do transformador
#define Div_Tensao 0.15     // Divisor de tensão
// Parâmetros do sinal de temperatura
#define P_Temp 100          // Proporção temperatura/tensão do LM35 (0,01mV/°C)
#define G_Temp 8.81         // Ganho do ampop
// Parâmetros do sinal de luminância
#define R_Ilum 4600         // Resistência utilizada
#define P_Ilum 0.0000008   // Proporção iluminância/corrente

void setup() {
  Serial.begin(19200);
  
  pinMode(A0, INPUT);  
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);

  // ADMUX se trata do multiplexador do conversor analógico para digital do arduino, abaixo configuraremos seu sistema
  // Configura o primeiro bit do ADMUX como 0
  ADMUX = 0x00;
  
  // Para que possamos realizar uma medição é necessario que seja definida uma tensão de referência para o sistema, onde as medições seram feitas referentes a mesma
  // A linha de código abaixo define a tensão de refereência do sistema para a tensão do arduino (5V) REFS1 REFS0 
  ADMUX |= 0x40;

  // Configura o prescaler
  // Inicializa os bits como 0
  ADCSRA = 0x00;
  
  // Define o fator de divisão para 128
  // ADPS2 ADPS1 ADPS0  
  ADCSRA |= 0x07;
  
  // Permite interrupções
  ADCSRA |= 0x08;
  
  // enable do auto-trigger
  // ADATE
  ADCSRA |= 0x20;

  // TIMER
  ADCSRB = 0x03;  //Define o modo timer0 compare match A

  TCCR0A = 0x02;  // Desconecta as portas  OC0a e OC0b, liga clear timer on compare
  TCCR0B = 0x00;  // Desliga o clock
  TCNT0 = 0;      // Retorna o counter para 0
  OCR0A = 10;     // Irá disparar quando o clock rodar 10 vezes (De 0 a 9)
  OCR0B = 0;      // O clock TCCROB foi desligo então nunca será acionado
  TIMSK0 = 0x02;  // Gera um interrupção no proximo timer

  TCCR0B = 0x04;  // Inicia o timer com frequência clock/64 = 62500 Hz
                  // Período de leitura será de 10/62500 = 0,016 s
                  // Frequência de leitura será de 1/0,0064 = 6250 Hz

  DIDR0 = 0xFF; // Desativa os buffers digitais

  SREG |= 0x80; // Liga as interrupções globais (serão utilizadas no TIMSK0)
 
  ADCSRA |= 0x80; // Liga o conversor
}

// variáveis globais
const int N = 4;      // 4 canais
const int tam = 150;  // tamanho dos buffers de dados

bool procStatus = false;  // flag para iniciar o processamento de dados
int dataVector[N][tam];   // vetores de dados para cada canal
int counter = 0;          // controla o número de amostras

void loop() {
  int i;
  int bufferCmax = -2^31;
  int bufferCmin = 2^31;
  int bufferTmax = -2^31;
  int bufferTmin = 2^31;
  if(procStatus == true){
    // itera pelo buffer de dados printando a corrente, tensão e potência com frequência maior
    for(i = 0; i < tam; i++){
      // Exibe um ponto da corrente instantânea e calcula seus valores máximos e mínimos
      Serial.print("Corrente_mA:"); Serial.print((((dataVector[0][i] * q) - 2.4) * 1000) / (R_shunt * G_Iso * G_FiltroC)); Serial.print(",");
      if (((dataVector[0][i] * q) - 2.25) * 1000 / (R_shunt * G_Iso * G_FiltroC) > bufferCmax) {
        bufferCmax = ((dataVector[0][i] * q) - 2.25) * 1000 / (R_shunt * G_Iso * G_FiltroC);
      } else if (((dataVector[0][i] * q) - 2.25) * 1000 / (R_shunt * G_Iso * G_FiltroC) < bufferCmin) {
        bufferCmin = ((dataVector[0][i] * q) - 2.25) * 1000 / (R_shunt * G_Iso * G_FiltroC);
      }
      
      // Exibe um ponto da tensão instantânea e calcula seus valores máximos e mínimos
      Serial.print("Tensao_V:"); Serial.print(((dataVector[1][i] * q) - 2) * (Rel_T / Div_Tensao)); Serial.print(","); 
      if (((dataVector[1][i] * q) - 2) * (Rel_T / Div_Tensao) > bufferTmax) {
        bufferTmax = ((dataVector[1][i] * q) - 2) * (Rel_T / Div_Tensao);
      } else if (((dataVector[1][i] * q) - 2) * (Rel_T / Div_Tensao) < bufferTmin) {
        bufferTmin = ((dataVector[1][i] * q) - 2) * (Rel_T / Div_Tensao);
      }
      
      // Exibe um ponto da potência instantânea
      Serial.print("Potência_W:"); Serial.print((((dataVector[0][i] * q) - 2.4) / (R_shunt * G_Iso * G_FiltroC)) * (((dataVector[1][i] * q) - 2.25) * (Rel_T / Div_Tensao))); Serial.print(",");
      
      // itera pelos valores amostrados de temp. e ilum.
      if(i % 4 == 0){
        // Exibe um ponto da temperatura
        Serial.print("Temperatura_C:"); Serial.print((dataVector[2][i] * q * P_Temp) / G_Temp); Serial.print(",");
        
        // Exibe um ponto da iluminância
        Serial.print("Iluminancia_Lumens:"); Serial.print((dataVector[3][i] * q) / (R_Ilum * P_Ilum)); Serial.print(",");           
      }
      if (i == tam - 1) { 
        Serial.print("CorrenteRMS_mA:"); Serial.print(abs(bufferCmax - bufferCmin) / (2 * sqrt(14))); Serial.print(",");
        Serial.print("TensãoRMS_V:"); Serial.print(abs(bufferTmax - bufferTmin) / (2 * sqrt(2))); Serial.print(",");
        Serial.print("PotênciaRMS_mW:"); Serial.print((abs(bufferCmax - bufferCmin) / (2 * sqrt(14))) * (abs(bufferTmax - bufferTmin) / (2 * sqrt(2)))); Serial.print(",");
      }
      Serial.println();
    }
    noInterrupts();
    procStatus = false;
    interrupts();
  }
}

ISR(ADC_vect) {
  int sample, CH;
  //Lê o utlimo dado
  sample = ADCL;        //Primeiro byte
  sample += ADCH << 8;  //Ultimo byte
  if(procStatus == false) {
    CH = ADMUX & 0x0F;
    //Corrente (canal 0) e tensão (canal 1) serão amostradas em todas as iterações
    //Temperatura (canal 2) e iluminância (canal 3) só são amostradas a cada iteração divisível por 4
    if(CH == 2 || CH == 3) {
      if(counter % 4 == 0) {
        dataVector[CH][counter] = sample; 
      }
    }
    else {
      dataVector[CH][counter] = sample;
    }
    //Altera o sinal a ser lido
    if(++CH < N) {
      ADMUX += 1;  
    }
    else {
      //Altera a iteração do sinal a ser lido
      ADMUX &=0xF0;
      counter++;
    }
    //Roda o loop quando todos os valores do vetor forem preenchidos
    if(counter == tam) {
      counter = 0;
      procStatus = true;  
    }
  }
}

ISR(TIMER0_COMPA_vect) {
  // just clears the interrupt flag
}
