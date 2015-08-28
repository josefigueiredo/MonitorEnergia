#include <TimerOne.h>
#include <LiquidCrystal.h>

#define AMOSTRAS 64
#define FREQ 60

// initialize the library with the numbers of the interface pins
//http://www.hobbytronics.co.uk/arduino-lcd-keypad-shield
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);


//variaveis para medida de tempo
unsigned long microseconds,duration,T,tAmostra, tAmostraMinusADTime;
unsigned int vetorA[AMOSTRAS],vetormA[AMOSTRAS],vetorV[AMOSTRAS];
float vetorASemDC[AMOSTRAS],vetormASemDC[AMOSTRAS],vetorVSemDC[AMOSTRAS];
unsigned int timeOffset = 0;

uint16_t tempoLoop = 250,tmpVar;
byte sensorA = A0 ,sensormA = A1, sensorV = A3,led = 13;
boolean estadoLed = true,mAdbg=false,Adbg=false,Vdbg=false;
char cmd;
float ganhoA=9.52,ganhomA=9.4,ganhoV=76450;//ganhoV=52250;

float rmsAnterior=0.001;
boolean houveAlteracaoRMS=false;
byte numVezesDiferente=0,limiteDiferencas=3;

//http://www.microsmart.co.za/technical/2014/03/01/advanced-arduino-adc/
// Define various ADC prescaler
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

void setup(){
  Serial.begin(9600);
  pinMode(sensorA, INPUT);     
  pinMode(sensormA, INPUT);     
  pinMode(sensorV, INPUT);     

  initDisplay();
  ajustaBrilho();


  // set up the ADC
  analogReference(EXTERNAL);
  ADCSRA &= ~PS_128;  // remove bits set by Arduino library
  // you can choose a prescaler from above.
  // PS_16, PS_32, PS_64 or PS_128
  ADCSRA |= PS_32;    // set our own prescaler here

  if(FREQ == 60){
    Serial.println("60");
    T = 16666; //tempo do ciclo em microssegundos 
  }
  else if(FREQ == 50){
    Serial.println("50");
    T = 20000; //tempo do ciclo em microssegundos 
  }
  //executa a primeira leitura de cada canal - isto  necessrio porque a primeira leitura  mais lenta que as outras.
  tmpVar =  analogRead(sensorA);
  tmpVar =  analogRead(sensormA);
  tmpVar =  analogRead(sensorV);

  //calcula o tempo de TRES leituras do AD no modo ACELERADO
  microseconds = micros();
  tmpVar =  analogRead(sensorA);
  tmpVar =  analogRead(sensormA);
  tmpVar =  analogRead(sensorV);
  duration = micros() - microseconds;

  tAmostra = T/AMOSTRAS; //calcula o maximo de tempo de cada amostra dentro do ciclo T amostra 
  // +2 para 64 e 128 (usando preescaler de 32) com 2 leituras
  // -1 para 64 e 128 (usando preescaler de 32) com 3 leituras (media de 16.58
  tAmostraMinusADTime = tAmostra-duration-1; //subtrai o tempo de cada amostra do tempo de converso.

  Serial.print("T: ");
  Serial.print(T,10);
  Serial.println("uS");
  Serial.print("tAmostra: ");
  Serial.print(tAmostra);
  Serial.println("uS");
  Serial.print("tLeiutraAD: ");
  Serial.print(duration);
  Serial.println("uS");
  Serial.print("tAmostraMinusADTime: ");
  Serial.print(tAmostraMinusADTime);
  Serial.println("uS");

  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);     
  Serial.println("---------------------------------------------------------"); 

  //inicializa contador do temporizador para leitura em modo ISR
  Timer1.initialize(100000); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  Timer1.attachInterrupt( timerIsr ); // attach the service routine here
  //controle tipo semaforos (spinlock)
  //spinLock  uma chave para contrle de acesso ao vetor de leituras
}


void loop(){
  if(Serial.available()){
    cmd = Serial.read();

    switch(cmd){
    case 'a':
      mAdbg = !mAdbg;
      break;
    case 'b':
      Adbg = !Adbg;
      break;
    case 'c':
      Vdbg = !Vdbg;
      break;
    case 'm':
      //chama função memoriaLivre()
      Serial.print("memoria livre: ");
      Serial.println(memoriaLivre(),DEC);
      break;
    default:
      Serial.println("Ativar/desativar debug a(mA), b(A), c(V), m (mem livre) ");
      break;  
    }
  }

  fazLeitura();
  delay(1000);
}


//executado por fora do loop
void timerIsr(){
  piscaLed();
}


void fazLeitura(){
  unsigned int accV=0,accA=0,accmA=0,CorrmARMS=0,VoltsRMS=0;
  float mediaV=0.0,mediaA=0.0,mediamA=0.0,CorrRMS=0,accVflt=0,accAflt=0,accmAflt=0;

  //microseconds = micros();
  for(uint16_t i=0; i<AMOSTRAS; i++){
    vetorA[i] = analogRead(sensorA);
    vetormA[i] = analogRead(sensormA);
    vetorV[i] = analogRead(sensorV);
    accA += vetorA[i];
    accmA += vetormA[i];
    accV += vetorV[i];
    delayMicroseconds(tAmostraMinusADTime);
  }
  //duration = micros() - microseconds;
  //Serial.print(" - tempo leitura dois canais 64 amostras: ");   
  //Serial.println(duration);
  //duration = 0;

  mediaA = (float)accA / AMOSTRAS;
  mediamA = (float)accmA / AMOSTRAS;
  mediaV = (float)accV / AMOSTRAS;  

  for(uint8_t i=0;i<AMOSTRAS;i++){
    vetorASemDC[i] = (float)vetorA[i] - mediaA;
    vetormASemDC[i] = (float)vetormA[i] - mediamA;
    vetorVSemDC[i] = (float)vetorV[i] - mediaV;
  }

  if(Adbg == true){
    Serial.print("A,");
    for(uint8_t i=0;i<AMOSTRAS;i++){
      Serial.print(vetorASemDC[i]); 
      Serial.print(",");
    }
    Serial.println();
  }
  if(mAdbg == true){
    Serial.print("mA,");
    for(uint8_t i=0;i<AMOSTRAS;i++){
      Serial.print(vetormASemDC[i]); 
      Serial.print(",");
    }
    Serial.println();
  }
  if(Vdbg == true){
    Serial.print("V,");
    for(uint8_t i=0;i<AMOSTRAS;i++){
      Serial.print(vetorVSemDC[i]); 
      Serial.print(",");
    }
    Serial.println();
  }

  //somatorio dos quadrados
  for(uint8_t i=0;i<AMOSTRAS;i++){
    accAflt += (vetorASemDC[i] * vetorASemDC[i]);
    accmAflt += (vetormASemDC[i] * vetormASemDC[i]);    
    accVflt += (vetorVSemDC[i] * vetorVSemDC[i]);
  }

  //raiz quadrada do somatorio dos quadrados peloa raiz do numero de amostras
  CorrRMS = sqrt(accAflt/AMOSTRAS)/ganhoA;
  CorrmARMS = sqrt(accmAflt/AMOSTRAS)/ganhomA;
  VoltsRMS = (sqrt(accVflt/AMOSTRAS)/ganhoV)*224900;


  testaAlteracaoRMS(CorrRMS);

  //SE o RMS calculado  maior que 999mA descarta a medida e assume  sensor de alta amperagem.
  //imprime a coisa
  if(CorrmARMS > 1000){
    //atualizaDisplay(CorrRMS,VoltsRMS, 0.86);
    Serial.print(CorrRMS);
    Serial.print(" A, ");
    Serial.print(VoltsRMS);
    Serial.println(" V");

    //testaAlteracaoRMS(CorrRMS);
  }
  else{
    //atualizaDisplay(CorrmARMS,VoltsRMS, 0.86);
    Serial.print(CorrRMS);
    Serial.print(" A, ");
    Serial.print(VoltsRMS);
    Serial.println(" V");

    //testaAlteracaoRMS(CorrmARMS);    
  }

}

void testaAlteracaoRMS(float newRMS){
  //Quando teste der 'positivo' n vezes zera contador e envia esta amostra
  float dif = newRMS - rmsAnterior;
  if(abs(dif) > 0.15){
    numVezesDiferente++;
    Serial.println(numVezesDiferente);
    //se testou n vezes e deu diferença entao tem que alterar.
    if(numVezesDiferente >= limiteDiferencas){
      numVezesDiferente=0;
      rmsAnterior = newRMS;
      Serial.println("RMS Alterado");
      //sendAmostra();
    }
  }else{numVezesDiferente=0;}
}

void piscaLed(){
  estadoLed = !estadoLed;
  digitalWrite(led, estadoLed);
}


//função que 'quantifica' memoria livre
extern int __bss_end;
extern void *__brkval;

int memoriaLivre(){
  int memLivre;
  if((int)__brkval == 0)
    memLivre = ((int)&memLivre) - ((int)&__bss_end);
  else
    memLivre = ((int)&memLivre) - ((int)&__brkval);
  return memLivre;
}


void initDisplay(){
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.setCursor(0,0);
  lcd.print("Inicializando:");
  for(byte i=0;i<14;i++){
    lcd.setCursor(i,1);
    lcd.print(".");
    delay(100);
  }
  lcd.setCursor(14,1);
  lcd.print("ok");
  delay(500);

  limparDisplay();
}


void atualizaDisplay(float iRMS,unsigned int vRMS, float fp){
  //mostra corrente
  lcd.setCursor(0,0);
  lcd.print("I=");
  lcd.print(iRMS);
  lcd.print("A");

  //mostra tensao
  lcd.setCursor(0,1);
  lcd.print("V=");
  lcd.print(vRMS);
  lcd.print("V");

  //mostra consumo
  lcd.setCursor(9,1);
  lcd.print("Cons:");


  //mostra fatorPotencia
  lcd.setCursor(9,1);
  lcd.print("FP:");
  lcd.print(fp);

}

void limparDisplay(){
  //limpa o display
  lcd.setCursor(0,0);
  lcd.print("                ");
  lcd.setCursor(0,1);
  lcd.print("                ");
}

void ajustaBrilho(){
  //ajusta o brilho
  analogWrite(10,150);
}




