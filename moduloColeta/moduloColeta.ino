#include <TimerOne.h>
#include <LiquidCrystal.h>
#include <Ethernet.h>
#include <SPI.h>

//define o numero de amostras de cada captura
#define AMOSTRAS 64
//define a frequencia da rede (pode ser 50)
#define FREQ 60
// define os limites superior e infoerir para deteçcao de pico de tensao
#define TENSAO_LIMIT_S 640
#define TENSAO_LIMIT_I 400


// initialize the library with the numbers of the interface pins
//http://www.hobbytronics.co.uk/arduino-lcd-keypad-shield
// usando esta pinagem para arduinoMEga
LiquidCrystal lcd(41, 40, 33, 32, 31, 30);


//variaveis para medida de tempo
unsigned long microseconds,duration,T,tAmostra, tAmostraMinusADTime;
unsigned int vetorA[AMOSTRAS],vetormA[AMOSTRAS],vetorV[AMOSTRAS];
float vetorASemDC[AMOSTRAS],vetormASemDC[AMOSTRAS],vetorVSemDC[AMOSTRAS];
unsigned int timeOffset = 0;

uint16_t tempoLoop = 250,tmpVar;
byte sensorA = A1 ,sensormA = A0, sensorV = A3,led = 13;
boolean estadoLed = true,mAdbg=false,Adbg=false,Vdbg=false,Sdbg=false,rmsTestdbg=false,overflowDBG=false;
char cmd,tmpBuf[9];
//calibraço feita em 1/9/15 com multimetro do prof. Trentin
//utilizando rotina switch/case para ajuste fino de cada um dos ganhos.
//float ganhoA=8.52,ganhomA=156,ganhoV=79850;//ganhoV=52250;
float ganhoA=9,ganhomA=150,ganhoV=75850;//ganhoV=52250; //CHANGE IN 6/11/15

//inicializa com variavel rmsAnterior com corrente mnima
float rmsAnterior=0.001;
boolean houveAlteracaoRMS=false;
//variaveis para controle e teste das diferenças
byte numVezesDiferente=0,limiteDiferencas=3;

//http://www.microsmart.co.za/technical/2014/03/01/advanced-arduino-adc/
// Define various ADC prescaler
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

//inicializaao socket de rede
byte mac[] = { 
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
  //meu IP
IPAddress ip(172,20,6,254);
// ip do servidor
IPAddress server(172,20,6,41);

EthernetClient client;

void setup(){
  Serial.begin(9600);
  pinMode(sensorA, INPUT);     
  pinMode(sensormA, INPUT);     
  pinMode(sensorV, INPUT);     

  initDisplay();
  ajustaBrilho();


  // set up the ADC
  analogReference(EXTERNAL); //usando uma referncia externa - de 3.3V
  ADCSRA &= ~PS_128;  // remove bits set by Arduino library
  // you can choose a prescaler from above.
  // PS_16, PS_32, PS_64 or PS_128
  ADCSRA |= PS_32;    // set our own prescaler here

  //este teste define o periodo do ciclo conforme a frequencia da rede
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

  //mostra valores iniciais...
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

  //ativaçao da rede
  Ethernet.begin(mac, ip);
  delay(100);
}


void loop(){
  //verifica se servidor mandou algo para o mim
  if(client.available()){
    char c = client.read();
    Serial.print(c);
  }

  //verifica se o firmware recebeu algum comando via serial (debugs e calibraço)
  if(Serial.available()){
    cmd = Serial.read();

    
    switch(cmd){
    case '+':
      ganhoV+=1000;
      Serial.println(ganhoV);
      break;
    case '-':
      ganhoV-=1000;
      Serial.println(ganhoV);
      break;
    case 'a':
      mAdbg = !mAdbg;
      break;
    case 'b':
      Adbg = !Adbg;
      break;
    case 'c':
      Vdbg = !Vdbg;
      break;
    case 's':
      Sdbg = !Sdbg;
      break;
    case 't':
      rmsTestdbg = !rmsTestdbg;
      break;
    case 'm':
      //chama função memoriaLivre()
      Serial.print("memoria livre: ");
      Serial.println(memoriaLivre(),DEC);
      break;
    case 'o':
      overflowDBG = !overflowDBG;
      break;
    default:
      Serial.println("Ativar/desativar debug a(mA), b(A), c(V), m (mem livre), s(Serial),t(alteracaoRMS),+/- (ajuste ganho), o(overflow)");
      break;  
    }
  }

  fazLeitura();
  
  testaSobreTensao();

  delay(1000);
}

//testar se houve picos de tensao - implementado em 23/9
void testaSobreTensao(){
  //percorrer o vetor buscando valores acima de PIC_SUP_T e abaixo PIC_INF_T
  for(uint16_t i=0; i<AMOSTRAS; i++){
    if (vetorV[i] >= TENSAO_LIMIT_S || vetorV[i] <= TENSAO_LIMIT_I){
      //o a eh de artefato (nome cientificio da sobretensao)
      sendtoSocket(3, vetorV, 2, vetorA,'a');
      Serial.print("Detectado sobre tensao");
       break;
    }
  }
}

//executado por fora do loop
void timerIsr(){
  piscaLed();
}


void fazLeitura(){
  unsigned int accV=0,accA=0,accmA=0,CorrmARMS=0,VoltsRMS=0;
  float mediaV=0.0,mediaA=0.0,mediamA=0.0,CorrRMS=0,accVflt=0,accAflt=0,accmAflt=0;

  //laço para coleta dos valores 
  // este laço faz a leitura sequncial dos tres sensores da rede..
  // o primeiro para correntes acima de 1A
  // o segundo para correntes abaixo de 1A
  // o terceiro para tensao
  // as variaveis acc* so responsaveis por acumular o valor de cada leitura (sera usado mais tarde para remocao do nivel DC)
  for(uint16_t i=0; i<AMOSTRAS; i++){
    vetorA[i] = analogRead(sensorA);
    vetormA[i] = analogRead(sensormA);
    vetorV[i] = analogRead(sensorV);
    accA += vetorA[i];
    accmA += vetormA[i];
    accV += vetorV[i];
    //este delay eh necessario para ajustar o tempo de cada captura [ este tempo eh calculado na inicializaço ]
    delayMicroseconds(tAmostraMinusADTime);
  }
  //calcula a media de cada vetor para remoçao do nivel DC
  mediaA = (float)accA / AMOSTRAS;
  mediamA = (float)accmA / AMOSTRAS;
  mediaV = (float)accV / AMOSTRAS;  

  //remove o nivel DC de cada amostra
  for(uint8_t i=0;i<AMOSTRAS;i++){
    vetorASemDC[i] = (float)vetorA[i] - mediaA;
    vetormASemDC[i] = (float)vetormA[i] - mediamA;
    vetorVSemDC[i] = (float)vetorV[i] - mediaV;
  }

  //debug pra leitura do sensor acima de 1A
  if(Adbg == true){
    Serial.print("A,");
    for(uint8_t i=0;i<AMOSTRAS;i++){
      Serial.print(vetorASemDC[i]); 
      Serial.print(",");
    }
    Serial.println();
  }
  //debug pra leitura do sensor abaixo de 1A
  if(mAdbg == true){
    Serial.print("mA,");
    for(uint8_t i=0;i<AMOSTRAS;i++){
      Serial.print(vetormASemDC[i]); 
      Serial.print(",");
    }
    Serial.println();
  }
  //debug pra leitura do sensor de tensao  
  if(Vdbg == true){
    Serial.print("V,");
    for(uint8_t i=0;i<AMOSTRAS;i++){
      Serial.print(vetorVSemDC[i]); 
      Serial.print(",");
    }
    Serial.println();
  }
  //debug pra overflow (nao sei o que e isso)
  if(overflowDBG == true ){
    Serial.println("Tensao: ");
    for(uint16_t i=0; i<AMOSTRAS; i++){
      Serial.print(vetorV[i]);
      Serial.print(",");
    }
  }

  //somatorio dos quadrados
  //para calculo RMS
  for(uint8_t i=0;i<AMOSTRAS;i++){
    accAflt += (vetorASemDC[i] * vetorASemDC[i]);
    accmAflt += (vetormASemDC[i] * vetormASemDC[i]);    
    accVflt += (vetorVSemDC[i] * vetorVSemDC[i]);
  }

  //raiz quadrada do somatorio dos quadrados peloa raiz do numero de amostras
  CorrRMS = sqrt(accAflt/AMOSTRAS)/ganhoA;
  CorrmARMS = (sqrt(accmAflt/AMOSTRAS)/ganhomA)*1000; //multiplica por 1000 para mostrar em mA
  VoltsRMS = (sqrt(accVflt/AMOSTRAS)/ganhoV)*224900; //divide pelo valor do resistor para encontrar a tenso
 
  //se Sdbg is true envia valores RMS pela serial tmb.
  if(Sdbg == true){
    Serial.print(CorrRMS);
    Serial.print(" A, ");
    Serial.print(CorrmARMS);
    Serial.print(" mA, ");
    Serial.print(VoltsRMS);
    Serial.println(" V");

  }
  //SE o RMS calculado  maior que 999mA descarta a medida e assume  sensor de alta amperagem.
  //imprime a coisa
  if(CorrmARMS > 1000){
    atualizaDisplay(CorrRMS,VoltsRMS, 0.86);
    testaAlteracaoRMS(CorrRMS,VoltsRMS);
  }
  else{
    atualizaDisplaymA(CorrmARMS,VoltsRMS, 0.86);
    testaAlteracaomARMS(CorrmARMS,VoltsRMS);
  }
}

// o limite para teste de mA de 0.10A
// esta funçao serve para detectar se houve alteraçao do valor RMS
// o teste  aplicado a cada 3 leituras (limiteDiferencas=3;)
// se 3 leituras de corrente tiverem uma diferença maior que 100mA entao temos um evento
void testaAlteracaoRMS(float newRMS, float volts){
  //Quando teste der 'positivo' n vezes zera contador e envia esta amostra
  float dif = newRMS - rmsAnterior;
  if(abs(dif) > 0.10){
    numVezesDiferente++;
    if(rmsTestdbg == true){
      Serial.println(numVezesDiferente);
    }
    //se testou n vezes e deu diferença entao tem que alterar.
    if(numVezesDiferente >= limiteDiferencas){
      numVezesDiferente=0;
      rmsAnterior = newRMS;
      sendtoSocket(3, vetorV, 2, vetorA,testaDif(dif));
      Serial.print("Detectado um evento");

      if(rmsTestdbg == true){
        Serial.println("RMS Alterado");
      }
    }
  }
  else{
    numVezesDiferente=0;
  }
}


// idem para funcao anterior mas limite para teste de mA eh de 50mA 
void testaAlteracaomARMS(float newRMS, float volts){
  //Quando teste der 'positivo' n vezes zera contador e envia esta amostra
  float dif = newRMS - rmsAnterior;
  if(abs(dif) > 50){
    numVezesDiferente++;
    if(rmsTestdbg == true){
      Serial.println(numVezesDiferente);
    }
    //se testou n vezes e deu diferença entao tem que alterar.
    if(numVezesDiferente >= limiteDiferencas){
      numVezesDiferente=0;
      rmsAnterior = newRMS;
      sendtoSocket(3, vetorV, 1, vetormA,testaDif(dif));  

      if(rmsTestdbg == true){
        Serial.println("RMS Alterado");
      }
    }
  }
  else{
    numVezesDiferente=0;
  }
}

//esta funçao apenas retorna se o evento foi ligar (l) ou desligar(d)
char testaDif(float x){
  if (x > 0) return 'l';
  else if (x < 0) return 'd';
}

//esta versao envia 2 vetores (tensao e corrente)
//funçao socekt tcp
// os parametros sao: (numero do sensor, vetor de tensao, numero sensor corrente, vetor corrente, tipo evento) 
void sendtoSocket(byte vSensor, unsigned int vToSend[AMOSTRAS], byte iSensor, unsigned int iToSend[AMOSTRAS], char evento){
  if(client.connect(server,10002)){
    Serial.println("-> Conectado.");
    //retirei o envio do numero do sensor de tensao [para esta versao eh sempre o mesmo]
    //sprintf(tmpBuf,"%d:",vSensor);
    //client.print(tmpBuf); //envia nome do sensor
    client.print(evento); //envia tipo do evento pelo socket
    client.print(":"); //envia separador pelo socket
    
    //percore o vetor de tensao para enviar todo pelo socket
    for(uint8_t i=0; i<AMOSTRAS; i++){
      sprintf(tmpBuf,"%d,",vToSend[i]); //converte valor da posiçao para char*
      client.print(tmpBuf); // envia pelo socket
    }
    sprintf(tmpBuf,":%d:",iSensor); ////converte numero do sensor para char* (concatena com separador :)
    client.print(tmpBuf); //envia pelo socket
    //percore o vetor de corrente para enviar todo pelo socket
    for(uint8_t i=0; i<AMOSTRAS; i++){
      sprintf(tmpBuf,"%d,",iToSend[i]); //converte valor da posiçao para char*
      client.print(tmpBuf); // envia pelo socket
    }
  }
  else{
    // mensagem de erro se no conectar ao servidor..
    Serial.println("-> Falha de conexao.");
    
  }
  client.stop(); //encerra a comunicaçao
}

//FUNCAO PARA PISCAR MOSTRANDO QUE ESTA VIVO
void piscaLed(){
  estadoLed = !estadoLed;
  digitalWrite(led, estadoLed);
}


//função que 'quantifica' ou 'mede' a memoria livre
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


//**********************************
//************* funcoes para display
//**********************************
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
  lcd.setCursor(0,1);
  lcd.print("                ");
  
  
  //mostra corrente
  lcd.setCursor(6,1);
  lcd.print("/I=");
  lcd.print(iRMS);
  lcd.print("A");

  //mostra tensao
  lcd.setCursor(0,1);
  lcd.print("V=");
  lcd.print(vRMS);
  lcd.print("V");

  ////mostra consumo
  //lcd.setCursor(9,1);
  //lcd.print("Cons:");


  //mostra fatorPotencia
  //lcd.setCursor(9,1);
  //lcd.print("FP:");
  //lcd.print(fp);
}

void atualizaDisplaymA(int iRMS,unsigned int vRMS, float fp){
  lcd.setCursor(0,1);
  lcd.print("                ");
  //mostra corrente
  lcd.setCursor(6,1);
  lcd.print("/I=");
  lcd.print(iRMS);
  lcd.print("mA");

  //mostra tensao
  lcd.setCursor(0,1);
  lcd.print("V=");
  lcd.print(vRMS);
  lcd.print("V");

  //mostra consumo
  //lcd.setCursor(9,1);
  //lcd.print("Cons:");

  //mostra fatorPotencia
  // desativado em 14/1 (fotos dissertaçao)
  //lcd.setCursor(9,1);
  //lcd.print("FP:");
  //lcd.print(fp);

}


void limparDisplay(){
  //limpa o display
  lcd.setCursor(0,0);
  lcd.print("                ");
  lcd.setCursor(0,1);
  lcd.print("                ");
  
  lcd.setCursor(0,0);
  lcd.print(" Prototipo MCCT ");
}


void ajustaBrilho(){
  //ajusta o brilho
  //analogWrite(10,150);
  //o que fazer aqui?
}


/* O QUE ESTA DAQUI PARA BAIXO NAO ESTA MAIS SENDO USADO
########################################################################################/
//esta versao manda 1 vetor corrente + tensao RMS
void sendToSocket(byte sensor, int volts, unsigned int vetorParaEnviar[AMOSTRAS]){  
  if(client.connect(server,10002)){
    Serial.println("-> Conectado.");
    sprintf(tmpBuf,"%d:",sensor);
    client.print(tmpBuf); //envia nome do sensor
    sprintf(tmpBuf,"%d:",volts); //envia tensao lida (*10) para ir como inteiro
    client.print(tmpBuf); //envia valor de tensao
    //pega o vetor, converte para array de char para envio pelo socket
    for(uint8_t i=0; i<AMOSTRAS; i++){
      sprintf(tmpBuf,"%d,",vetorParaEnviar[i]); //enviando somente o valor
      client.print(tmpBuf);
    }
  }
  else{
    Serial.println("-> Falha de conexao.");
  }
  client.stop();
}

*/






