//////////////////////////////////////////////////////////
// CONTROL Y MONITORIZACIÓN DE UN INVERNADERO A TRAVÉS//// 
// DE UNA APLICACIÓN MÓVI                             ////
// Trabajo Final de Master Ingeniera Electromecanica  ////
// Esp. Mecatronica                                   ////
// Departamento de Electronica, Automatica e Infor.   ////
// Indusrial.                                         ////
// Escuela Tec. Superior de Ing. y Diseño Ind.        ////
// Universidad Politecnica de Madrid                  ////
// Alumno: ANDRES BARROSO GARCIA                      ////
// NºMatricula: 2053                                  ////
// Curso: 2014-2015                                   ////
// Fecha: 13-07-15                                    ////
//////////////////////////////////////////////////////////
#include <DHT.h>

#define T0 18.00 //Cota inferior rango temperatura plantacion
#define T1 22.00 //Cota superior rango temperatura plantacion
#define TEMP_MAX 32.00 //Valor maximo de temperatura
#define TEMP_MIN 14.00 //Valor minimo de temperatura
#define H0 45.00 //Cota inferior rango humedad plantacion
#define H1 55.00 //Cota superior rango humedad plantacion
#define HUM_MAX 70.00 //Valor maximo de humedad
#define HUM_MIN 30.00 //Valor minimo de humedad
#define CONTAMINACION 50 //Valor limite de pureza del aire

//Pines Arduino
#define LED_POWER 13//Panel de control
#define LED_TEMP 8
#define LED_HUM 7
#define LED_GAS 10
#define SWITCH 2
#define LED_SPEED_1 3//Panel velocidad ventilador
#define LED_SPEED_2 4
#define LED_SPEED_3 5
#define LED_SPEED_4 6
#define ZUMB 9//Zumbador Alarma
#define VENT 11//Ventilador
#define SONDA 12//Sensor Temperatura-Humedad
#define LAMP 1//Sistema Calefaccion
#define HUM 19//Sistema Humidificaccion
#define GAS A0//Sensor analogico gas
#define INTERRUP_GAS 0//Pin interrupcion sensor gas

boolean DANGER=false;//ESTADO PELIGRO AIRE
int aux_vent;//Variable aux velocidad ventilador

boolean PAUSE=false;//ESTADO PAUSA SISTEMA

boolean SwAndroid=false;
boolean pauseAndroid=false;

boolean pauseSwitch=false;

///////////////////////////////////////////
////////////CLASE VENTILADOR///////////////
///////////////////////////////////////////
// Control del periferico de renovacion de/
// aire
///////////////////////////////////////////

class Ventilador{
  private:
    int pin;
    int estado;//Modo funcionamiento
  public:
    Ventilador(int );
    void SetEstado(int e){estado=e;};//Establece estado  
    int GetEstado(){return estado;};//Retorna estado
    void SetVentilador();//Establece velocidad{0,1,2,3,4}
    void Monitor();//Salida por pantalla
    void ToServer();//Comunicacion con Server
};

Ventilador::Ventilador(int p){

  pin=p;
  estado=0;  
  pinMode(pin,OUTPUT);
}

void Ventilador::SetVentilador(){
  
  switch(estado){    
    case 0: analogWrite(pin,0);break;    
    case 1: analogWrite(pin,95);break;    
    case 2: analogWrite(pin,145);break;
    case 3: analogWrite(pin,195);break;
    case 4: analogWrite(pin,255);break;
    default: break;
  }
}

void Ventilador::Monitor(){
  
  Serial.print("Ventilador....");
  Serial.println(estado);
}

void Ventilador::ToServer(){

  Serial.print("V");
  Serial.print(estado);
}

///////////////////////////////////////////
////////////CLASE CALEFACCION//////////////
///////////////////////////////////////////
// Control de las luminarias que aportan
// calor al invernadero
///////////////////////////////////////////

class Calefaccion{
  private:
    int pin;
    boolean estado;
  public:  
    Calefaccion(int );
    void SetCalefaccion(boolean );//ON/OFF Lamparas
    void Monitor();//Salida por pantalla
    void ToServer();//Comunicacion con Server
};

Calefaccion::Calefaccion(int p){

  pin=p;
  pinMode(pin,OUTPUT);
  estado=false;
}

void Calefaccion::SetCalefaccion(boolean a){
  
  if(a)
    digitalWrite(pin,HIGH);

  else 
    digitalWrite(pin,LOW);

  estado=a; 
}

void Calefaccion::Monitor(){

  Serial.print("SISTEMA CALEFACCION: ");
  Serial.println(estado);  
}

void Calefaccion::ToServer(){

  Serial.print("C");
  Serial.print(estado);  
}

///////////////////////////////////////////
////////////CLASE HUMIDIFICADOR//////////////
///////////////////////////////////////////
// Control del actuador que aporta humedad
// al invernadero
///////////////////////////////////////////

class Humidificador{
  private:
    int pin;
    boolean estado;
  public:  
    Humidificador(int );
    void SetHumidificador(boolean );//ON/OFF Humidificador
    void Monitor();//Salida por pantalla
    void ToServer();//Comunicacion con Server
};

Humidificador::Humidificador(int p){

  pin=p;
  pinMode(pin,OUTPUT);
  estado=false;
}

void Humidificador::SetHumidificador(boolean a){
  
  if(a)
    digitalWrite(pin,HIGH);

  else 
    digitalWrite(pin,LOW);

  estado=a; 
}

void Humidificador::Monitor(){

  Serial.print("SISTEMA HUMIDIFICACION: ");
  Serial.println(estado);  
}

void Humidificador::ToServer(){

  Serial.print("W");
  Serial.print(estado);  
}

///////////////////////////////////////////
///////////////CLASE PANEL/////////////////
///////////////////////////////////////////
///////////////////////////////////////////
// Control del encendido/apagado de leds,
// interruptores y zumbadores
///////////////////////////////////////////

class Panel{//Control de iluminacion de LED, sonido zumbador y pulsador
  private:
    int ledTemp,ledHum,ledGas,ledPower;//LED Alarmas
    int ledV1,ledV2,ledV3,ledV4;//LED Velocidad Ventilador
    int sw,estadoSw;//Switch
    int sound;boolean aux;//Zumbador
    boolean alarmas[3];//Vector estado Alarmas
  public:
    Panel(int ,int ,int ,int ,int ,int ,int ,int ,int ,int );
    void SetLed(int ,boolean );//ON/OFF Led
    boolean GetSw(){return(digitalRead(sw));};
    void Ventilador(int );//Control LED Ventilador
    void Alarm(boolean []);//Control LED Alarmas
    void Pause();//OFF panel
    void Monitor();//Salida por pantalla
    void ToServer();//Comunicacion con Server
};

Panel::Panel(int t,int h,int g,int p,int s,int z,int v1,int v2,int v3, int v4){
  
  ledTemp=t;
  ledHum=h;
  ledGas=g;
  ledPower=p;
  sw=s;
  sound=z;aux=true;
  ledV1=v1;ledV2=v2;ledV3=v3;ledV4=v4;
  
  for(int i=0;i<3;i++)
    alarmas[i]=false;
  
  pinMode(ledTemp,OUTPUT);
  pinMode(ledHum,OUTPUT);
  pinMode(ledGas,OUTPUT);
  pinMode(sw,INPUT);
  pinMode(sound,OUTPUT);
  pinMode(ledV1,OUTPUT);
  pinMode(ledV2,OUTPUT);
  pinMode(ledV3,OUTPUT);  
  pinMode(ledV4,OUTPUT);
}

void Panel::SetLed(int l,boolean a){
  digitalWrite(l,a);
}  

void Panel::Ventilador(int x){

  SetLed(ledV1,false);//OFF 
  SetLed(ledV2,false);
  SetLed(ledV3,false);  
  SetLed(ledV4,false);
  
  switch(x){//ON LED Velocidad actual
    case 0:break;
    case 1:SetLed(ledV1,true);break;
    case 2:SetLed(ledV2,true);break;
    case 3:SetLed(ledV3,true);break;
    case 4:SetLed(ledV4,true);break;
    default: break;
  }
  
}

void Panel::Alarm(boolean v[]){

  for(int i=0;i<3;i++)
    alarmas[i]=v[i];
  
  boolean acoustic=false;
  
  SetLed(ledTemp,LOW);
  SetLed(ledHum,LOW);
  SetLed(ledGas,LOW);
  

  if(v[0])SetLed(ledTemp,HIGH);//ALARMA TEMPERATURA   
  if(v[1])SetLed(ledHum,HIGH);//ALARMA HUMEDAD   
  if(v[2])SetLed(ledGas,HIGH);//ALARMA GAS   
  
  for(int i=0;i<=sizeof(v);i++)//Deteccion de alguna alarma
    acoustic+=v[i];

  //CONTROL ZUMBADOR
  if(acoustic){//Sonido si hay alguna alarma activa
    if(aux){//Sonido intermitente Zumbador. Duracion:1ciclo
      tone(sound,256);
      aux=false;
      }
    else{
      noTone(sound);
      aux=true;
      } 
  }
  else
    noTone(sound); 
  
}

void Panel::Pause(){
  
  SetLed(LED_SPEED_1,LOW);//OFF 
  SetLed(LED_SPEED_2,LOW);
  SetLed(LED_SPEED_3,LOW);  
  SetLed(LED_SPEED_4,LOW);
  SetLed(ledTemp,LOW);
  SetLed(ledHum,LOW);
  SetLed(ledGas,LOW);  
  SetLed(ledPower,LOW);
  noTone(sound); 
  
}

void Panel::Monitor(){
  //Envio de vector estados Alarmas
  Serial.println("ALARMAS");
  Serial.print("Temp: ");
  Serial.println(alarmas[0]);
  Serial.print("Hum: ");
  Serial.println(alarmas[1]);
  Serial.print("Gas: ");
  Serial.println(alarmas[2]);
}

void Panel::ToServer(){

  Serial.print("P");
  for(int i=0;i<3;i++)//Envio de vector estados pin
    Serial.print(alarmas[i]);
}

///////////////////////////////////////////
///////////////CLASE SONDA/////////////////
///////////////////////////////////////////
///////////////////////////////////////////
// Control del sensor temperatura/humedad
///////////////////////////////////////////

class Sonda{
  private:
    int pin;
    float temp,hum;//Variables contenedor
    DHT *dht;//Sensor Humedad Temperatura
  public: 
    Sonda(int ); 
    float CheckTemperature(){return temp=dht->readTemperature();}
    float CheckHumidity(){return hum=dht->readHumidity();}  
    boolean Monitor();//Salida por pantalla
    boolean ToServer();//Comunicacion con Server
};

Sonda::Sonda(int p){
  dht=new DHT(pin,DHT22);//Modelo DHT22 Sensor
  pin=p;
  temp=hum=0;
}

boolean Sonda::Monitor(){
  
  if (isnan(hum) || isnan(temp)){
    Serial.println("Failed to read from DHT sensor!");
    return false;
  }
 
    Serial.print("Humidity: "); 
    Serial.print(hum);
    Serial.print(" %\t");
    Serial.print("Temperature: "); 
    Serial.print(temp);
    Serial.println(" *C ");
    return true;
}

boolean Sonda::ToServer(){

  if (isnan(hum) || isnan(temp)){
    Serial.println("Failed to read from DHT sensor!");
    return false;
  }
  
  char t[10],h[10];
     
  Serial.print("T");
  dtostrf(temp ,5,2,t);//Ajuste del float al formato de envio XX.xx     
  Serial.print(t);
         
  Serial.print("H");//Hxx.xx
  dtostrf(hum ,5,2,h);      
  Serial.print(h);

}

///////////////////////////////////////////
////////////CLASE DETECTOR/////////////////
///////////////////////////////////////////
///////////////////////////////////////////
// Control del sensor de gas
///////////////////////////////////////////

class Detector{
  private:
    int pin;
    int val;//Variable contenedor
  public:
    Detector(int );
    int Check(){return val=map(analogRead(pin),50,512,0,100);}
    void Monitor();//Salida por pantalla
    void ToServer();//Comunicacion con Server
};

Detector::Detector(int p){

  pin=p;
  pinMode(pin,INPUT); 
  val=0; 
}

void Detector::Monitor(){

  Serial.print("Estado del aire...");
  Serial.println(val);
}

void Detector::ToServer(){

  char a[10];
  
  Serial.print("A");
  dtostrf(val,3,0,a);//Ajuste del int al formato de ajuste XXX    
  Serial.print(a);
}
///////////////////////////////////////////
////////////CLASE INVERNADERO//////////////
///////////////////////////////////////////
///////////////////////////////////////////
// Control de los sensores/actuadores del 
// proyecto ICRA
///////////////////////////////////////////

class Invernadero{
  private:
    float temp,hum,air;//Variables estado 
    float t0,t1,h0,h1;//Rangos de trabajo
    float tm,tM,hm,hM;//Limites min/MAX de alarmas    
    boolean ALARMAS[3];//Vector alarmas {TEMP,HUM,GAS}  
    //PERIFERICOS
    Ventilador *ventilador;//Ventilador
    Calefaccion *calefaccion;//Sistema calefaccion
    Humidificador *humidificador;//Sistema humedad
    Panel *panel;//Panel de control
    Sonda *sonda;//Sonda de temperatura/humedad
    Detector *gas;//Detector de gas inflamable
  public:    
    Invernadero();
    void Inicializa();//Puesta en marcha del sistema
    void SetAlarms(float ,float ,float ,float );//Definir umbral alarmas
    void SetLimits(float, float ,float ,float );//Definir zona trabajo
    void Check();//Control del invernadero
    void Adq();//Adquisicion valor sensores
    void Pause();//Estado PARO
    void SerialPort();//Comunicacion serie
    void Monitor();//Salida por pantalla
    void FromServer();//Recepcion variables usuario
    void ToServer();//Envio estado de variables
};

Invernadero::Invernadero(){

  t0=T0;
  t1=T1;
  h0=H0;
  h1=H1;
  
  tm=TEMP_MIN;
  tM=TEMP_MAX;
  hm=HUM_MIN;
  hM=HUM_MAX;  
  
  for(int i=0;i<3;i++)
    ALARMAS[i]=0;
      
  ventilador=new Ventilador(VENT);
  calefaccion=new Calefaccion(LAMP);
  humidificador=new Humidificador(HUM);
  panel=new Panel(LED_TEMP,LED_HUM,LED_GAS,LED_POWER,SWITCH,ZUMB,LED_SPEED_1,
  LED_SPEED_2,LED_SPEED_3,LED_SPEED_4);
  sonda=new Sonda(SONDA);
  gas=new Detector(GAS);
  temp=0;
  hum=0;
  air=0;
}

void Invernadero::Inicializa(){
  
  Serial.begin(9600);
  delay(5000);
  
  Serial.println("Checking componentes...");
  
  calefaccion->SetCalefaccion(true);
  Serial.println("...");
  delay(1000);
  panel->SetLed(LED_TEMP,true);
  Serial.println("...");
  delay(1000);
  panel->SetLed(LED_HUM,true);
  Serial.println("...");
  delay(1000);
  panel->SetLed(LED_GAS,true);
  Serial.println("...");
  delay(1000);
  panel->SetLed(LED_POWER,true);
  Serial.println("...");
  delay(1000);  
 
  panel->SetLed(LED_SPEED_1,true);
  Serial.println("...");
  delay(1000);  
  
  panel->SetLed(LED_SPEED_2,true);
  Serial.println("...");
  delay(1000);  
  
  panel->SetLed(LED_SPEED_3,true);
  Serial.println("...");
  delay(1000);  
  
  panel->SetLed(LED_SPEED_4,true);
  Serial.println("...");
  delay(1000);  
  
  ventilador->SetEstado(4);
  ventilador->SetVentilador();
  Serial.println("...");
  delay(1000);

  calefaccion->SetCalefaccion(false);
  panel->SetLed(LED_HUM,false);
  panel->SetLed(LED_TEMP,false);
  panel->SetLed(LED_GAS,false);
  panel->SetLed(LED_SPEED_1,false);
  panel->SetLed(LED_SPEED_2,false);
  panel->SetLed(LED_SPEED_3,false);
  panel->SetLed(LED_SPEED_4,false);
  
  ventilador->SetEstado(aux_vent=0);//Ventilador en velocidad defecto
  ventilador->SetVentilador();
  calefaccion->SetCalefaccion(true);//Arranca Calefaccion ON
  humidificador->SetHumidificador(true);//Arranca Humidificador ON
  
  attachInterrupt(2,danger, FALLING);//Rutina atencion GAS
  attachInterrupt(1,pause, HIGH);//Rutina atencion SWITCH
}

void Invernadero::SetAlarms(float tmin, float tmax, float hmin, float hmax){

  tm=tmin;
  tM=tmax;
  hm=hmin;
  hM=hmax;

}

void Invernadero::SetLimits(float t,float T,float h,float H){

  t0=t;
  t1=T;
  h0=h;
  h1=H;
}

void Invernadero::Check(){

  //CONTROL TEMPERATURA

  //Regulacion valores ideales 
  if(temp<=t0)
    calefaccion->SetCalefaccion(true);
  if(temp>=t1)
    calefaccion->SetCalefaccion(false);  
  
  //Control valores peligrosos  
  if(temp>=tM||temp<=tm)
    ALARMAS[0]=true;    
  else
    ALARMAS[0]=false;
    
  //CONTROL HUMEDAD
  
  //Regulacion valores ideales 
  if(hum<=h0)
    humidificador->SetHumidificador(true);
  if(hum>=h1)
    humidificador->SetHumidificador(false);    
  
  //Control valores peligrosos 
  if(hum>=hM||hum<=hm)
    ALARMAS[1]=true;    
  else
    ALARMAS[1]=false;
  
  //CONTROL GAS
  
  if(DANGER){//Estado de PELIGRO
  
    ventilador->SetEstado(4);//Maxima evacuacion de aire
    
    if(air<CONTAMINACION)//Control fin DANGER
    {
      DANGER=false;//Fin ventilacion maxima
      ventilador->SetEstado(aux_vent);//Ventilacion previa  
      }
   }
   
  ALARMAS[2]=DANGER;//Asignacion en el vector de ALARMAS 
   
  //CONTROL PANEL
  
  panel->Ventilador(ventilador->GetEstado());//Indicadores velocidad
  panel->Alarm(ALARMAS);//Envio de estados al panel
      
  //CONTROL VENTILADOR    
  ventilador->SetVentilador();
}

void Invernadero::Adq(){
  
  temp=sonda->CheckTemperature();
  hum=sonda->CheckHumidity();
  air=gas->Check();

}

void Invernadero::Pause(){
  
  ventilador->SetEstado(0);//OFF actuadores
  ventilador->SetVentilador();
  calefaccion->SetCalefaccion(false);
  humidificador->SetHumidificador(false);
  
  panel->Pause();//OFF panel
  
  if((panel->GetSw()==0)&&pauseSwitch)
     pauseSwitch=false;  

  if((SwAndroid==0)&&pauseAndroid)
     pauseAndroid=false;
  
  if(pauseSwitch==0&&pauseAndroid==0){
    PAUSE=false;
    ventilador->SetEstado(aux_vent);//Ventilacion previa
    panel->SetLed(LED_POWER,HIGH); 
  }
}

void Invernadero::SerialPort(){
  
  //Monitor();//Herramientas->Monitor Serial
  
  if (Serial.available() > 0) {
  // read the incoming byte:
    char r= Serial.read();
      if(r=='y'){//Servidor envia ajuste Android
        FromServer(); 
        
        char r2=Serial.read();
        if(r2=='z')//Servidor solicita datos tras ajuste Android
          ToServer();
      }      
      else if(r=='z')//Servidor solicita datos
        ToServer();
      else if(r=='x'){//Servidor solicita estado de ajuste   
        Serial.print("a");Serial.print(t0);
        Serial.print("b");Serial.print(t1);
        Serial.print("c");Serial.print(tm);
        Serial.print("d");Serial.print(tM);
        Serial.print("e");Serial.print(h0);
        Serial.print("f");Serial.print(h1);
        Serial.print("g");Serial.print(hm);
        Serial.print("h");Serial.print(hM); 
        Serial.print("i");Serial.print(ventilador->GetEstado());
        Serial.print("j");Serial.print(SwAndroid);         
      }     
  }
}

void Invernadero::Monitor(){

  ventilador->Monitor();
  sonda->Monitor();
  gas->Monitor();
  calefaccion->Monitor();
  humidificador->Monitor();
  panel->Monitor();
  Serial.println("********************");
}

void Invernadero::ToServer(){

       sonda->ToServer();       
       gas->ToServer();       
       ventilador->ToServer();
       calefaccion->ToServer();
       humidificador->ToServer();
       panel->ToServer();
       Serial.print("S");
       Serial.print(PAUSE);
}

void Invernadero::FromServer(){
       //RECEPCION valores usuario
        String CMD="axx.xxbxx.xxcxx.xxdxx.xxexx.xxfxx.xxgxx.xxhxx.xxixjx";//Formato recepcion
        char c[CMD.length()];//buffer
        char _t0[10],_t1[10],_T0[10],_T1[10];
        char _h0[10],_h1[10],_H0[10],_H1[10];
        char _v[5],_s[5];//variables aux
        char *p;//puntero recepcion
        int ii=0;//indice puntero
         
        Serial.readBytes(c,CMD.length());//(buffer,tamaño)       
       
        for(int i=0;i<CMD.length();i++){//segmentacion buffer  
          
          if(c[i]=='a'){
            ii=0;
            p=_t0;
          }
          else if(c[i]=='b'){
            ii=0;
            p=_t1;
          }
          else if(c[i]=='c'){
            ii=0;
            p=_T0;
          }
          else if(c[i]=='d'){
            ii=0;
            p=_T1;
          }
          /////////////////////
          
          else if(c[i]=='e'){
            ii=0;
            p=_h0;
          }
          else if(c[i]=='f'){
            ii=0;
            p=_h1;
          }
          else if(c[i]=='g'){
            ii=0;
            p=_H0;
          }
          else if(c[i]=='h'){
            ii=0;
            p=_H1;
          } 
          
          ////////////////////////
          
          else if(c[i]=='i'){
            ii=0;
            p=_v;
          }
          else if(c[i]=='j'){
            ii=0;
            p=_s;
          }
          
          ////////////////////////
          
          else{          
            p[ii++]=c[i];
            p[ii]='\0';         
          }          
        }
        
        SetAlarms(atof(_T0),atof(_T1),atof(_H0),atof(_H1));
        SetLimits(atof(_t0),atof(_t1),atof(_h0),atof(_h1));
        ventilador->SetEstado(aux_vent=atof(_v));
        SwAndroid=atof(_s);        
       
        if(SwAndroid){
          PAUSE=pauseAndroid=true;
        } 
}

//MAIN PROGRAM
Invernadero invernadero;

void setup(){

  invernadero.Inicializa();
}

void loop(){

  invernadero.Adq();//Refresh sensores
  
  if(pauseSwitch||pauseAndroid) 
    invernadero.Pause();//Paro  
  else
    invernadero.Check();//Gestion actuadores     

  invernadero.SerialPort();//Comunicacion serie

  delay(1000); 
}

///////////////////////////////////////////
//SUBRUTINAS DE ATENCION INTERRUPCIONES////
///////////////////////////////////////////

void danger(){//Rutina atencion GAS
   
  DANGER=true;//Estado de PELIGO
  analogWrite(VENT,255);//ON inmedato de Ventilacion MAX
}

void pause(){//Rutina atencion SWITCH

    PAUSE=pauseSwitch=true;
    digitalWrite(LAMP,LOW);//OFF inmediato de actuadores de potencia
    digitalWrite(VENT,LOW);
    digitalWrite(HUM,LOW);
}
