#include <EEPROM.h>

// Include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal.h>
// Date and time functions uspiing a DS1307 RTC connected via I2C and Wire lib
#include <Wire.h>
#include "RTClib.h"
/******************************* WIRELESS ********************/
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
//SPI: 10 (SS), 11 (MOSI), 12 (MISO), 13 (SCK). 
#define CE_PIN 40
#define CSN_PIN 53 //Since we are using 3 pin configuration we will use same pin for both CE and CSN
RF24 radio(CE_PIN, CSN_PIN);

// initialize the library by providing the nuber of pins to it
LiquidCrystal lcd(6,7,8,9,10,11);
RTC_DS1307 rtc;
// Cable de datos conectado al pin 3
#define ONE_WIRE_BUS 3
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
DateTime now;

#undef HORA_DEBUG

#define TXD   //If you comment this line, the DPRINT & DPRINTLN lines are defined as blank.
#ifdef TXD    //Macros are usually in all capital letters.
  #define TXDEBUG(...)    Serial.print(__VA_ARGS__)     //DPRINT is a macro, debug print
  #define TXDEBUGLN(...)  Serial.println(__VA_ARGS__)   //DPRINTLN is a macro, debug print with new line
#else
  #define TXDEBUG(...)     //now defines a blank line
  #define TXDEBUGLN(...)   //now defines a blank line
#endif

#define DEBUG   //If you comment this line, the DPRINT & DPRINTLN lines are defined as blank.
#ifdef DEBUG    //Macros are usually in all capital letters.
  #define DPRINT(...)    Serial.print(__VA_ARGS__)     //DPRINT is a macro, debug print
  #define DPRINTLN(...)  Serial.println(__VA_ARGS__)   //DPRINTLN is a macro, debug print with new line
#else
  #define DPRINT(...)     //now defines a blank line
  #define DPRINTLN(...)   //now defines a blank line
#endif



#define GRABAR_SD 0
#define CANT  8

#define SET_TARGET     'S'
#define GET_TARGET     'G'
#define SET_CONFIG     'C'
#define GET_CONFIG     'I'
#define GET_TEMPS     'T'

#define WAIT_START    1
#define IN_COMMAND     2
#define IN_MESSAGE      3
#define TRANSMIT       4
#define UNDEFINED       5

long RECEP_STATE = 1;
long comando = 1;

bool habilitados[CANT];
bool fulltime;
bool frio[CANT];    // Guardo si hay que enfriar o calentar
bool calor[CANT];    // Guardo si hay que enfriar o calentar
int salidas_frio[CANT];
int salidas_calor[CANT];
bool estado_salida_frio[CANT];
bool estado_salida_calor[CANT];
bool bomba_frio, bomba_calor;
int cont = 0;
int leidos = 0;
#define PIN_BOMBA_FRIO 23
#define PIN_BOMBA_CALOR 24

long valores[CANT];
float values[CANT];

float temp[CANT+2];
float set[CANT];
int estado = 0;

int interpretar = 0;
unsigned long actual_time, back_time;

byte address[11] = "SimpleNod1";    // Lectura
byte address2[11] = "SimpleNod2";   // Excritura
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };       // Pipe 0 Writing - Pipe 1 Reading
char payload = 0;
String theMessage = ""; //Received Message
String inputString = ""; //Hold incoming Message from Serial
String value_recv = "";
/************************************ END WIRELESS ****************/


// constants won't change. Used here to set a pin number :
const int S11 = 31;      // Salida 1 agua caliente
const int S12 = 33;      // Salida 1 agua fria
const int S21 = 35;      // Salida 2 agua caliente
const int S22 = 37;      // Salida 2 agua fria
const int Button1 = 30;      // Salida 1 agua caliente
const int Button2 = 32;      // Salida 1 agua fria
const int Button3 = 34;      // Salida 2 agua caliente
const int Button4 = 36;      // Salida 2 agua fria
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
const int chipSelect = 25;

struct conf{
    uint8_t frio;       // Utilizo los bits como booleanos
    uint8_t calor;      // Utilizo los bits como booleanos
    uint8_t salida_frio[8];
    uint8_t salida_calor[8];
    uint8_t habilitado; // Utilizo los bits como booleanos
    uint8_t fulltime;
    float temp[2];
    uint8_t cte1;
    uint8_t estado_bombas;
    uint8_t estado_salidas_cold, estado_salidas_hot;
};

struct sensores{
    float temp[8];
};

struct target{
    float set_temp[8];
};

void cargar(uint8_t *destino, bool* origen, uint8_t nbits);
void descargar(uint8_t origen, bool* destino, uint8_t nbits);
void LeerEEPROM();
void GuardarConfigEEPROM();
void GuardarTargetEEPROM();
void checkWaterPumps();

conf st_conf;
sensores st_sens;
target st_target;

unsigned long startTime;
unsigned long elapsedTime;
/*
 * The setup function. We only start the sensors here
 */
void setup(void)
{
  // start serial port
  Serial.begin(9600);
  
  DPRINTLN("Bienvenido!");
  
  // Inicializo los valores.
  for(int i = 0; i < CANT; i++)
  {
    temp[i] = 0.0;
    set[i] = 18.0;
    habilitados[i] = false;
    frio[i] = false;
    calor[i] = true;
    salidas_frio[i] = 0;
    salidas_calor[i] = 0;
    estado_salida_frio[i] = 0;
    estado_salida_calor[i] = 0;
    
  }
  set[1] = 10.0;
  bomba_frio = 0; bomba_calor = 0;
  fulltime = 0;
  // Pines de salidas 
  salidas_frio[0] = 31;
  salidas_frio[1] = 35;
  salidas_calor[0] = 31;
  salidas_calor[1] = 35;
  
 
  // Inicio LCD
  lcd.begin(16,2);
  // set cursor position to start of first line on the LCD
  lcd.clear();

  rtcInit();
  radioInit();

  // Start up the library
  sensors.begin();


  // set the digital pin as output:
  // BOMBAS DE AGUA
  pinMode(PIN_BOMBA_FRIO, OUTPUT);
  pinMode(PIN_BOMBA_CALOR, OUTPUT);
  pinMode(S11, OUTPUT);
  pinMode(S12, OUTPUT);
  pinMode(S21, OUTPUT);
  pinMode(S22, OUTPUT);
  
  digitalWrite(S11, HIGH);
  digitalWrite(S12, HIGH);
  digitalWrite(S21, HIGH);
  digitalWrite(S22, HIGH);
  digitalWrite(PIN_BOMBA_FRIO, HIGH);
  digitalWrite(PIN_BOMBA_CALOR, HIGH);

  // botones de entrada
  pinMode(Button1,INPUT_PULLUP);
  pinMode(Button2,INPUT_PULLUP);
  pinMode(Button3,INPUT_PULLUP);
  pinMode(Button4,INPUT_PULLUP);

  back_time = millis();

  LeerEEPROM();
  resetTime();

/*
  fulltime = true;
  
  set[1] = 18.0;
  habilitados[1] = true;
  frio[1] = false;
  calor[1] = true;
  salidas_frio[1] = 35;
  salidas_calor[1] = 35;
  
  
  set[0] = 19.0;
  habilitados[0] = true;
  frio[0] = false;
  calor[0] = true;
  salidas_frio[0] = 33;
  salidas_calor[0] = 33;
  */
  }
 
/*
 * Main function, get and show the temperature
 */
void loop(void)
{
  
  actual_time = millis();
  if( (actual_time - back_time) >1500)
  {
    back_time = actual_time;
    now = rtc.now();      // Reloj de tiempo real!!!!!!!
    readTemps();
    mostrarHora();
    ControlFunction();
    Display();
  }
  
  transmisor();
}

/* 
#define SET_TARGET     'S'       #define WAIT_START      1
#define GET_TARGET     'G'          #define IN_COMMAND     2
#define SET_CONFIG     'C'          #define IN_MESSAGE      3
#define GET_CONFIG     'I'          #define TRANSMIT       4
#define GET_TEMPS     'T'           #define UNDEFINED       5
º
 */
void transmisor()
{
	bool ok;
	/********************* ESTADO ESPERANDO INICIO *********************/
	if(RECEP_STATE == WAIT_START)
	{
		radio.startListening();
		TXDEBUGLN("ESTADO = WAIT_START");
		if(radio.available())
		{
      resetTime();
			radio.read(&comando, sizeof(comando));
      TXDEBUG("ESTADO = WAIT_START - Comando =");TXDEBUGLN(comando);
			//ok = ComandoValido(comando);
			if(comando == 'G' || comando == 'I' || comando == 'T')
			{
				//radio.flush_tx();
				RECEP_STATE = TRANSMIT;
			}
			else if(comando == 'S' || comando == 'C')
			{
				RECEP_STATE = IN_MESSAGE;
			}
     else
     {
        TXDEBUGLN("Error recibiendo comando - comando invalido");
     }
		}
    else
    {
      TXDEBUGLN("Radio not available - checking time");
      checkTime();
    }
	}
	/********************* ESTADO EN MENSAJE ***************************/
	else if(RECEP_STATE == IN_MESSAGE)
	{
		TXDEBUG("ESTADO = IN_MESSAGE");
		radio.startListening();
		if(radio.available())
		{
      TXDEBUGLN(" - radio Available");
			char len;
			if(comando == SET_TARGET)
			{
				len = radio.getPayloadSize();
				TXDEBUG("SET-TARGET - Longitud del Payload =");TXDEBUGLN(len);
				radio.read(&st_target, sizeof(st_target));
				for(int i = 0; i< CANT; i++)
				{
				  set[i] = st_target.set_temp[i];
         TXDEBUG("Target [");TXDEBUG(i);TXDEBUG("] = ");TXDEBUGLN(set[i]);
				}
        GuardarTargetEEPROM();
			}
			else if(comando == SET_CONFIG)
			{
				len = radio.getPayloadSize();
				TXDEBUG("SET-CONFIG - Longitud del Payload =");TXDEBUGLN(len);
				radio.read(&st_conf, sizeof(st_conf));
				GuardarConfig();
        GuardarConfigEEPROM();
			}
			RECEP_STATE = WAIT_START;
			resetTime();
		}
   else
   {
    TXDEBUGLN(" - Radio not available");
    checkTime();
   }
	}
	/********************* ESTADO TRANSMITIENDO ***************************/
	else if(RECEP_STATE == TRANSMIT)
	{
		TXDEBUG("ESTADO = TRANSMIT - Comando =");TXDEBUGLN((char)comando);
		radio.stopListening(); 
		if(comando == GET_TARGET)
		{
      TXDEBUGLN("GET TARGET");
			for(int i = 0; i< CANT; i++)
			{
			  st_target.set_temp[i] = set[i];
			}
			ok = radio.write(&st_target, sizeof(st_target));
		}
		else if(comando == GET_TEMPS)
		{
      TXDEBUGLN("GET TEMPS");
			for(int i = 0; i< CANT; i++)
			{
			  st_sens.temp[i] = temp[i];
			}
			ok = radio.write(&st_sens, sizeof(st_sens));
		}
		else if(comando == GET_CONFIG)
		{
      TXDEBUGLN("GET CONFIG");
			CargarConfig();
			ok = radio.write(&st_conf, sizeof(st_conf));
		}
		else
		{
      TXDEBUGLN("Comando erroneo - No deberia entrar nunca");
			resetTime();
			RECEP_STATE = WAIT_START;
      ok = 0;
		}
   
		if(ok == 1)
		{
		  TXDEBUGLN("OK = 1");
			resetTime();
			RECEP_STATE = WAIT_START;
		}
		else
		{
      TXDEBUGLN("OK = 0 - Checking time");
			checkTime();
		}
		radio.startListening();
		TXDEBUG("Transmitiendo comando: ");TXDEBUG(comando);TXDEBUG(" - ok =");TXDEBUGLN(ok);
	}
}

void checkTime()
{
  elapsedTime = millis() - startTime;
  if(elapsedTime > (1000*8))
  {
    radioTimeout();
  }
}

void radioTimeout()
{
    TXDEBUG("Error Timeout - Tiempo espera excedido - Tiempo = ");TXDEBUGLN(elapsedTime);
    TXDEBUG("Comando = ");TXDEBUG((char)comando);TXDEBUG(" Estado = ");TXDEBUGLN(RECEP_STATE);
    
    comando = NULL;
    RECEP_STATE = WAIT_START;
    // Aca habria que reiniciar la radio. 
    //radio.closeReadingPipe(pipes[1]);
    //radio.flush_tx();
    //radio.flush_rx();
    //radio.closeWritingPipe(pipes[0]);
    //radio.openWritingPipe(pipes[0]); // Write to device address 'SimpleNode'
    //radio.openReadingPipe(1, pipes[1]); // Write to device address 'SimpleNode'
    resetTime();
}

void resetTime()
{
  TXDEBUGLN("REINCIANDO TIMER");
  startTime = millis();
}

// Cargo la configuracion en la estructura st_conf
// paso de usar la forma de vectores frio[CANT], calor[CANT], habilitados[CANT], etc. a usar bits para identificar lo mismo.
void CargarConfig()
{
  DPRINTLN("Cargando config");
  cargar(&st_conf.frio , frio, CANT);
  cargar(&st_conf.calor , calor , CANT);
  cargar(&st_conf.habilitado , habilitados , CANT);
  st_conf.fulltime = fulltime;
  for(int i = 0; i< CANT; i++)
  {
    st_conf.salida_frio[i] =  salidas_frio[i];
    st_conf.salida_calor[i] =  salidas_calor[i];
  }
  st_conf.temp[0] = 0.0;
  st_conf.temp[1] = 0.0;
  /****************************************** ACA ESTAMOS ***********************************/ 
  st_conf.estado_bombas = (bomba_frio<<1) + bomba_calor;  // Bit 1  
  cargar(&st_conf.estado_salidas_cold, estado_salida_frio, CANT);
  cargar(&st_conf.estado_salidas_hot, estado_salida_calor, CANT);
  
  st_conf.cte1 = 0xF0;
}

// Descargo la configuracion desde la estructura st_conf a la forma de array. 
void GuardarConfig()
{
  descargar(st_conf.frio , frio, CANT);
  descargar(st_conf.calor , calor , CANT);
  descargar(st_conf.habilitado , habilitados , CANT);
  fulltime = st_conf.fulltime;
  for(int i = 0; i< CANT; i++)
  {
    salidas_frio[i] = st_conf.salida_frio[i];
    salidas_calor[i] = st_conf.salida_calor[i];
  }
  st_conf.temp[0] = 0.0;
  st_conf.temp[1] = 0.0;
}

bool ComandoValido(long cmd)
{
  if(cmd == 'S' || cmd == 'G'|| cmd == 'C'|| cmd == 'I'|| cmd == 'T')
    return 1;
  else
    return 0;
}

void Display()
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("T1= ");
  lcd.print(temp[0]);
  lcd.print(" C");

  lcd.setCursor(0,1);
  lcd.print("T2= ");
  lcd.print(temp[1]);
  lcd.print(" C");
}

void ControlFunction()
{
  int minuto = now.minute();
  DPRINT("Funcion de control - Minuto:"); DPRINT(minuto); DPRINT(" Estado = ");DPRINTLN(estado);
  if(minuto <=30 || fulltime)
  {
      for(int i =0; i < CANT; i++)
      {
        //Serial.print("Control temp (");Serial.print(i);Serial.println(")");

        /*************************+++ VER LA FUNCION HABILITADOS, SI SE DESHABILITA estando la salida activa QUEDA activa al deshabilitar *********/
        if(habilitados[i])
        {
            ControlTempFrio(i);
            ControlTempCalor(i);
        }
        else
        {
          // Apagago salidas
          DPRINT("Salida [");DPRINT(i);DPRINTLN("] = Deshabilitada - Apagando salidas");
          estado_salida_frio[i] = 0;
          estado_salida_calor[i] = 0;
          digitalWrite(salidas_calor[i],HIGH);
          digitalWrite(salidas_frio[i],HIGH);
        }
      }
      checkWaterPumps();
  }
    
}

void checkWaterPumps()
{
      DPRINT("Checking water pumps");
      int bfrio = 0;
      int bcalor = 0;
      for(int i = 0; i<CANT; i++)
      {
        if(estado_salida_frio[i] == true)
        {
          bfrio++;
        }
        if(estado_salida_calor[i] == true)
        {
          bcalor++;
        }
      }
      if(bfrio !=0)
      {
        // Prendo bomba de agua fria
        bomba_frio = 1;
        digitalWrite(PIN_BOMBA_FRIO,LOW);
        
      }
      else if(bfrio == 0)
      {
        // Apago Bomba de agua fria
        bomba_frio = 0;
        digitalWrite(PIN_BOMBA_FRIO,HIGH);
      }
      if(bcalor !=0)
      {
        // Prendo bomba de agua caliente
        bomba_calor = 1; 
        digitalWrite(PIN_BOMBA_CALOR,LOW);
        
      }
      else if(bcalor == 0)
      {
        // Apago Bomba de agua caliente
        bomba_calor = 0;
        digitalWrite(PIN_BOMBA_CALOR, HIGH);
      }

      DPRINT("Bomba frio = ");DPRINTLN(bomba_frio);
      DPRINT("Bomba calor = ");DPRINTLN(bomba_calor);
}

void ControlTempFrio(int i)
{
  DPRINT("Control temp frio ");DPRINTLN(i);
  if( (temp[i] > set[i] + 0.2) && frio[i] ) /******************** TEMP ALTA - Abrir agua fria *****************/
  {
    digitalWrite(salidas_frio[i],LOW);
    //digitalWrite(S12,HIGH);
    estado = i;
    estado_salida_frio[i] = 1;
  }
  else if( (temp[i] < set[i] - 0.2) || !frio[i])     /******************** TEMP BAJA - Cerrar agua fria *****************/
  {
    digitalWrite(salidas_frio[i],HIGH);
    estado = 0;                   // Dejo libre
    estado_salida_frio[i] = 0;
  }
}

void ControlTempCalor(int i)
{
    DPRINT("Control temp calor ");DPRINTLN(i);
    if( (temp[i] < set[i] - 0.2) && calor[i] ) /******************** TEMP BAJA - Abrir agua caliente *****************/
      {
        digitalWrite(salidas_calor[i],LOW);
        //digitalWrite(S12,HIGH);
        estado = i;
        estado_salida_calor[i] = 1;
      }
      else if( (temp[i] > set[i] + 0.2) || !calor[i] )     /******************** TEMP ALTA - Cerrar agua caliente *****************/
      {
        digitalWrite(salidas_calor[i],HIGH);
        //digitalWrite(S12,LOW);
        estado = 0;                   // Dejo libre
        estado_salida_calor[i] = 0;
      }
}

void readTemps()
{
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  DPRINTLN("Leyendo temperatura...");
  
  sensors.requestTemperatures(); // Send the command to get temperatures
  // After we got the temperatures, we can print them here.
  // We use the function ByIndex, and as an example get the temperature from the first sensor only.

  for(int i = 0; i < CANT; i++)
  {
    temp[i] = sensors.getTempCByIndex(i);
    
    DPRINT("Dev[");DPRINT(i);DPRINT("] =");DPRINTLN(temp[i]);
  }

}

void mostrarHora()
{
  #ifdef HORA_DEBUG
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
  #endif
}

void radioInit()
{
    /********************************* WIRELESS ***************************/
  radio.begin(); // Start up the radio
  radio.setDataRate(RF24_1MBPS);
  radio.setAutoAck(1); // Ensure autoACK is enabled
  //radio.setChannel(1);                 //Set RF channel to 1
  //radio.setPALevel(RF24_PA_MAX);       //PA level to output
  //radio.setCRCLength(RF24_CRC_8);  
  radio.setRetries(15,15); // Max delay between retries & number of retries
  radio.openWritingPipe(pipes[0]); // Write to device address 'SimpleNode'
  radio.openReadingPipe(1, pipes[1]); // Write to device address 'SimpleNode'
  radio.startListening();
  /********************************* END WIRELESS ***************************/
  bool rf = radio.testCarrier();
  TXDEBUGLN("Radio init () END = ");TXDEBUGLN(rf);
}

void rtcInit()
{
  if (! rtc.begin()) {
    
    DPRINTLN("Couldn't find RTC");
    while (1);
  }
  
  if (! rtc.isrunning()) {
    DPRINTLN("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }  

}


/*
void WriteSD()
{
  
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("data.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print("T1,");
    dataFile.print(temp1);
    dataFile.print(",");
    dataFile.print("T2,");
    dataFile.print(temp2);
    dataFile.print(",");
    dataFile.print(now.hour());
    dataFile.print(":");dataFile.print(now.minute());dataFile.print(":");dataFile.print(now.second());dataFile.print(",");
    dataFile.print(now.year());dataFile.print("/");dataFile.print(now.month());dataFile.print("/");dataFile.print(now.day());
    dataFile.println(';');
    dataFile.close();
    // print to the serial port too:
    #if DEBUG
    Serial.println("Escribiendo la tarjeta");
    #endif
  }
  // if the file isn't open, pop up an error:
  else {
    #if DEBUG
    Serial.println("Error Abriendo datalog.txt");
    #endif
  }
}
*/
/*
void state_machine(char c)
{
  
  switch(STATE) {                                                           // Switch the current state of the message
    case WAIT_START:                                                          // If waiting for start [$], examine each char
        if(c == START_MSG) {                                            // If the char is $, change STATE to IN_MESSAGE
            STATE = IN_COMMAND;
            recibiendo = 1;
            Serial.print("$");
            // Borro variables                                            // Clear temporary QString that holds the message
            break;                                                            // Break out of the switch
        }
        break;
    case IN_COMMAND:
        if( isAlpha(c) ) {                                            // If the char is $, change STATE to IN_MESSAGE
              STATE = IN_MESSAGE;
              comando = c;
              Serial.print(" Comando = ");
              Serial.println(c);
              cont = -1;
              value_recv = "";
              leidos = 0;
              // Borro variables                                            // Clear temporary QString that holds the message
              break;                                                            // Break out of the switch
          }

      break;
    case IN_MESSAGE:                                                          // If state is IN_MESSAGE
        if(c == END_MSG) {                                              // If char examined is ;, switch state to END_MSG
            STATE = WAIT_START;
            recibiendo = 0;
            interpretar = 1;

            valores[cont] = value_recv.toInt();
            values[cont] = value_recv.toFloat();
            value_recv = "";
            cont++;
            
            // Terminamos de obtener los valores
            leidos = cont;
            Serial.println(";"); 
            break;
        }
        else if(isspace(c) ) {                      // If examined char is a digit, and not '$' or ';', append it to temporary string
            //termino un valor
            values[cont] = value_recv.toFloat();
            value_recv = "";
            cont++;
            if(cont>0)
              Serial.print(" ");
        }
        else
        {
          if(cont < 10)
          {
            value_recv +=(char)c;
          }
          Serial.print("Valor[");Serial.print(cont);Serial.print("] recibido = ");Serial.println(c);
        }
        break;
    default:
    {
        STATE = WAIT_START;
        recibiendo = 0;
        break;
    }
  }
    
}
*/

void cargar(uint8_t *destino, bool* origen, uint8_t nbits)
{
  uint8_t temp = 0;
  for(char i = 0; i < nbits; i++)
  {
    bool temp_val;
    temp_val = origen[i];
    temp += temp_val<<i;
  }
  *destino = temp;
}


void descargar(uint8_t origen, bool* destino, uint8_t nbits)
{
  uint8_t temp = 0;
  for(char cont = nbits-1; cont >= 0; cont--)
  {
    bool s;
    s = (bool)(( origen>>cont)&0x01);
    destino[cont] = s;
  }
}

void GuardarConfigEEPROM()
{
  DPRINTLN("Guardando configuracion en EEPROM");
  int eeAddress = 0;   //Location we want the data to be put.
  //Data to store.
  byte *pointer = (byte*)&st_conf;
  for(int i = 0; i < 32; i++)
  {
    EEPROM.write(eeAddress, *pointer);
    pointer++;
    eeAddress++;
  }
}

void GuardarTargetEEPROM()
{
  DPRINTLN("Guardando targets en EEPROM");
  int eeAddress = 32;   //Location we want the data to be put.
  byte *pointer = (byte*)&st_target;
  for(int i = 0; i < 32; i++)
  {
    EEPROM.write(eeAddress, *pointer);
    pointer++;
    eeAddress++;
  }
  
}

void GuardarEnEEPROM()
{
  DPRINTLN("Guardando TODO en EEPROM");
  // Estructuras a escribir: st_conf st_target
  int eeAddress = 0;   //Location we want the data to be put.
  //Data to store.
  byte *pointer = (byte*)&st_conf;
  for(int i = 0; i < 32; i++)
  {
    EEPROM.write(eeAddress, *pointer);
    pointer++;
    eeAddress++;
  }
  *pointer = (byte*)&st_target;
  for(int i = 0; i < 32; i++)
  {
    EEPROM.write(eeAddress, *pointer);
    pointer++;
    eeAddress++;
  }
  DPRINT("Grabado TARGET posicion final = ");DPRINTLN(eeAddress);
  
}

void LeerEEPROM()
{
  DPRINT("Leyendo EEPROM: ");
  int eeAddress = 0;   //Location we want the data to be put.
  //Data to store.
  byte *pointer = (byte*)&st_conf;
  for(int i = 0; i < 32; i++)
  {
    *pointer = EEPROM.read(eeAddress);
    DPRINT(*pointer);
    pointer++;
    eeAddress++;
  }
  DPRINTLN("");
  DPRINT("Leida CONFIGURACION posicion = ");DPRINTLN(eeAddress);
  DPRINT(" Leyendo TARGETS :");
  *pointer = (byte*)&st_target;
  for(int i = 0; i < 32; i++)
  {
    *pointer = EEPROM.read(eeAddress);
    DPRINT(*pointer);
    pointer++;
    eeAddress++;
  }
  DPRINTLN("");
  DPRINT("Leida TARGET tamaño = ");DPRINTLN(eeAddress);
  for(int i = 0; i< CANT; i++)
  {
    set[i] = st_target.set_temp[i];
  }
  GuardarConfig();
}


