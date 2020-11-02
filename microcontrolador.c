#define ESP32_BOARD
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>
#ifdef ESP32_BOARD
#include <BlynkSimpleEsp32.h>
#else
#include <BlynkSimpleEsp8266.h>
#endif
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <string.h>
#define Hall sensor 2  //  Pino digital 2
Adafruit_BME280 bme;
#define BLYNK_PRINT Serial
#define BLYNK_DEBUG 1
#include <SoftwareSerial.h>

#include "SIM808.h"
#include "inetGSM.h"
//MOD GSM
InetGSM inet;
boolean started = false;
char smsbuffer[160];
char n[20];
byte valor;
char valx;
//PH measurement
float calibration = 0.00;
//mude este valor para calibrar
const int analogInPin = A0;
int sensorValue = 0;
unsigned long int avgValue;
float b;
int buf[10], temp;
//Anemometro
//Difinicoes de constantes
const float pi = 3.14159265;
// Numero pi
int period = 5000;
// Tempo de medida(miliseconds)
int delaytime = 2000;
//Tempo entre amostras(miliseconds)
int radius = 147;
// Raio do anemometro(mm)
//Definicao de variaveis
unsigned int Sample = 0;
//numero de amostra
unsigned int counter = 0;
//contador magnetico para sensor
unsigned int RPM = 0;
//Revolucoes por minuto
float speedwind = 0;
//Velocidade do vento (m/s)
float windspeed = 0;
//Velocidade do vento (km/h)
//Umidade do solo
int umidade;
//pluviometro
//Constantes:
const int REED = 6;
//A chave reed sai para o pino digital 9
// Variaveis:
int val = 0;      //Valor atual do interruptor reed
int old_val = 0;  //Valor antigo do interruptor reed
int REEDCOUNT = 0;
//Esta e a variavel que mantem a contagem de comutacao
//as variaveis a seguir sao do tipo long
//pois o tempo, medido em milissegundos,
//rapidamente se tornara em um numero grande demais para ser
//armazenado em uma int.

void setup() {
    Serial.begin(9600);
    //Iniciar Serial
    pinMode(13, OUTPUT);
    //Defina os pinos
    pinMode(2, INPUT);
    digitalWrite(2, HIGH);
    //pull-up interno ativo
    Serial.println("###################");
    delay(5000);
    //Esse BME280 esta no endereco 0x76
    if (!bme.begin(0x76)) {
        Serial.println("BME280 nao inciado. Aguardando.");
        while (true)
            ;
    }
    Serial.println("Iniciando.");
    Blynk.begin(auth, ssid, pass, SRV, 8080);
    //para servidor local, esse e o formato
    timer.setInterval(2000L, tempSend);
    delay(1000);
    //initializa o pino do switch como entrada
    pinMode(REED, INPUT_PULLUP);
    //Isso ativa o resistor pull up interno
    //initializa a comunicacao serial:
    digitalWrite(2, HIGH);
    powerUpOrDown();
    Serial.println(F("Testando GSM Shield SIM900"));
    if (gsm.begin(2400)) {
        Serial.println(F("nstatus=READY"));
        started = true;
    } else
        Serial.println(F("nstatus=IDLE"));
}
void loop() {
    //###########pH############
    for (int i = 0; i < 10; i++) {
        buf[i] = analogRead(analogInPin);
        delay(30);
    }
    for (int i = 0; i < 9; i++) {
        for (int j = i + 1; j < 10; j++) {
            if (buf[i] > buf[j]) {
                temp = buf[i];
                buf[i] = buf[j];
                buf[j] = temp;
            }
        }
    }
    avgValue = 0;
    for (int i = 2; i < 8; i++)
        avgValue += buf[i];
    float pHVol = (float)avgValue * 5.0 / 1024 / 6;
    float phValue = -5.70 * pHVol + calibration;
    Serial.print("sensor = ");
    Serial.println(phValue);
    delay(500);
    //########end############
    //########sensor umidade do solo###########
    umidade = analogRead(A0);
    int Porcento = map(umidade, 1023, 0, 0, 100);
    Serial.print(Porcento);
    Serial.println("%");
    if (Porcento <= 70) {
        Serial.println("Irrigando...");
        digitalWrite(13, HIGH);
    } else {
        digitalWrite(13, LOW);
    }
    delay(1000);
    //##########anemometro#############
    Sample++;
    Serial.print(Sample);
    Serial.print(": Start measurement...");
    windvelocity();
    Serial.println("   finished.");
    Serial.print("Counter: ");
    Serial.print(counter);
    Serial.print(";  RPM: ");
    RPMcalc();
    Serial.print(RPM);
    Serial.print(";  Wind speed: ");
    //******************************************
    //print m/s
    WindSpeed();
    Serial.print(windspeed);
    Serial.print(" [m/s] ");
    //******************************************
    //print km/h
    SpeedWind();
    Serial.print(speedwind);
    Serial.print(" [km/h] ");
    Serial.println();
    delay(delaytime);  //delay between prints
    //###############Sensor BME280##############
    f_temp = bme.readTemperature();
    Blynk.run();
    timer.run();
    delay(100);
    //#############Sensor Pluviometro###############
    // ler o estado do switch pelo pino de entrada:
    val = digitalRead(REED);
    //Leia o status do interruptor Reed
    if ((val == LOW) && (old_val == HIGH)) {
        //Verifique se o status mudou
        delay(10);
        //Atraso colocado para lidar com qualquer
        //"salto" no switch.
        REEDCOUNT = REEDCOUNT + 1;
        //Adicione 1 a contagem de numeros
        old_val = val;
        //Faca o valor antigo igual ao valor atual
        Serial.print("Medida de chuva (contagem): ");
        Serial.print(REEDCOUNT);
        //*0.2794);
        Serial.println(" pulso");
        Serial.print("Medida de chuva (calculado): ");
        Serial.print(REEDCOUNT * 0.25);
        Serial.println(" mm");
    } else {
        old_val = val;
        //Se o status nao mudou, nao faca nada
    }
    //###############mod GSM#####################
    if (started) {
        //Aguarda novo SMS e envia para o servidor web
        if (gsm.readSMS(smsbuffer, 160, n, 20)) {
            String str(smsbuffer);
            envia_GSM(smsbuffer);
            delay(10000);
        }
        delay(1000);
    }
}
void tempSend() {
    String s_temp = String(f_temp);
    Serial.println(s_temp);
    Blynk.virtualWrite(V1, f_temp);
}
// Measure wind speed
void windvelocity() {
    speedwind = 0;
    windspeed = 0;
    counter = 0;
    attachInterrupt(0, addcount, RISING);
    unsigned long millis();
    long startTime = millis();
    while (millis() < startTime + period) {
    }
}
void RPMcalc() {
    RPM = ((counter)*60) / (period / 1000);
    //Calcular revolucoes por minuto (RPM)
}
void WindSpeed() {
    windspeed = ((4 * pi * radius * RPM) / 60) / 1000;
    //Calcular a velocidade do vento em m/s
}
void SpeedWind() {
    speedwind = (((4 * pi * radius * RPM) / 60) / 1000) * 3.6;
    //Calcular a velocidade do vento em km/h
}
void addcount() {
    counter++;
}
void powerUpOrDown() {
    //Liga o GSM Shield
    Serial.print(F("Liga GSM..."));
    pinMode(6, OUTPUT);
    digitalWrite(6, LOW);
    delay(1000);
    digitalWrite(6, HIGH);
    delay(1000);
    Serial.println(F("OK!"));
    digitalWrite(6, LOW);
    delay(500);
}
void envia_GSM(String texto) {
    char temp_string[55];
    char msg[10];
    int numdata;
    if (inet.attachGPRS("opcel.br", "opcel", "opcel"))
        Serial.println(F("status=Conectado..."));
    else
        Serial.println(F("status=Nao conectado!"));
    delay(100);
    String valor = "MSG_Texto1=" + texto;
    valor.toCharArray(temp_string, 55);
    valx = "A2P2.webatu.com", 80, "/add.php", temp_string, msg, 50 numdata = inet.httpPOST(valx);
    delay(5000);
}
