//Подключение по WiFi для отправки сообщений по MQTT
#include <PubSubClient.h>
#include <WiFi.h>

#define ZMCT116A_PIN 33   //Датчик тока
#define ZMPT101B_PIN 32   //Датчик напряжения

const int NUM_READ = 30;   //Количество элементов выборки для фильтрации

const float MAX_ANALOG_READ = 4095.0; //2047,5  2005  Максимальное считываемое значение АЦП
const float MAX_ANALOG_READ_COEFF = 2002;   //Должна быть половина от максимального значения \
//но видимо из-за просадок напряжения по пути питания немного падает значение, должно быть 2047,5
const float MAX_ANALOG_READ_V = 3.3;  //V Напряжение питания
const int COUNTS_OF_TURN = 2500;  //n Количество витков катушки
const int RESISTANS = 39;   //Om    (AREF * CT TURNS) / (2√2 * максимальный первичный ток)  =  (3.3 * 2500) / (2 * sqrt(2) * 70A)
const int MAX_AMPESR_COIL = 70; //A Максимальное значение силы тока на катушке

uint32_t printTimer = 0;  //Таймер для отправки сообщений

//Параметры для работы по WiFi
const char* ssid = "FreeNet";
const char* password = "password2";
const char* mqtt_server = "arhiopteryx.ga"; //Адрес mqtt сервера
const int mqtt_port = 1883; //Порт mqtt сервера
WiFiClient espClient;
PubSubClient client(espClient);

void setup()
{
  Serial.begin(9600);  //  Запуск serial

  //Настройки подключения по mqtt
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop()
{
  double currentA = getA();   //Получить значение силы тока
  double currentV = getV();   //Получить значение напряжения

  if ( millis() - printTimer > 1000) {  //Каждую секунду отправлять значения по MQTT
    printTimer = millis();
    Serial.println(String(currentA) + " A       "  + String(currentV) + " V");
    getMessageMQTT(currentV , currentA);
  }
}

double getV() { //Получить значение напряжения

  int sensor_Value  = 0;
  float  v_Out = 0.0; //  Переменная для получения значения напряжения сети из аналогового порта

  static uint32_t timerV = 0;   //таймер для усреднения значений

  //Переменные для расчетов максимального и минимального значений с АЦП
  static float Vmin = 1024;
  static float Vmax = 0;

  static float result_V = 0;

  sensor_Value = analogRead (ZMPT101B_PIN);     // read the analog in value
  v_Out = 250 * (sensor_Value / MAX_ANALOG_READ); //Перевод считанного значения в напряжение

  //Нахождение полученного максимума и минимума
  if (v_Out > Vmax) {
    Vmax = v_Out;
  }
  if (v_Out < Vmin) {
    Vmin = v_Out;
  }

  if (millis() > timerV + 100) {   //Каждую 1/10 секунды обновлять результат
    timerV = millis();
    float result = ((Vmax - Vmin) * 2) / sqrt(2); //Расчет реального значения напряжения с учетом корня двух, то есть без пикового напряжения

    if (Vmin > 0) result = 0; //Когда напряжения есть, Vmin равно 0

    Vmin = MAX_ANALOG_READ;
    Vmax = 0;
    result_V = runMiddleArifmOptimV(result);  //Фильтрация напряжения, чтобы не было скачков
  }
  return (result_V);
}

double getA() { //Получить значение силы тока

  //Таймеры для считывания значений и для фидьтрации
  static uint32_t timerA1 = 0;
  static uint32_t timerA2 = 0;
  static int maxICur = 0;
  static int maxI = 0;
  static float result_I = 0;

  if ((millis() - timerA1) > 1) { //Каждую миллисекунду считываем значение и находим максимально
    timerA1 = millis();
    int readValue = analogRead(ZMCT116A_PIN);
    if (readValue > maxICur) maxICur = readValue;
  }

  if ((millis() - timerA2) > 100) { //Каждую 1/10 секунды обрабатываем полученное максимальное значение
    timerA2 =  millis();
    maxI = maxICur;
    maxICur = 0;

    //float res = ((maxI - MAX_ANALOG_READ/2.0) * MAX_ANALOG_READ_V) / MAX_ANALOG_READ;   //Должно быть так, то есть половина от максимума
    float res = ((maxI - MAX_ANALOG_READ_COEFF) * MAX_ANALOG_READ_V) / MAX_ANALOG_READ; //Поправка на просадки
    float   nCurrThruResistorPP = (res / RESISTANS) * COUNTS_OF_TURN; //Перевод напряжения в силу тока
    float nCurrThruResistorRMS = nCurrThruResistorPP * 0.707;  //0.707 = 1 / sqrt(2)   220 = 310 / sqrt(2) = 310 * 0.707  Вычет пиковой составляющей
    // float nCurrentThruWire = nCurrThruResistorRMS * 1000;  //перевод в миллиамперы

    if (nCurrThruResistorRMS < 0) { //Из-за моих неточностей в наспройке схемы иногды выдает отрицатильное значение силы тока, подгоняю под 0
      nCurrThruResistorRMS = 0;
    }
    result_I = runMiddleArifmOptim(nCurrThruResistorRMS); //Обработка результата фильтром
  }
  return (result_I);
}

// оптимальное бегущее среднее арифметическое для силы тока
float runMiddleArifmOptim(float newVal) {
  static int t = 0;
  static float vals[NUM_READ];
  static float average = 0;
  if (++t >= NUM_READ) t = 0; // перемотка t
  average -= vals[t];         // вычитаем старое
  average += newVal;          // прибавляем новое
  vals[t] = newVal;           // запоминаем в массив
  return ((float)average / NUM_READ);
}

// оптимальное бегущее среднее арифметическое для напряжения
float runMiddleArifmOptimV(float newVal2) {
  static int t2 = 0;
  static float vals2[NUM_READ];
  static float average2 = 0;
  if (++t2 >= NUM_READ) t2 = 0; // перемотка t
  average2 -= vals2[t2];         // вычитаем старое
  average2 += newVal2;          // прибавляем новое
  vals2[t2] = newVal2;           // запоминаем в массив
  return ((float)average2 / NUM_READ);
}

void getMessageMQTT(float valueV, float valueI) { //Функция отправк сообщений по MQTT

  if (!client.connected()) {
    reconnect();
  }
  if (!client.loop())
    client.connect("ESP8266Client");

  String helloMessage = "hello world /n, time = " + String(millis());
  String isV = "-";
  String valueV_str = String(valueV) + " V";
  String valueI_str = String(valueI) + " A";
  String valueW_str = String(valueI * valueV) + " W";

  if (valueV > 0) { //Если напряжение = 0, значит не подключено к питанию
    isV = "+";
  }

  //Отправка сообщений в соответствующие топики
  client.publish("test", helloMessage.c_str());
  client.publish("test/isV", isV.c_str());
  client.publish("test/countV", valueV_str.c_str());
  client.publish("test/countA", valueI_str.c_str());
  client.publish("test/countW", valueW_str.c_str());
}

void setup_wifi() { //Функция запуска WiFi
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected - ESP IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(String topic, byte* message, unsigned int length) { //Функция получения сообщения из топика *** просто ради теста ***
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
  if (messageTemp == "+") {
    Serial.println("Turn On");
  }
  else if (messageTemp == "-") {
    Serial.println("Turn Off");
  }
  else {
    Serial.println("Error Message");
  }
}

void reconnect() {  //Функция перезапуска MQTT
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      client.subscribe("test/isTurnRelay");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
