#include <arduinoFFT.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <FastLED.h>

const char* ssid = "Pixel_7";
const char* password = "987654321";

#define LED_PIN    4       // Пин, к которому подключена светодиодная лента
#define LED_COUNT  296     //Количество светодиодов в ленте

// Инициализация WebSocket сервера
WebSocketsServer webSocket = WebSocketsServer(81);

// Переменные для управления режимами и параметрами ленты
enum LedMode { STATIC, RAINBOW, AUDIO_REACTIVE };
LedMode currentMode = STATIC;
CRGB staticColor = CRGB(0, 0, 0);  // Красный цвет по умолчанию
uint8_t brightness = 50;  // Яркость по умолчанию (50%)



////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

#define LOW_F_LIMIT 200
#define MEDIUM_F_LIMIT 2000
#define SAMPLES 4000
#define NOISE 500
#define AMPLITUDE 1000
#define NUM_BANDS 3
#define NUM_LEDS 296


#define TEST_SAMPLE 128

int newTime;

double vReal[TEST_SAMPLE];
double vImag[TEST_SAMPLE];

const double SAMPLING_FREQ = 40000; 
unsigned int sampling_period_us;
unsigned long microseconds;
int bandValues[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


ArduinoFFT FFT = ArduinoFFT(vReal, vImag, SAMPLES, SAMPLING_FREQ, true);

CRGB leds[NUM_LEDS];

void setup() {
  Serial.begin(115200);

  // Подключаемся к Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.print("Connected to WiFi with IP ");
  Serial.println(WiFi.localIP());

  // Запускаем WebSocket сервер
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server started.");

  // Инициализация ленты
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, LED_COUNT);  // Изменили на GRB
  FastLED.setBrightness(brightness);
  FastLED.clear();

  //FastLED.show();  // Очищаем ленту
  Serial.println("Strip started.");


  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQ));

}

void loop(){

  webSocket.loop();



  //delay(1000);
}



// Обработчик WebSocket сообщений
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_TEXT:
      // Получаем текстовую команду
      handleTextCommand((char*)payload);
      break;
    case WStype_BIN:
      // Обрабатываем бинарные аудиоданные
      processAudioData(payload, length);
      break;
  }
}

// Обработчик текстовых команд
void handleTextCommand(const char* message) {
  String cmd = String(message);

  // Парсим JSON команду
  if (cmd.indexOf("changeMode") != -1) {
    if (cmd.indexOf("static") != -1) {
      currentMode = STATIC;
    } else if (cmd.indexOf("rainbow") != -1) {
      currentMode = RAINBOW;
    } else if (cmd.indexOf("audio") != -1) {
      currentMode = AUDIO_REACTIVE;
    }
  }

  // Обработка команды изменения цвета (RGB)
  if (cmd.indexOf("changeColor") != -1) {
    int r = getValueFromJSON(cmd, "r");
    int g = getValueFromJSON(cmd, "g");
    int b = getValueFromJSON(cmd, "b");
    staticColor = CRGB(r, g, b);
    showStaticColor(staticColor);
    // Serial.print("This is your color: ");
    // Serial.print(r);
    // Serial.print(" ");
    // Serial.print(g);
    // Serial.print(" ");
    // Serial.println(b);

  }

  // Обработка команды изменения яркости
  if (cmd.indexOf("changeBrightness") != -1) {
    int newBrightness = getValueFromJSON(cmd, "brightness");
    brightness = constrain(newBrightness, 0, 255);
    FastLED.setBrightness(brightness);
    FastLED.show();
    // Serial.print("This is your brightness: ");
    // Serial.println(brightness);
  }
}

// Функция для получения значения из JSON
int getValueFromJSON(String json, String key) {
  // Serial.println(json);
  int startIndex = json.indexOf("\"" + key + "\":") + key.length() + 3;
  int endIndex = json.indexOf(",", startIndex);
  if (endIndex == -1) {
    endIndex = json.indexOf("}", startIndex);
  }

  // Serial.println(startIndex);
  // Serial.println(endIndex);


  
  return json.substring(startIndex, endIndex).toInt();
}

// Отображение статичного цвета
void showStaticColor(CRGB color) {
  for (int i = 0; i < LED_COUNT; i++) {
    leds[i] = color;
  }
  FastLED.show();
}




// Пример обработки аудиоданных для светомузыки
void processAudioData(uint8_t *data, size_t length) 
{

   for (int i = 0; i < length; i++)
   {
     Serial.print("Data ["); Serial.print(i); Serial.print("]:" );
     Serial.println(data[i]);
   }

    // Reset bandValues[]
  for (int i = 0; i<NUM_BANDS; i++){
    bandValues[i] = 0;
  }


  for (int i = 0; i < TEST_SAMPLE ;i++)
  {
    newTime = micros();
    vReal[i] = (double)data[i];
    vImag[i] = 0;
    while (micros() < (newTime + sampling_period_us)) { /* Wait */}
  }


  FFT.dcRemoval();
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);  // Apply windowing to reduce spectral leakage
  FFT.compute(FFT_FORWARD);                          // Compute the FFT
  FFT.complexToMagnitude();                        // Convert complex numbers to magnitudes

  for (int i = 2; i < (TEST_SAMPLE/2); i++)
  {
      if (vReal[i] > NOISE)
      {
        if (i<=14 )           bandValues[0]  += (int)vReal[i];
        if (i>14   && i<=166  ) bandValues[1]  += (int)vReal[i];
        if (i>166             ) bandValues[2]  += (int)vReal[i];
      }
  }

  // Find the dominant frequency component
  double peakFrequency = FFT.majorPeak(vReal, TEST_SAMPLE, SAMPLING_FREQ);           // Find the major frequency peak

  // Classify the frequency as Low, Medium, or High
  classifyFrequency(peakFrequency);


  for (int i = 0; i < TEST_SAMPLE / 2; i++) {  // Only the first half contains useful data
    Serial.print("Frequency bin ");
    Serial.print(i * ((double)SAMPLING_FREQ / TEST_SAMPLE));
    Serial.print(" Hz: ");
    Serial.println(vReal[i]);  // Magnitude of the frequency bin
  }






  // for (size_t i = 0; i < SAMPLES / 2; i++) 
  // {
  //  Serial.print((i * 1.0 * SAMPLING_FREQ) / SAMPLES, 1);
  //  Serial.print(" ");
  //  Serial.println(vReal[i], 1); // These are the magnitudes of each frequency bin
  // }



  // for (byte band = 0; band < NUM_BANDS; band++)
  // {

  //   ranibowBars(band);

  // }

  // FastLED.show();
}


void classifyFrequency(double frequency) {
  if (frequency < LOW_F_LIMIT) {
    Serial.print("Low Frequency Detected: ");
  } 
  else if (frequency < MEDIUM_F_LIMIT) {
    Serial.print("Medium Frequency Detected: ");
  } 
  else {
    Serial.print("High Frequency Detected: ");
  }

  Serial.print(frequency);
  Serial.println(" Hz");
}




//////////////////    DRAWING    //////////////////
//////////////////               //////////////////

#define BAR_WIDTH (NUM_LEDS / NUM_BANDS - 1)

void ranibowBars (int band)
{
  int xStart = BAR_WIDTH * band;

  for (int x = 0; x < xStart + BAR_WIDTH; x++) {
    leds[x] = CHSV((x  / BAR_WIDTH) * (255 / NUM_BANDS), 255, 255);  // Set the color
  }

}

