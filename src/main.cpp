#include <Adafruit_GFX.h>    
#include <Adafruit_SSD1306.h> 
#include "MAX30105.h"           
#include "heartRate.h"       
#include "SPI.h"
#include <Wire.h>
#include "Arduino.h"
#include "driver/i2s.h"
#include "logo.hpp"
#include "spo2_hr_display.hpp"
#include "Fonts/FreeMono12pt7b.h"
#include "map_convert.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_task_wdt.h"

//Cau hinh I2S cho cam bien
#define SAMPLE_RATE 4000
#define BCLK_PIN GPIO_NUM_32
#define DOUT_PIN GPIO_NUM_33
#define LRCL_PIN GPIO_NUM_25
#define I2S_PORT I2S_NUM_0

//Cau hinh OLED
#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64
#define OLED_RESET    4 
#define OLED_SCLK    18
#define OLED_MOSI    23
#define OLED_DC      2
#define OLED_CS      5 

//Cau hinh chan doc tin hieu AC & nguong gia tri
#define ADC_PIN GPIO_NUM_36 //Chan VP(SP) - GPIO 36 (ADC1_CHANNEL0)
#define MAX_ADC_VALUE 4000 //Bien do tin hieu ADC lon nhat 
#define MIN_ADC_VALUE 300 //Bien do tin hieu ADC nho nhat

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_SCLK, OLED_DC, OLED_RESET, OLED_CS); //Khoi tao giao thuc SPI cho OLED 
Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
MAX30105 particleSensor; 
spo2_hr_display_t param = {0};
x_y_position_t position; //Cau truc vi tri pixels
int adc_read;
int sample_taken = 0;

//Task
TaskHandle_t read_max30102 = NULL;
TaskHandle_t read_ad8232 = NULL;

void ssd1306_welcome_display(){
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setFont(&FreeMono12pt7b); //Set font chu cho toan chuong trinh

  display.setCursor(15, 40);
  display.print("WELCOME");
  display.display();
  vTaskDelay(pdMS_TO_TICKS(1000));
  display.clearDisplay();

  display.setCursor(30, 30);
  display.print("PULSE");
  display.setCursor(10, 50);
  display.print("OXIMETER");
  display.display();
  vTaskDelay(pdMS_TO_TICKS(1000));
  display.clearDisplay();
}

void ssd1306_welcome_display1(){
  display1.setTextColor(WHITE);
  display1.setTextSize(1);
  display1.setFont(&FreeMono12pt7b); //Set font chu cho toan chuong trinh

  display1.setCursor(15, 40);
  display1.print("WELCOME");
  display1.display();
  vTaskDelay(pdMS_TO_TICKS(1000));
  display1.clearDisplay();

  display1.setCursor(30, 30);
  display1.print("PULSE");
  display1.setCursor(10, 50);
  display1.print("OXIMETER");
  display1.display();
  vTaskDelay(pdMS_TO_TICKS(1000));
  display1.clearDisplay();
}

void pwm_init(){
  //Phai khai bao dung theo thu tu struct
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_13_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 1000,
  };
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ledc_channel = {
    .gpio_num = GPIO_NUM_13,
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
  };
  ledc_channel_config(&ledc_channel);

  ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, ledc_channel.duty);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
  vTaskDelay(100 / portTICK_PERIOD_MS);
}

//Ham khoi tao giao thuc I2S cho sph0645
void i2s_sph0645_install(){
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 6,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0,
  };

  i2s_pin_config_t i2s_pin_config = {
    .bck_io_num = BCLK_PIN,
    .ws_io_num = LRCL_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = DOUT_PIN,
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &i2s_pin_config);
  Serial.println(F("Cau hinh I2S thanh cong ~"));
}

//Ham khoi tao giao thuc I2C cho max30102
void i2c_max30102_install(){
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)){ //Use default I2C port, 400kHz speed
    Serial.println(F("MAX30102 not found !"));
    while(1);
  }

  //Set cac thong so cho cam bien
  byte ledBrightness = 0x1F; 
  byte sampleAverage = 4; 
  byte ledMode = 2; 
  int sampleRate = 500; 
  int pulseWidth = 411; 
  int adcRange = 16384;
  // Set up the wanted parameters
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  particleSensor.enableDIETEMPRDY();
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
}

//Ham khoi tao cac thong so cho ADC
void adc_ad8232_install(){
  pinMode(ADC_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PIN, ADC_11db);
}

void draw_ecg_plot(){
  if(position.x_pos > 127){
    display1.fillRect(0, 0, 128, 64, BLACK);
    position.x_pos = 0;
    position.last_x = 0;
  }

  int y_signal = adc_read;
  if(y_signal > MAX_ADC_VALUE) y_signal = MAX_ADC_VALUE;
  if(y_signal < MIN_ADC_VALUE) y_signal = MIN_ADC_VALUE;

  int y_signal_map = map_convert(y_signal, MIN_ADC_VALUE, MAX_ADC_VALUE, 0, 60);
  position.y_pos = 60 - y_signal_map;

  display1.writeLine(position.last_x, position.last_y, position.x_pos, position.last_y, WHITE);
  display1.display();

  position.last_x = position.x_pos;
  position.last_y = position.y_pos;

  position.x_pos++;
}

void read_max30102_task(void *pvParameter){
  while(1){
    esp_task_wdt_reset();
    long irValue = particleSensor.getIR();    //Reading the IR value it will permit us to know if there's a finger on the sensor or not
    if (irValue > FINGER_ON) {  
      
      if (checkForBeat(irValue) == true) {
        display.clearDisplay(); 
        display.drawBitmap(0, 0, logo3_bmp, 32, 32, WHITE);
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(42, 20);
        display.drawBitmap(0, 35, O2_bmp, 32, 32, WHITE);
        display.setCursor(42, 55);
       
        if (param.beatAvg > 30) display.print(String(ESpO2) + "%");
        else display.print("---- %" );
        display.display();    
        tone(BUZZER_PIN, 1000);
        vTaskDelay(pdMS_TO_TICKS(100));
        noTone(BUZZER_PIN);
      
        heart_rate_calculate(&param);
        vTaskDelay(pdMS_TO_TICKS(10));
      }
  
      uint32_t ir, red ;
      particleSensor.check(); //Check the sensor, read up to 3 samples
      if (particleSensor.available()) {
        sample_taken++;
        ir = particleSensor.getFIFOIR(); 
        red = particleSensor.getFIFORed(); 
        
        low_pass_for_IR_RED(red, ir, &param, sample_taken);
        particleSensor.nextSample(); //We're finished with this sample so move to next sample
      }
    
      display.clearDisplay();
      display.drawBitmap(5, 5, logo2_bmp, 24, 21, WHITE);
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(42, 20);    
      display.print(param.beatAvg); 
      display.println("BPM");
      display.drawBitmap(0, 35, O2_bmp, 32, 32, WHITE);
      display.setCursor(42, 55);
      if (param.beatAvg > 30) display.print(String(ESpO2) + "%");
      else display.print("---- %" );
      display.display();
    }
    else {
      for (byte rx = 0 ; rx < RATE_SIZE ; rx++) rates[rx] = 0;
      param.beatAvg = 0;
      param.SpO2 = 0;
      param.aveir = 0;
      param.avered = 0;
      param.sumirrms = 0;
      param.sumredrms = 0;
      rateSpot = 0; 
      lastBeat = 0;
      ESpO2 = 90.0;
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(22, 30);
      display.println("Finger");
      display.setCursor(22, 50);
      display.println("Please");
      display.display();
    }
  }
}

void read_ad8232_task(void *pvParameter){ 
  while(1){
    esp_task_wdt_reset(); //Reset watchdog 
    adc_read = analogRead(ADC_PIN);
    draw_ecg_plot();  
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("System Start");

  //Khoi tao man hinh 1
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
    Serial.println(F("Failed to connected to SPI SSD1306"));
  } 
  display.display();

  //Khoi tao man hinh 2
  if(!display1.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
    Serial.println(F("Failed to connected to I2C SSD1306"));
  }
  display1.display();
  vTaskDelay(pdMS_TO_TICKS(1000));
  display.clearDisplay();
  display1.clearDisplay();
  ssd1306_welcome_display();
  ssd1306_welcome_display1();

  //Khoi tao 
  adc_ad8232_install();
  i2s_sph0645_install();
  i2c_max30102_install();
  pwm_init();

  xTaskCreatePinnedToCore(read_ad8232_task, "ECG task", 1024 * 5, NULL, 5, &read_ad8232, 0);
  xTaskCreatePinnedToCore(read_max30102_task, "PPG task", 1024 * 15, NULL, 5, &read_max30102, 1);
}

void loop() {
  /**
   * @note Trong vong loop, cac task se tu dong duoc freeRTOS thuc thi song song 
   * Do do, khong can phai thuc thi bat ky ma nao o day nua
   */
  vTaskDelete(NULL);
}
