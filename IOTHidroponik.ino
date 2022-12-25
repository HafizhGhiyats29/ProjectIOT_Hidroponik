#define BLYNK_TEMPLATE_ID "TMPLu39O2Y7P"
#define BLYNK_DEVICE_NAME "Posyandu Rumpun Bambu"
#define BLYNK_AUTH_TOKEN "OFWiF8pGeGKqkRzFQkye1mewCWzOIV_I"

#define BLYNK_PRINT Serial
#include <EEPROM.h>
#include "GravityTDS.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#define EEPROM_SIZE 1024
#define RED2RED 0
#define GREEN2GREEN 1
#define BLUE2BLUE 2
#define BLUE2RED 3
#define GREEN2RED 4
#define RED2GREEN 5
#define splashWidth 320
#define splashHeight 480
 #define TFT_GREY 0x2104

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "RumpunBambu08";
char pass[] = "posyandu08";

 
//TFT_eSPI tft = TFT_eSPI(); // Invoke custom library with default width and height
 
uint32_t runTime = -99999; // time for next update
int d = 0;       // Variable used for the sinewave test waveform
boolean range_error = 0;
 
unsigned long lastDebounceTime = 0; // the last time the output pin was toggled
unsigned long debounceDelay = 1000; // the debounce time; increase if the output flickers

//TDS
#define TdsSensorPin 33
GravityTDS gravityTds;
float tdsValue = 0;
static unsigned long timepoint = millis();

//pH
#include "DFRobot_PH.h"
#define PH_PIN 32
float phVoltage,phValue;
DFRobot_PH ph;
#define ArrayLenth 40
int pHArray[ArrayLenth];
int pHArrayIndex=0;
int avrg;

//Water
#define TRIGGER  13
#define ECHO 12
#define USONIC_DIV 0.034
#define BUZZER 26
long duration;
long distance;
int percentage;

//Temp
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature temp(&oneWire);

//TFT
TFT_eSPI tft = TFT_eSPI();
unsigned long drawTime = 0;

double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error");
    return 0;}
    if(number<5){
      for(i=0;i<number;i++){
        amount+=arr[i];
        }
      avg = amount/number;
      return avg;
    }else{
      if(arr[0]<arr[1]){
        min = arr[0];max=arr[1];
        }
        else{
          min=arr[1];max=arr[0];
          }
         for(i=2;i<number;i++){
          if(arr[i]<min){
            amount+=min;
            min=arr[i];
           }else {
            if(arr[i]>max){
              amount+=max;
              max=arr[i];
             }else{
              amount+=arr[i];
              }
            }
          }
          avg = (double)amount/(number-2);
        }
        return avg;
      }

BlynkTimer timer;

void setup(void)
{
  EEPROM.begin(EEPROM_SIZE);
  Blynk.begin(auth, ssid, pass);
  Serial.begin(115200);
  Blynk.virtualWrite(V3,0);
  Blynk.virtualWrite(V1,0);
  Blynk.virtualWrite(V2,0);
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(3.3);  //reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(4096);  //1024 for 10bit ADC;4096 for 12bit ADC
  gravityTds.begin();  //initialization
  temp.begin();
  ph.begin();
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Kale Monitor", 10, 10, 2);
  pinMode(TRIGGER, OUTPUT); 
  pinMode(ECHO, INPUT); 
  pinMode (BUZZER, OUTPUT);
  timer.setInterval(1000L, sendSensor);

}
 
void loop()
{
   Blynk.run();
   timer.run();
   if(millis()-timepoint>1000U){
    timepoint=millis();
    temp.requestTemperatures();
    float tempC=temp.getTempCByIndex(0);
    if(tempC != DEVICE_DISCONNECTED_C){
      Serial.print("Temp: ");
      Serial.println(tempC);
      }
      else tempC = 25.0;
    if (millis() - runTime >= 0L)
  { // Execute every TBD ms
    runTime = millis();
 
    // Test with a slowly changing value from a Sine function
    //d += 4; if (d >= 360) d = 0;
 
    // Set the the position, gap between meters, and inner radius of the meters
    int xpos = 0, ypos = 5, gap = 4, radius = 52;
 
    // Draw meter and get back x position of next meter
 
    // Test with Sine wave function, normally reading will be from a sensor
    //reading = 250 + 250 * sineWave(d+0);
    //xpos = gap + ringMeter(reading, 0, 500, xpos, ypos, radius, "mA", GREEN2RED); // Draw analogue meter
 
    //reading = 20 + 30 * sineWave(d+60);
    //xpos = gap + ringMeter(reading, -10, 50, xpos, ypos, radius, "degC", BLUE2RED); // Draw analogue meter
 
    //reading = 50 + 50 * sineWave(d + 120);
    //ringMeter(reading, 0, 100, xpos, ypos, radius, "%RH", BLUE2BLUE); // Draw analogue meter
 
    // Draw two more larger meters
    //xpos = 20, ypos = 115, gap = 24, radius = 64;
 
    //reading = 1000 + 150 * sineWave(d + 90);
    //xpos = gap + ringMeter(reading, 850, 1150, xpos, ypos, radius, "mb", BLUE2RED); // Draw analogue meter
 
    //reading = 15 + 15 * sineWave(d + 150);
    //xpos = gap + ringMeter(reading, 0, 30, xpos, ypos, radius, "Volts", GREEN2GREEN); // Draw analogue meter
 
    // Draw a large meter
    xpos = 40, ypos = 30, gap = 15, radius = 70;
    // Comment out above meters, then uncomment the next line to show large meter
    ringMeter(tdsValue, 0, 800, 160, 140, 80, "ppm", GREEN2RED); // Draw analogue meter
    ringMeter(tempC, -10, 50, 40, 30, 70, "C", BLUE2RED);
    ringMeter(phValue, 0, 14, 300, 30, 70, "pH", RED2GREEN);
  }
        
   //temperature = readTemperature();
   gravityTds.setTemperature(tempC);
   gravityTds.update();
   tdsValue = gravityTds.getTdsValue();
   Serial.print(tdsValue,0);
   Serial.println("ppm");
   pHArray[pHArrayIndex++]=analogRead(PH_PIN);
   if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
   avrg = avergearray(pHArray, ArrayLenth);
   phVoltage = avrg/4096.0*3300;
   phValue = ph.readPH(phVoltage,tempC);
   Blynk.virtualWrite(V3,tdsValue);
   Blynk.virtualWrite(V1,tempC);
   Blynk.virtualWrite(V2,phValue);
}
}
 
// #########################################################################
//  Draw the meter on the screen, returns x coord of righthand side
// #########################################################################
int ringMeter(int value, int vmin, int vmax, int x, int y, int r, const char *units, byte scheme)
{
  // Minimum value of r is about 52 before value text intrudes on ring
  // drawing the text first is an option
 
  x += r;
  y += r; // Calculate coords of centre of ring
 
  int w = r / 3; // Width of outer ring is 1/4 of radius
 
  int angle = 150; // Half the sweep angle of meter (300 degrees)
 
  int v = map(value, vmin, vmax, -angle, angle); // Map the value to an angle v
 
  byte seg = 3; // Segments are 3 degrees wide = 100 segments for 300 degrees
  byte inc = 6; // Draw segments every 3 degrees, increase to 6 for segmented ring
 
  // Variable to save "value" text colour from scheme and set default
  int colour = TFT_BLUE;
 
  // Draw colour blocks every inc degrees
  for (int i = -angle + inc / 2; i < angle - inc / 2; i += inc)
  {
    // Calculate pair of coordinates for segment start
    float sx = cos((i - 90) * 0.0174532925);
    float sy = sin((i - 90) * 0.0174532925);
    uint16_t x0 = sx * (r - w) + x;
    uint16_t y0 = sy * (r - w) + y;
    uint16_t x1 = sx * r + x;
    uint16_t y1 = sy * r + y;
 
    // Calculate pair of coordinates for segment end
    float sx2 = cos((i + seg - 90) * 0.0174532925);
    float sy2 = sin((i + seg - 90) * 0.0174532925);
    int x2 = sx2 * (r - w) + x;
    int y2 = sy2 * (r - w) + y;
    int x3 = sx2 * r + x;
    int y3 = sy2 * r + y;
 
    if (i < v)
    { // Fill in coloured segments with 2 triangles
      switch (scheme)
      {
      case 0:
        colour = TFT_RED;
        break; // Fixed colour
      case 1:
        colour = TFT_GREEN;
        break; // Fixed colour
      case 2:
        colour = TFT_BLUE;
        break; // Fixed colour
      case 3:
        colour = rainbow(map(i, -angle, angle, 0, 127));
        break; // Full spectrum blue to red
      case 4:
        colour = rainbow(map(i, -angle, angle, 70, 127));
        break; // Green to red (high temperature etc)
      case 5:
        colour = rainbow(map(i, -angle, angle, 127, 63));
        break; // Red to green (low battery etc)
      default:
        colour = TFT_BLUE;
        break; // Fixed colour
      }
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, colour);
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, colour);
      //text_colour = colour; // Save the last colour drawn
    }
    else // Fill in blank segments
    {
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_GREY);
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_GREY);
    }
  }
  // Convert value to a string
  char buf[10];
  byte len = 3;
  if (value > 999)
    len = 5;
  dtostrf(value, len, 0, buf);
  buf[len] = ' ';
  buf[len + 1] = 0; // Add blanking space and terminator, helps to centre text too!
  // Set the text colour to default
  tft.setTextSize(1);
 
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  // Uncomment next line to set the text colour to the last segment value!
  tft.setTextColor(colour, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  // Print value, if the meter is large then use big font 8, othewise use 4
  if (r > 84)
  {
    tft.setTextPadding(55 * 3);   // Allow for 3 digits each 55 pixels wide
    tft.drawString(buf, x, y, 8); // Value in middle
  }
  else
  {
    tft.setTextPadding(3 * 14);   // Allow for 3 digits each 14 pixels wide
    tft.drawString(buf, x, y, 4); // Value in middle
  }
  tft.setTextSize(1);
  tft.setTextPadding(0);
  // Print units, if the meter is large then use big font 4, othewise use 2
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  if (r > 84)
    tft.drawString(units, x, y + 60, 4); // Units display
  else
    tft.drawString(units, x, y + 15, 2); // Units display
 
  // Calculate and return right hand side x coordinate
  return x + r;
}
 
// #########################################################################
// Return a 16 bit rainbow colour
// #########################################################################
unsigned int rainbow(byte value)
{
  // Value is expected to be in range 0-127
  // The value is converted to a spectrum colour from 0 = blue through to 127 = red
 
  byte red = 0;   // Red is the top 5 bits of a 16 bit colour value
  byte green = 0; // Green is the middle 6 bits
  byte blue = 0;  // Blue is the bottom 5 bits
 
  byte quadrant = value / 32;
 
  if (quadrant == 0)
  {
    blue = 31;
    green = 2 * (value % 32);
    red = 0;
  }
  if (quadrant == 1)
  {
    blue = 31 - (value % 32);
    green = 63;
    red = 0;
  }
  if (quadrant == 2)
  {
    blue = 0;
    green = 63;
    red = value % 32;
  }
  if (quadrant == 3)
  {
    blue = 0;
    green = 63 - 2 * (value % 32);
    red = 31;
  }
  return (red << 11) + (green << 5) + blue;
}
 
// #########################################################################
// Return a value in range -1 to +1 for a given phase angle in degrees
// #########################################################################
float sineWave(int phase)
{
  return sin(phase * 0.0174532925);
}
 
//====================================================================================
// This is the function to draw the icon stored as an array in program memory (FLASH)
//====================================================================================
 
// To speed up rendering we use a 64 pixel buffer
#define BUFF_SIZE 64
 
// Draw array "icon" of defined width and height at coordinate x,y
// Maximum icon size is 255x255 pixels to avoid integer overflow
 
void drawIcon(const unsigned short *icon, int16_t x, int16_t y, int8_t width, int8_t height)
{
 
  uint16_t pix_buffer[BUFF_SIZE]; // Pixel buffer (16 bits per pixel)
 
  tft.startWrite();
 
  // Set up a window the right size to stream pixels into
  tft.setAddrWindow(x, y, width, height);
 
  // Work out the number whole buffers to send
  uint16_t nb = ((uint16_t)height * width) / BUFF_SIZE;
 
  // Fill and send "nb" buffers to TFT
  for (int i = 0; i < nb; i++)
  {
    for (int j = 0; j < BUFF_SIZE; j++)
    {
      pix_buffer[j] = pgm_read_word(&icon[i * BUFF_SIZE + j]);
    }
    tft.pushColors(pix_buffer, BUFF_SIZE);
  }
 
  // Work out number of pixels not yet sent
  uint16_t np = ((uint16_t)height * width) % BUFF_SIZE;
 
  // Send any partial buffer left over
  if (np)
  {
    for (int i = 0; i < np; i++)
      pix_buffer[i] = pgm_read_word(&icon[nb * BUFF_SIZE + i]);
    tft.pushColors(pix_buffer, np);
  }
 
  tft.endWrite();
}

void sendSensor(){

  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(11);
  digitalWrite(TRIGGER, LOW);

  duration = pulseIn(ECHO, HIGH);
  distance = duration * USONIC_DIV/2;
  percentage = map(distance, 27, 5, 0,100);

  if(percentage < 0) {
    percentage = 0;
  } else if(percentage > 100) {
    percentage = 100;
  }

  if (distance > 20 ){
    digitalWrite(BUZZER, HIGH);
    delay(400);
    digitalWrite(BUZZER, LOW);
    delay(300);
    digitalWrite(BUZZER, HIGH);
    delay(400);
    digitalWrite(BUZZER, LOW);
    delay(300);
  } else {
    digitalWrite(BUZZER, LOW);
  }
  
  Serial.print("Percentage :");
  Serial.print(percentage);
  Serial.print("% Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  Blynk.virtualWrite(V4,percentage);
  delay(1000);
  
}


void onChange()
{
 
}
