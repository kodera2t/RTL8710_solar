#include <ESP_SSD1306.h>    
#include <Adafruit_GFX.h>   
#include <SPI.h>            
#include <Wire.h>
#include <WiFi.h>
#include <AllAboutEE_MCP3021.h>
//#include <Wire.h>

using namespace AllAboutEE;

MCP3021 mcp3021;



#define BME280_ADDRESS 0x76
#define red_led 6
#define green_led 7
#define blue_led 8
#define white_led 9

#define PIN_RESET 16  // Connect RST to pin 9
#define PIN_DC    11  // Connect DC to pin 8
#define PIN_CS    10 // Connect CS to pin 10
#define DC_JUMPER 0  // For I2C Communication - this pin pulled high by default
ESP_SSD1306 oled(PIN_RESET);    // I2C declaration

char ssid[] = "yourssid";  //  your network SSID (name)
char pass[] = "yourwifi_passwd";       // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status
WiFiServer server(80);
int count=1;
unsigned long time_1;
unsigned long interval = 3600000; //one hour interval
int disp;
unsigned int d,dh,dl;

unsigned long int hum_raw, temp_raw, pres_raw;
signed long int t_fine;
boolean disp_index = false;

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;
int8_t  dig_H1;
int16_t dig_H2;
int8_t  dig_H3;
int16_t dig_H4;
int16_t dig_H5;
int8_t  dig_H6;

void setup()
{
  uint8_t osrs_t = 1;             //Temperature oversampling x 1
  uint8_t osrs_p = 1;             //Pressure oversampling x 1
  uint8_t osrs_h = 1;             //Humidity oversampling x 1
  uint8_t mode = 3;               //Normal mode
  uint8_t t_sb = 5;               //Tstandby 1000ms
  uint8_t filter = 0;             //Filter off
  uint8_t spi3w_en = 0;           //3-wire SPI Disable

  uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
  uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
  uint8_t ctrl_hum_reg  = osrs_h;
//  pinMode(red_led, OUTPUT);
//  pinMode(green_led, OUTPUT);
//  pinMode(blue_led, OUTPUT);
//  pinMode(white_led, OUTPUT);

//  Serial.begin(9600);

//  Serial.print("Hello");

  Wire.begin();

  //blink_allLED();
  writeReg(0xF2, ctrl_hum_reg);
  writeReg(0xF4, ctrl_meas_reg);
  writeReg(0xF5, config_reg);
  readTrim();                    //
  oled.begin();
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(WHITE);
    oled.setCursor(0,0);
//  oled.clear(ALL);
  oled.setTextSize(1);
  //  delay(1000);
  //  oled.setCursor(90,0);
  //  oled.print("hPa");
  if (WiFi.status() == WL_NO_SHIELD) {
    //Serial.println("WiFi shield not present");
    while (true);       // don't continue
  }
  
  String fv = WiFi.firmwareVersion();
  if (fv != "1.1.0") {
    //Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    //Serial.print("Attempting to connect to Network named: ");
    //Serial.println(ssid);                   // print the network name (SSID);

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(3000);
  }
    printWifiStatus(); 
  server.begin();                           // start the web server on port 80

 // mcp3021.begin(0,0);
//      oled.setCursor(33, 48);
//    //  oled.print(hum_act);
//    oled.print(mcp3021.read(5,3.3));  
//    oled.display();
//  Wire.beginTransmission(0b1001101);
//  //Wire.write(0x88);//configuration register
//  Wire.endTransmission();
//  delay(200);

//  Wire.requestFrom(0b1001101,2);
//  dh = Wire.read();
//  dl = Wire.read();
//  d = dh *256 + dl;
//      oled.setCursor(33, 40);
////    //  oled.print(hum_act);
//    oled.print(dh); 
//          oled.setCursor(33, 48);
////    //  oled.print(hum_act);
//    oled.print(dl);  
//    oled.display();
//    delay(5000);
  oled.setCursor(33,40);
  oled.print("LiPo voltage");
  oled.setCursor(33,48);
  oled.print(mcp3021.read(5,3.3)*2);
  oled.print(" V");
  oled.display();
  readData();
}


void loop()
{
  //oled.fillRect(32,32,64,32,BLACK);
  double temp_act = 0.0, press_act = 0.0, hum_act = 0.0;
  signed long int temp_cal;
  unsigned long int press_cal, hum_cal;

  float index;
  //if(count%3000==0){


//  temp_cal = calibration_T(temp_raw);
//  press_cal = calibration_P(pres_raw);
//  hum_cal = calibration_H(hum_raw);
//  temp_act = (double)temp_cal / 100.0;
//  press_act = (double)press_cal / 100.0;
//  hum_act = (double)hum_cal / 1024.0;
//  index = 0.81 * temp_act + 0.01 * (0.99 * temp_act - 14.3) + 46.3;
  //}
  //    Serial.print("TEMP : ");
  //    Serial.print(temp_act);
  //    Serial.print(" DegC  PRESS : ");
  //    Serial.print(press_act);
  //    Serial.print(" hPa  HUM : ");
  //    Serial.print(index);
  //    Serial.println(" %");
//    if(count==1){
//    oled.fillRect(32,32,64,32,BLACK);
//    oled.setTextSize(1);
//    oled.setCursor(33, 32);
//    oled.print(press_act, 0);
//    oled.setCursor(33, 40);
//    oled.print(temp_act, 1);
//
//    //  oled.setTextSize(1);
//    oled.setCursor(70, 32);
//    oled.print("hPa");
//    oled.setCursor(70, 40);
//    oled.print("deg");
//    oled.setCursor(33, 48);
//    //  oled.print(hum_act);
//    oled.print(hum_act, 1);
//    oled.setCursor(70, 48);
//    oled.print("%");
////    oled.setCursor(33, 48);
////    //  oled.print(hum_act);
////    oled.print(mcp3021.read(5,3.3));    
//    oled.display();
//    }else{
////    oled.fillRect(32,32,64,32,BLACK);
////    oled.clearDisplay();
////    oled.display();
//    //delay(3000);
//    }
//    count++;
//  if (disp_index == false) {
 //   disp_index = not(disp_index);

    //

  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {  
  readData();
  temp_cal = calibration_T(temp_raw);
  press_cal = calibration_P(pres_raw);
  hum_cal = calibration_H(hum_raw);
  temp_act = (double)temp_cal / 100.0;
  press_act = (double)press_cal / 100.0;
  hum_act = (double)hum_cal / 1024.0;
  index = 0.81 * temp_act + 0.01 * (0.99 * temp_act - 14.3) + 46.3;
    // if you get a client,
    oled.fillRect(32,32,64,32,BLACK);
    oled.display();
    //oled.setCursor(32,32);
    //oled.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        //Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            //client.print("Click <a href=\"/H\">here</a> turn the LED on pin 13 on<br>");
            client.print("Measured values by BME280 with RTL8710<BR>");
            client.print("Full Solar-powered system<BR>");
            client.print("Pressure (hpa):");
            client.print(press_act);
            client.print("<br>");
            client.print("Temperature (deg):");            
            client.print(temp_act);
            client.print("<br>");
            client.print("Humidity (%):");            
            client.print(hum_act);
            client.print("<br>");
            client.print("LiPo battery voltage (V):");
            client.print(mcp3021.read(5,3.3)*2);
            client.print("<br>");            
               oled.fillRect(0,0,128,64,BLACK); 
            //client.print("Click <a href=\"/L\">here</a> turn the LED on pin 13 off<br>");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
//        if (currentLine.endsWith("GET /H")) {
//          digitalWrite(13, HIGH);               // GET /H turns the LED on
//        }
//        if (currentLine.endsWith("GET /L")) {
//          digitalWrite(13, LOW);                // GET /L turns the LED off
//        }
      }
    }
    // close the connection:
    client.stop();
    //Serial.println("client disonnected");
  }




//  delay(3000);
//count++;
}
void readTrim()
{
  uint8_t data[32], i = 0;                   // Fix 2014/04/06
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 24);      // Fix 2014/04/06
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }

  Wire.beginTransmission(BME280_ADDRESS);    // Add 2014/04/06
  Wire.write(0xA1);                          // Add 2014/04/06
  Wire.endTransmission();                    // Add 2014/04/06
  Wire.requestFrom(BME280_ADDRESS, 1);       // Add 2014/04/06
  data[i] = Wire.read();                     // Add 2014/04/06
  i++;                                       // Add 2014/04/06

  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(0xE1);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 7);       // Fix 2014/04/06
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }
  dig_T1 = (data[1] << 8) | data[0];
  dig_T2 = (data[3] << 8) | data[2];
  dig_T3 = (data[5] << 8) | data[4];
  dig_P1 = (data[7] << 8) | data[6];
  dig_P2 = (data[9] << 8) | data[8];
  dig_P3 = (data[11] << 8) | data[10];
  dig_P4 = (data[13] << 8) | data[12];
  dig_P5 = (data[15] << 8) | data[14];
  dig_P6 = (data[17] << 8) | data[16];
  dig_P7 = (data[19] << 8) | data[18];
  dig_P8 = (data[21] << 8) | data[20];
  dig_P9 = (data[23] << 8) | data[22];
  dig_H1 = data[24];
  dig_H2 = (data[26] << 8) | data[25];
  dig_H3 = data[27];
  dig_H4 = (data[28] << 4) | (0x0F & data[29]);
  dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F); // Fix 2014/04/06
  dig_H6 = data[31];                                   // Fix 2014/04/06
}
void writeReg(uint8_t reg_address, uint8_t data)
{
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(reg_address);
  Wire.write(data);
  Wire.endTransmission();
}


void readData()
{
  int i = 0;
  uint32_t data[8];
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 8);
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }
  pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
  temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
  hum_raw  = (data[6] << 8) | data[7];
}


signed long int calibration_T(signed long int adc_T)
{

  signed long int var1, var2, T;
  var1 = ((((adc_T >> 3) - ((signed long int)dig_T1 << 1))) * ((signed long int)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T >> 4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;

  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

unsigned long int calibration_P(signed long int adc_P)
{
  signed long int var1, var2;
  unsigned long int P;
  var1 = (((signed long int)t_fine) >> 1) - (signed long int)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((signed long int)dig_P6);
  var2 = var2 + ((var1 * ((signed long int)dig_P5)) << 1);
  var2 = (var2 >> 2) + (((signed long int)dig_P4) << 16);
  var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((signed long int)dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((signed long int)dig_P1)) >> 15);
  if (var1 == 0)
  {
    return 0;
  }
  P = (((unsigned long int)(((signed long int)1048576) - adc_P) - (var2 >> 12))) * 3125;
  if (P < 0x80000000)
  {
    P = (P << 1) / ((unsigned long int) var1);
  }
  else
  {
    P = (P / (unsigned long int)var1) * 2;
  }
  var1 = (((signed long int)dig_P9) * ((signed long int)(((P >> 3) * (P >> 3)) >> 13))) >> 12;
  var2 = (((signed long int)(P >> 2)) * ((signed long int)dig_P8)) >> 13;
  P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
  return P;
}

unsigned long int calibration_H(signed long int adc_H)
{
  signed long int v_x1;

  v_x1 = (t_fine - ((signed long int)76800));
  v_x1 = (((((adc_H << 14) - (((signed long int)dig_H4) << 20) - (((signed long int)dig_H5) * v_x1)) +
            ((signed long int)16384)) >> 15) * (((((((v_x1 * ((signed long int)dig_H6)) >> 10) *
                (((v_x1 * ((signed long int)dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) *
                ((signed long int) dig_H2) + 8192) >> 14));
  v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)dig_H1)) >> 4));
  v_x1 = (v_x1 < 0 ? 0 : v_x1);
  v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
  return (unsigned long int)(v_x1 >> 12);
}



void blink_allLED(void) {
  int delay_time = 20;
  for (int i = 1; i < 5; i++) {
    led_blink(red_led, delay_time);
    led_blink(green_led, delay_time);
    led_blink(blue_led, delay_time);
    led_blink(white_led, delay_time);
  }
}


void led_blink(int led_num, int delay_time2) {
  digitalWrite(led_num, HIGH);
  delay(delay_time2);
  digitalWrite(led_num, LOW);
  delay(delay_time2);
}

void printWifiStatus() {
  IPAddress ip = WiFi.localIP();
  oled.setCursor(22,32);
  oled.print(ip);
  oled.display();
  long rssi = WiFi.RSSI();
  delay(2000);
}
