// Tanana AQ sensor
// Bill Simpson (wrsimpson@alaska.edu)
// v1.3 14 Nov 2021
// Uses Plantower PMS5003 air quality sensor
//
// The 3D print files for the case can be found here
// https://www.thingiverse.com/thing:5135798
// The case is a remix of one by creator "stevesch" -- see link above.

#include <SPIFFS.h>
#include <TFT_eSPI.h>   /// by Bodmer, Version 2.3.70
//After loading this, go in the Arduino libraries, find the folder for TFT_eSPI, and edit the file called "User_Setup_Select.h". 
// Comment OUT the line that says:
// #include <User_Setup.h>           // Default setup is root library folder
// Remove the comment slashes from (e.g. activate) the line that is below.
// #include <User_Setups/Setup25_TTGO_T_Display.h>    // Setup file for ESP32 and TTGO T-Display ST7789V SPI bus TFT
#include <SoftwareSerial.h>
// Note that I'm using "ESPSoftwareSerial" by Dirk Kaar, Version 6.14.1
// https://github.com/plerup/espsoftwareserial/
#include <Button2.h>  // Button2 by Lennart Hennings, Version 1.6.5
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <time.h>
#include <Preferences.h>

// defines
#define VERSION "Version 1.3.1" // string for version number
#define PM_SET 27 // ESP32 pin for the SET command of the PM monitor
#define PM_TX 26 // ESP32 pin for the TX to the PM monitor
#define PM_RX 25 // ESP32 pin for the RX from the PM monitor
#define BATT_ADC_EN          14   // TTGO T-display battery enable (set high to measure Battery even if on USB power)
#define BATT_ADC_PIN         34   // TTGO T-display Battery/bus power check ADC pin
#define BATT_CHK_TIMEOUT 5000     // milliseconds between battery checks
#define BATT_THRESH_MV  4100      // threshold below which we are battery powered
#define BATT_LOWV_MV  3000        // threshold below which we turn the device fully off
#define BATT_LOWV_TRIES 5         // number of times battery must read below the lowv threshold to shutdown
#define TFT_MAIN 0 // an enumerator for which TFT screen to show...
#define TFT_WIFI 1 
#define TFT_HARDWARE 2 
#define TFT_WARNING 3
#define TFT_BL_TIMER 0  // ESP32 timer for PWM dimming of the TFT backlight.
#define TFT_BL_DIM  4 // value for "dim"
#define TFT_BL_MED  128 // value for "medium"
#define TFT_BL_HI  255 // value for "hi"
#define AVGTIME_S_LONG 60  // seconds to average in long run (slow) mode
#define AVGTIME_S_FAST 3   // seconds to average in fast mode
#define SCREENHOLD_MILLIS 15000 // how long to hold the last button push
#define BUTTON_TOP_PIN 0 // ESP32 T-Display button 
#define BUTTON_BOTTOM_PIN 35  // ESP32 T-Display button 
#define NANVAL -9999 // a value for NAN, but it will generally print "NaN"
#define STARTOFTIME 1577869200 // This is 1 Jan 2020, AKST

// internet and time.  Note that ssid and password are handled by Preferences.h
char ssid[32];
char password[32];
char hostname[] = "TananaAQ-XX-XX-XX";
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -9*3600;
const int   daylightOffset_sec = 3600;
#define TZ_ST "AKST"
#define TZ_DT "AKDT"

// averaging
long avgtime_s = AVGTIME_S_LONG;
time_t epoch_start = 0;
int last_timemod = avgtime_s;
long sum_aqi, avg_aqi;
int count_n_summed;

// drivers / objects:
TFT_eSPI tft = TFT_eSPI();
SoftwareSerial pmSerial;
Button2 buttonTOP, buttonBOTTOM;
WebServer server(80);
String webpage;
Preferences preferences;

// Adapted PM serial data reading from Adafruit_PM25AQI driver
// Structure holding Plantower's standard packet 
typedef struct PMSdata {
  uint16_t framelen;       ///< How long this data chunk is
  uint16_t pm10_standard,  ///< Standard PM1.0
      pm25_standard,       ///< Standard PM2.5
      pm100_standard;      ///< Standard PM10.0
  uint16_t pm10_env,       ///< Environmental PM1.0
      pm25_env,            ///< Environmental PM2.5
      pm100_env;           ///< Environmental PM10.0
  uint16_t particles_03um, ///< 0.3um Particle Count
      particles_05um,      ///< 0.5um Particle Count
      particles_10um,      ///< 1.0um Particle Count
      particles_25um,      ///< 2.5um Particle Count
      particles_50um,      ///< 5.0um Particle Count
      particles_100um;     ///< 10.0um Particle Count
  uint16_t unused;         ///< Unused
  uint16_t checksum;       ///< Packet checksum
} PMS_Data;

// global variables
PMS_Data pmdata;
unsigned char pmdatabuffer[128];
int ix_pmdata = 0;
int loopcount = 0;
int tft_mode = TFT_MAIN;
unsigned long timeout_millis;
File logfile;
uint64_t batt_timer_millis = 0;
bool batt_powered;
int batt_lowv_count = 0;

// fragments of the html pages
char html_pre_title[] = {"<html>\n<head>\n<meta content=\"text/html; charset=ISO-8859-1\" http-equiv=\"content-type\">\n"};
char html_reload[] = {"<meta http-equiv=\"refresh\" content=\"5\">\n"};
char html_post_title[] = {"</title>\n</head>\n<body>\n"};
char html_ending[] = {"</body>\n</html>\n"};

// subroutines

void setEpochTime(time_t utc_epoch) {
  struct timeval tval;
  if (utc_epoch < STARTOFTIME) utc_epoch = STARTOFTIME;  // do not allow time before STARTOFTIME
  tval.tv_sec = utc_epoch;  // UTC epoch time (seconds)
  tval.tv_usec = 0;    // microseconds
  settimeofday(&tval, NULL);   // NULL specifies that this time is UTC
}

void correct_crazy_time() {
  time_t currepoch;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    setEpochTime(STARTOFTIME);   // do not allow time before STARTOFTIME
  }
  time(&currepoch);
  if (currepoch < STARTOFTIME) setEpochTime(STARTOFTIME);  // do not allow time before STARTOFTIME
}

void espLightSleepDelay(int ms) {   
  esp_sleep_enable_timer_wakeup(ms * 1000);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,ESP_PD_OPTION_ON);
  esp_light_sleep_start();
}

void set_tft_brightness(uint16_t bright) {
  ledcWrite(TFT_BL_TIMER, bright); // TFT brightness, which varies 0 to 255
}

void config_tft_bl() {
  pinMode(TFT_BL, OUTPUT);
  ledcSetup(TFT_BL_TIMER, 5000, 8); // set up timer at 5000Hz, 8 bit resolution
  ledcAttachPin(TFT_BL, TFT_BL_TIMER); // attach TFT_BL pin to its timer
}

String localDateTimeString(){
  struct tm timeinfo;
  char buf[30]; 

  getLocalTime(&timeinfo);
  strftime(buf, 30,"%Y-%m-%d %H:%M:%S\t", &timeinfo);
  if (timeinfo.tm_isdst) snprintf(&(buf[20]),5,TZ_DT);
  else snprintf(&(buf[20]),5,TZ_ST);
  return(String(buf));
}

String epochToDateTimeString(time_t utc_epoch){
  struct tm* tm_ptr;
  char buf[30]; 

  tm_ptr = localtime(&utc_epoch);
  strftime(buf, 30,"%Y-%m-%d %H:%M:%S,", tm_ptr);
  if (tm_ptr->tm_isdst) snprintf(&(buf[20]),5,TZ_DT);
  else snprintf(&(buf[20]),5,TZ_ST);
  return(String(buf));
}

void printLocalDateTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Time not available");
    return;
  }
  Serial.print(&timeinfo, "%Y-%m-%d %H:%M:%S\t");
  if (timeinfo.tm_isdst) Serial.println(TZ_DT);
  else Serial.println(TZ_ST);
}

void tftdisplayLocalTime(){
  struct tm timeinfo;
  char timechars[10];
  String timestr;

  if(!getLocalTime(&timeinfo)){
    timestr = "-----";
  }
  else {
    strftime(timechars, 10, "%H:%M:%S", &timeinfo);
    timestr = String(timechars);
  }
  tft.setTextDatum(TC_DATUM);
  tft.setTextFont(7);  
  tft.drawString(timestr, tft.width() / 2, 30);
}

void control_powersave_mode(bool powersave) {
  if (batt_powered == powersave) return;
  batt_powered = powersave;
  if (batt_powered) {    // this is entering powersave mode
    set_tft_brightness(TFT_BL_DIM); // Set to DIM intensity
    digitalWrite (PM_SET, LOW); // Send the sleep signal to the PM sensor, which is the "SET" signal into low
    WiFi.disconnect();  // disconnect WiFi
    WiFi.mode( WIFI_MODE_NULL );  // turn wifi radio off
  }
  else {    // this is exiting powersave mode
    set_tft_brightness(TFT_BL_MED); // set tft to normal intensity      
    digitalWrite (PM_SET, HIGH); // Turn the PM sensor on, which is "SET" = high
    start_wifi(false);  // start wifi from after being off
  }
}

void check_batt_powered() {
  if ((millis() - batt_timer_millis) > BATT_CHK_TIMEOUT) {
    batt_timer_millis = millis();
	  uint16_t batt_mv = (analogRead(BATT_ADC_PIN) * 2 * 3300) / 4095;
//	  Serial.println("Battery :" + String(batt_mv) + " mV");
	  if (batt_mv > BATT_THRESH_MV) {
      control_powersave_mode(false);
    }
    else {
      control_powersave_mode(true);
      show_tft_pwrsave();  // display message
    }
  }
}

void clear_aq_sum() {
  sum_aqi = 0;
  count_n_summed = 0;
}

void coadd_aq() {
  sum_aqi += aqi_from_pm25(pmdata.pm25_standard);
  count_n_summed++;
}

void write_aq(time_t epoch_start) {
  if (count_n_summed > 0) {
    avg_aqi = sum_aqi / count_n_summed;
  }
  else { 
    avg_aqi = NANVAL;
  }
  logfile.print(epochToDateTimeString(epoch_start)+",");
  if (avg_aqi != NANVAL) 
    logfile.println(String(avg_aqi));
  else 
    logfile.println("NaN");
  Serial.println("Wrote a line to the data file with timestamp " + epochToDateTimeString(epoch_start));
}

void try_write_avg() {
  time_t curr_epoch, curr_epoch_start;
  int curr_timemod; // time modulo the avgtime_s

  time(&curr_epoch);
  curr_timemod = curr_epoch % avgtime_s;
  if (curr_timemod < last_timemod) {  // modulotime reduced, meaning we went to the next averaging period
    curr_epoch_start = avgtime_s * (curr_epoch / avgtime_s);
    if ( curr_epoch_start == (epoch_start + avgtime_s) ) {  // we moved into the next avgtime, report data
      write_aq(epoch_start);
    }
    clear_aq_sum();
    epoch_start = curr_epoch_start;
  }
  last_timemod = curr_timemod;
}

// Breakpoints for AQI from PM2.5.  Note the _hi breakpoints are larger by 0.1 and 1
// See https://forum.airnowtech.org/t/the-aqi-equation/169
static float pm_conc_breakpoint_lo[8] = {0, 12.1, 35.5, 55.5, 150.5, 250.5, 500.5, 10000.5};
static float pm_aqi_breakpoint_lo[8] = {0, 51, 101, 151, 201, 301, 501, 10001};

float aqi_from_pm25(float pm25) {
  int i = 0;

  float pm25_trunc = float( floor(pm25*10) / 10);
  while (pm25_trunc >= pm_conc_breakpoint_lo[i]) i++;
  float conc_hi = pm_conc_breakpoint_lo[i]-0.1;
  float conc_lo = pm_conc_breakpoint_lo[i-1];
  float aqi_hi = pm_aqi_breakpoint_lo[i]-1;
  float aqi_lo = pm_aqi_breakpoint_lo[i-1];
  return ( (aqi_hi-aqi_lo)/(conc_hi-conc_lo)*(pm25_trunc-conc_lo)+aqi_lo ); 
}

bool pollpmserial(PMS_Data *data) {
  while (pmSerial.available() && (ix_pmdata < 120) ) { // read all available serial data into the buffer
    pmdatabuffer[ix_pmdata] = pmSerial.read();
// code to print the last character received in Hex.
//    char buffer[4];
//    sprintf (buffer, "%02x ", pmdatabuffer[ix_pmdata]);
//    Serial.print(buffer);
    ix_pmdata++;
  }
  if (ix_pmdata >= 32) { // there is enough data for a frame
    int ix_start = ix_pmdata - 32;
    bool seeking = true;
    int ix_frame = -1;
    while (seeking) {      
      if ( (pmdatabuffer[ix_start] == 0x42) && (pmdatabuffer[ix_start+1] == 0x4d) ) {
        ix_frame = ix_start;
        seeking = false;
      }
      ix_start--;
      if ( ix_start < 0) seeking = false;
    }
    if (ix_frame >=0) {
      // calculate checksum
      uint16_t chksum = 0;
      for (int i = 0; i < 30; i++) {
        chksum += pmdatabuffer[i+ix_frame];
      }
      // from LadyAda's Adafruit driver code
      // The data comes in endian'd, this solves it so it works on all platforms
      uint16_t buffer_u16[15];
      for (uint8_t i = 0; i < 15; i++) {
        buffer_u16[i] = pmdatabuffer[ix_frame + 2 + i * 2 + 1];
        buffer_u16[i] += (pmdatabuffer[ix_frame + 2 + i * 2] << 8);
      }
      // copy data into the PMS_data structure
      memcpy((void *)data, (void *)buffer_u16, 30); 
      // clear front of data buffer moving newer characters back
      int j = 0;
      for (int i = (ix_frame+32); i<ix_pmdata; i++) {
        pmdatabuffer[j] = pmdatabuffer[i];
        j++;
      }
      ix_pmdata = j;
      if (chksum != data->checksum) return false;
      else return true;
    }
    if (ix_pmdata > 64) {
      // buffer too large, chew it back, keep last 32 characters
      for (int i = 0; i<32; i++) {
        pmdatabuffer[i] = pmdatabuffer[i+ix_pmdata-32];
      }
      ix_pmdata = 32;
      return false;
    }
    return false;
  }
  return false;
}

void tap_handler(Button2& btn) {
  if (btn == buttonTOP) {
    Serial.println("Top button clicked");
    switch (tft_mode) {
      case TFT_MAIN:
        tft_mode = TFT_WIFI;
        show_tft_wifi();
        break;
      case TFT_WIFI:
        tft_mode = TFT_HARDWARE;
        show_tft_hardware();
        break;
      case TFT_HARDWARE:
      default:
        tft_mode = TFT_MAIN;
    }
    timeout_millis = millis() + SCREENHOLD_MILLIS;
  }
}

void pressed_handler(Button2& btn) {
  if (btn == buttonBOTTOM) {
    Serial.println("Bottom button clicked");
    tft_mode = TFT_WARNING;
    timeout_millis = millis() + 5000;
    show_tft_warning();
  }
}

void longpress_handler(Button2& btn) {
  if (btn == buttonBOTTOM) {
    unsigned int time = btn.wasPressedFor();
    Serial.println("Very long press on the lower button to reset");
    if (time > 15000) {
      ESP.restart();
    }
    if (time > 5000) {
      Serial.println("Restarting WiFi");
      start_wifi(true);
    }
  }
}

void show_tft_main() {
  // Clear screen
  tft.fillScreen(TFT_NAVY);
  tft.setTextColor(TFT_YELLOW,TFT_NAVY);
  // print the name of the device
  tft.setTextDatum(TC_DATUM);
  tft.setTextFont(4);  // font size 4 has all characters, others don't.
  tft.drawString(hostname, tft.width() / 2, 5);    
  //note that the time print has to happen after the clear screen and color setup
  tftdisplayLocalTime();
  // Show PM data as AQI
  tft.setTextDatum(TL_DATUM);
  tft.setTextFont(4);  // font size 4 has all characters, others don't.
  if (avgtime_s == AVGTIME_S_LONG) tft.drawString("Long", 10, 82);
  else tft.drawString("Fast", 10, 82);
  tft.drawString("mode", 10, 108);
  tft.setTextDatum(TR_DATUM);
  tft.drawString("AQI", tft.width() / 2 + 10, 98);
  tft.setTextFont(7);  
  String aqi_string = String(((int)aqi_from_pm25(pmdata.pm25_standard)));
  tft.drawString(aqi_string, tft.width() - 5, tft.height() / 2 + 20);
}

void show_tft_wifi() {
  // Clear screen
  tft.fillScreen(TFT_NAVY);
  tft.setTextColor(TFT_YELLOW,TFT_NAVY);
  // print the name of the device
  tft.setTextDatum(TC_DATUM);
  tft.setTextFont(4);  // font size 4 has all characters and is 26 pixels high.
  tft.drawString(hostname, tft.width() / 2, 0);    
  tft.drawString("Network:", tft.width() / 2, 27); 
  if (WiFi.status() == WL_CONNECTED) { // we are in connected mode
    tft.drawString(WiFi.SSID(), tft.width() / 2, 54);
    tft.drawString("Connect to:", tft.width() / 2, 81);\
    String URL_string = String("http://"+WiFi.localIP().toString()+"/");
    tft.drawString(URL_string, tft.width() / 2, 108);
    // information to serial port
    Serial.println("Current local IP address: "+WiFi.localIP().toString());
  }
  else { // we are in self-hosted mode
    tft.drawString(WiFi.softAPSSID(), tft.width() / 2, 54);
    tft.drawString("Connect to:", tft.width() / 2, 81);\
    String URL_string = String("http://"+WiFi.softAPIP().toString()+"/");
    tft.drawString(URL_string, tft.width() / 2, 108);
    // information to serial port
    Serial.println("Current local IP address: "+WiFi.softAPIP().toString());
  }
}

void show_tft_hardware() {
  // Clear screen
  tft.fillScreen(TFT_NAVY);
  tft.setTextColor(TFT_YELLOW,TFT_NAVY);
  // print the name of the device
  tft.setTextDatum(TC_DATUM);
  tft.setTextFont(4);  // font size 4 has all characters and is 26 pixels high.
  tft.drawString(VERSION, tft.width() / 2, 0);    
  tft.drawString("MAC Address:", tft.width() / 2, 27); 
  tft.drawString(WiFi.macAddress(), tft.width() / 2, 54);
  tft.drawString("Mem used/avail:", tft.width() / 2, 81);\
  String mem_string = String(SPIFFS.usedBytes()/1024)+"k / "+String(SPIFFS.totalBytes()/1024)+"k";
  tft.drawString(mem_string, tft.width() / 2, 108);
}

void show_tft_warning() {
  // Clear screen
  tft.fillScreen(TFT_NAVY);
  tft.setTextColor(TFT_YELLOW,TFT_NAVY);
  tft.setTextDatum(TC_DATUM);
  tft.setTextFont(4);  // font size 4 has all characters and is 26 pixels high.
  tft.drawString("HOLD 5 SEC", tft.width() / 2, 20);    
  tft.drawString("TO RESET WIFI", tft.width() / 2, 55);
  tft.drawString("15 SEC TO REBOOT", tft.width() / 2, 100);
}

void show_tft_wifi_reset() {
  // Clear screen
  tft.fillScreen(TFT_NAVY);
  tft.setTextColor(TFT_YELLOW,TFT_NAVY);
  tft.setTextDatum(TC_DATUM);
  tft.setTextFont(4);  // font size 4 has all characters and is 26 pixels high.
  tft.drawString(hostname, tft.width() / 2, 20); 
  tft.drawString("RESETTING WIFI", tft.width() / 2, 55);
  tft.drawString("please be patient", tft.width() / 2, 100);
}

void show_tft_pwrsave() {
  // Clear screen
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_YELLOW,TFT_BLACK);
  tft.setTextDatum(TC_DATUM);
  tft.setTextFont(4);  // font size 4 has all characters and is 26 pixels high.
  tft.drawString(hostname, tft.width() / 2, 5);    
  //note that the time print has to happen after the clear screen and color setup
  tftdisplayLocalTime();
  tft.setTextDatum(TC_DATUM);
  tft.setTextFont(4);  // font size 4 has all characters and is 26 pixels high.
  tft.drawString("POWERSAVE MODE", tft.width() / 2, 82);
  tft.drawString("USB power to run", tft.width() / 2, 108);
}

void setdatetime(String in_date, String in_time) {
  char in_buf[80];
  struct tm in_tm;
  time_t epochUTC;
  bool parsedtime = false;
  int arg1,arg2,arg3;

  getLocalTime(&in_tm);  // set the time to the current local time (believed)
  // I found that all elements of the "struct tm" must be set, including seconds
  String in_dt = in_date+" "+in_time;
  Serial.println("Received local date/time as ["+in_dt+"]");
  in_dt.toCharArray(in_buf,80);
  if ( strptime(in_buf, "%Y-%m-%d %H:%M", &in_tm) != NULL ) {
    epochUTC = mktime(&in_tm); 
    Serial.println("Parsed datetime to "+epochToDateTimeString(epochUTC));
    setEpochTime(epochUTC);      // set the datetime in the internal clock
    Serial.print("set time to ");
    Serial.println(localDateTimeString());
  }
}

void setwificreds(String in_ssid, String in_pass) {
  Serial.println("SSID received as: "+in_ssid);
  Serial.println("password received as: "+in_pass);
  in_ssid.toCharArray(ssid,30);
  in_pass.toCharArray(password,30);
  storeWiFiCreds();
  Serial.println("saved ssid and password -- restarting wifi with new creds");
  delay(250);
  start_wifi(true);
}

void wipelog(String wipephrase) {
  if (wipephrase == "WIPELOG") {
    Serial.println("User requested to delete the logfile -- doing that");
    logfile.close();
    delay(250);
    SPIFFS.remove("/logfile");
    delay(250);
    logfile = SPIFFS.open("/logfile", "w");
    logfile.println("datetime,timezone,aqi");
  }
  // secret format option
  if (wipephrase = "FORMAT!") {
    Serial.println("User requested the disk to be formatted");
    // unmount (end) SPIFFS
    SPIFFS.end();
    delay(250);
    SPIFFS.format();
    delay(250);
    SPIFFS.begin();
    delay(250);
    logfile = SPIFFS.open("/logfile", "w");
    logfile.println("datetime,timezone,aqi");
  }

}

void toggleavg() {
  if (avgtime_s == AVGTIME_S_LONG) avgtime_s = AVGTIME_S_FAST;
  else avgtime_s = AVGTIME_S_LONG;
}

void handleRoot() {
  // parse values
  if (server.hasArg("in_date") && server.hasArg("in_time")) setdatetime(server.arg("in_date"),server.arg("in_time")); 
  if (server.hasArg("ssid") && (server.arg("ssid").length()>0)) setwificreds(server.arg("ssid"),server.arg("password")); 
  if (server.hasArg("wipephrase")) wipelog(server.arg("wipephrase")); 
  if (server.hasArg("toggleavg")) toggleavg();
  webpage = "";  //clear webpage
  webpage += html_pre_title;
  webpage += html_reload;
  webpage += "<title>TananaAQ Sensor</title>\n";
  webpage += html_post_title;
  webpage += "<H1>Welcome to the TananaAQ sensor homepage</H1>\n";
  webpage += "<H2>"+localDateTimeString()+"<\H2>\n";
  webpage += "<H2>AQI : "+String(((int)aqi_from_pm25(pmdata.pm25_standard)))+"</H2>\n";
  if (avgtime_s == AVGTIME_S_LONG) webpage+= "<H2>LONG (60s) averaging mode<\H2>\n";
  else webpage+= "<H2>FAST (3s) averaging mode<\H2>\n";
  webpage += "<form action=\"/\" method=\"post\"><p>\n";
  webpage += "<input type=\"submit\" name=\"toggleavg\" value=\"Toggle averaging mode\"></p>\n";
  webpage += "</form>\n";
  if (batt_powered) webpage+= "<H2>ON BATTERY POWER<\H2>\n";
  else webpage+= "<H2>Normal USB-C powered<\H2>\n";
  webpage += "<H2>Used space             : "+String(SPIFFS.usedBytes()/1024)+"k</H2>\n";
  webpage += "<H2>Filesystem total space : "+String(SPIFFS.totalBytes()/1024)+"k</H2>\n";
  webpage += "<H2><A HREF=\"/logfile\">View the logfile</A></H2>\n";
  webpage += "<H2>You can copy and paste data from the logfile into a spreadsheet for analysis</H2>\n";
  webpage += "<H2><A HREF=\"/settime\">Click here to set the time</A></H2>\n";
  webpage += "<H2><A HREF=\"/setwifi\">Click here to set the WiFi credentials</A></H2>\n";
  webpage += "<H2><A HREF=\"/wipelog\">Click here to wipe (delete) the log file (will prompt for verification)</A></H2>\n";
  webpage += html_ending;
  server.send(200, "text/html", webpage);
}

void handleSettime() {
  Serial.println("generating set time webpage");
  webpage = "";  //clear webpage
  webpage += html_pre_title;
  webpage += "<title>Set time page</title>\n";
  webpage += html_post_title;
  webpage += "<H1>Enter the current time local timezone</H1>\n";
  webpage += "<H2>Leave either blank to keep prior time</H2>\n";
  webpage += "<form action=\"/\" method=\"post\">\n<p>\n<label>Date:</label>\n";
  webpage += "<input type=\"date\" name=\"in_date\"><br>\n";
  webpage += "<form action=\"/\" method=\"post\">\n<p>\n<label>Time:</label>\n";
  webpage += "<input type=\"time\" name=\"in_time\"><br>\n";
  webpage += "<input type=\"submit\" value=\"Set Date/Time\">\n</p>\n</form>\n";
  webpage += html_ending;
  server.send(200, "text/html", webpage);
}

void handleSetwifi() {
  Serial.println("generating set wifi webpage");
  webpage = "";  //clear webpage
  webpage += html_pre_title;
  webpage += "<title>Set wifi page</title>\n";
  webpage += html_post_title;
  webpage += "<h1>Enter your WiFi credentials</h1>\n";
  webpage += "<h2>To select self-hosted access point mode, enter \"selfhost\" for SSID.<BR>\n";
  webpage += "To cancel (keep prior network), leave SSID blank</h2>\n";  
  webpage += "<form action=\"/\" method=\"post\">\n";
  webpage += "<p>\n<label>Network name (SSID):</label>\n<input maxlength=\"30\" name=\"ssid\"><br>\n";
  webpage += "<label>Key (network password):</label><input maxlength=\"30\" name=\"password\"><br>\n";
  webpage += "<input type=\"submit\" value=\"Save\">\n</p>\n</form>\n";
  webpage += html_ending;
  server.send(200, "text/html", webpage);
}

void handleWipeLog() {
  Serial.println("generating wipelog verification webpage");
  webpage = "";  //clear webpage
  webpage += html_pre_title;
  webpage += "<title>Wipe Log Verify Page</title>\n";
  webpage += html_post_title;
  webpage += "<h1>This action will delete the old logfile</h1>\n";
  webpage += "<h2>THERE IS NO WAY TO RECOVER THE DELETED DATA<BR>\n";
  webpage += "To confirm, type \"WIPELOG\"<BR>Leave blank or enter anything else to cancel</h2>\n";  
  webpage += "<form action=\"/\" method=\"post\">\n";
  webpage += "<p>\n<label>Confirmation key:</label>\n<input maxlength=\"30\" name=\"wipephrase\">\n";
  webpage += "<input type=\"submit\" value=\"Confirm delete log\">\n</p>\n</form>\n";
  webpage += html_ending;
  server.send(200, "text/html", webpage);
}

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

bool handleFileRead(String path) {
  bool retval = false; // default is failure

  Serial.println("handleFileRead: " + path);
  if (path == "/logfile") logfile.close();  // close the logfile if we are being asked to read it.
  File file = SPIFFS.open(path, "r");
  if (!file.isDirectory()) {
    server.streamFile(file, "text/plain");
    file.close();
    retval = true;
  }
  if (path == "/logfile") logfile = SPIFFS.open("/logfile", "a");
  return retval;
}

bool storeWiFiCreds() {
  preferences.putString("ssid", String(ssid)); 
  preferences.putString("password", String(password));
}

bool getWiFiCreds(){
  preferences.getString("ssid", "").toCharArray(ssid, 30); 
  preferences.getString("password", "").toCharArray(password, 30);
  Serial.print("Retrieved ssid:");
  Serial.println(ssid);
  Serial.print("Number of characters in password is: ");
  Serial.println(strlen(password));
}

void start_wifi(bool reset_wifi) {
  if (reset_wifi) {
    WiFi.disconnect();
    delay(500);
  }
  getWiFiCreds(); // retrieve Creds

  show_tft_wifi_reset();
  // try station mode first
  WiFi.mode(WIFI_STA);
  if (strlen(password)>0) {
    WiFi.begin(ssid, password);
  } else {
    Serial.print("Connecting without password to: ");
    Serial.println(ssid);
    WiFi.begin(ssid);
  }
  
  Serial.print("Attempting to connect as station ");
  int tries = 30;
  while ( (WiFi.status() != WL_CONNECTED) && (tries > 0) ) {
  // Loop to attempt STAtion mode connection
    delay(500);
    tries--;
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to "+String(ssid));
  }
  else {
    Serial.println("\nStation mode failed, reverting to access point as: "+String(hostname));
    WiFi.mode(WIFI_AP);
    WiFi.softAP(hostname, NULL);
  }
}

void setup(void) {
  delay(5000);
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Welcome users
  Serial.println("Tanana Air Quality PM2.5 Sensor");
  Serial.println("using the Plantower PMS5003");

  // setup to measure the bus (batt or USB) voltage always
  pinMode(BATT_ADC_EN, OUTPUT);
  digitalWrite(BATT_ADC_EN, LOW);  // always measure the bus voltage (battery or USB)
  // setup to be able to turn the PM sensor on and off with its "SET" pin
  pinMode (PM_SET, OUTPUT);
  digitalWrite (PM_SET, HIGH);  // Turn the sensor on 

  // Start the TFT display
  tft.init();
  // set up brightness with dimmer
  config_tft_bl();
  set_tft_brightness(TFT_BL_MED); // 0 (off) to 255
  // show splash screen
  tft.setRotation(3);
  tft.fillScreen(TFT_NAVY);
  tft.setCursor(20, 20);
  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(TFT_YELLOW, TFT_NAVY);    
  tft.setTextFont(4);
  tft.drawString("Tanana", tft.width() / 2, 20); 
  tft.drawString("AQ sensor", tft.width() / 2, 55); 
  tft.drawString("...... starting", tft.width() / 2, 90); 
  delay(5000);
  
  //Setup buttons
  Serial.println("Enabling buttons");
  buttonTOP.begin(BUTTON_TOP_PIN);
  buttonTOP.setTapHandler(tap_handler);
  buttonBOTTOM.begin(BUTTON_BOTTOM_PIN);
  buttonBOTTOM.setPressedHandler(pressed_handler);
  buttonBOTTOM.setLongClickHandler(longpress_handler);

  // start the file system
  if (!SPIFFS.begin()) {
    Serial.println("SPIFFS failed to begin, so we will format it");
    if (SPIFFS.format()) {
      Serial.println("SPIFFS formatted successfully");
      SPIFFS.begin();
      logfile = SPIFFS.open("/logfile", "w");
      logfile.println("datetime,timezone,aqi");
      logfile.close();
    }
    else {
      Serial.println("SPIFFS seems very broken -- failing to use the filesystem");
    }
  }

  // open the logfile to append data
  logfile = SPIFFS.open("/logfile", "a");
  logfile.println(" "); // start a new line because power downs can leave partial lines

  // start preferences credentials namespace for storing WiFi credentials
  preferences.begin("credentials", false);

  // start the WiFi system
  start_wifi(false);

  // print out the MAC id if needed for whitelisting on a network
  Serial.print("WiFi MAC address: ");
  String mac = WiFi.macAddress();
  Serial.println(mac);
  // Generate instrument unique name from MAC address of ESP32
  char *mac_ending = &mac[9];
  mac_ending[2]='-'; mac_ending[5]='-';
  snprintf(hostname, 78, "TananaAQ-%s", mac_ending);
  Serial.println(hostname);

  // Attempt to set time from internet
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  correct_crazy_time();
  printLocalDateTime();

  if (MDNS.begin(hostname)) {
    Serial.println("MDNS responder started");
  }

  server.on("/", handleRoot);
  server.on("/settime", handleSettime);
  server.on("/setwifi", handleSetwifi);
  server.on("/wipelog", handleWipeLog);
  server.on("/logfile", HTTP_GET, []() {
    if (!handleFileRead("/logfile")) {
      server.send(404, "text/plain", "FileNotFound");
    }
    });
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started");

  // Setup the sleep signal for the PM sensor, which is the "SET" signal
  // pinMode (PM_SET, OUTPUT);
  // digitalWrite (PM_SET, HIGH);  
  // Start the software serial communications with the board
  pmSerial.begin(9600, SWSERIAL_8N1, PM_RX, PM_TX, false);
  if (!pmSerial) { // If the object did not initialize, then its configuration is invalid
    Serial.println("Invalid SoftwareSerial pin configuration, check config"); 
    while (1) { // Don't continue with invalid configuration
      delay (1000);
    }
  }
  tft.fillScreen(TFT_NAVY);
  Serial.println("Startup sequence complete");
}

void loop(void) {
  // handle http server
  if (!batt_powered) server.handleClient();

  // check for serial data and show screen  
  if (pollpmserial(&pmdata)) {
    // coadd datea and do averaging
    coadd_aq();
    try_write_avg();
    if (tft_mode == TFT_MAIN) show_tft_main();
  }
  // check buttons
  buttonTOP.loop();
  buttonBOTTOM.loop();
  // return to main screen if last event timed out
  if (millis() > timeout_millis ) {
    tft_mode = TFT_MAIN;
  }
  // check battery/bus voltage
  check_batt_powered();
  delay(100);//allow the cpu to switch to other tasks
}
