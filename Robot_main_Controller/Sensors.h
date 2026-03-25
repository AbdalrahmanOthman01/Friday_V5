#ifndef SENSORS_H
#define SENSORS_H

// Arduino.h is REQUIRED - provides essential functions and types
#include <Arduino.h>

// ============================================================================
// SENSOR LIBRARY HEADER FILE - Robot Main Controller
// ============================================================================
// Extended sensor control library for main controller
// Contains communication and storage modules not in Robot_Head_Controller
// 
// Author: Senior Embedded Engineer (20+ years experience)
// Platform: Arduino/AVR microcontrollers
// 
// This file includes:
// - GPS Module (NEO-6M, NEO-7M, etc.) - Uses Serial (RX/TX)
// - SIM Module (SIM800L, SIM900, etc.) - Uses Serial (RX/TX)
// - SD Card Module - Uses SPI (MOSI, MISO, SCK, CS)
// - Bluetooth Module (HC-05, HC-06, HM-10) - Uses Serial (RX/TX)
// - RTC Module (DS1307, DS3231) - Uses I2C (SDA/SCL)
// 
// Note: Basic sensors (ultrasonic, flame, light, rain, sound, gas, DHT)
// are in Robot_Head_Controller/Sensors.h
// ============================================================================

// ============================================================================
// GPS MODULE CLASS (NEO-6M, NEO-7M, NEO-M8N, etc.)
// ============================================================================
// Supports GPS modules via Serial (UART) communication
// - NMEA sentence parsing (GPGGA, GPRMC)
// - Location, altitude, speed, satellite count
// - Date and time extraction
// 
// Default pins: RX=0, TX=1 (Hardware Serial)
// Common modules: NEO-6M, NEO-7M, NEO-M8N, GT-U7
// ============================================================================

class GPSModule {
private:
    HardwareSerial* gpsSerial;
    uint32_t baudRate;
    float latitude, longitude, altitude, speed, course;
    uint8_t satellites, fixQuality;
    bool hasFix;
    uint8_t hour, minute, second, day, month;
    uint16_t year;
    String nmeaBuffer;
    bool sentenceReady;
    unsigned long lastParseTime, updateIntervalMS, lastUpdateTime;

public:
    GPSModule(HardwareSerial* serial = &Serial, uint32_t baud = 9600)
        : gpsSerial(serial), baudRate(baud), latitude(0), longitude(0), altitude(0), speed(0), course(0),
          satellites(0), fixQuality(0), hasFix(false), hour(0), minute(0), second(0), day(0), month(0), year(0),
          nmeaBuffer(""), sentenceReady(false), lastParseTime(0), updateIntervalMS(1000), lastUpdateTime(0) {}
    
    void begin(uint32_t baud = 0) { if (baud > 0) baudRate = baud; gpsSerial->begin(baudRate); nmeaBuffer.reserve(128); }
    
    bool update() {
        while (gpsSerial->available()) {
            char c = gpsSerial->read();
            if (c == '$') nmeaBuffer = "";
            nmeaBuffer += c;
            if (c == '\n') { sentenceReady = true; if (parseNMEA()) { lastUpdateTime = millis(); return true; } nmeaBuffer = ""; sentenceReady = false; }
        }
        return false;
    }
    
    float getLatitude() const { return latitude; }
    float getLongitude() const { return longitude; }
    float getAltitude() const { return altitude; }
    float getSpeed() const { return speed; }
    float getCourse() const { return course; }
    uint8_t getSatellites() const { return satellites; }
    uint8_t getFixQuality() const { return fixQuality; }
    bool isValid() const { return hasFix; }
    uint8_t getHour() const { return hour; }
    uint8_t getMinute() const { return minute; }
    uint8_t getSecond() const { return second; }
    uint8_t getDay() const { return day; }
    uint8_t getMonth() const { return month; }
    uint16_t getYear() const { return year; }
    String getTimeString() const { char buf[9]; sprintf(buf, "%02d:%02d:%02d", hour, minute, second); return String(buf); }
    String getDateString() const { char buf[11]; sprintf(buf, "%02d/%02d/%04d", day, month, year); return String(buf); }
    unsigned long getTimeSinceLastUpdate() const { return millis() - lastUpdateTime; }
    String getGoogleMapsURL() const { return "https://www.google.com/maps?q=" + String(latitude, 6) + "," + String(longitude, 6); }
    float distanceTo(float lat2, float lon2) const { float R = 6371000; float dLat = radians(lat2 - latitude); float dLon = radians(lon2 - longitude); float a = sin(dLat/2) * sin(dLat/2) + cos(radians(latitude)) * cos(radians(lat2)) * sin(dLon/2) * sin(dLon/2); float c = 2 * atan2(sqrt(a), sqrt(1-a)); return R * c; }

private:
    bool parseNMEA() {
        if (nmeaBuffer.length() < 10) return false;
        if (nmeaBuffer.indexOf("$GPGGA") >= 0 || nmeaBuffer.indexOf("$GNGGA") >= 0) return parseGPGGA();
        if (nmeaBuffer.indexOf("$GPRMC") >= 0 || nmeaBuffer.indexOf("$GNRMC") >= 0) return parseGPRMC();
        return false;
    }
    
    bool parseGPGGA() {
        int commaIndex[15]; int idx = 0;
        for (int i = 0; i < nmeaBuffer.length() && idx < 15; i++) if (nmeaBuffer[i] == ',') commaIndex[idx++] = i;
        if (idx < 9) return false;
        fixQuality = nmeaBuffer.substring(commaIndex[5]+1, commaIndex[6]).toInt(); hasFix = (fixQuality > 0);
        satellites = nmeaBuffer.substring(commaIndex[6]+1, commaIndex[7]).toInt();
        String latStr = nmeaBuffer.substring(commaIndex[1]+1, commaIndex[2]); String latDir = nmeaBuffer.substring(commaIndex[2]+1, commaIndex[3]);
        if (latStr.length() > 0) { float latDeg = latStr.substring(0, 2).toFloat(); float latMin = latStr.substring(2).toFloat(); latitude = latDeg + (latMin / 60.0); if (latDir == "S") latitude = -latitude; }
        String lonStr = nmeaBuffer.substring(commaIndex[3]+1, commaIndex[4]); String lonDir = nmeaBuffer.substring(commaIndex[4]+1, commaIndex[5]);
        if (lonStr.length() > 0) { float lonDeg = lonStr.substring(0, 3).toFloat(); float lonMin = lonStr.substring(3).toFloat(); longitude = lonDeg + (lonMin / 60.0); if (lonDir == "W") longitude = -longitude; }
        altitude = nmeaBuffer.substring(commaIndex[8]+1, commaIndex[9]).toFloat();
        return hasFix;
    }
    
    bool parseGPRMC() {
        int commaIndex[12]; int idx = 0;
        for (int i = 0; i < nmeaBuffer.length() && idx < 12; i++) if (nmeaBuffer[i] == ',') commaIndex[idx++] = i;
        if (idx < 9) return false;
        char status = nmeaBuffer.charAt(commaIndex[1]+1); hasFix = (status == 'A');
        String timeStr = nmeaBuffer.substring(commaIndex[0]+1, commaIndex[1]); if (timeStr.length() >= 6) { hour = timeStr.substring(0, 2).toInt(); minute = timeStr.substring(2, 4).toInt(); second = timeStr.substring(4, 6).toInt(); }
        speed = nmeaBuffer.substring(commaIndex[6]+1, commaIndex[7]).toFloat() * 1.852;
        course = nmeaBuffer.substring(commaIndex[7]+1, commaIndex[8]).toFloat();
        String dateStr = nmeaBuffer.substring(commaIndex[8]+1, commaIndex[9]); if (dateStr.length() >= 6) { day = dateStr.substring(0, 2).toInt(); month = dateStr.substring(2, 4).toInt(); year = 2000 + dateStr.substring(4, 6).toInt(); }
        return hasFix;
    }
};

// ============================================================================
// SIM MODULE CLASS (SIM800L, SIM900, A7, A6, etc.)
// ============================================================================
// Supports GSM/GPRS modules via Serial communication
// - SMS send/receive
// - Phone calls
// - GPRS data connection
// - HTTP requests
// 
// Default pins: RX=0, TX=1 (Hardware Serial)
// Common modules: SIM800L, SIM900, A7, A6, SIM7600
// ============================================================================

class SIMModule {
private:
    HardwareSerial* simSerial;
    uint32_t baudRate;
    bool isReady, isRegistered;
    uint8_t signalStrength;
    String operatorName, lastSMS, lastSMSNumber, responseBuffer, apn, apnUser, apnPass;
    unsigned long commandTimeout;

public:
    SIMModule(HardwareSerial* serial = &Serial, uint32_t baud = 9600)
        : simSerial(serial), baudRate(baud), isReady(false), isRegistered(false), signalStrength(0),
          operatorName(""), lastSMS(""), lastSMSNumber(""), responseBuffer(""), apn("internet"), apnUser(""), apnPass(""), commandTimeout(5000) {}
    
    bool begin() {
        simSerial->begin(baudRate); delay(3000); responseBuffer.reserve(256);
        if (!sendATCommand("AT", 1000).startsWith("OK")) return false;
        sendATCommand("ATE0", 1000);
        String creg = sendATCommand("AT+CREG?", 2000); isRegistered = (creg.indexOf(",1") >= 0 || creg.indexOf(",5") >= 0);
        String csq = sendATCommand("AT+CSQ", 1000); int idx = csq.indexOf(":"); if (idx >= 0) signalStrength = csq.substring(idx+1, csq.indexOf(",")).toInt();
        isReady = true; return true;
    }
    
    bool sendSMS(const String& number, const String& message) {
        if (!isReady) return false;
        sendATCommand("AT+CMGF=1", 1000);
        simSerial->print("AT+CMGS=\""); simSerial->print(number); simSerial->println("\""); delay(100);
        simSerial->print(message); simSerial->write(0x1A);
        String response = waitForResponse("OK", 10000); return response.indexOf("OK") >= 0;
    }
    
    bool checkSMS() {
        if (!isReady) return false;
        String response = sendATCommand("AT+CMGL=\"REC UNREAD\"", 5000);
        if (response.indexOf("+CMGL:") >= 0) {
            int msgStart = response.indexOf("\n", response.indexOf("+CMGL:"));
            if (msgStart >= 0) { lastSMS = response.substring(msgStart + 1); lastSMS.trim();
                int numStart = response.indexOf("\"", response.indexOf("+CMGL:")); int numEnd = response.indexOf("\"", numStart + 1);
                if (numStart >= 0 && numEnd >= 0) lastSMSNumber = response.substring(numStart + 1, numEnd);
                return true; }
        }
        return false;
    }
    
    String getLastSMS() const { return lastSMS; }
    String getLastSMSNumber() const { return lastSMSNumber; }
    void deleteAllSMS() { sendATCommand("AT+CMGD=1,4", 2000); }
    void makeCall(const String& number) { if (!isReady) return; simSerial->print("ATD"); simSerial->print(number); simSerial->println(";"); }
    void answerCall() { sendATCommand("ATA", 1000); }
    void hangUp() { sendATCommand("ATH", 1000); }
    void setAPN(const String& apnName, const String& user = "", const String& pass = "") { apn = apnName; apnUser = user; apnPass = pass; }
    bool startGPRS() {
        if (!isReady) return false;
        sendATCommand("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"", 1000);
        sendATCommand("AT+SAPBR=3,1,\"APN\",\"" + apn + "\"", 1000);
        if (apnUser.length() > 0) sendATCommand("AT+SAPBR=3,1,\"USER\",\"" + apnUser + "\"", 1000);
        if (apnPass.length() > 0) sendATCommand("AT+SAPBR=3,1,\"PWD\",\"" + apnPass + "\"", 1000);
        String response = sendATCommand("AT+SAPBR=1,1", 10000); return response.indexOf("OK") >= 0;
    }
    void stopGPRS() { sendATCommand("AT+SAPBR=0,1", 2000); }
    String getIP() { String response = sendATCommand("AT+SAPBR=2,1", 2000); int start = response.indexOf("\""); int end = response.indexOf("\"", start + 1); if (start >= 0 && end >= 0) return response.substring(start + 1, end); return ""; }
    String httpGet(const String& url) {
        if (!isReady) return "";
        sendATCommand("AT+HTTPINIT", 1000); sendATCommand("AT+HTTPPARA=\"CID\",1", 1000);
        sendATCommand("AT+HTTPPARA=\"URL\",\"" + url + "\"", 1000);
        String response = sendATCommand("AT+HTTPACTION=0", 10000);
        String data = sendATCommand("AT+HTTPREAD", 5000); sendATCommand("AT+HTTPTERM", 1000); return data;
    }
    String httpPost(const String& url, const String& data, const String& contentType = "application/json") {
        if (!isReady) return "";
        sendATCommand("AT+HTTPINIT", 1000); sendATCommand("AT+HTTPPARA=\"CID\",1", 1000);
        sendATCommand("AT+HTTPPARA=\"URL\",\"" + url + "\"", 1000);
        sendATCommand("AT+HTTPPARA=\"CONTENT\",\"" + contentType + "\"", 1000);
        sendATCommand("AT+HTTPDATA=" + String(data.length()) + ",10000", 1000); simSerial->print(data); delay(100);
        String response = sendATCommand("AT+HTTPACTION=1", 10000);
        String result = sendATCommand("AT+HTTPREAD", 5000); sendATCommand("AT+HTTPTERM", 1000); return result;
    }
    bool isModuleReady() const { return isReady; }
    bool isNetworkRegistered() const { return isRegistered; }
    uint8_t getSignalStrength() const { return signalStrength; }
    float getSignalPercent() const { if (signalStrength == 99) return 0; return (signalStrength / 31.0) * 100.0; }
    String getOperator() const { return operatorName; }
    String sendATCommand(const String& command, unsigned long timeout = 1000) { simSerial->println(command); return waitForResponse("OK", timeout); }
    String waitForResponse(const String& expected, unsigned long timeout) { String response = ""; unsigned long start = millis(); while ((millis() - start) < timeout) { if (simSerial->available()) { char c = simSerial->read(); response += c; if (response.indexOf(expected) >= 0) return response; } } return response; }
};

// ============================================================================
// SD CARD CLASS
// ============================================================================
// Supports SD card modules via SPI communication
// - File read/write operations
// - Directory listing
// - Data logging
// 
// Default SPI pins: MOSI=11, MISO=12, SCK=13, CS=10 (Arduino Uno)
// Common modules: SD card module, SD breakout
// ============================================================================

#include <SD.h>
#include <SPI.h>

class SDCard {
private:
    uint8_t csPin;
    bool isInitialized;
    String currentFile;
    uint8_t lastError;

public:
    SDCard(uint8_t cs = 10) : csPin(cs), isInitialized(false), currentFile(""), lastError(0) {}
    bool begin() { isInitialized = SD.begin(csPin); return isInitialized; }
    bool isReady() const { return isInitialized; }
    bool write(const String& filename, const String& data, bool append = true) {
        if (!isInitialized) return false;
        File file = SD.open(filename, FILE_WRITE); if (!file) { lastError = 1; return false; }
        file.print(data); file.close(); return true;
    }
    bool writeLine(const String& filename, const String& data, bool append = true) {
        if (!isInitialized) return false;
        File file = SD.open(filename, FILE_WRITE); if (!file) { lastError = 1; return false; }
        file.println(data); file.close(); return true;
    }
    String read(const String& filename) {
        if (!isInitialized) return "";
        File file = SD.open(filename, FILE_READ); if (!file) { lastError = 2; return ""; }
        String content = ""; while (file.available()) content += (char)file.read(); file.close(); return content;
    }
    String readLine(const String& filename, uint32_t lineNumber) {
        if (!isInitialized) return "";
        File file = SD.open(filename, FILE_READ); if (!file) return "";
        uint32_t currentLine = 0; String line = "";
        while (file.available()) { char c = file.read(); if (c == '\n') { currentLine++; if (currentLine == lineNumber) { file.close(); return line; } line = ""; } else if (c != '\r') line += c; }
        file.close(); return "";
    }
    bool exists(const String& filename) { if (!isInitialized) return false; return SD.exists(filename); }
    bool remove(const String& filename) { if (!isInitialized) return false; return SD.remove(filename); }
    uint32_t fileSize(const String& filename) { if (!isInitialized) return 0; File file = SD.open(filename, FILE_READ); if (!file) return 0; uint32_t size = file.size(); file.close(); return size; }
    bool logData(const String& filename, const String& data) { String timestamp = String(millis()); return writeLine(filename, timestamp + "," + data); }
    bool logCSVHeader(const String& filename, const String& header) { if (exists(filename)) return true; return writeLine(filename, header, false); }
    bool logCSVRow(const String& filename, const String& data) { return writeLine(filename, data); }
    String listFiles() {
        if (!isInitialized) return "";
        File root = SD.open("/"); if (!root) return "";
        String fileList = ""; File entry = root.openNextFile();
        while (entry) { if (!entry.isDirectory()) { fileList += entry.name(); fileList += " ("; fileList += String(entry.size()); fileList += " bytes)\n"; } entry.close(); entry = root.openNextFile(); }
        root.close(); return fileList;
    }
    bool mkdir(const String& path) { if (!isInitialized) return false; return SD.mkdir(path); }
    bool rmdir(const String& path) { if (!isInitialized) return false; return SD.rmdir(path); }
    uint8_t getLastError() const { return lastError; }
};

// ============================================================================
// BLUETOOTH MODULE CLASS (HC-05, HC-06, HM-10, etc.)
// ============================================================================
// Supports Bluetooth modules via Serial communication
// - Data transmission/reception
// - Device pairing
// - AT command configuration
// 
// Default pins: RX=0, TX=1 (Hardware Serial)
// Common modules: HC-05, HC-06, HM-10 (BLE), JDY-31
// ============================================================================

class BluetoothModule {
private:
    HardwareSerial* btSerial;
    uint32_t baudRate;
    bool isConnected, isConfigMode;
    String deviceName, pin, receiveBuffer;
    unsigned long lastReceiveTime;

public:
    BluetoothModule(HardwareSerial* serial = &Serial, uint32_t baud = 9600)
        : btSerial(serial), baudRate(baud), isConnected(false), isConfigMode(false),
          deviceName("Robot"), pin("1234"), receiveBuffer(""), lastReceiveTime(0) {}
    
    void begin(const String& name = "Robot", const String& pairingPin = "1234") {
        btSerial->begin(baudRate); deviceName = name; pin = pairingPin; receiveBuffer.reserve(256);
        delay(1000); configure();
    }
    
    void send(const String& data) { btSerial->print(data); }
    void sendLine(const String& data) { btSerial->println(data); }
    void send(uint8_t data) { btSerial->write(data); }
    void send(const uint8_t* data, size_t length) { btSerial->write(data, length); }
    bool available() { return btSerial->available(); }
    char read() { return btSerial->read(); }
    String readLine() { String line = ""; while (btSerial->available()) { char c = btSerial->read(); if (c == '\n') return line; else if (c != '\r') line += c; } return line; }
    String readAll() { String data = ""; while (btSerial->available()) data += (char)btSerial->read(); return data; }
    String readUntil(char delimiter) { String data = ""; while (btSerial->available()) { char c = btSerial->read(); if (c == delimiter) return data; data += c; } return data; }
    bool isDeviceConnected() { return isConnected; }
    void setConnected(bool connected) { isConnected = connected; }
    bool configure() { sendATCommand("AT+NAME=" + deviceName); delay(500); sendATCommand("AT+PIN=" + pin); delay(500); sendATCommand("AT+BAUD4"); delay(500); return true; }
    bool setName(const String& name) { deviceName = name; return sendATCommand("AT+NAME=" + name); }
    bool setPIN(const String& newPin) { pin = newPin; return sendATCommand("AT+PIN=" + newPin); }
    String getName() const { return deviceName; }
    void enterConfigMode() { isConfigMode = true; }
    void exitConfigMode() { isConfigMode = false; }
    bool sendATCommand(const String& command) { btSerial->println(command); delay(100); String response = ""; unsigned long start = millis(); while ((millis() - start) < 1000) { if (btSerial->available()) response += (char)btSerial->read(); } return response.indexOf("OK") >= 0; }
    void flush() { btSerial->flush(); }
    void clearBuffer() { receiveBuffer = ""; }
    uint32_t getBaudRate() const { return baudRate; }
};

// ============================================================================
// RTC MODULE CLASS (DS1307, DS3231)
// ============================================================================
// Supports Real-Time Clock modules via I2C communication
// - Date and time tracking
// - Battery backup
// - Alarm functions (DS3231)
// - Temperature sensor (DS3231)
// 
// Default I2C pins: SDA=20, SCL=21 (Arduino Mega) or SDA=A4, SCL=A5 (Uno/Nano)
// Common modules: DS1307, DS3231
// ============================================================================

#include <Wire.h>

class RTCModule {
private:
    uint8_t i2cAddress;
    bool isInitialized, isRunning, hasTemperature;
    uint8_t second, minute, hour, day, month, dayOfWeek;
    uint16_t year;
    float temperature;

public:
    RTCModule(uint8_t address = 0x68)
        : i2cAddress(address), isInitialized(false), isRunning(false), hasTemperature(false),
          second(0), minute(0), hour(0), day(1), month(1), dayOfWeek(1), year(2024), temperature(0) {}
    
    bool begin() {
        Wire.begin();
        Wire.beginTransmission(i2cAddress); if (Wire.endTransmission() != 0) return false;
        Wire.beginTransmission(i2cAddress); Wire.write(0x00); Wire.endTransmission();
        Wire.requestFrom(i2cAddress, (uint8_t)1);
        if (Wire.available()) { uint8_t status = Wire.read(); isRunning = !(status & 0x80); }
        Wire.beginTransmission(i2cAddress); Wire.write(0x11); Wire.endTransmission();
        Wire.requestFrom(i2cAddress, (uint8_t)2);
        if (Wire.available() >= 2) hasTemperature = true;
        isInitialized = true; read(); return true;
    }
    
    bool isReady() const { return isInitialized; }
    bool isClockRunning() const { return isRunning; }
    bool read() {
        if (!isInitialized) return false;
        Wire.beginTransmission(i2cAddress); Wire.write(0x00); Wire.endTransmission();
        Wire.requestFrom(i2cAddress, (uint8_t)7);
        if (Wire.available() >= 7) {
            second = bcdToDec(Wire.read() & 0x7F); minute = bcdToDec(Wire.read());
            hour = bcdToDec(Wire.read() & 0x3F); dayOfWeek = bcdToDec(Wire.read());
            day = bcdToDec(Wire.read()); month = bcdToDec(Wire.read() & 0x1F);
            year = bcdToDec(Wire.read()) + 2000; return true;
        }
        return false;
    }
    
    bool setDateTime(uint16_t yr, uint8_t mo, uint8_t dy, uint8_t hr, uint8_t mn, uint8_t sc) {
        if (!isInitialized) return false;
        year = yr; month = mo; day = dy; hour = hr; minute = mn; second = sc;
        dayOfWeek = calculateDayOfWeek(yr, mo, dy);
        Wire.beginTransmission(i2cAddress); Wire.write(0x00);
        Wire.write(decToBcd(second)); Wire.write(decToBcd(minute)); Wire.write(decToBcd(hour));
        Wire.write(decToBcd(dayOfWeek)); Wire.write(decToBcd(day)); Wire.write(decToBcd(month));
        Wire.write(decToBcd(year - 2000)); Wire.endTransmission();
        isRunning = true; return true;
    }
    
    bool setTime(uint8_t hr, uint8_t mn, uint8_t sc) { return setDateTime(year, month, day, hr, mn, sc); }
    bool setDate(uint16_t yr, uint8_t mo, uint8_t dy) { return setDateTime(yr, mo, dy, hour, minute, second); }
    uint8_t getSecond() const { return second; }
    uint8_t getMinute() const { return minute; }
    uint8_t getHour() const { return hour; }
    uint8_t getDay() const { return day; }
    uint8_t getMonth() const { return month; }
    uint16_t getYear() const { return year; }
    uint8_t getDayOfWeek() const { return dayOfWeek; }
    String getTimeString() const { char buf[9]; sprintf(buf, "%02d:%02d:%02d", hour, minute, second); return String(buf); }
    String getDateString() const { char buf[11]; sprintf(buf, "%02d/%02d/%04d", day, month, year); return String(buf); }
    String getDateTimeString() const { return getDateString() + " " + getTimeString(); }
    String getDayOfWeekName() const { const char* days[] = {"", "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"}; if (dayOfWeek >= 1 && dayOfWeek <= 7) return String(days[dayOfWeek]); return "Unknown"; }
    String getMonthName() const { const char* months[] = {"", "January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"}; if (month >= 1 && month <= 12) return String(months[month]); return "Unknown"; }
    float readTemperature() {
        if (!hasTemperature) return 0;
        Wire.beginTransmission(i2cAddress); Wire.write(0x11); Wire.endTransmission();
        Wire.requestFrom(i2cAddress, (uint8_t)2);
        if (Wire.available() >= 2) { int8_t tempMSB = Wire.read(); uint8_t tempLSB = Wire.read() >> 6; temperature = tempMSB + (tempLSB * 0.25); return temperature; }
        return 0;
    }
    float getTemperature() const { return temperature; }
    bool hasTempSensor() const { return hasTemperature; }
    uint32_t getUnixTimestamp() const { uint32_t days = 0; for (uint16_t y = 1970; y < year; y++) days += isLeapYear(y) ? 366 : 365; for (uint8_t m = 1; m < month; m++) days += daysInMonth(m, year); days += day - 1; return days * 86400UL + hour * 3600UL + minute * 60UL + second; }
    bool isLeapYear(uint16_t yr) const { return (yr % 4 == 0 && yr % 100 != 0) || (yr % 400 == 0); }
    uint8_t daysInMonth(uint8_t mo, uint16_t yr) const { const uint8_t days[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}; if (mo == 2 && isLeapYear(yr)) return 29; return days[mo]; }

private:
    uint8_t bcdToDec(uint8_t bcd) const { return (bcd / 16 * 10) + (bcd % 16); }
    uint8_t decToBcd(uint8_t dec) const { return (dec / 10 * 16) + (dec % 10); }
    uint8_t calculateDayOfWeek(uint16_t yr, uint8_t mo, uint8_t dy) const { if (mo < 3) { mo += 12; yr--; } int k = yr % 100; int j = yr / 100; int h = (dy + (13 * (mo + 1)) / 5 + k + k / 4 + j / 4 - 2 * j) % 7; return ((h + 5) % 7) + 1; }
};

#endif // SENSORS_H