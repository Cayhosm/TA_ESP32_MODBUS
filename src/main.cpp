#include <Arduino.h>
#include <WiFi.h>
#include <ModbusServerTCPasync.h>
#include <SPI.h>
#include <CoilData.h>
#include <AccelStepper.h>
#include <ESP32_Servo.h>
#include <DRV8825.h>

/*
ToDo:
0. pembuatan svg untuk dashboard
1. pembagian register dan coils sesuai fungsinya
2. pembuatan diagram blok dan flow chart yang lebih baik

*/



#define pin_dir 32
#define pin_step 33

#define pin_servo1 18
#define pin_servo2 19

AccelStepper stepper(AccelStepper::DRIVER, pin_step, pin_dir);

char ssid[] = "coba_cobs";
char pass[] = "qwerty123";

enum coilName
{
  coil_On,
  coil_Off,
  coil_servo1,
  coil_servo2,
  coil_detect
};

enum holdingName
{
  holding_speed,
  holding_count_mentah,
  holding_count_setMateng,
  holding_count_mateng,
  holding_count_total
};

uint32_t totalBuah;
uint32_t total_mentah;
uint32_t total_setMatang;
uint32_t total_matang;

uint32_t speedStepper;

// create server
ModbusServerTCPasync MBserver;

uint16_t memo[32];    // test server memory: 32 words || memory holding register yang dimiliki slave addr 0-31
CoilData myCoils(20); // memory coils yang dimiliki slave addr 0-20
bool coilTrigger = false;

Servo myServo1;
Servo myServo2;
int pos = 0;
// const int pinServo = 19;

uint32_t lastMillis;

// Some functions to be called when function codes 0x01, 0x05 or 0x15 are requested
// FC_01: act on 0x01 requests - READ_COIL || Membaca Single Coil
ModbusMessage FC_01(ModbusMessage request)
{
  ModbusMessage response;
  // Request parameters are first coil and number of coils to read
  uint16_t start = 0;
  uint16_t numCoils = 0;
  request.get(2, start, numCoils);

  // Are the parameters valid?
  if (start + numCoils <= myCoils.coils())
  {
    // Looks like it. Get the requested coils from our storage
    vector<uint8_t> coilset = myCoils.slice(start, numCoils);
    // Set up response according to the specs: serverID, function code, number of bytes to follow, packed coils
    response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)coilset.size(), coilset);
  }
  else
  {
    // Something was wrong with the parameters
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
  }
  // Return the response
  return response;
}

// Server function to handle FC 0x03 and 0x04
ModbusMessage FC03(ModbusMessage request)
{
  ModbusMessage response; // The modbus message we are going to give back
  uint16_t addr = 0;      // start address
  uint16_t words = 0;     // # of words requsted
  request.get(2, addr);   // read address from request
  request.get(4, words);  // read # of words from request

  // address overflow?
  if ((addr + words) > 20)
  {
    // yes - send respective error response
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
  }
  // set up response
  response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));
  // requset for fc 0x03
  if (request.getFunctionCode() == READ_HOLD_REGISTER)
  {
    // yes complete response
    for (uint8_t i = 0; i < words; ++i)
    {
      // send increasing data values
      response.add((uint16_t)(memo[addr + i]));
    }
  }
  else
  {
    // no, this is for fc 0x04 response is random
    for (uint8_t i = 0; i < words; ++i)
    {
      // send increasing data values
      response.add((uint16_t)random(1, 65535));
    }
  }
  return response;
}

// FC05: act on 0x05 requests - Write Single Coil
ModbusMessage FC_05(ModbusMessage request)
{
  ModbusMessage response;
  // Request parameters are coil number and 0x0000 (OFF) or 0xFF00 (ON)
  uint16_t start = 0;
  uint16_t state = 0;
  request.get(2, start, state);

  // Is the coil number valid?
  if (start <= myCoils.coils())
  {
    // Looks like it. Is the ON/OFF parameter correct?
    if (state == 0x0000 || state == 0xFF00)
    {
      // Yes. We can set the coil
      if (myCoils.set(start, state))
      {
        // All fine, coil was set.
        response = ECHO_RESPONSE;
        // Pull the trigger
        coilTrigger = true;
      }
      else
      {
        // Setting the coil failed
        response.setError(request.getServerID(), request.getFunctionCode(), SERVER_DEVICE_FAILURE);
      }
    }
    else
    {
      // Wrong data parameter
      response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
    }
  }
  else
  {
    // Something was wrong with the coil number
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
  }
  // Return the response
  return response;
}

ModbusMessage FC06(ModbusMessage request)
{                         // Write single holding register
  ModbusMessage response; // The modbus message we are going to give back
  uint16_t arrayByte[5];
  uint16_t addr = 0;     // start address
  uint16_t words = 0;    // # of words requsted
  request.get(2, addr);  // read address from request
  request.get(4, words); // read # of words from request
  request.get(0, arrayByte[0], arrayByte[1], arrayByte[2], arrayByte[3], arrayByte[4]);

  response.add(request.getServerID(), request.getFunctionCode());
  response.add((uint8_t)0);
  response.add((uint8_t)addr);
  response.add((uint16_t)arrayByte[2]);
  memo[addr] = arrayByte[2];

  return response;
}
//====================================================================
int millisServo;
void servoBack(Servo &servo)
{
  if (millis() - millisServo > 5)
  {
    if (pos > 0)
    {
      pos -= 1;
      servo.write(pos);
    }
    millisServo = millis();
  }
}
void servoSwing(Servo &servo)
{
  if (millis() - millisServo > 5)
  {
    if (pos < 180)
    {
      pos += 1;
      servo.write(pos);
    }
    millisServo = millis();
  }
}

bool stateServoSwing1 = true;
bool stateServoSwing2 = true;

// bool stateServoBack = false;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    Serial.println("__OK__");
  }
  Serial.println("halo");
  myServo1.attach(pin_servo1);
  myServo2.attach(pin_servo2);

  WiFi.begin(ssid, pass);
  delay(200);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print('.');
    delay(500);
  }

  IPAddress wIP = WiFi.localIP();
  Serial.printf("WiFi IP Address: %u.%u.%u.%u\n", wIP[0], wIP[1], wIP[2], wIP[3]);
  MBserver.registerWorker(1, READ_HOLD_REGISTER, &FC03); // FC=03 for serverID=1
  MBserver.registerWorker(1, WRITE_HOLD_REGISTER, &FC06);
  MBserver.registerWorker(1, WRITE_COIL, &FC_05);
  MBserver.registerWorker(1, READ_COIL, &FC_01);

  MBserver.start(502, 1, 20000);
  lastMillis = millis();

  stepper.setMaxSpeed(1000);
  stepper.setSpeed(100);

  millisServo = millis();
}
// bool stateTimeSwingServo1;
// bool stateTimeSwingServo2;

// bool stateTimeBackServo;
/*
*/
uint32_t timeOutSwingServo1 = 1000;
uint32_t timeOutSwingServo2 = 2000;

uint32_t millisServoSwing;
uint32_t millisServoBack;

void loop()
{
  //baca data serial yang dikirim pc 
 
  /*
  List data:
  <0>: Mentah
  <1>: Setengah Matang
  <2>: Matang
  */

  if (Serial.available()){
    String data = Serial.readStringUntil('>');
    data = data.substring(0,2);
    Serial.println("isi data: " + data);
    if (data=="0"){
      if (!stateServoSwing1){
        servoSwing(myServo1);
        stateServoSwing1 = true;
        millisServoSwing = millis();
        stateServoSwing1 = true;
        data = "";
        Serial.println("ada mentah");
      }
    }else if(data=="1"){
      if (!stateServoSwing2){
        servoSwing(myServo2);
        stateServoSwing2 = true;
        millisServoSwing = millis();
        stateServoSwing2 = true;
        data = "";
        Serial.println("ada 1/2 matang");
      }
    }else if(data=="2"){
      Serial.println("ada matang");
    }
  }

  if (stateServoSwing1){
    if (millis()-millisServoSwing>=timeOutSwingServo1){
      servoBack(myServo1);
      stateServoSwing1 = false;
      Serial.println("servor swing");
    }
  }else if (stateServoSwing2){
    if (millis()-millisServoSwing>=timeOutSwingServo2){
      servoBack(myServo2);
      stateServoSwing2 = false;
      Serial.println("servor back");
    }
  }






  //=======================
  if (myCoils[0]==true){
    Serial.println("system ON");
  }else{
    Serial.println("system OFF");
  }

  stepper.runSpeed();

  if (millis() - lastMillis > 1000)//asli 10000
  {
    lastMillis = millis();
    Serial.printf("free heap: %d\n", ESP.getFreeHeap());
    Serial.printf("nilai Hreg: %d", memo[0]);
    memo[0] += 100;
    stepper.setSpeed(500);
  }
  // int modul = memo[0] / 100;
  // if (modul % 2 != 0)
  // {
  //   servoSwing();
  // }
  // else
  // {
  //   servoBack();
  // }
}
