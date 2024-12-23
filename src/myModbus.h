#ifndef _MY_MODBUS_
#define _MY_MODBUS_


#include <Arduino.h>
#include <WiFi.h>
#include <ModbusServerTCPasync.h>
#include <SPI.h>
#include <CoilData.h>

#ifndef MY_SSID
#define MY_SSID "coba_cobs"
#endif

#ifndef MY_PASS
#define MY_PASS "qwerty123"
#endif

uint32_t totalBuah;
uint32_t total_mentah;
uint32_t total_setMatang;
uint32_t total_matang;
uint32_t speedStepper;

char ssid[] = MY_SSID;
char pass[] = MY_PASS;

enum coilName{
  coil_On,
  coil_Off,
  coil_servo1,
  coil_servo2,
  coil_detect
};

enum holdingName{
  holding_speed,
  holding_count_mentah,
  holding_count_setMateng,
  holding_count_mateng,
  holding_count_total
};

bool connectWiFi(){

}

//====================================================================

//create server
ModbusServerTCPasync MBserver;

uint16_t memo[32]; // test server memory: 32 words || memory holding register yang dimiliki slave addr 0-31
CoilData myCoils(20);//memory coils yang dimiliki slave addr 0-20
bool coilTrigger = false;

// Some functions to be called when function codes 0x01, 0x05 or 0x15 are requested
// FC_01: act on 0x01 requests - READ_COIL || Membaca Single Coil
ModbusMessage FC_01(ModbusMessage request) {
  ModbusMessage response;
// Request parameters are first coil and number of coils to read
  uint16_t start = 0;
  uint16_t numCoils = 0;
  request.get(2, start, numCoils);

// Are the parameters valid?
  if (start + numCoils <= myCoils.coils()) {
    // Looks like it. Get the requested coils from our storage
    vector<uint8_t> coilset = myCoils.slice(start, numCoils);
    // Set up response according to the specs: serverID, function code, number of bytes to follow, packed coils
    response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)coilset.size(), coilset);
  } else {
    // Something was wrong with the parameters
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
  }
// Return the response
  return response;
}

//Server function to handle FC 0x03 and 0x04
ModbusMessage FC03(ModbusMessage request){ 
  ModbusMessage response; //The modbus message we are going to give back
  uint16_t addr = 0; //start address
  uint16_t words = 0; //# of words requsted
  request.get(2,addr); //read address from request
  request.get(4, words); //read # of words from request

  //address overflow?
  if ((addr+words)>20){
    //yes - send respective error response
    response.setError(request.getServerID(),request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);

  }
  //set up response 
  response.add(request.getServerID(),request.getFunctionCode(), (uint8_t)(words * 2));
  //requset for fc 0x03
  if (request.getFunctionCode() == READ_HOLD_REGISTER){
    //yes complete response
    for (uint8_t i=0; i<words;++i){
      //send increasing data values
      response.add((uint16_t)(memo[addr+i]));
    }
  }else{
    //no, this is for fc 0x04 response is random
    for (uint8_t i=0; i<words;++i){
      //send increasing data values
      response.add((uint16_t)random(1,65535));
    }
  }
  return response;
}



// FC05: act on 0x05 requests - Write Single Coil
ModbusMessage FC_05(ModbusMessage request) {
  ModbusMessage response;
// Request parameters are coil number and 0x0000 (OFF) or 0xFF00 (ON)
  uint16_t start = 0;
  uint16_t state = 0;
  request.get(2, start, state);

// Is the coil number valid?
  if (start <= myCoils.coils()) {
    // Looks like it. Is the ON/OFF parameter correct?
    if (state == 0x0000 || state == 0xFF00) {
      // Yes. We can set the coil
      if (myCoils.set(start, state)) {
        // All fine, coil was set.
        response = ECHO_RESPONSE;
        // Pull the trigger
        coilTrigger = true;
      } else {
        // Setting the coil failed
        response.setError(request.getServerID(), request.getFunctionCode(), SERVER_DEVICE_FAILURE);
      }
    } else {
      // Wrong data parameter
      response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
    }
  } else {
    // Something was wrong with the coil number
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
  }
// Return the response
  return response;
}

ModbusMessage FC06(ModbusMessage request){//Write single holding register
  ModbusMessage response; //The modbus message we are going to give back
  uint16_t arrayByte[5];
  uint16_t addr = 0; //start address
  uint16_t words = 0; //# of words requsted
  request.get(2,addr); //read address from request
  request.get(4, words); //read # of words from request
  request.get(0,arrayByte[0],arrayByte[1],arrayByte[2],arrayByte[3],arrayByte[4]);

  response.add(request.getServerID(),request.getFunctionCode());
  response.add((uint8_t)0);
  response.add((uint8_t)addr);
  response.add((uint16_t)arrayByte[2]);
  memo[addr] = arrayByte[2];
  
  return response;
}
//====================================================================


#endif