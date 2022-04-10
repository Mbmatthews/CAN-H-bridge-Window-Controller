/*
 *   Brandon Matthews
 *   BTS7960 Module
 *   CAN bus Window Motor Controller 
 */

#include <mcp2515.h>
#include <TimerOne.h>

//H-bridge Module
#define ENABLE 7 //left enable pin (active high)
#define LPWM 5 //Active high PWM signal
#define RPWM 6 //Active high PWM signal
#define HBRIDGE_IS A6 //current sense pin
#define SOURCE_VOLTAGE_DIVIDER A7

//MCP2515 Module
#define CS_PIN 10
#define INTERRUPT_PIN 2

//General definitions
#define BUTTON_STATE_MSG_ID    0x767 
#define PERIODIC_SEND_ID  0x768 // Left Side Window Module
//#define PERIODIC_SEND_ID       0x769 //Right Side Window Module

bool mcp_interrupt = false;
bool periodicSentFlag = true;
struct can_frame windowButtonMsg;
struct can_frame windowPeriodicBroadcastMsg;
struct can_frame rcvMsg;

bool leftDown = 0;
bool leftUp = 0;
bool rightDown = 0;
bool rightUp = 0;

MCP2515 mcp2515(CS_PIN);

// Interrupt Callback Functions
void mcpInterruptCallback(){
  mcp_interrupt=true;
}

void timerOneInterruptCallback(){
  periodicSentFlag=false;
}


void setup() {
  pinMode(ENABLE, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  

  Serial.begin(115800);

  bool MCP_OK = false;
  while(!MCP_OK){
    if(mcp2515.reset() == MCP2515::ERROR_OK){
      Serial.println("MCP OK");
      MCP_OK = true;
    }
    delay(5);
  } 

  windowPeriodicBroadcastMsg.can_id = PERIODIC_SEND_ID;
  windowPeriodicBroadcastMsg.can_dlc = 1;
  windowPeriodicBroadcastMsg.data[0] = 0xAA;  //packet that contains an incremeter just for letting the master device know that this device is still functioning

  // ------ CONFIGURING MCP2515 -------
  mcp2515.setConfigMode();
  mcp2515.setFilterMask(MCP2515::MASK0, false, 0x7FF);
  mcp2515.setFilterMask(MCP2515::MASK1, false, 0x7FF);
  mcp2515.setFilter(MCP2515::RXF0, false, BUTTON_STATE_MSG_ID);
  mcp2515.setFilter(MCP2515::RXF2, false, BUTTON_STATE_MSG_ID);

  //Set up interrupts
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), mcpInterruptCallback, FALLING);
  Timer1.initialize(2000000); //Set Timer1 to trigger interrupt every 2s
  Timer1.attachInterrupt(timerOneInterruptCallback); 

  mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  mcp2515.sendMessage(&windowPeriodicBroadcastMsg);
}

unsigned long currentMillis;
unsigned long lastInterruptTime = 0;
unsigned long lastReceiveMillis = 0;

void loop() {
  currentMillis=millis();

  checkCAN();
  inputProcessor();
  processCurrent();
  outputProcessor();
  periodicSend();
    
}

//###################################################################################################################################################################################

void checkCAN(){
  /*
   if(!mcp_interrupt && !digitalRead(INTERRUPT_PIN) && (currentMillis - lastInterruptTime)>450){ //Periodically in case an interrupt is missed for some reason
   mcp_interrupt=true;
   }
   */
   if(mcp_interrupt){
    lastInterruptTime=currentMillis;
    Serial.println("Interrupt Received");
    mcp_interrupt = false;
    uint8_t irq = mcp2515.getInterrupts();
    if(irq & (MCP2515::CANINTF_RX0IF | MCP2515::CANINTF_RX1IF)){ //if the interrupt came from one of the receive buffers, do the following
      //Serial.println("Interrupt came from a receive buffer");
      if (mcp2515.readMessage(&rcvMsg) == MCP2515::ERROR_OK) {  
        //Serial.print("CAN ID: 0x");
        //Serial.println(rcvMsg.can_id, HEX);            
        if(rcvMsg.can_id == (BUTTON_STATE_MSG_ID)){
          lastReceiveMillis = currentMillis;
          //Do stuff with received message
          //---------------------------------
          //incoming data should have a byte of 0xA followed by a byte of the button states in the order Left Up, Left Down, Right Up, Right Down
          byte relevantData = rcvMsg.data[0] & 0x0F;
          leftUp = (relevantData >> 3) & 0b1;
          leftDown = (relevantData >> 2) & 0b1;
          //rightDown = (relevantData >> 1) & 0b1;
          //rightUp = relevantData & 0b1;
        }
      }
    }
    if(irq & MCP2515::CANINTF_MERRF){
      mcp2515.MCP2515::clearMERR();
    }
    if(irq & MCP2515::CANINTF_WAKIF){
      
    }
    if(irq & MCP2515::CANINTF_ERRIF){
      //Serial.println("Error Interrupt Flag bit (multiple sources from EFLG Register)");
      if(mcp2515.MCP2515::checkError()){
        /*
          Serial.println("Check Error Returned True");
          uint8_t erf = mcp2515.MCP2515::getErrorFlags();
          if(erf & 0x01)_libSerial->println("EWARN: Error Warning Flag bit");
          if(erf & 0x02)_libSerial->println("RXWAR: Receive Error Warning Flag bit");
          if(erf & 0x04)_libSerial->println("TXWAR: Transmit Error Warning Flag bit");
          if(erf & 0x08)_libSerial->println("RXEP: Receive Error-Passive Flag bit");
          if(erf & 0x10)_libSerial->printlnn("TXEP: Transmit Error-Passive Flag bit");
          if(erf & 0x20)_libSerial->println("TXBO: Bus-Off Error Flag bit");
          if(erf & 0x40)_libSerial->println("RX0OVR: Receive Buffer 0 Overflow Flag bit");
          if(erf & 0x80)_libSerial->println("RX1OVR: Receive Buffer 1 Overflow Flag bit");
        */
      }
    }    
  }
  //Take care of any errors that have appeared
  uint8_t erf = mcp2515.MCP2515::getErrorFlags();
  if(erf & (MCP2515::EFLG_EWARN | MCP2515::EFLG_TXWAR | MCP2515::EFLG_TXEP | MCP2515::EFLG_TXBO)){
      mcp2515.setListenOnlyMode();
      mcp2515.clearTXInterrupts();
      mcp2515.clearERRIF();
      mcp2515.setNormalMode();
  }
  if(erf & (MCP2515::EFLG_EWARN | MCP2515::EFLG_RXWAR | MCP2515::EFLG_RXEP | MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR)){
      mcp2515.setListenOnlyMode();
      mcp2515.clearRXnOVR();
      mcp2515.clearERRIF();
      mcp2515.setNormalMode();
  }
}

//###################################################################################################################################################################################

void periodicSend(){
  if(!periodicSentFlag){
    mcp2515.sendMessage(&windowPeriodicBroadcastMsg);
    periodicSentFlag = true;
  }
}

//###################################################################################################################################################################################

void moveFunc(int inputVal){ //value between -255 and 255
    if(inputVal<0){ //move in reverse
      digitalWrite(ENABLE,1);
      analogWrite(LPWM, 0);
      analogWrite(RPWM, -inputVal);
    }
    else if(inputVal>0){ //move forward
      digitalWrite(ENABLE,1);
      analogWrite(RPWM, 0);
      analogWrite(LPWM, inputVal);
    }
    else{
      digitalWrite(ENABLE,0);
      analogWrite(LPWM, 0);
      analogWrite(RPWM, 0);
    }
}

//###################################################################################################################################################################################

bool downAutoReady=0;
bool upAutoReady=0;
bool upButtonState[2] = {0,0}; //these states have the last state in the 0th index and the newest state in the 1st index
bool downButtonState[2] = {0,0};
bool automaticMove[2] = {0,0};
unsigned long upButtonLastPress = 0;
unsigned long upButtonLastDepress = 0;
unsigned long downButtonLastPress = 0;
unsigned long downButtonLastDepress = 0;
const int delayForQuickPress = 300; //ms
const int debounceTime = 10; //ms
unsigned long automaticMoveBegin = 0;
const int automaticTimeout = 6000; //millisecond timeout on any automatic move
uint8_t inputStateNumber = 0;
/*
 * InputStateNumber:
 * 0 = nothing pressed
 * 1 = up pressed and down not pressed
 * 2 = down pressed and up not pressed
 * 3 = up button quick pressed
 * 4 = down button quick pressed
 * 5 = both buttons pressed
 */

void inputProcessor(){
  //Shifting/refreshing input state registers
  upButtonState[0]=upButtonState[1];
  downButtonState[0]=downButtonState[1];
  automaticMove[0] = automaticMove[1];
  upButtonState[1] = leftUp;
  downButtonState[1] = leftDown;
  //upButtonState[1] = rightUp;
  //downButtonState[1] = rightDownw;
  

  //Keeping track of event times
  if(upButtonState[1] && !upButtonState[0]){ //rising edge of up button
    upButtonLastPress = currentMillis;
  }
  if(!upButtonState[1] && upButtonState[0]){ //falling edge of up button
    upButtonLastDepress = currentMillis;
  }
  if(downButtonState[1] && !downButtonState[0]){ //rising edge of down button
    downButtonLastPress = currentMillis;
  }
  if(!downButtonState[1] && downButtonState[0]){ //falling edge of down button
    downButtonLastDepress = currentMillis;
  }

  //Process input logic into a state variable
  if(!upButtonState[1] && !downButtonState[1]){ //nothing pressed
    inputStateNumber = 0;
    automaticMove[1] = 0;
  }
  else if(upButtonState[1] && !downButtonState[1]){ //up pressed and down not pressed
    inputStateNumber = 1;
    automaticMove[1] = 0;
  }
  else if(downButtonState[1] && !upButtonState[1]){ //down pressed and up not pressed
    inputStateNumber = 2;
    automaticMove[1] = 0;
  }
  if(!upButtonState[1] && (upButtonLastDepress-upButtonLastPress < delayForQuickPress) && (upButtonLastDepress-upButtonLastPress > debounceTime)){ //if up button is not pressed and the press to depress time is short enough for quick press to be engaged
    inputStateNumber = 3;
    automaticMove[1] = 1;
  }
  else if(!downButtonState[1] && (downButtonLastDepress-downButtonLastPress < delayForQuickPress) && (downButtonLastDepress-downButtonLastPress > debounceTime)){ //if down button is not pressed and the press to depress time is short enough for quick press to be engaged
    inputStateNumber = 4;
    automaticMove[1] = 1;
  }
  if(upButtonState[1] && downButtonState[1]){ //both buttons pressed
    inputStateNumber = 5;
    automaticMove[1] = 0;
  }
  
  //set time for beginning of automatic move
  if(automaticMove[1] && !automaticMove[0]){ //rising edge of the automatic move
    automaticMoveBegin = currentMillis;
  }

  //extra conditionals
  if(inputStateNumber == 3 || inputStateNumber == 4){
    if(upButtonState[1] || downButtonState[1]){
      resetInputState();
    }
    if(currentMillis-automaticMoveBegin>automaticTimeout){
      resetInputState();
    }
  }
  if(inputStateNumber == 3 && !upAutoReady){
    resetInputState();
  }
  if(inputStateNumber == 4 && !downAutoReady){
    resetInputState();
  }
  //Serial.println(inputStateNumber);
}

void resetInputState(){
  inputStateNumber = 0;
  upButtonLastPress = 0;
  upButtonLastDepress = 0;
  downButtonLastPress = 0;
  downButtonLastDepress = 0;
}

//###################################################################################################################################################################################

const int sampleRateMillis = 5;
double motorCurrent;
double maxUpCurrent = 0.0;
double maxDownCurrent = 0.0;
double minUpCurrent = 25.0;
double minDownCurrent = 25.0;
unsigned long lastCurrentSampleMillis;
const int delayOnMinCurrent = 800; //ms delay between when the button is pressed and when a minimum current can be sampled
const int delayOnMaxCurrent = 300; //ms delay between when the button is pressed and when a maximum current can be sampled
double downTriggerCurrent = 0;
double upTriggerCurrent = 0;

void processCurrent(){
  if(currentMillis-lastCurrentSampleMillis>=sampleRateMillis){
    lastCurrentSampleMillis = currentMillis;
    motorCurrent = readCurrent();
    if(upButtonState[1]){
      if(currentMillis - upButtonLastPress > delayOnMaxCurrent){
        if(motorCurrent > maxUpCurrent){
          maxUpCurrent=motorCurrent;
          //Serial.print("New Max Up Current:  ");
          //Serial.println(maxUpCurrent);
          upTriggerCurrent = minUpCurrent + 0.93*(maxUpCurrent-minUpCurrent);
        }
      }
      if(currentMillis - upButtonLastPress > delayOnMinCurrent){
        if(motorCurrent < minUpCurrent && motorCurrent > 5.0){
          minUpCurrent=motorCurrent;
          //Serial.print("New Min Up Current:  ");
          //Serial.println(minUpCurrent);
          upTriggerCurrent = minUpCurrent + 0.93*(maxUpCurrent-minUpCurrent);
        }
      }
    }
    if(downButtonState[1]){
      if(currentMillis - downButtonLastPress > delayOnMaxCurrent){
        if(motorCurrent > maxDownCurrent){
          maxDownCurrent=motorCurrent;
          //Serial.print("New Max Down Current:  ");
          //Serial.println(maxDownCurrent);
          downTriggerCurrent = minDownCurrent + 0.93*(maxDownCurrent-minDownCurrent);
        }
      }
      if(currentMillis - downButtonLastPress > delayOnMinCurrent){
        if(motorCurrent < minDownCurrent && motorCurrent > 5.0){
          minDownCurrent=motorCurrent;
          //Serial.print("New Min Down Current:  ");
          //Serial.println(minDownCurrent);
          downTriggerCurrent = minDownCurrent + 0.93*(maxDownCurrent-minDownCurrent);
        }
      }
    }
    
    if(minDownCurrent<maxDownCurrent)downAutoReady=1;
    if(minUpCurrent<maxUpCurrent)upAutoReady=1;
    
  }
}

int currentIndex = 0;
double currentReadings[25];
int numSamples = 10;

double readCurrent(){
    double analogVal = analogRead(A6);
    //Serial.println(analogVal);
    double Vis = double(analogVal) * double(5000.0/1023.0);     // mV
    double Iint = Vis / double(10000.0/2.0);                    // mA (10k for each Is on the module, leaving both in parallel with how I have it connected
    double Iext = Vis / double(2200.0);                         // mA (2.2k resistor connnected external to the module)
    double currentRead = 8.5 * (Iint+Iext);                     // A
    currentReadings[currentIndex] = currentRead;
    if(currentIndex>=numSamples-1)currentIndex=0;
    else currentIndex++;
    double averageCurrent = 0.0;
    double numValidSamples = 0;
    for(int i=0;i<numSamples;i++){
      if(currentReadings[i] > 0.1){
        averageCurrent += currentReadings[i];
        numValidSamples ++;
      }
    }
    averageCurrent = averageCurrent/(double)numValidSamples;
    return averageCurrent;
}

void clearCurrentArray(){
  for(int i=0;i<25;i++){
      currentReadings[i] = 0;
    }
}

//###################################################################################################################################################################################

/*
 * InputStateNumber:
 * 0 = nothing pressed
 * 1 = up pressed and down not pressed
 * 2 = down pressed and up not pressed
 * 3 = up button quick pressed
 * 4 = down button quick pressed
 * 5 = both buttons pressed
 */

void outputProcessor(){
  if(inputStateNumber == 0)moveFunc(0);
  if(inputStateNumber == 1)moveFunc(255);
  if(inputStateNumber == 2)moveFunc(-255);
  if(inputStateNumber == 3 && upAutoReady){
    motorCurrent = readCurrent();
    if(motorCurrent >= upTriggerCurrent){
      resetInputState();
      clearCurrentArray();
    }
  }
  if(inputStateNumber == 4 && downAutoReady){
    motorCurrent = readCurrent();
    if(motorCurrent >= downTriggerCurrent){
      resetInputState();
      clearCurrentArray();
    }
  }
  if(inputStateNumber == 5 && upAutoReady)moveFunc(0);
}
