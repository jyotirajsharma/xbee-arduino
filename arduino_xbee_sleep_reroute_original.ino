
#include <XBee.h>
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
volatile int f_wdt=1;
#define INVALID_VALUE 100
uint8_t previous_temperature = INVALID_VALUE, previous_humidity=INVALID_VALUE;
uint8_t sleep_state=0;
#include <dht.h>
dht DHT;
const int XBee_wake = 9; // Arduino pin used to sleep the XBee
#define DHT11_PIN 9
#define MOISTURE_A0 A0
#define MAX_RETRY_COUNT 20
int statusLed = 13;
void readDHTSensorDataStatus(int chk);
void send_data_to_destination(void);
void send_data_to_different_path(void);
 uint8_t retry_counter = 0;
/**
 * Struct to keep info about discovered nodes.
 */
struct node_info {
  XBeeAddress64 addr64;
  uint8_t addr64_lsb[4];
  uint8_t hops_count;
  uint8_t battery_voltage;
  uint16_t addr16;
  uint8_t type: 2;
  uint8_t visited: 1;
};
node_info nodes[10];
void send_data_after_sleep_disable();
/***************************************************
 *  Name:        ISR(WDT_vect)
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Watchdog Interrupt Service. This
 *               is executed when watchdog timed out.
 *
 ***************************************************/
ISR(WDT_vect)
{
  if(f_wdt == 0)
  {
    f_wdt=1;
    Serial.println("Again Makeing f_wdt = 1");
  }
  else
  {
    Serial.println("WDT Overrun!!!");
  }
}

/***************************************************
 *  Name:        enterSleep
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Enters the arduino into sleep mode.
 *
 ***************************************************/
void enterSleep(void)
{
  set_sleep_mode(SLEEP_MODE_PWR_SAVE/*SLEEP_MODE_PWR_DOWN*/);   /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
  sleep_enable();
  
  /* Now enter sleep mode. */
  sleep_mode();
  
  /* The program will continue from here after the WDT timeout*/
  Serial.println("Before Sleep Disable");
  sleep_disable(); /* First thing to do is disable sleep. */
  Serial.println("After Sleep Disable");
  ////flashLed(statusLed, 1, 100);
  /* Re-enable the peripherals. */
  power_all_enable();
  delay(5000);
  send_data_after_sleep_disable();
}

/*
This example is for Series 2 XBee
 Sends a ZB TX request with the value of analogRead(pin5) and checks the status response for success
*/

// create the XBee object
XBee xbee_request_response = XBee();

uint8_t payload[] = { 0, 0,0,0 };
//uint8_t payload[10] ;//= { 22, 45 ,450};
// SH + SL Address of receiving XBee
XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, /*0x40F16F1D */0x40BD3AA1);
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));

ZBTxStatusResponse txStatus = ZBTxStatusResponse();

int pin5 = 0;

char str1[34] = {0,};
int failure_count=0;
int errorLed = 13;
int next=0;
void flashLed(int pin, int times, int wait) {

  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(wait);
    digitalWrite(pin, LOW);

    if (i + 1 < times) {
      delay(wait);
    }
  }
}

void setup() {
  pinMode(statusLed, OUTPUT);
  pinMode(errorLed, OUTPUT);
   //pinMode(ledPin, OUTPUT);//sets the pin as an output of the arduino
// wake up the XBee
  pinMode(XBee_wake, OUTPUT);
  digitalWrite(XBee_wake, LOW);

    /*** Setup the WDT ***/
  /* Clear the reset flag. */
  MCUSR &= ~(1<<WDRF);
  
  /* In order to change WDE or the prescaler, we need to
   * set WDCE (This will allow updates for 4 clock cycles).
   */
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  /* set new watchdog timeout prescaler value */
  WDTCSR = 1<<WDP0 | 1<<WDP3; /* 8.0 seconds */
  
  /* Enable the WD interrupt (note no reset). */
  WDTCSR |= _BV(WDIE);
 
  Serial.begin(9600);
  xbee_request_response.setSerial(Serial);
}
void readDHTSensorDataStatus(int chk)
{
  switch (chk)
  {
    case DHTLIB_OK:  
        Serial.print("OK,\t"); 
        break;
    case DHTLIB_ERROR_CHECKSUM: 
         Serial.print("Checksum error,\t");
         break;
    case DHTLIB_ERROR_TIMEOUT: 
         Serial.print("Time out error,\t");
         break;
    case DHTLIB_ERROR_CONNECT:
        Serial.print("Connect error,\t");
        break;
    case DHTLIB_ERROR_ACK_L:
         Serial.print("Ack Low error,\t");
         break;
    case DHTLIB_ERROR_ACK_H:
         Serial.print("Ack High error,\t");
         break;
    default: 
        Serial.print("Unknown error,\t"); 
        break;
  }
}
int iterate = 0;
void send_data_after_sleep_disable()
{
int index=0;

  int chk = DHT.read11(DHT11_PIN);

  readDHTSensorDataStatus(chk);
  
  unsigned short moisture,moisture_new;
 
  Serial.print(DHT.humidity);
  Serial.print(",\t");
  Serial.println(DHT.temperature);
  int t =DHT.temperature;
  int h = DHT.humidity;

  if(previous_temperature == INVALID_VALUE)
  {
    previous_temperature =t;
    previous_humidity = h;
    //moisture = analogRead(MOISTURE_A0);
    //moisture_new = map(moisture, 0, 1023, 255, 0);
    payload[0] = 29; //previous_temperature;//t;//temperature;
    payload[1] = 63;//previous_humidity;//h;//humidity;
   // payload[2] = moisture_new >> 8 & 0xff;
    //payload[3] = moisture_new & 0xff;
    //Serial.println(moisture_new);
      Serial.println("Waking UP xbee");
      pinMode(XBee_wake, OUTPUT);
      digitalWrite(XBee_wake, LOW);
    send_data_to_destination();
  }
  else if((previous_temperature - t) <= 2)
  {
    previous_temperature = (previous_temperature + t)/2;
    previous_humidity = (previous_humidity + h)/2;
    moisture = analogRead(MOISTURE_A0);
    moisture_new = map(moisture, 0, 1023, 255, 0);
    payload[0] = 29; //previous_temperature;//t;//temperature;
    payload[1] = 63;//previous_humidity;//h;//humidity;
    payload[2] = moisture_new >> 8 & 0xff;
    payload[3] = moisture_new & 0xff;
    Serial.println(moisture_new);
  
    Serial.println("Sleeping xbee because difference is less than or equal to 2");
    //put the XBee to sleep
    if(iterate == 1000)
    {
        Serial.println("Waking UP xbee: iterate 10");
        pinMode(XBee_wake, OUTPUT);
        digitalWrite(XBee_wake, LOW);
        iterate = 0;
        previous_temperature = t;
        previous_humidity =  h;
        //moisture = analogRead(MOISTURE_A0);
        //moisture_new = map(moisture, 0, 1023, 255, 0);
        payload[0] = 27; //previous_temperature;//t;//temperature;
        payload[1] = 61;//previous_humidity;//h;//humidity;
       // payload[2] = moisture_new >> 8 & 0xff;
        //payload[3] = moisture_new & 0xff;
        //Serial.println(moisture_new);
        sleep_state = 0;
      
        send_data_to_destination();
    }
    else if(sleep_state == 0)
    {
       Serial.println(" Xbee going in sleeping state ");
       pinMode(XBee_wake, INPUT); // put pin in a high impedence state
       digitalWrite(XBee_wake, HIGH);
       sleep_state = 1;
    }
    else
    {
       Serial.println(" already in sleeping state ");
    }
  }
  else if((previous_temperature - t) > 2)
  {
    // wake up the XBee
    if(sleep_state == 1)
    {
      Serial.println("Waking UP xbee");
      pinMode(XBee_wake, OUTPUT);
      digitalWrite(XBee_wake, LOW);
      sleep_state = 0;
    }
    previous_temperature = t;
    previous_humidity =  h;
    //moisture = analogRead(MOISTURE_A0);
    //moisture_new = map(moisture, 0, 1023, 255, 0);
    payload[0] = 27; //previous_temperature;//t;//temperature;
    payload[1] = 61;//previous_humidity;//h;//humidity;
    //payload[2] = moisture_new >> 8 & 0xff;
    //payload[3] = moisture_new & 0xff;
    //Serial.println(moisture_new);
  }
 // delay(1000);
}

void loop() {   
  iterate++;
  if(f_wdt == 1 &&  retry_counter == 0)
  {
    /* Don't forget to clear the flag. */
    f_wdt = 0;  
    /* Re-enter sleep mode. */
    Serial.println("Entering sleep mode");
    delay(100);
    enterSleep();
  }
  else
  {
    Serial.println("f_wdt is 0");
    /* Do nothing. */
  } 
   delay(1000);
}

/*****************************************************************************************/
void send_data_to_destination(void)
{
  
   addr64 = XBeeAddress64(0x0013a200,0x40BD3AA1/*0x40F16F1D*/);
   zbTx = ZBTxRequest(addr64, payload, sizeof(payload));

   while (retry_counter++ < MAX_RETRY_COUNT)
   {
    Serial.println("First destination: trying to send");

    xbee_request_response.send(zbTx);
    // flash TX indicator
    //flashLed(statusLed, 1, 100);
    // after sending a tx request, we expect a status response wait up to half second for the status response
    if (xbee_request_response.readPacket(500)) {
      // got a response! should be a znet tx status              
      if (xbee_request_response.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
      xbee_request_response.getResponse().getZBTxStatusResponse(txStatus);
    
      // get the delivery status, the fifth byte
      if (txStatus.getDeliveryStatus() == SUCCESS) {
        // success.  time to celebrate
         //flashLed(statusLed, 5, 50);
         //digitalWrite(statusLed, HIGH);
         Serial.println("time to celebrate");
         Serial.println(retry_counter);
        
         //retry_counter=MAX_RETRY_COUNT+1;
      } else {
        // the remote XBee did not receive our packet. is it powered on?
        //flashLed(errorLed, 3, 500);
         Serial.println("local XBee did not power");
         
         Serial.println(retry_counter);
         //send_data_to_different_path();
      }
      }
    } else if (xbee_request_response.getResponse().isError()) {
      //nss.print("Error reading packet.  Error code: ");  
      //nss.println(xbee.getResponse().getErrorCode());
    } else {
        Serial.println("local XBee no response");
        Serial.println(retry_counter);
        
        //send_data_to_different_path();
    }
    retry_counter++;
    delay(1000);/* put a very little pause 200ms between successive retries*/
   }
   
   if(retry_counter == MAX_RETRY_COUNT)
   {  
     retry_counter = 0;
    Serial.println("current destination: RESET data sending flag");
    //send_data_to_different_path();
   }
    retry_counter = 0;
  }
/*****************************************************************************************/
void send_data_to_different_path(void)
{
       int retry_count_1= 0;
       while (retry_count_1 < MAX_RETRY_COUNT)
       {      
          Serial.println("Next best neighbor: trying to send");
          addr64 = XBeeAddress64(0x0013a200,0x40BD3AA1/*0x40F16F1D*/);
          zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
          xbee_request_response.send(zbTx);
          //flashLed(statusLed, 1, 100);
           // after sending a tx request, we expect a status response
          // wait up to half second for the status response
          if (xbee_request_response.readPacket(500)) {
            // got a response!
        
            // should be a znet tx status              
            if (xbee_request_response.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
              xbee_request_response.getResponse().getZBTxStatusResponse(txStatus);
        
              // get the delivery status, the fifth byte
              if (txStatus.getDeliveryStatus() == SUCCESS) {
                // success.  time to celebrate
                //flashLed(statusLed, 20, 50);
                 Serial.println("Next best neighbor: time to celebrate1");
                 retry_count_1= MAX_RETRY_COUNT+1;
                
                 
              } else {
                // the remote XBee did not receive our packet. is it powered on?
                //flashLed(errorLed, 3, 500);
               
              }
            }
          } else if (xbee_request_response.getResponse().isError()) {
            //nss.print("Error reading packet.  Error code: ");  
              Serial.println("Next best neighbor:error reading packet");
          
            //nss.println(xbee.getResponse().getErrorCode());
          } else {
            Serial.println("Next best neighbor: local XBee did not power 2");
           // xbee.send(zbTx_new);
            
            //flashLed(errorLed, 2, 50);
          }
            retry_count_1++;
            delay(200);/* put a very little pause 200ms between successive retries*/
       }
       Serial.println("next best destination: RESET data sending flag");
}



