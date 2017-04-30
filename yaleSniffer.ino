/************************************************************************************************************
 *                     Rx/Tx COMMUNICATIONS WITH THE YALE HSA6400 433MHz ALARM SYSTEM
 ************************************************************************************************************                     
 Author: Steve Williams

 This is currently running on a NANO
 
 Initial version - Cheap Rx module does not work very well used more expensive module & rewrite Rx
                   New Rx working fine - still nosie in absense of signal but thats the way it is
                   Re-write to try and detect remote
                   Re-write up and running, still can't detect remote
                   Remote now working was due to shorter Tx idle at start of message
                   Add Tx funtionallity (Tx out = Pin 9)
 
 
   Protocol:

   Logic one coveyed by Line low = 320uS followed by line high for 1.4mS
   Logic zero conveyed by line low for 1100us followed by line high for 800us
   Each byte is sent like this:
      B1010CDDDDDDDDS

      Where: B = line high for 5mS to mark the beginning
             1010 is the pre-amble
             C = 1 for a normal databyte or zero for this is the checksum
             D = data bits 7 down to zero, forming the byte being transmitted
             S = the stop bit (sometimes one, sometimes zero - don't know why)

   Each message consists of 5 data bytes plus one checksum byte. The checksum is calculated
   as follows:
              (5 + each data byte + csum) inverted should be zero
              To create the CSUM seed it with 5 add each byte of the message, invert then send the result

   each message is repeated 9 times forming the "packet" sent over the air for each event

   The data its self works like this:

   ABCDEF

   Where: ABC = the unique "mac" address of the device transmitting (3 bytes)
            D Uppder nibble is the device type
            D (lower nibble) plus E (byte) are 12 bits of discreets that the device is sending
            F = the checksum

  OPERATION
  
  This sketch prints out of the serial port a line containing the hex bytes decoded off the air folled by
  a new line. This is the format described above including the checksum.

  To tranmit send a line of hex as descibed above but without the CSUM followed by a new line. The actual data
  sent over the air will be sent out of the serial port in such a way as it apprears that we received the data
  

*/

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////////////////////////////////
const byte RxGPIOPin = 2;
const byte LEDPin = 13;
const byte DIAGPin = 3;
const byte TxDrivePin = 9;

#define SHORT_RX_PULSE 510
#define LONG_RX_PULSE 720
#define RESET_RX_PULSE 4000

#define YALE_MESSAGE_SIZE 6
#define YALE_MESSAGE_BUFFER_SZ 64
#define YALE_PREAMBLE 0xA000
#define YALE_BITS_PER_SYMBOL 14
#define YALE_MESSAGE_REPEATS 9

typedef enum
{
  YALEDRIVER_STATE_IDLE,
  YALEDRIVER_STATE_SEEN_RESET
} yaleDriverState_t;

typedef struct
{
  volatile byte full;
  byte count;
  byte data[YALE_MESSAGE_SIZE + 3];
} yaleMessageBuff_t;

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Globals
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// The message bufeer - passing Rx messages to the main loop from the RxISR
volatile yaleMessageBuff_t g_messageBuff[YALE_MESSAGE_BUFFER_SZ];

volatile int g_debug;
volatile int g_txBitCounter;
volatile byte g_TxSymbolSent;
volatile unsigned int g_currentTxSymbol;
volatile byte g_txOn;

/*
   Rx ISR: Arrive here on every edge of the Rx data pin
*/
void RxISR(void)
{
  static unsigned long lastTime;
  unsigned int pulseDuration;
  unsigned long now;
  static byte csum = 5;
  static yaleDriverState_t state;
  volatile yaleMessageBuff_t *p_buffer;
  static int ignoreNext;
  static unsigned int bitMask;
  static int byteIndex;
  static int msgBuffIdx;
  static unsigned int offAirBits;
  byte tData;

  now = micros();
  pulseDuration = (now - lastTime);
  lastTime = now;

  // Mute Rx if we are transmitting
  if( g_txOn )
  {
    return;
  }

  // Filter noise
  if ( pulseDuration < 250 )
  {
    return;
  }

  // Get the current buffer
  p_buffer = &g_messageBuff[msgBuffIdx];

  // Don't let the number get silly
  if ( pulseDuration > 10000 )
  {
    pulseDuration = 10000;
    state = YALEDRIVER_STATE_IDLE;
  }

  // Have we got a reset pulse
  if ( (pulseDuration > RESET_RX_PULSE) )
  {
    state = YALEDRIVER_STATE_SEEN_RESET;

    ignoreNext = 0;
    bitMask = 0x8000;
    offAirBits = 0;

    // The main loop should have cleared this by now
    if ( p_buffer->full )
    {
      g_debug = msgBuffIdx;
    }
  }

  // Next pulse after a valid state we can collect bits
  else if ( state == YALEDRIVER_STATE_SEEN_RESET )
  {
    if ( ignoreNext == 0 )
    {
      // Ignore the next one
      ignoreNext = 1;
      if ( pulseDuration < SHORT_RX_PULSE )
      {
        // Stick the one in
        offAirBits |= bitMask;
        digitalWrite(DIAGPin, HIGH);
      }
      else
      {
        digitalWrite(DIAGPin, LOW);
      }

      // Next bit
      bitMask >>= 1;

      // Look for the preamble 1010 in the first 4 bits received (bitMask has been shifted here)
      if ( bitMask == 0x0100 )
      {
        if ( (offAirBits & 0xF000) != YALE_PREAMBLE )
        {
          // Start over again - we have received junk
          state = YALEDRIVER_STATE_IDLE;
        }
      }

      // The stop bit tells us we reached the end of the message
      else if ( bitMask == 0x0002 )
      {
        // The data byte
        tData = (offAirBits & 0x07f8) >> 3;

        // Stash the data including the CSUM - if there is space available
        if ( byteIndex <= YALE_MESSAGE_SIZE )
        {
          p_buffer->data[byteIndex++] = tData;
          p_buffer->count++;
          csum += tData;
        }
        else
        {
          // Something wrong - start over
          byteIndex = 0;
          state = YALEDRIVER_STATE_IDLE;
        }

        // This bit is one for normal data & zero for the CSUM
        if ( ((offAirBits & 0x0800) == 0) )
        {
          // Time to check the CSUM
          csum = ~csum;
          if ( (csum == 0) && (byteIndex == YALE_MESSAGE_SIZE) )
          {
            // We have a new good message
            p_buffer->full = 1;
            byteIndex = 0;

            // Ready the next message buffer
            msgBuffIdx++;
            msgBuffIdx &= (YALE_MESSAGE_BUFFER_SZ - 1);
          }
          else
          {
            // Re-use this message buffer
            p_buffer->count = 0;
            byteIndex = 0;
          }

          // Preset CSUM for next time
          csum = 5;
        }

        state = YALEDRIVER_STATE_IDLE;
      }
    }
    else
    {
      ignoreNext = 0;
    }
  }
}

/*
 * Timer 1 output compare.
 * Toggles the Tx pin (Pin 9) implementing the Yale protocol
 * Sets a flag after each symbol has been sent. Main loop provides
 * the next symbol as required
 */
ISR(TIMER1_COMPA_vect)
{
  static unsigned int nextOC;

  if( nextOC )
  {
    // Have we finised this symbol
    if( g_txBitCounter == 0 )
    {
      // Timer1 OFF leaving Tx on
      TCCR1B = 0;
      
      // Flag main loop
      g_TxSymbolSent = 1;
    }
    else
    {
      OCR1A = nextOC;
    }
    
     nextOC = 0;
    
    return;
  }

  if( g_currentTxSymbol & 0x8000 )
  {
    // Logic 1 is 320us low followed by 1.4ms high
    OCR1A = 320 / 4;
    nextOC = 1400 / 4;
  }
  else
  {
    // Logic 0 is 1.1ms low followed by 800us high   
    OCR1A = 1100 / 4;
    nextOC = 800 / 4;
  }

  // Next bit
  g_currentTxSymbol <<= 1;

  g_txBitCounter--;
}

/*
 * Start the Tx sequence
 * Turn ON the transmitter, Arrange for the Timer1 int to fire in 5ms
 * the main loop will provide the data as its needed
 */
void startTxSequence()
{
  // Number of bits per symbol on the air
  g_txBitCounter = YALE_BITS_PER_SYMBOL;
  
  // Force Tx ON if it was OFF
  if( g_txOn == 0 )
  {
    TCCR1C = (1 << FOC1A);
    g_txOn = 1;
  }

  // Set O/C in 5ms
  OCR1A = 5000 / 4;

  // Start the timer
  TCCR1B |= (1 << WGM12) | (1 << CS10) | (1 << CS11);
}

// Initialise the board
void setup()
{
  noInterrupts();

  pinMode(LEDPin, OUTPUT);
  pinMode(DIAGPin, OUTPUT);
  pinMode(RxGPIOPin, INPUT);
  pinMode(TxDrivePin, OUTPUT);

  // This is the receiver
  attachInterrupt(digitalPinToInterrupt(RxGPIOPin), RxISR, CHANGE);

  // Talking to the PC
  Serial.begin(9600);

  /*
     Setup timer clock  = 16Mhz / 64 = 250Khz (4us)
     Output compare: Arrive in OC int in OCR1A * 4us
     Timer counts to OCR1A then reset back to zero
     We are toggling D9 on every match - This is the Tx pin
  */
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  TCCR1A |= (1 << COM1A0);
  TCCR1B |= (1 << WGM12);   // CTC mode
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt

  interrupts();
}

// Convert two ASCII encoded hex bytes to binary
byte hexToBin(char *hexByte)
{
  byte retValue = 0;
  byte h,l;

  h = toupper(hexByte[0]);
  l = toupper(hexByte[1]);

  if( isxdigit(h) )
  {
    if( h > '9' ) h -= 7;
    h -= '0';
    h <<= 4;
  }

  if( isxdigit(l) )
  {
    if( l > '9' )  l -= 7;
    l -= '0';

    retValue = h | l;
  }

  return retValue;
}

void loop()
{
  int n;
  char str[25];
  unsigned long now = 0;
  int count;
  static unsigned long lastTime = 0;
  static byte lastCsum = 0;
  byte thisCsum;
  static int msgIdx = 0;
  volatile yaleMessageBuff_t *p_buffer;
  static int blink;
  byte blinkState = 0;
  static byte lastBlinkState;
  static byte messageToSend[YALE_MESSAGE_SIZE];
  byte txMsgByte;
  static int outGoingTxPosn;
  static byte outGoingCsum;
  static byte triggerTx;
  static byte sendOnAir;
  static byte txMsgRepeatCount;
  byte serIn;
  char rawSerialInput[12];
  static byte serialInPos;

  now = millis();

  if ( g_debug )
  {
    sprintf(str, "Debug: %02X", g_debug);
    Serial.print(str);
    g_debug = 0;
  }

  p_buffer = &g_messageBuff[msgIdx];

  // Handle new messages as they are buffered
  if ( p_buffer->full )
  {
    count = p_buffer->count;
    thisCsum = p_buffer->data[count - 1];

    // If we have a different message or we timeout
    if ( ((lastCsum != thisCsum) || ((now - lastTime) > 1000L)) )
    {
      lastCsum = thisCsum;
      sprintf(str, "Data: ");

      Serial.print(str);
      for (n = 0; n < count; n++)
      {
        sprintf(str, "%02X ", p_buffer->data[n]);
        Serial.print(str);
      }

      Serial.println("");
    }

    p_buffer->count = 0;
    p_buffer->full = 0;
    lastTime = now;
  }

  msgIdx++;
  msgIdx &= (YALE_MESSAGE_BUFFER_SZ - 1);

  /*
   * Handle the on Air transmitter
   */

  // This is the outer loop than repeats complete messages
  if( sendOnAir )
  {
    txMsgRepeatCount = YALE_MESSAGE_REPEATS;
    sendOnAir = 0;
    triggerTx = 1; 
  }

  // Start the sequence with the first byte of the message
  if( triggerTx )
  {
    triggerTx = 0;
    outGoingTxPosn = 0;
    outGoingCsum = 5;
    txMsgByte = messageToSend[outGoingTxPosn++];
    g_currentTxSymbol = YALE_PREAMBLE | (1 << 11) | (txMsgByte << 3);
    outGoingCsum += txMsgByte;
    g_TxSymbolSent = 0;
    startTxSequence();
  }

  // Handle the rest of the individual bytes followed by the CSUM
  if( g_TxSymbolSent && (outGoingTxPosn != -1) )
  {
    g_TxSymbolSent = 0;

    if( outGoingTxPosn > (YALE_MESSAGE_SIZE -2) )
    {
      // Send the CSUM
      outGoingTxPosn = -1;
      outGoingCsum = ~outGoingCsum;
      messageToSend[YALE_MESSAGE_SIZE-1] = outGoingCsum;
      g_currentTxSymbol = YALE_PREAMBLE | (outGoingCsum << 3);
    }
    else
    {
      // Send next byte
      txMsgByte = messageToSend[outGoingTxPosn++];
      g_currentTxSymbol = YALE_PREAMBLE | (1 << 11) | (txMsgByte << 3);
      outGoingCsum += txMsgByte;
    }

    startTxSequence();
  }

  // Kick of the next message in the repeated sequence. Turn OFF Tx when done
  if( g_TxSymbolSent && (outGoingTxPosn == -1) )
  {
    g_TxSymbolSent = 0;
    
    if( txMsgRepeatCount )
    {
      txMsgRepeatCount--;

      // Stop after all repeats done
      if( txMsgRepeatCount == 0 )
      {
         // Turn Tx OFF after 5ms
        delay(5);
        TCCR1C = (1 << FOC1A);

        // Print it as if we had just received it
        sprintf(str, "Data: ");
        Serial.print(str);
       
        for(n=0; n < (YALE_MESSAGE_SIZE); n++)
        {
          sprintf(str, "%02X ",messageToSend[n]);
          Serial.print(str);
        }

        Serial.println("");

        g_txOn = 0;      
      }
      else
      {
        // Send another one
        triggerTx = 1;
      }
    }
  }

  /*
   * Hanlde serial message from the PC for Tx on the air
   * format is 10 hex digits followed by NL
   */
  if( Serial.available() > 0 )
  {
    serIn = Serial.read();
    rawSerialInput[serialInPos++] = serIn;
    rawSerialInput[serialInPos] = 0;

    if( serialInPos >= (sizeof(rawSerialInput) -1) )
    {
      serialInPos--;
    }

    // Terminator
    if( serIn == 0x0a || serIn == 0x0d )
    {
      serialInPos = 0;

      if( strlen(rawSerialInput) == 11 )
      {
        for(n=0; n < 5; n++)
        {
          messageToSend[n] = hexToBin(&rawSerialInput[n*2]);
        }

        // Send it out over the air
        sendOnAir = 1;
      }
    }
  }

  /*
   * Handle blinking the LED
   */
  if ( (now & 0x400) )
  {
    blinkState = 1;
  }
  else
  {
    blinkState = 0;
  }

  if ( blinkState != lastBlinkState )
  {
    lastBlinkState = blinkState;
    blink = 1000;
  }

  if ( blink )
  {
    blink--;
    digitalWrite(LEDPin, HIGH);
  }
  else
  {
    digitalWrite(LEDPin, LOW);
  }

}
