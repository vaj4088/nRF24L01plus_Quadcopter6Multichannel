// Do not remove the include below
#include "nRF24L01plus_Quadcopter6Multichannel.h"

#include "RF24.h"

// Comment to not get Serial debug message.
// Uncomment for Serial debug message.
// #define RC_DEBUG

const unsigned long version = 20170824 ;
const char versionSuffix = 'a' ;
//#define myRF24_NOTRANSMIT

RF24 myRF24(8, 10);

// address and message needed for the pairing
uint8_t pairingAddress[] = {101, 101, 101, 101, 101};
uint8_t pairingMessage[] = {223, 25, 85, 87, 86, 170, 50, 0};

// pipes share the	four most significant bytes
// 					least significant byte is unique for each pipe
// Hex is DF 19 55 57 C1 / C3 / C4 / C5 / C6
// Command: MSBit first to LSBit
// Data: LSByte first to MSByte, MSBit first to LSBit in each byte
uint8_t myAddress[][5] = {{223, 25, 85, 87, 193}, {195}, {196}, {197}, {198}};

/*
 *   The message template according to Hamster Cage Drones

  payload[0]=throttle;
  payload[1]=yaw;
  payload[2]=yawtrim;
  payload[3]=pitch;
  payload[4]=roll;
  payload[5]=pitchtrim;
  payload[6]=rolltrim;
  payload[7]=flyrun;

 *
 */
// the message template to be sent out during normal operation
// byte		meaning
// 0		throttle
// 1		yaw
// 2		counter 0
// 3		pitch
// 4		roll
// 5		counter 1
// 6		counter 2
// 7		flag. bit 0-3 for flips, bit 4 for +/- switch (bit 4 set = '-')
uint8_t commandMessage[] = {0, 128, 128, 128, 128, 128, 128, 0};

int channelNum ;
const uint8_t channelList[] = {
		0x02, 0x0b, 0x12, 0x1b, 0x21,
		0x31, 0x3c, 0x41, 0x4b, 0x4c,
		0x51, 0x5b, 0x6c
} ;

// buffer for receiving
const int bufferSize = 8;
uint8_t myBuffer[bufferSize];

// User commands and input position.
byte hotItemNumber = 0 ;
int userCommand[8] = {0, 0, 0, 0, 0, 0, 0, 0} ;
//uint8_t commandMessage[] = {0, 128, 11, 128, 128, 124, 125, 0};
float userCommandTimeSeconds ;
float userCommandTimeSecondsInitial ;
const float userCommandTimeSecondsThreshold = 0.400 ;

unsigned long commandTimer;
unsigned long commandThresh = 20;


// initialize the receiver buffer
void initBuffer() {
	memset(myBuffer, 0, bufferSize);
} // void initBuffer() {

// initialize the RF24 transceiver
void initRF24(RF24 rf24) {
	rf24.begin();

	// needed, since the nRF24L01 is not resetting itself at startup!
	rf24.stopListening();

	rf24.setAutoAck(true);

	rf24.setAddressWidth(5);

	// setting for 10 retries with 750 microsecond limit
	rf24.setRetries(2, 10);

	rf24.setChannel(60);

	rf24.setPayloadSize(8);

	rf24.openReadingPipe(2, myAddress[1]);
	rf24.openReadingPipe(3, myAddress[2]);
	rf24.openReadingPipe(4, myAddress[3]);
	rf24.openReadingPipe(5, myAddress[4]);

	// RF24_PA_MIN RF24_PA_LOW RF24_PA_HIGH RF24_PA_MAX
//	rf24.setPALevel(RF24_PA_HIGH);
	rf24.setPALevel(RF24_PA_MIN) ;  //  Works better when close.

	rf24.setDataRate(RF24_1MBPS);

	rf24.enableDynamicPayloads();
	rf24.enableDynamicAck();
	rf24.enableAckPayload();

	rf24.setCRCLength(RF24_CRC_8);
//	rf24.startListening() ;
}

void clearFlagsAndWrite(RF24 rf24, uint8_t * msg, uint8_t msgLength) {
	rf24.flush_tx() ;
	bool tx_ds ;
	bool max_rt ;
	bool rx_dr ;
	rf24.whatHappened(tx_ds, max_rt, rx_dr) ;
	//  whatHappened also happens to clear the flags.

	Serial.print(tx_ds?F("Data Sent      "):F("Data NOT Sent  ")) ;
	Serial.print(
			max_rt?F("    Max Retransmits  "):F("NOT Max Retransmits  ")) ;
	Serial.print(rx_dr?F("New Data Rcvd."):F("NO  Data Rcvd.")) ;
	rf24.write(msg, msgLength) ;
}

boolean pairingSuccessful(RF24 rf24, uint8_t * msg, uint8_t msgLength) {
//	rf24.write(msg, msgLength) ;
#ifndef myRF24_NOTRANSMIT
 	moveCursorTo(16, 1) ;
 	//  Write for pairing.
	clearFlagsAndWrite(rf24, msg, msgLength) ;
#endif
	long responseTimer = millis();
	while (1) {
		if (rf24.available()) {
			rf24.read(myBuffer, bufferSize);
			String outString = "Received: ";

			for (int k=0; k<bufferSize-1; k++) {
				outString += String(myBuffer[k]) + " ";
			} // for (int k=0; k<bufferSize-1; k++) {

			outString += String(myBuffer[bufferSize-1]);
			Serial.println(outString);
			initBuffer();
		}
		if(millis() - responseTimer > 20) { // if (myRF24.available()) {
			break;
		}
	}
	return true ;
}
// pairing with the quadcopter
// this is a blocking procedure
void pair() {
#ifdef RC_DEBUG
	Serial.print("pairing");
#endif
	myRF24.openWritingPipe(pairingAddress);
	myRF24.openReadingPipe(0, pairingAddress);

#ifdef RC_DEBUG
	uint8_t heartbeatCounter = 0;
#endif

	while(1) {
		myRF24.flush_tx();
		if(pairingSuccessful(myRF24, pairingMessage, sizeof(pairingMessage))) {
#ifdef RC_DEBUG
			Serial.println(String(millis()) +
					"milliseconds, had successful pairing.");
#endif
			break;
		} else { // if(myRF24.write(pairingMessage, sizeof(pairingMessage))) {
#ifdef RC_DEBUG
			if (heartbeatCounter++ < 9) {
				Serial.print(".");
			} else { // if (heartbeatCounter++ < 9) {
				Serial.println(".");
				heartbeatCounter = 0;
			} // } else { // if (heartbeatCounter++ < 9) {
#endif
			delay(500);
		} // } else { // if(myRF24.write(pairingMessage, sizeof(pairingMessage))) {
	} // while(1) {

	myRF24.openWritingPipe(myAddress[0]);
	myRF24.openReadingPipe(0, myAddress[0]);
} // void pair() {

// Return number of seconds.
float seconds() {
	return millis()/1000.0 ;  //  The ".0" is important to make this a float.
}

void setChannel(unsigned int x) {
//	myRF24.stopListening() ;
	myRF24.setChannel(x) ;
//	myRF24.startListening() ;
}

// send a command to the quadcopter
void sendCommand(RF24 rf24) {
//	enum UserCommandStates {reset, idle, throttleUp, positive1, negative1,
//		positive2, negative2, zero} ;
	static boolean firstTime = true ;
	static int itemSelect = 1 ;
	userCommandTimeSeconds = seconds() ;
	switch (userCommandState) {
	case reset:
		userCommand[0] = 0 ;  //  Throttle off.
		if (firstTime) {
			firstTime = false ;
			screenUpdate(0, userCommand, channelList[channelNum]) ;
			channelNum = 0 ;
		}
		if (userCommandTimeSeconds - 5.0 >= userCommandTimeSecondsInitial) {
			userCommandState  = idle ;
			firstTime = true ;
			userCommandTimeSecondsInitial = userCommandTimeSeconds ;
		}
		break ;
	case idle:
		userCommand[0] = 0 ;  //  Throttle off.
		if (firstTime) {
			firstTime = false ;
			screenUpdate(0, userCommand, channelList[channelNum]) ;
		}
		if (userCommandTimeSeconds - userCommandTimeSecondsThreshold >=
				userCommandTimeSecondsInitial) {
			userCommandState  = throttleUp ;
			firstTime = true ;
			userCommandTimeSecondsInitial = userCommandTimeSeconds ;
		}
		break ;
	case throttleUp:
		userCommand[0] = 15 ;
		if (firstTime) {
			firstTime = false ;
			screenUpdate(0, userCommand, channelList[channelNum]) ;
		}
		if (userCommandTimeSeconds - userCommandTimeSecondsThreshold >=
				userCommandTimeSecondsInitial) {
			userCommandState  = positive1 ;
			firstTime = true ;
			userCommandTimeSecondsInitial = userCommandTimeSeconds ;
		}
		break ;
	case positive1:
		userCommand[itemSelect] = 15 ;
		if (firstTime) {
			firstTime = false ;
			screenUpdate(itemSelect, userCommand, channelList[channelNum]) ;
		}
		if (userCommandTimeSeconds - userCommandTimeSecondsThreshold >=
				userCommandTimeSecondsInitial) {
			userCommandState  = negative1 ;
			firstTime = true ;
			userCommandTimeSecondsInitial = userCommandTimeSeconds ;
		}
		break ;
	case negative1:
		userCommand[itemSelect] = -15 ;
		if (firstTime) {
			firstTime = false ;
			screenUpdate(itemSelect, userCommand, channelList[channelNum]) ;
		}
		if (userCommandTimeSeconds - userCommandTimeSecondsThreshold >=
				userCommandTimeSecondsInitial) {
			userCommandState  = positive2 ;
			firstTime = true ;
			userCommandTimeSecondsInitial = userCommandTimeSeconds ;
		}
		break ;
	case positive2:
		userCommand[itemSelect] = 15 ;
		if (firstTime) {
			firstTime = false ;
			screenUpdate(itemSelect, userCommand, channelList[channelNum]) ;
		}
		if (userCommandTimeSeconds - userCommandTimeSecondsThreshold >=
				userCommandTimeSecondsInitial) {
			userCommandState  = negative2 ;
			firstTime = true ;
			userCommandTimeSecondsInitial = userCommandTimeSeconds ;
		}
		break ;
	case negative2:
		userCommand[itemSelect] = -15 ;
		if (firstTime) {
			firstTime = false ;
			screenUpdate(itemSelect, userCommand, channelList[channelNum]) ;
		}
		if (userCommandTimeSeconds - userCommandTimeSecondsThreshold >=
				userCommandTimeSecondsInitial) {
			userCommandState  = zero ;
			firstTime = true ;
			userCommandTimeSecondsInitial = userCommandTimeSeconds ;
		}
		break ;
	case zero:
		userCommand[itemSelect] = 0 ;  // Zero command ;
		userCommand[0] = 0 ;  //  Throttle off.
		if (firstTime) {
			firstTime = false ;
			screenUpdate(0, userCommand, channelList[channelNum]) ;
		}
		if (userCommandTimeSeconds - userCommandTimeSecondsThreshold >=
				userCommandTimeSecondsInitial) {
			userCommandState  = throttleUp ;
			firstTime = true ;
			if (++itemSelect > 6) {
				itemSelect = 1 ;
				channelNum += 1 ;
				if ((long)channelNum>=(long)sizeof(channelList)) {
					channelNum = 0 ;
				}
				setChannel(channelList[channelNum]) ;
			}
			userCommandTimeSecondsInitial = userCommandTimeSeconds ;
		}
		break ;
	default:
		Serial.println(F("Error in switch userCommandState")) ;
		while (true) ;
	}

	const int conversionOffset = 0 ;
	//
	// Convert user commands to quadcopter commands.
	//
	// the command message template to be sent out during normal operation
	// byte		meaning
	// 0		throttle
	// 1		yaw
	// 2		counter 0
	// 3		pitch
	// 4		roll
	// 5		counter 1
	// 6		counter 2
	// 7		flag. bit 0-3 for flips, bit 4 for +/- switch (bit 4 set = '-')
	commandMessage[0] = userCommand[0] ;
	commandMessage[1] = userCommand[1] + conversionOffset ;
	commandMessage[2] = userCommand[2] + conversionOffset ;
	commandMessage[3] = userCommand[3] + conversionOffset ;
	commandMessage[4] = userCommand[4] + conversionOffset ;
    commandMessage[5] = userCommand[5] + conversionOffset ;
	commandMessage[6] = userCommand[6] + conversionOffset ;
	commandMessage[7] = userCommand[7] ;
////	if (
////		(sizeof(commandMessage)!=8 )||
////		(userCommand[0]!=5)||
////		(commandMessage[0]!=5)
////		) {
////	moveCursorTo(24, 1) ;
////		Serial.print("sizeof(commandMessage)=") ;
////		Serial.print(sizeof(commandMessage)) ;
////	Serial.println(".") ;
////	Serial.print("   userCommand[0]=") ;
////	Serial.print(userCommand[0]) ;
////	Serial.print(".  0x") ;
////	Serial.println(userCommand[0], HEX) ;
////	Serial.print("commandMessage[0]=") ;
////	Serial.print(commandMessage[0]) ;
////	Serial.print(".  0x") ;
////	Serial.println(commandMessage[0], HEX) ;
////	while (true) ;  ////  Intentional infinite loop.
////}
//	myRF24.write(commandMessage, sizeof(commandMessage), 0) ;
#ifndef myRF24_NOTRANSMIT
 	moveCursorTo(16, 1) ;
	//  Write for commands.
	clearFlagsAndWrite(rf24, commandMessage, sizeof(commandMessage)) ;
#endif
} // void sendCommand() {

/*

 CSI is ESC [

                   Key            Normal
                  -------------+----------+
                  Cursor Up    | CSI A    |
                  Cursor Down  | CSI B    |
                  Cursor Right | CSI C    |
                  Cursor Left  | CSI D    |
                  -------------+----------+
 *
 */
void processUserCommand(char c) {
//
//	The char datatype is a signed type, meaning
//	that it encodes	numbers from -128 to 127.
//	For an unsigned, one-byte (8 bit) data type, use the byte data type.
//
	if (c < 0)
		return;
	switch (rateState) {
	case waiting: {
		if (c == ESC) {
			rateState = receivedEsc;
		}
	}
	break;
	case receivedEsc:
		if (c == LEFT_SQUARE_BRACKET) {
			rateState = receivedLeftSquareBracket;
		} else {
			rateState = waiting;
		}
		break;
	case receivedLeftSquareBracket:
		if (c == 'A') {
//			countA++;           // Up Arrow
			rateState = waiting;
			processUpArrow();
		} else if (c == 'B') {
//			countB++;           // Down Arrow
			rateState = waiting;
			processDownArrow();
		} else if (c == 'C') {
//			countC++;           // Right Arrow
			rateState = waiting;
			processRightArrow();
		} else if (c == 'D') {
//			countD++;           // Left Arrow
			rateState = waiting;
			processLeftArrow();
		} else {
//			countUnknown++;
			rateState = waiting;
		}
		break;
	default:
		// Anything else.
//		countUnknown++;
		break;
	}
}

void processUpArrow() {
	int limitHigh[8] = { 255, 127, 127, 127, 127, 127, 127, 255} ;
	if (userCommand[hotItemNumber]<limitHigh[hotItemNumber]) {
		userCommand[hotItemNumber]++ ;
		screenUpdate(hotItemNumber, userCommand, channelList[channelNum]) ;
	}
}

void processDownArrow() {
	int limitLow[8]  = {   0,-128,-128,-128,-128,-128,-128,   0} ;
	if (userCommand[hotItemNumber]>limitLow[hotItemNumber]) {
		userCommand[hotItemNumber]-- ;
		screenUpdate(hotItemNumber, userCommand, channelList[channelNum]) ;
	}
}

void processRightArrow() {
	hotItemNumber++ ;
	if (hotItemNumber>7) hotItemNumber=0 ;
	screenUpdate(hotItemNumber, userCommand, channelList[channelNum]) ;
}

void processLeftArrow() {
	hotItemNumber-- ;
	if (hotItemNumber>7) hotItemNumber=7 ;
	screenUpdate(hotItemNumber, userCommand, channelList[channelNum]) ;  ////
}


// prepare a command
void getCommandData() {
	// check if we get data
#ifdef RC_DEBUG
	int throttle = commandMessage[0] ;
	int yaw      = commandMessage[1] ;
	int pitch    = commandMessage[3] ;
	int roll     = commandMessage[4] ;
	boolean flipFlag = (commandMessage[7] && 15) == 15 ;

	String delimiter = "\t";

	String outString = "";

	outString += String(throttle) + delimiter;
	outString += String(yaw) + delimiter;
	outString += String(pitch) + delimiter;
	outString += String(roll) + delimiter;
	outString += String(flipFlag);
	Serial.println(outString);
#endif
/****************************************************************/
#ifdef RC_DEBUG
	throttle =   5 ;
	yaw      = 128 ;
	pitch    = 128 ;
	roll     = 128 ;
	flipFlag = false ;
#else
	int throttle =   5 ;
	int yaw      = 128 ;
	int pitch    = 128 ;
	int roll     = 128 ;
	boolean flipFlag = false ;
#endif

	// put the calculated values in the command message
	commandMessage[0] = throttle;
	commandMessage[1] = yaw;
	commandMessage[3] = pitch;
	commandMessage[4] = roll;
	commandMessage[7] = (flipFlag) ? 15 : 0 ;
/*****************************************************************/
} // void getCommandData() {

boolean screenUpdateItem(byte _line, byte _column, int _value) {
	moveCursorTo(_line, _column) ;
	int x = _value ;
	if (x>=0) {
		Serial.print("+") ;
	} else {
		Serial.print("-") ;
	}
	x = abs(x) ;
	byte leadingSpaces = 2-(int)log10(x) ;
	for (byte j = 1; j<=leadingSpaces; j++) {
		Serial.print(" ") ;
	}
	Serial.print(x) ;
	return true ;
}

boolean screenUpdate(int _hotItemNumber, int _item[8],  byte channel) {
	//
	// hotItemNumber is in the range [0,7].
	// Each item is in the range [-128, 127].
	//
	moveCursorTo(3, 1);
	eraseWholeLine();
	setCharacterAttributesOff() ;
	byte column = 2 ;
	for (byte i = 0; i<8; i++) {
		if (i == _hotItemNumber) setCharacterAttributeReverseVideo() ;
		screenUpdateItem(3, column, _item[i]) ;
		if (i == _hotItemNumber) setCharacterAttributesOff() ;
		column +=9 ;
	}
	moveCursorTo(3, 80) ;
	Serial.print(F("         0x")) ;
	if (channel<16) Serial.print('0') ;
	Serial.print(channel, HEX) ;

	return true ;
}

void outputVersionInfo() {
	Serial.print(F("Version "));
	Serial.print(version);
	Serial.println(versionSuffix);
	hideCursor();
}

void localScreenSetup() {
	moveCursorTo(1, 1) ;
	setBold() ;
	userCommandTimeSecondsInitial = seconds() ;
	//
	// Width for each item is 9 characters (9 columns).
	//
//	Serial.println(F("0-Full   Rotate            Fwd/Aft  Left/Rt")) ;
//	Serial.print(  F("Throttle Yaw      Counter0 Pitch    Roll     ")) ;
//	Serial.print(F("Counter1 Counter2 Flags")) ;
	/*
	  The message template according to Hamster Cage Drones:
	  payload[0]=throttle;
	  payload[1]=yaw;
	  payload[2]=yawtrim;
	  payload[3]=pitch;
	  payload[4]=roll;
	  payload[5]=pitchtrim;
	  payload[6]=rolltrim;
	  payload[7]=flyrun;
	 */
	Serial.println(F("0->Full  Fwd/Aft  Left/Rt")) ;
	Serial.print(  F("Throttle Pitch    Roll     PitchTrm RollTrm? ")) ;
	Serial.print(F("???????? ???????? Flags (128=>Lights Off)")) ;
	Serial.print(F(" Channel")) ;
	setNormal() ;
	moveCursorTo(12, 1) ;
	outputVersionInfo() ;
}

void setup() {
	screenSetup(115200UL, localScreenSetup) ;
	screenUpdate(hotItemNumber, userCommand, channelList[channelNum]) ;

	// init receiver buffer
	initBuffer();

	// init transceiver
	initRF24(myRF24);

	// pair with the quadcopter
	// this call is blocking until the quadcopter is paired
	pair();

//	Wait a few seconds before sending the first command.
 	moveCursorTo(14, 1) ; Serial.println(F("Delaying...")) ;
	delay(4000) ;

	// set the timer for the command intervals
	commandTimer = millis();

 	moveCursorTo(14, 1) ; Serial.println(F("Delay complete.")) ;
} // void setup() {
// get the data for commands every <commandThresh> ms
// calculate the controller values
// send the data to the quadcopter
void loop() {
	// check timer for command input
	if (millis() - commandTimer > commandThresh) {
		// get the command data
		getCommandData();

		// reset command timer
		commandTimer = millis();
	} // if (millis() - commandTimer > commandThresh) {
	if (Serial.available()) {
		processUserCommand(Serial.read()) ;
	}
	// keep sending commands regardless
	sendCommand(myRF24);
} // void loop() {
