/*
  This is the NanoSD IO Card's Arduino Nano firmware code. This is the code that communicates with TVC
  and reads/writes the SD card.

  Pins to avoid: D0, D1 (RX/TX)
  The Nano board has to have
  - 8 lines to the data bus of TVC -  PORTC0-3: BD0-BD3 (A0-A3)
                                      PORTD4-7: BD4-BD7 (pin 4-pin 7)
  - 6 lines to the control
    - /SLOTn + /RD PD2 (pin 2)  Always IN, INT0, TVC port in
    - /SLOTn + /WR PD3 (pin 3)  Always IN, INT1, TVC port out
    - /LE on TVC-IN register  PB0 (pin 8)    Always OUT
    - /OE on TVC-OUT register PB1 (pin 9)    Always OUT
    - BANK-Sel (pin A4)   Always OUT   - as of now, NanoSD EEPROM code fits in one bank, this feature is not in use
    - Card Detect PC5 (pin A5) Always IN, shall be pulled UP internally
  - 4 lines to SD card (SPI):
    - PB3, PB4, PB5 (MOSI, MISO, CLK; pin 11, 12, 13)
    - PB2 (CS; pin 10)                - Always OUTPUT

  Always TVC initiates a command
  OPEN: opens a file for reading
    TVC OUTs to address 00 an OPEN command
    then OUTs a fileName with 0 delimiter. The fileName is max 64 chars long
    TVC INs and waits for OPEN_OK or FILE_NOT_FOUND.
    WAITING_FOR_INPUT (string was not terminated properly) and DATA_NOT_READY_YET are normal, intermediate responses.
    In case of FILE_NOT_FOUND the file cannot be opened. In case of OPEN_OK then the file size can be read from stream
    (low/high byte)
  CLOSE: close a previously opened file
    TVC OUTs to address 00 a CLOSE command
    TVC INs and waits for CLOSE_OK or CLOSE_FAILED.
  GETDATA: get data content from an open file
    TVC OUTs to 00 a GETDATA command
    -- repeat from here
    TVC INs until OK, END_OF_FILE or FILE_NOT_OPEN. DATA_NOT_READY_YET is normal, intermediate response.
    END_OF_FILE means TVC read all the bytes, operation shall finish here.
    After OK, TVC continues reading and the card returns a number, how many bytes (n) are available in its
    buffer, then the data. After that TVC reads (IN from address 00) n bytes.
    After n bytes read TVC must ack the read by OUTing ACK_GETDATA and repeat the read procedure. Anything else
    than ACK_GETDATA will cancel the getData stream, empties the buffer of the card.
  LIST: list the directory of the current dir
    TVC OUTs to 00 a LIST command
    -- repeat from here
    TVC INs until OK, END_OF_LIST, CARD_NOT_READY, DATA_NOT_READY_YET. DATA_NOT_READY_YET is a normal, intermediate response.
    END_OF_LIST means there is no more files in the list and the operation shall finish here.
    After OK, the card returns null terminated string, then a type, then a size (0 in case of directory).
    After all bytes received, TVC must ACK_LIST file record and repeat the procedure.
  CHDIR: changes directory
    TVC OUTs to 00 a CHDIR command
    TVC OUTs a path as a null terminated string. Max length is 64 chars. Can be absolute or relative path. Nano code stores the working path
    as the SD library doesn't support that.
    "/" is the root directory.
    If a path starts with "/" it is handled as absolute path and that will be working directory (if it exists).
    Otherwise it is added to the stored working directory and that will be used for the subsequent CHDIR, LIST and OPEN commands
    The path may or may not ends with "/"
    TVC can receive CHDIR_OK or CHDIR_FAILED and and intermediate DATA_NOT_READY_YET or WAITING_FOR_INPUT (if path is not terminated yet).
*/

#include <SPI.h>
#include <SD.h>
#include <wiring_private.h>
#include "CircularQueue.h"
#include "PathStack.h"
#include <string.h>
#include <EEPROM.h>

#define INPUTDATABUFFER_SIZE 64+16
#define OUTPUTDATABUFFER_SIZE 64+16
#define GETDATA_SIZE 64

#define LINE_TVCOUT_OE  PB0
#define LINE_TVCIN_CLK  PB1
#define LINE_SLOTN_RD   PD2
#define LINE_SLOTN_WR   PD3
#define LINE_BANK       PC4
#define LINE_CD         PC5

#define LINE_DATA0      PC0
#define LINE_DATA1      PC1
#define LINE_DATA2      PC2
#define LINE_DATA3      PC3
#define LINE_DATA4      PD4
#define LINE_DATA5      PD5
#define LINE_DATA6      PD6
#define LINE_DATA7      PD7

#define LINE_SS         PB2
#define LINE_MISO       PB3
#define LINE_MOSI       PB4
#define LINE_CLK        PB5
#define LINE_CS         PB2  // SS is not used in Master SPI mode

#define DEBUG

enum COMMANDS { OPEN = 0, // Open a file. Parameter is a null terminated string. Returns 0: ok
                CLOSE,   // Closes the lastopen file. Returns OK
                CHDIR,   // Change dir. Parameter is a null terminated string. Returns 0:ok, 1:FILE_NOT_FOUND
                LIST,    // Gets the list of files in the current directory (fName, type, size)
                ACK_LIST, //
                GETDATA,   // Returns OK, then [num of bytes to return (GETDATA_SIZE)]. After num of bytes read TVC must send ACK_GETDATA;
                FINFO,   // Returns the file info. Param: null terminated file name.
                GETCDIR,  // Returns the current path: length of string + string
                BANKTO0,  // NOTIMPLEMENTED YET!
                BANKTO1,
                CREATE, // 0x0a gets a fileName (length + chars), fileType (0x01 or 0x11), return CREATE_OK of CREATE_FAILED
                PUTDATA,
                CLOSEWRITE,
                GETPARAMETER, // 0x0D gets a parameter from Nano
                SETPARAMETER, // 0x0E sets a parameter to Nano
                MKDIR,
                RMDIR,  // 0x10 - removes an empty directory from SDs
                DELETE,
                CDPINCHANGE_DETECTED,
                NONE = 255
              };
enum STATUS  {OK = 0,
              OPEN_OK,
              DATA_NOT_READY_YET = 0x80,  // not sent from outputQueue
              WAITING_FOR_INPUT,
              FILE_NOT_FOUND,
              END_OF_FILE,
              END_OF_LIST,
              FILE_NOT_OPEN,              // 0x85
              CARD_NOT_READY,
              READ_OUT_OF_SYNC,
              BUFFER_OVERRUN,
              BUFFER_UNDERRUN,
              CLOSE_OK,                     // 0x8a
              CLOSE_FAILED,
              CHDIR_OK,
              CHDIR_FAILED,                 // 0x8d
              BANK_SELECT_DONE,
              CREATE_OK,
              CREATE_FAILED,                // 0x90
              DATA_RECEIVED,
              DATA_FAILED_TO_RECEIVE,
              MKDIR_OK,                     // 0x93
              MKDIR_FAILED,
              RMDIR_OK,                     // 0x95
              RMDIR_FAILED,
              DELETE_OK,
              DELETE_FAILED,                // 0x98
              INVALID_PARAMETER_ID,
              PARAMETER_SET,                 // 0x9
              STATUS_NO_SD  = 0xFF
             };

enum FILETYPE { DIRECTORY = 0,
                PLAINFILE
              };

enum PARAMETERS {
                PARAM_MENU_STATUS = 0,
                PARAM_DEFAULT_SORT,
                PARAM_SD_STATE,
                PARAM_NANO_VERSION
                };

enum MENUSTATUS {MENUSTATUS_OFF = 0,
                 MENUSTATUS_ON
                };

const int chipSelect = 10; // LINE_CS!!
const char NANOSD_ID[] PROGMEM = "NanoSD Arduino fw v0.32";

Sd2Card card;
SdVolume volume;
SdFile root;

CircularQueue inputQueue = new CircularQueue(INPUTDATABUFFER_SIZE);
CircularQueue outputQueue = new CircularQueue(OUTPUTDATABUFFER_SIZE);
enum COMMANDS currentCommand = NONE;
volatile enum STATUS status = WAITING_FOR_INPUT;
File openFile;
File currentDirFile;
union {
  long remainingBytes;
  long bytesWritten;
} fileSizeCounter;
byte savingFileType;
//long remainingBytes = 0;
bool fileIsOpen = false;
String currentDir = "/";
volatile bool sdInitialized = false;
byte memoryBankMapStore = 0;

void setup() {

  DDRD =  DDRD & 0x03;    // Leave RX and TX as they are, the rest is set to input (d4-d7, SLOTn, d3?)
  DDRC =  (1 << LINE_BANK); // Setting data lines and card detect to input and BANK line (PC4) to output
  PORTC = (1 << LINE_CD); // BANK to low, enable pullup on LINE_CD
  DDRB = (1 << LINE_TVCIN_CLK) | (1 << LINE_TVCOUT_OE) | (1 << LINE_SS) | (1 << LINE_MOSI) | (1 << LINE_CLK); // Clock, Output enable to output
  PORTB = (1 << LINE_TVCIN_CLK) | (1 << LINE_TVCOUT_OE); // set clock and output enable set to high

#ifdef DEBUG
  Serial.begin(9600);
  while (!Serial) {}
  Serial.print(F("PINC: ")); Serial.println(PINC, BIN);
#endif

  if (sdInitialized = SD.begin(chipSelect)) { // SD.begin must be called when timer0 overflow INT is enabled.
    status = DATA_NOT_READY_YET;
    currentDirFile = SD.open("/");
  }
  if ((!sdInitialized) || (!currentDirFile)) {
    status = CARD_NOT_READY;
  }

#ifndef DEBUG
  TIMSK0 &= ~_BV(TOIE0); // disable timer0 overflow interrupt, Serial uses it
#endif

  initTVC_IN_register();

  noInterrupts();
  EICRA = _BV(ISC00) | _BV(ISC01) | _BV(ISC10) | _BV(ISC11) ; // Falling edge on on INT0/1 generates an interrupt request.
  EIMSK |= (1 << INT0) | (1 << INT1);

  PCICR = _BV(PCIE1);   // enable port change int on PC5(PCI13)
  PCIFR = _BV(PCIF1);            // clear earlier interrupts on PCI - if any..
  PCMSK1 = _BV(PCINT13); // enable PCI on pin PCI13 which is PORTC5
  interrupts();
#ifdef DEBUG
  Serial.println(F("NanoSD setup() executed!"));
  Serial.print(F("SD initialized: ")); Serial.println(sdInitialized ? F("true") : F("false"));
  Serial.print(F("root dir: ")); Serial.println(currentDirFile ? F("valid") : F("invalid"));
  Serial.print(F("PINC: ")); Serial.println(PINC, BIN);
#endif
}

void initTVC_IN_register() {
  cbi(PORTB, LINE_TVCIN_CLK);  // CLOCK into register

  DDRD = B11110000;
  DDRC = B00001111 | _BV(LINE_BANK);

  PORTD = status & 0xf0;
  PORTC = (status & 0x0f) | _BV(LINE_CD);

  asm("NOP");
  asm("NOP");
  PORTB |= _BV(LINE_TVCIN_CLK);
  asm("NOP");
  PORTB &= ~_BV(LINE_TVCIN_CLK);
  asm("NOP");

  DDRD = B00000000;
  DDRC = _BV(LINE_BANK);
}

// IO access
ISR(INT0_vect) { // pin 2 (PD2) fell - RD from register happened - several cycles ago
    #ifdef DEBUG
      Serial.println(F("Z80 IN execution..."));
    #endif


  DDRD = B11110000;
  DDRC = B00001111 | _BV(LINE_BANK);

  byte localStatus = status;

  if ((localStatus & 0x80) != 0) {
    PORTD = localStatus & 0xf0;
    PORTC = (localStatus & 0x0f) | memoryBankMapStore | (1 << LINE_CD);
  } else if (!outputQueue.isEmpty()) {
    byte data = outputQueue.pop();
    PORTD = data & 0xf0;
    PORTC = (data & 0x0f) | memoryBankMapStore | (1 << LINE_CD);
    #ifdef DEBUG
      Serial.print(F("Data on PORTC and PORTD: "));Serial.println(data);
    #endif
  } else {
    PORTD = BUFFER_UNDERRUN & 0xf0;
    PORTC = (BUFFER_UNDERRUN & 0x0f) | memoryBankMapStore | (1 << LINE_CD);
  }
  // the data is set on data bus

  asm("NOP");   // let's leave some time to stabilize the data lines
  asm("NOP");

  cbi(PORTB, LINE_TVCIN_CLK);  // CLOCK into register t
  asm("NOP");
  sbi(PORTB, LINE_TVCIN_CLK);  // CLOCK into register done. TVC will read it next time
  asm("NOP");   // let's leave some time to keep the data lines (5-10 ns required)

  DDRD = B00000000;
  DDRC = _BV(LINE_BANK);
}

ISR(INT1_vect) { // pin 3 (PD3) change - WR to register happened
  // TVC wrote into register, let's read it and pass for processing
  cbi(PORTB, LINE_TVCOUT_OE);
  asm("NOP");
  asm("NOP");
  asm("NOP");
  byte datal = PINC;
  byte datah = PIND;
  sbi(PORTB, LINE_TVCOUT_OE);

  if (!inputQueue.isFull()) {
    datal = (datal & 0x0f) | (datah & 0xf0);
    inputQueue.push(datal);
  } else {
    status = BUFFER_OVERRUN;
    // TODO handle buffer is full!
  }
}

ISR (PCINT1_vect) {
  inputQueue.push(CDPINCHANGE_DETECTED);
  PCICR = 0x00; // let's not disturb the main loop for a while...
}

void handleOpen() {
  String fileName = "";
  byte c;
  status = WAITING_FOR_INPUT;
  if (fileIsOpen) {
    openFile.close();
  }
  while ((c = inputQueue.pop()) != 0) {
    fileName += (char)c;
  }

#ifdef DEBUG
  Serial.print(F("Received fileName: '")); Serial.print(fileName); Serial.println('\'');
#endif

  status = DATA_NOT_READY_YET;

  if (fileName.equals("")) {
    // no filename given, return the first/next file in the directory
    while (openFile = currentDirFile.openNextFile()) {
      if (openFile.isDirectory()) {
        continue;
      } else {
        fileName = openFile.name();
        if (fileName.endsWith(F(".CAS"))) {
#ifdef DEBUG
          Serial.print(F("File found: ")); Serial.println(fileName);
#endif
          break;
        }
      }
    }
  } else {
    if (!fileName.startsWith("/")) {
      fileName = currentDir + fileName;
    }
    fileName.toUpperCase();
    if (!SD.exists(fileName) && !fileName.endsWith(F(".CAS"))) {
      fileName += F(".CAS");
    }
#ifdef DEBUG
    Serial.print(F("File to open: ")); Serial.println(fileName);
#endif
    openFile = SD.open(fileName);
  }

  if (openFile) {
    fileName = openFile.name();
    fileName.toUpperCase();
#ifdef DEBUG
    Serial.print(F("fileName: ")); Serial.println(fileName);
#endif
    byte fileType = openFile.read();
    boolean casFile = fileName.endsWith(F(".CAS")) &&
                      ((fileType == 0x01) || (fileType == 0x11));
    fileName = fileName.substring(0, fileName.length() - 4);
#ifdef DEBUG
    Serial.print(F("File found, sending back the fileName: "));
    Serial.print(fileName); Serial.print(F(" (length: ")); Serial.print(fileName.length()); Serial.println(')');
#endif
    outputQueue.push((byte)fileName.length());
    for (int i = 0; i < fileName.length(); i++) {
      outputQueue.push(fileName.charAt(i));
    }
    fileSizeCounter.remainingBytes = openFile.size();
    if (casFile) {
      fileSizeCounter.remainingBytes -= 0x80;
      openFile.seek(0x80);
    } else {
#ifdef DEBUG
      Serial.println(F("Not a CAS file."));
#endif
      // in case of non-cas file, the full file is available
      openFile.seek(0);
    }
#ifdef DEBUG
    Serial.print(F("File size to TVC is: ")); Serial.println(fileSizeCounter.remainingBytes);
#endif
    outputQueue.push((byte)(fileSizeCounter.remainingBytes &  0x000000ff));
    outputQueue.push((byte)((fileSizeCounter.remainingBytes & 0x0000ff00) >> 8));
    outputQueue.push((byte)((fileSizeCounter.remainingBytes & 0x00ff0000) >> 16));
    outputQueue.push((byte)((fileSizeCounter.remainingBytes & 0xff000000) >> 24));

    status = OPEN_OK;
    fileIsOpen = true;
  } else {
    status = FILE_NOT_FOUND;
    fileIsOpen = false;
#ifdef DEBUG
    Serial.println(F("not found"));
#endif
  }
}

void handleClose() {
  if (fileIsOpen) {
    openFile.close();
    fileIsOpen = false;
#ifdef DEBUG
    Serial.println(F("File closed!"));
  } else {
    Serial.println(F("No file to close.."));
#endif
  }
  fileSizeCounter.remainingBytes = 0;
  outputQueue.clear();
  status = CLOSE_OK;
}

void handleGetData() {
  byte readBuffer[GETDATA_SIZE + 1];
  int len = 0;
  byte toRead = min(fileSizeCounter.remainingBytes, GETDATA_SIZE);
  if(toRead>0) {
    do {
      len = openFile.read(&readBuffer[1], toRead);
    } while (len==0 && openFile.available()>0);
  } else {
    len = 0;
  }
#ifdef DEBUG
  Serial.print(F("To Read: "));Serial.print(toRead);Serial.print(F(", read len: "));Serial.print(len);Serial.print(F(", available: "));Serial.println(openFile.available());
  Serial.print(F("File buffer read, len: "));Serial.println(len);
#endif
  if (len > 0) {
    readBuffer[0] = len;
    outputQueue.pushBlock(len + 1, readBuffer);
    fileSizeCounter.remainingBytes -= len;
#ifdef DEBUG
    Serial.print(F("Content added to outputQueue: "));Serial.print(len);
    Serial.print(F(" Queue size(): "));Serial.print(outputQueue.getSize());
    Serial.print(F(" Remaining size:"));Serial.println(fileSizeCounter.remainingBytes);
#endif
    status = OK;
  } else {
    // EOF
    status = END_OF_FILE;
#ifdef DEBUG
    Serial.println(F("EOF reached!"));
#endif
  }
}

byte fillFileInfoBuffer(File f, byte* data) {
  char *name = f.name();
  int pos = strlen(name);
  int len = pos;
  for (int i = 0; i < 12; i++) {
    if (i < pos) {
      data[i] = name[i];
    } else {
      data[i] = 0x20;
    }
  }
  pos = 12;
  data[pos++] = f.isDirectory() ? DIRECTORY : PLAINFILE;
  unsigned long fSize = f.size();
  if (f.isDirectory()) {
    data[pos++] = 0;
    data[pos++] = 0;
    data[pos++] = 0;
    data[pos++] = 0;
  } else {
    if((len > 3) && !strcmp(f.name()+len-4, ".CAS")) {
      fSize -= 128;  // Cheat here, let's not check the first byte..
    }
    data[pos++] = (byte)(fSize & 0xff);
    data[pos++] = (byte)((fSize & 0xff00) >> 8);
    data[pos++] = (byte)((fSize & 0xff0000) >> 16);
    data[pos++] = (byte)((fSize & 0xff000000) >> 24);
  }
  return pos;
}

bool endsWith(char *str, char *post) {
  int lenStr = strlen(str);
  int lenPst = strlen(post);
  int pos = lenStr - lenPst;
  int idx = 0;
  while (pos + idx < lenStr && str[pos + idx] == post[idx]) {
    idx++;
  }
  return pos + idx == lenStr;
}

void handleList() {
  byte data[18];  // OK(1) filename8.3(8+1+3 = 12) type(1) filesize(4)
  data[0] = OK;
  File cDir = SD.open(currentDir);
  File entry;
#ifdef DEBUG
  int c = 0;
  long delayFirst = 0;
  long delayLast = 0;
  long timeStamp = millis();
#endif
  while (entry = cDir.openNextFile()) {
#ifdef DEBUG
    if(c==0) {
      delayFirst = millis() - timeStamp;
    }else {
      delayLast = millis() - timeStamp;
    }
    c++;
#endif
    fillFileInfoBuffer(entry, &data[1]);
    entry.close();
// #ifdef DEBUG
//     for (int i = 0; i < 18; i++) {
//       Serial.print(data[i], HEX); Serial.print(' ');
//     }
//     Serial.println();
// #endif
    outputQueue.pushBlock(18, data);
    status = OK;
// #ifdef DEBUG
//     Serial.println(F("Data sent"));
// #endif
    byte ack = inputQueue.pop();
    status = DATA_NOT_READY_YET;
// #ifdef DEBUG
//     Serial.print(F("Answer: ")); Serial.println(ack);
// #endif

    if (ack != ACK_LIST) {
      status = READ_OUT_OF_SYNC;
      outputQueue.clear();
#ifdef DEBUG
  Serial.print(F("No ACK_LIST arrived, instead: "));Serial.println(ack);
#endif
      break;
    }
#ifdef DEBUG
    timeStamp = millis();
#endif
  }

  status = END_OF_LIST;
#ifdef DEBUG
  Serial.println(F("List done"));
  Serial.print(F("First delay: ")); Serial.print(delayFirst); Serial.print(F("ms, Last delay: ")); Serial.print(delayLast);
  Serial.print(F("ms, count: "));Serial.println(c);
#endif

  cDir.close();
}

String getFilePath() {
  PathStack ps;
  byte c;
  String fileName = "";
  while ((c = inputQueue.pop()) != 0) {
    fileName += (char)c;
  }
  status = DATA_NOT_READY_YET;
#ifdef DEBUG
  Serial.print(F("CD path arrived: ")); Serial.println(fileName);
#endif
  if (fileName.startsWith("./")) {
    fileName = fileName.substring(2);
  }
  if (!fileName.startsWith("/")) {
    fileName = currentDir + fileName;
  }
  if (!fileName.endsWith("/")) {
    fileName += "/";
  }
#ifdef DEBUG
  Serial.print(F("Preconverted name: ")); Serial.println(fileName);
#endif

  byte pos = 0;
  byte startPos = 0;
  byte len = fileName.length();
  while ((startPos < len) && (pos = fileName.indexOf("/", startPos)) != -1) {
#ifdef DEBUG
    Serial.print(F("startPos: ")); Serial.print(startPos);
    Serial.print(F(", pos: ")); Serial.println(pos);
#endif

    String p = fileName.substring(startPos, pos + 1);
#ifdef DEBUG
    Serial.print(F("after substring: '")); Serial.println(fileName);
#endif

    if (p != "../") {
      ps.push(startPos, pos + 1);
    } else {
      ps.dropTop();
    }
    startPos = pos + 1;
  }
  fileName = ps.getAbsolutePath(fileName);
#ifdef DEBUG
  Serial.print(F("Converted path: '")); Serial.print(fileName); Serial.println('\'');
#endif
  return fileName;
}

void handleChDir() {
  status = WAITING_FOR_INPUT;
  
  String fileName = getFilePath();

  File dirFile;
  if ( (dirFile = SD.open(fileName)) && dirFile.isDirectory()) {
    if (currentDirFile) {
      currentDirFile.close();
    }
    currentDirFile = dirFile;
    currentDir = fileName;
    status = CHDIR_OK;
  } else {
    if (dirFile)
      dirFile.close();
    status = CHDIR_FAILED;
  }
}

void handleFileInfo() {
  String fileName;
  byte c;
  status = WAITING_FOR_INPUT;
  while ((c = inputQueue.pop()) != 0) {
    fileName += (char)c;
  }
  if (!fileName.startsWith("/")) {
    fileName = currentDir + fileName;
  }
  status = DATA_NOT_READY_YET;
  File f;
  if ( f = SD.open(fileName) ) {
    byte buffer[17];
    buffer[0] = OK;
    byte len = fillFileInfoBuffer(f, &buffer[1]);
    outputQueue.pushBlock(len, buffer);
    status = OK;
  } else {
    status = FILE_NOT_FOUND;
  }
}

void handleGetCDir() {
  byte len = min(min(currentDir.length(), 127), OUTPUTDATABUFFER_SIZE-1);
  outputQueue.push(len);
#ifdef DEBUG
Serial.print(F("currentDir: "));Serial.print(currentDir);Serial.print(F(", currentDir len: "));Serial.println(len);
#endif

  for (int i = 0; i < len; i++) {
    outputQueue.push((byte)currentDir.charAt(i));
  }
  status = OK;
}

void handleBank(byte bank) {
  memoryBankMapStore = (bank << LINE_BANK);
  status = BANK_SELECT_DONE;
}

void handleCDPinChangeDetected() {
  // let's leave some time to the switch to rest its final position
  for (long i = 0; i < 400000; i++) { // about  300ms
    asm("nop");
  }

#ifdef DEBUG
  Serial.print(F("handleCDPinChangeDetected. PCIFR: $"));
  Serial.print(PCIFR, HEX); Serial.print(F(", PCICR: $"));
  Serial.print(PCICR, HEX); Serial.print(F(", PINC: $"));
  Serial.println(PINC, HEX);
#endif
  // let's enable PCINT1 again
  noInterrupts();
  PCICR = _BV(PCIE1);
  PCMSK1 = _BV(PCINT13); // enable PCI on pin PCI13 which is PORTC5
  PCIFR = _BV(PCIF1); // writing logical 1 clears the flag
  interrupts();

  if ((PINC & _BV(LINE_CD)) != 0) {
    // no card in socket
    status = CARD_NOT_READY;
    sdInitialized = false;
    if(currentDirFile) {
      currentDirFile.close();
    }
    if (fileIsOpen)
      openFile.close();
#ifdef DEBUG
    Serial.println(F("SD card removed."));
#endif
  } else if (!sdInitialized) {  // let's not reinitialize if the card is already detected and being used (strange, though...
#ifndef DEBUG
    TIMSK0 |= _BV(TOIE0); // enable timer0 overflow interrupt
#endif
    if (SD.begin(chipSelect)) { // SD.begin must be called when timer0 overflow INT is enabled.
      currentDirFile = SD.open(currentDir);
      if (!currentDirFile) {
        currentDirFile = SD.open("/");
        currentDir = "/";
      }
      if(currentDirFile) {
        sdInitialized = true;
        status = DATA_NOT_READY_YET;
      }
#ifdef DEBUG
    } else {
      Serial.println("Re-init of SD card failed..");
#endif
    }

#ifndef DEBUG
    TIMSK0 &= ~_BV(TOIE0); // disable timer0 overflow interrupt
#else
    Serial.print(F("SD initialized: ")); Serial.println(sdInitialized ? F("true") : F("false"));
    Serial.print(F("root dir: ")); Serial.println(currentDirFile ? F("valid") : F("invalid"));
#endif
  }
}

void handleCreate() {
  // fileName - length + chars
  // savingFileType - 0x11:non-buffered, 0x01:buffered, otherwise: just binary
  String fileName = "";
  byte len = inputQueue.pop();
  for (int i = 0; i < len && i < 63; i++) {
    fileName += (char)inputQueue.pop();
  }
#ifdef DEBUG
  Serial.print(F("fileName arrived: ")); Serial.println(fileName);
#endif
  if (fileName.equals(""))
    fileName = F("NONAME.CAS");

  savingFileType = inputQueue.pop();
#ifdef DEBUG
  Serial.print(F("fileType arrived: ")); Serial.println(savingFileType);
#endif


  if (fileName.lastIndexOf('.') == -1) {
    fileName += F(".CAS");
  } else if (fileName.lastIndexOf(F(".CAS")) != fileName.length() - 4) {
    savingFileType = 0x00;
  }

  if (!fileName.startsWith("/")) {
    fileName = currentDir + fileName;
  }

  if (openFile)
    openFile.close();
  openFile = SD.open(fileName, FILE_WRITE);
  if (openFile) {
    if ((savingFileType == 0x01) || (savingFileType == 0x11)) {
      openFile.write(savingFileType);
      for (int i = 1; i < 128; i++)
        openFile.write((byte)0x00);
    }
    fileSizeCounter.bytesWritten = 0x00;
    status = CREATE_OK;
  } else {
    status = CREATE_FAILED;
  }
}

void handlePutData() {
  byte size = inputQueue.pop();
  byte data;
#ifdef DEBUG
  Serial.print(F("size to receive: ")); Serial.println(size);
#endif
  byte retVal = 1;
  for (int i = 0; i < size; i++) {
    data = inputQueue.pop();
#ifdef DEBUG
  Serial.print(F("data: ")); Serial.println(data, HEX);
#endif
    retVal &= openFile.write(data);
  }
#ifdef DEBUG
  Serial.print(F("receive done, retVal: ")); Serial.println(retVal);
#endif
  fileSizeCounter.bytesWritten += size;
  status = (retVal == 1 ? DATA_RECEIVED : DATA_FAILED_TO_RECEIVE);
}

void handleCloseForWrite() {
  if (openFile) {
    if ((savingFileType == 0x01) || (savingFileType == 0x11)) {
      byte lastBlock = fileSizeCounter.bytesWritten % 128;
      int missingLen = 128 - (lastBlock);
      if ((savingFileType == 0x01) && (missingLen != 128)) {
        for (int i = 0; i < missingLen; i++) {
          openFile.write((byte)0x00);
        }
      }
      openFile.flush();
      // does not work on SD filesystem during write!
      // openFile.seek(2);
      // openFile.write((byte)((fileSizeCounter.bytesWritten / 128) % 256));
      // openFile.write((byte)((fileSizeCounter.bytesWritten / 128) / 256));
      // openFile.write((byte)(lastBlock));
    }
    openFile.close();
    status = CLOSE_OK;
  } else {
#ifdef DEBUG
    Serial.println(F("No file to close.."));
#endif
    status = CLOSE_FAILED;
  }
}

bool checkIdInEeprom() {
  int pos = 0;
  int len = strlen_P(NANOSD_ID);
  #ifdef DEBUG
  Serial.print(F("IDcheck len: "));Serial.println(len);
  #endif
  while ( (pos < len) && 
          ( (char)EEPROM.read(pos) == pgm_read_byte_near(NANOSD_ID + pos) ) ) {
    pos++;
  }
  #ifdef DEBUG
  Serial.print(F("pos: ")); Serial.println(pos);
  #endif
  return pos == len;
}

void initEeprom() {
  int pos = 0;
  int len = strlen_P(NANOSD_ID);
  for (pos = 0; pos < len; pos++) {
    EEPROM.update(pos, pgm_read_byte_near(NANOSD_ID + pos));
  }
  EEPROM.update(pos++, MENUSTATUS_ON); // show menu
  EEPROM.update(pos++, 0);             // don't sort
}

void handleGetParameter() {
  if (!checkIdInEeprom()) {
#ifdef DEBUG
  Serial.println(F("EEPROM is not initialized yet."));
#endif
    initEeprom();
  }
  byte paramIdx = inputQueue.pop();
  #ifdef DEBUG
  Serial.print(F("Getting parameter ID: "));Serial.println(paramIdx);
  #endif
  byte retStatus = OK;
  byte retVal = 0;
  switch (paramIdx) {
    case PARAM_MENU_STATUS:
      outputQueue.push(1); // byte
      outputQueue.push(retVal = EEPROM.read(strlen(NANOSD_ID)));
      break;
    case PARAM_DEFAULT_SORT:
      outputQueue.push(1); // byte
      outputQueue.push(retVal = EEPROM.read(strlen(NANOSD_ID)+1));
      break;
    case PARAM_SD_STATE:
      outputQueue.push(1); // byte
      outputQueue.push(retVal = (sdInitialized ? 1 : 0));
      break;
    case PARAM_NANO_VERSION:
      outputQueue.push(3); // string
      byte len = strlen(NANOSD_ID);
      for(byte i=0; i<len; i++) {
        outputQueue.push(pgm_read_byte_near(NANOSD_ID + i));
      }
#ifdef DEBUG
      const char DEBUG_POSTFIX[] PROGMEM = "-dbg";
      for(byte i=0; i<4; i++)
        outputQueue.push(pgm_read_byte_near(DEBUG_POSTFIX + i));
#endif
      outputQueue.push(0);
      retVal = 255;
      break;
    default:
      retStatus = INVALID_PARAMETER_ID;
      break;
  }
  #ifdef DEBUG
  Serial.print(F("Found value: ")); Serial.println(retVal);
  #endif
  status = retStatus;
}

void handleSetParameter() {
  if (!checkIdInEeprom()) {
  #ifdef DEBUG
  Serial.println(F("EEPROM is not initialized yet."));
#endif
    initEeprom();
  }
  byte paramIdx = inputQueue.pop();
  #ifdef DEBUG
  Serial.print(F("Setting parameter ID: "));Serial.println(paramIdx);
  #endif
  byte value = 0;
  switch (paramIdx) {
    case PARAM_MENU_STATUS:
      value = inputQueue.pop();
      EEPROM.update(strlen(NANOSD_ID), value);
      break;
    case PARAM_DEFAULT_SORT:
      value = inputQueue.pop();
      EEPROM.update(strlen(NANOSD_ID)+1, value);
      break;
    default:
      #ifdef DEBUG
      Serial.println(F("Param not found.."));
      #endif

      status = INVALID_PARAMETER_ID;
      return;
  }
  #ifdef DEBUG
  Serial.print(F("Value: "));Serial.println(value);
  #endif DEBUG
  status = PARAMETER_SET;
}

void handleMkDir() {
  String pathName = getFilePath();
  if(!SD.exists(pathName) && SD.mkdir(pathName)) {
#ifdef DEBUG
Serial.print(F("mkdir ok: "));Serial.println(pathName);
#endif
    status = MKDIR_OK;
  } else {
    #ifdef DEBUG
Serial.print(F("mkdir nok: "));Serial.println(pathName);
#endif
    status = MKDIR_FAILED;
  }
}

void handleRmDir() {
  String pathName = getFilePath();
  if(SD.exists(pathName) && SD.rmdir(pathName)) {
    status = RMDIR_OK;
  } else {
    status = RMDIR_FAILED;
  }
}

void handleDelete() {
  String fullPath = getFilePath();
  if(SD.exists(fullPath) && SD.remove(fullPath)) {
    status = DELETE_OK;
  } else {
    status = DELETE_FAILED;
  }
}

void loop() {
  currentCommand = inputQueue.pop(); // blocks until a command is pushed
  status = DATA_NOT_READY_YET;
  //  Commands only with SD card present
  if (sdInitialized) {
#ifdef DEBUG
    Serial.print(F("New command: "));
    Serial.println(currentCommand);
#endif
    switch (currentCommand) {
      case OPEN:
        handleOpen();
        break;
      case CLOSE:
        handleClose();
        break;
      case GETDATA:
        handleGetData();
        break;
      case LIST:
        handleList();
        break;
      case CHDIR:
        handleChDir();
        break;
      case FINFO:
        handleFileInfo();
        break;
      case GETCDIR:
        handleGetCDir();
        break;
      case BANKTO0:
        handleBank(0);
        break;
      case BANKTO1:
        handleBank(1);
        break;
      case CREATE:
        handleCreate();
        break;
      case PUTDATA:
        handlePutData();
        break;
      case CLOSEWRITE:
        handleCloseForWrite();
        break;
      case GETPARAMETER:
        handleGetParameter();
        break;
      case SETPARAMETER:
        handleSetParameter();
        break;
      case MKDIR:
        handleMkDir();
        break;
      case RMDIR:
        handleRmDir();
        break;
      case DELETE:
        handleDelete();
        break;
     case CDPINCHANGE_DETECTED:
        handleCDPinChangeDetected();
        break;
     default:
#ifdef DEBUG
        Serial.print(F("unknown command: ")); Serial.println(currentCommand);
#endif
        break;
    }
  } else {
#ifdef DEBUG
  Serial.print(F("No SD card inserted, cmd: "));Serial.println(currentCommand);
#endif
    switch (currentCommand) {
      case GETPARAMETER:
        handleGetParameter();
        break;
      case CDPINCHANGE_DETECTED:
        handleCDPinChangeDetected();
        break;
      case 254:
        outputQueue.push(75);
        status=OK;
#ifdef DEBUG
  Serial.print(F("Sent 75!"));
#endif
        break;
      default:
        status = STATUS_NO_SD;
        noInterrupts();
        inputQueue.clear();
        interrupts();
    }
  }
}