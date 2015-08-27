// Required libraries

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <signal.h>
#include <sys/select.h>


#define TIMEOUT 1000
#define PAUSE 100

static volatile int CTRL_C = 0;

#define PBUFSIZE        180 // size of buffer for sprintf

// sprintf buffer
char buf[PBUFSIZE];

#define ORDER_DEFAULT   0
#define ORDER_WARM      1
#define ORDER_COLD      2


// get a specific bit string from the frame's data as long and normalize
long getVal (uint8_t *frame, int startbit, int endbit, int offset, int multiplier, int divisor) {

  int startbyte = startbit >> 3;
  int startbitt = startbit & 0x07;
  int endbyte   = endbit   >> 3;
  int endbitt   = endbit   & 0x07;
  unsigned long result = 0;

  if (startbitt > 0) {
    result = frame[startbyte] & (0xff>>startbitt);
  } else {
    result = frame[startbyte];
  }

  startbyte++;
  while (startbyte <= endbyte) {
    result = (result << 8) | frame[startbyte];
    startbyte++;
  }

  if (endbitt < 7) {
    result = result >> (7 - endbitt);
  }

  result = result - offset;
  result *= multiplier;
  result /= divisor;
 
  return result;
}


void fputsNow (char *s, FILE *f) {
   fputs (s, f);
   fflush(f);
}


void initSerial (int USB) {

   struct termios tty;

   /* Set Baud Rate */
   cfsetospeed (&tty, (speed_t)B38400);
   cfsetispeed (&tty, (speed_t)B38400);

   /* Setting other Port Stuff */
   tty.c_cflag     &=  ~PARENB;            // Make 8n1
   tty.c_cflag     &=  ~CSTOPB;
   tty.c_cflag     &=  ~CSIZE;
   tty.c_cflag     |=  CS8;

   tty.c_cflag     &=  ~CRTSCTS;           // no flow control
   tty.c_cc[VMIN]   =  1;                  // read doesn't block
   tty.c_cc[VTIME]  =  100;                // 10 seconds read timeout, as we handle that through "select()"
   tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

   /* Make raw */
   cfmakeraw(&tty);

   /* Flush Port, then applies attributes */
   tcflush( USB, TCIFLUSH );
   if ( tcsetattr ( USB, TCSANOW, &tty ) != 0) {
       perror("tcsetattr");
   }

}

void sendSerial (int USB, char *p) {
   while (*p != 0) {
      write(USB, p, 1);
      p++;
   }
}

void flushSerial (int USB) {
   /* Flush Port (input) */
   // tcflush( USB, TCIFLUSH );
   int n;
   int b;
   n = serialReady (USB, 100);
   while (n > 0 & CTRL_C == 0) {
      n = read(USB, &b, 1);
      n = serialReady (USB, 100);
   }
}


int serialReady (int tty, int msecs ) {
   fd_set rfds;
   struct timeval tv;
   int rv;

   /* Watch tty to see when it has input. */
   FD_ZERO( &rfds );
   FD_SET(tty, &rfds);

   /* Wait up to msecs milliseconds. */
   tv.tv_sec = 0;
   tv.tv_usec = 1000 * msecs ;
   rv = select( tty+1, &rfds, NULL, NULL, &tv);
   if( rv < 0 ){
      return rv ;
   }
   return FD_ISSET( tty, &rfds );
}


void restoreOrder (int USB, int hard) {
   char buffer[32];
 
   sendSerialCommandElm (USB, "x",       2, buffer, sizeof buffer);
   fputsNow ("\nresponse to x:", stdout);
   fputsNow (buffer, stdout);

   usleep (1000 * 1000);
   flushSerial (USB);
   switch (hard) {
   case ORDER_DEFAULT:
      sendSerialCommandElm (USB, "atd",    2, buffer, sizeof buffer);
      fputsNow ("\nresponse to atd:", stdout);
      break;
   case ORDER_COLD:
      sendSerialCommandElm (USB, "atz",    2, buffer, sizeof buffer);
      fputsNow ("\nresponse to atz:", stdout);
      break;
   case ORDER_WARM:
      sendSerialCommandElm (USB, "atws",   1, buffer, sizeof buffer);
      fputsNow ("\nresponse to atws:", stdout);
      break;
   }
   fputsNow (buffer, stdout);
   if (*buffer == '\0') {
      fputsNow ("\nNo contact with ELM, sorry\n", stdout);
      exit (2);
   }

   if (hard == ORDER_COLD) {
      if (strstr(buffer, "v1.5") == NULL) {
         fputsNow ("\nwrong version of ELM, sorry\n", stdout);
         exit (1);
      }
   }

   if (!sendSerialCommandElm (USB, "ate0",   0, buffer, sizeof buffer)) {fputsNow ("\nerror command e0",  stdout);} // no echo
   if (!sendSerialCommandElm (USB, "ats0",   0, buffer, sizeof buffer)) {fputsNow ("\nerror command s0",  stdout);} // no spaces
   if (!sendSerialCommandElm (USB, "atsp6",  0, buffer, sizeof buffer)) {fputsNow ("\nerror command sp",  stdout);} // CAN 500K 11 bit
   if (!sendSerialCommandElm (USB, "atat1",  0, buffer, sizeof buffer)) {fputsNow ("\nerror command at",  stdout);} // auto timing
   if (!sendSerialCommandElm (USB, "atcaf0", 0, buffer, sizeof buffer)) {fputsNow ("\nerror command caf", stdout);} // no formatting

}

void getSerialCommandLine (int USB, char stopChar, int timeout, char *p, int siz) {
   memset(p, '\0', siz);
   char *q;
   char b;
   int cnt;
   int n;
   cnt = 0;
   q = p;
   n = serialReady (USB, timeout);
   while (n > 0 & CTRL_C == 0) {
      n = read(USB, &b, 1);
      if (b == stopChar) {
         *q++ = '\0';
         return;
      }

      switch (b) {
      case '\0':
      case '\r':
      case '\n':
      case '>':
        break;

      default:
         *q++ = b;
         if (cnt++ >= siz-1) {
            *q++ = '\0';
            return;
         }
      }
      n = serialReady (USB, timeout);
   }
   *q++ = '\0';
   return;
}


int sendSerialCommandElm (int USB, char *p, int retries, char *buffer, int siz) {
   sendSerial (USB, p);
   sendSerial (USB, "\r");
   getSerialCommandLine (USB, '>', TIMEOUT, buffer, siz);
   return (strstr(buffer, "OK") != NULL);
}


int sendSerialMonitor (int USB, int id, int timeout, uint8_t *buffer, int *siz) {
   char recvBuffer [32];
   char transmitBuffer [32];
   int i;

   snprintf (transmitBuffer, sizeof transmitBuffer, "atcra%3x", id);
   sendSerialCommandElm (USB, transmitBuffer, 0, recvBuffer, sizeof recvBuffer);
   usleep (400 * 1000);
   flushSerial(USB);
   sendSerial (USB, "atma\r");
   getSerialCommandLine (USB, 0x0d, timeout, recvBuffer, sizeof recvBuffer);
   for (i=7; i>=0; i--)
   {
      buffer[i] = (int)strtol(recvBuffer + (2*i), NULL, 16);
      recvBuffer[i*2] = '\0';
   }
   sendSerial (USB, " "); // stop ma
   getSerialCommandLine (USB, '>', TIMEOUT, recvBuffer, sizeof recvBuffer);
   if (!sendSerialCommandElm (USB, "atar", 0, recvBuffer, sizeof recvBuffer)) {
      fputsNow ("\nerror command ar (1)",  stdout); 
   }
}


int sendSerialCommandIsoTp  (int USB, int id, char *p, uint8_t *buffer, int *siz, int checkUDP) {
   char recvBuffer [32];
   char transmitBuffer [32];
   uint8_t byte[8];
   int i;
   int bufferPtr;
   int frameCounter;

   snprintf (transmitBuffer, sizeof transmitBuffer, "atsh%3x", id);
   sendSerialCommandElm (USB, transmitBuffer, 0, recvBuffer, sizeof recvBuffer);

   sendSerialCommandElm (USB, "atfcsd300010", 0, recvBuffer, sizeof recvBuffer); // reply frame to a FIRST (data)

   snprintf (transmitBuffer, sizeof transmitBuffer, "atfcsh%3x", id);
   sendSerialCommandElm (USB, transmitBuffer, 0, recvBuffer, sizeof recvBuffer); // reply frame to a FIRST (header). This ordering is crucial!

   sendSerialCommandElm (USB, "atfcsm1", 0, recvBuffer, sizeof recvBuffer); // set mode to manual flow control, header and data given

   snprintf (transmitBuffer, sizeof transmitBuffer, "0%1x%s\r", (int)(strlen(p)/2), p); // send the command in ISO-TP. Assume this is always a SINGLE frame
   sendSerial (USB, transmitBuffer);

   getSerialCommandLine (USB, 0x0d, TIMEOUT, recvBuffer, sizeof recvBuffer); // get the reply

   if (strncmp (recvBuffer, "NO DATA", 7) == 0) {
      fputsNow ("\nNO DATA", stdout);
      restoreOrder (USB, ORDER_DEFAULT);
      return 0;
   }
   if (strncmp (recvBuffer, "CAN ERROR", 7) == 0) {
      fputsNow ("\nCAN ERROR", stdout);
      restoreOrder (USB, ORDER_DEFAULT);
      return 0;
   }


   if (checkUDP) {
      for (i = 0; i < strlen(p); i++) {
         if (p[i] + (i == 0 ? 0x04 : 0) != recvBuffer[i+2]) {
            printf ("\n[%s]", p);
            printf ("\n[%s]", recvBuffer);
            fputsNow ("\nno valid return ", stdout);
            restoreOrder (USB, ORDER_DEFAULT);
            return 0;
         }
      }
   }

   for (i=7; i>=0; i--)
   {
      byte[i] = (int)strtol(recvBuffer + (2*i), NULL, 16);
      recvBuffer[i*2] = '\0';
   }

   switch (byte[0] >> 4) {

   case 0: //single
      *siz = (int) (byte[0] & 0x0f);
      i = 1;
      for (bufferPtr=0; bufferPtr < *siz && i <= 7; bufferPtr++)
         buffer[bufferPtr] = byte [i++];
      break;

   case 1: //first
      *siz = (int) (((byte[0] & 0x0f) * 256) + byte[1]);
      i = 2;
      for (bufferPtr=0; bufferPtr < *siz && i <= 7; bufferPtr++)
         buffer[bufferPtr] = byte [i++];
      frameCounter = 1;
      while (bufferPtr < *siz) { // as we set up the ELM to send ONE flow control byte in a response to a FIRST, the remainder should flow in immediately
         getSerialCommandLine (USB, 0x0d, TIMEOUT, recvBuffer, sizeof recvBuffer);// NEXT frames
         if (recvBuffer[0] != '2' || recvBuffer[1] != (((frameCounter & 0x0f) < 10 ? '0' : ('A'-10)) + (frameCounter & 0x0f))) {
            printf ("\n[%s]", p);
            printf ("\n[%s]", recvBuffer);
            fputsNow ("\nno valid or out of sequence return ", stdout);
            fputsNow (p, stdout);
            restoreOrder (USB, ORDER_DEFAULT);
            return 0;
         }
         frameCounter++;
         for (i=7; i>=0; i--)
         {
            byte[i] = (int)strtol(recvBuffer + (2*i), NULL, 16);
            recvBuffer[i*2] = '\0';
         }
         i = 1;
         for (; bufferPtr < *siz && i <= 7; bufferPtr++)
            buffer[bufferPtr] = byte [i++];
      }
      break;

   default: // so, we should never see a NEXT or a FLOW CONTROL frame
      fputsNow ("\nno valid frame ", stdout);
      fputsNow (p, stdout);
      restoreOrder (USB, 0);
      return 0;
      break;
   }
   getSerialCommandLine (USB, 0x0d, TIMEOUT, recvBuffer, sizeof recvBuffer); // this is needed
   flushSerial (USB);
   sendSerialCommandElm (USB, "atfcsm0", 0, recvBuffer, sizeof recvBuffer); // and set the back to automatic flow control
   return -1;
}


// wrapper so we can redirect to where we want.
void toConsole (char *buf) {
  fputs (buf, stdout);
  fflush(stdout);
}

void clearScreen(){
  snprintf(buf, PBUFSIZE, "%c[2J", 27);
  toConsole (buf);
}



void intHandler(int dummy) {
    CTRL_C = -1;
}

int main(int argc, char *argv[])

{
   uint8_t isoTpBuffer[512]; // in theory, an ISO-TP frame can be 4096 bytes
   int isoTpSize;
   int id;
   int val;
   int n;

   int USB = open(argv[1], O_RDWR | O_NOCTTY); // | O_NONBLOCK
   if (USB == -1)
   {
      /* Could not open the port. */
      fputsNow ("\nopen_port: Unable to open port arg1 - ", stdout);
      return;
   }

   initSerial (USB);
   restoreOrder (USB, ORDER_COLD);

   clearScreen();

    // signal(SIGINT, intHandler);

   while (CTRL_C == 0) {

      // Some erandom examples

      // ODO meter from the EVC

      isoTpSize = sizeof isoTpBuffer; 
      if (sendSerialCommandIsoTp (USB, 0x7e4, "222006", isoTpBuffer, &isoTpSize, -1)) {
         val = (isoTpBuffer [3] << 16) + (isoTpBuffer [4] << 8) + isoTpBuffer [5];
         snprintf(buf, PBUFSIZE, "%c[%d;%df ==> Odo:%d", 27, 1, 1, val);
         toConsole (buf);
      }
      usleep (PAUSE * 1000);

      // soc from the EVC

      isoTpSize = sizeof isoTpBuffer;
      if (sendSerialCommandIsoTp (USB, 0x7e4, "222002", isoTpBuffer, &isoTpSize, -1)) {
         val = (isoTpBuffer [3] << 8) + isoTpBuffer [4];
         val = val / 50;
         snprintf(buf, PBUFSIZE, "%c[%d;%df ==> Soc:%d%%", 27, 2, 1, val);
         toConsole (buf);
      }
      usleep (PAUSE * 1000);

      // compartment temperatures from the LBC

      isoTpSize = sizeof isoTpBuffer;
      if (sendSerialCommandIsoTp (USB, 0x79b, "2104", isoTpBuffer, &isoTpSize, 0)) {
         snprintf(buf, PBUFSIZE, "%c[%d;%df ==> Temperature:%d %d %d %d %d %d %d %d %d %d %d %d C", 27, 3, 1, isoTpBuffer[4]-40, isoTpBuffer[7]-40, isoTpBuffer[10]-40, isoTpBuffer[13]-40, isoTpBuffer[16]-40, isoTpBuffer[19]-40, isoTpBuffer[22]-40, isoTpBuffer[25]-40, isoTpBuffer[28]-40, isoTpBuffer[31]-40, isoTpBuffer[34]-40, isoTpBuffer[37]-40);
         toConsole (buf);
      }
      usleep (PAUSE * 1000);

      // Max regen from the LBC

      isoTpSize = sizeof isoTpBuffer;
      if (sendSerialCommandIsoTp (USB, 0x79b, "2101", isoTpBuffer, &isoTpSize, 0)) {
         //val = (isoTpBuffer [24] << 8) + isoTpBuffer [25]; // fluence
         val = (isoTpBuffer [42] << 8) + isoTpBuffer [43]; // zoe?
         snprintf(buf, PBUFSIZE, "%c[%d;%df ==> Max ch:%d0 W", 27, 4, 1, val);
         toConsole (buf);
      }
      usleep (PAUSE * 1000);

      // Cell voltages block 1

      isoTpSize = sizeof isoTpBuffer;
      if (sendSerialCommandIsoTp (USB, 0x79b, "2141", isoTpBuffer, &isoTpSize, 0)) {
         snprintf(buf, PBUFSIZE, "%c[%d;%df ==> Cell voltages", 27, 5, 1);
         toConsole (buf);        
         n = 0;
         for (id = 2; id < isoTpSize; id = id + 2) {
            val = (isoTpBuffer [id] << 8) + isoTpBuffer [id+1];
            snprintf(buf, PBUFSIZE, "%c[%d;%df%d", 27, 6 + (n/12), 1 + (n%12) * 5, val);
            toConsole (buf);
            n++;
         }
         isoTpSize = sizeof isoTpBuffer;
         if (sendSerialCommandIsoTp (USB, 0x79b, "2142", isoTpBuffer, &isoTpSize, 0)) {
            for (id = 2; id < isoTpSize - 4; id = id + 2) {
               val = (isoTpBuffer [id] << 8) + isoTpBuffer [id+1];
               snprintf(buf, PBUFSIZE, "%c[%d;%df%d", 27, 6 + (n/12), 1 + (n%12) * 5, val);
               toConsole (buf);
               n++;
            }
         }
      }
      usleep (PAUSE * 1000);

      // PEB, Inverter temperature

      isoTpSize = sizeof isoTpBuffer;
      if (sendSerialCommandIsoTp (USB, 0x75a, "22302B", isoTpBuffer, &isoTpSize, -1)) {
         val = (isoTpBuffer [3] << 8) + isoTpBuffer [4];
         val = val * 100;
         val = val / 64;
         snprintf(buf, PBUFSIZE, "%c[%d;%df ==> PED inv temp:%d", 27, 14, 1, val);
         toConsole (buf);
      }
      usleep (PAUSE * 1000);

      // ABS

      isoTpSize = sizeof isoTpBuffer;
      if (sendSerialCommandIsoTp (USB, 0x740, "224B0E", isoTpBuffer, &isoTpSize, -1)) {
         val = isoTpBuffer [3];
         snprintf(buf, PBUFSIZE, "%c[%d;%df ==> Master cyl pres:%d bar", 27, 15, 1, val);
         toConsole (buf);
      }
      usleep (PAUSE * 1000);

      // serial monitor to some charging data

      sendSerialMonitor (USB, 0x654, 10*TIMEOUT, isoTpBuffer, &isoTpSize);
      val = (isoTpBuffer [4] << 2) + (isoTpBuffer [5]>>6);
      snprintf(buf, PBUFSIZE, "%c[%d;%df ==> Time to EOC:%d min", 27, 16, 1, val);
      toConsole (buf);
      usleep (PAUSE * 1000);

      sendSerialMonitor (USB, 0x42e, 10*TIMEOUT, isoTpBuffer, &isoTpSize);
      snprintf(buf, PBUFSIZE, "%c[%d;%df ==> 42e raw:", 27, 17, 1);
      toConsole (buf);
      for (id = 0; id < 8; id++) {
         snprintf(buf, PBUFSIZE, "%02X ", isoTpBuffer [id]);
         toConsole (buf);
      }
      val = (isoTpBuffer [5] >>4);
      snprintf(buf, PBUFSIZE, "%c[%d;%df ==> AC %cph", 27, 18, 1, val == 0x0f ? '3' : '1');
      toConsole (buf);
      usleep (PAUSE * 1000);

   }

}



