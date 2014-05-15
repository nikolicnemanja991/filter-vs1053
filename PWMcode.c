#define USE_SIMPLE_DSP_BOARD
//#define USE_PROTOTYPING_BOARD
// (define *one* of the PCB types before including board.h)
#include "board.h"

#define USE_MIC 0 // define 1 for mic-in, 0 for line-in

#define BLOCKSIZE 64 // amount of samples processed in one time
#define MY_SAMPLERATE 48000
#define DELAY_BUF_SZ 8192

#include <string.h>
#include <stdlib.h>
#include <vs1053.h>
#include <math.h>

void InitAudioExample(u_int16 srInput,int useMicIn,u_int16 coreClock);

main(void) {
	s_int16 auxBuffer[2*BLOCKSIZE]; // auxiliary audio buffer
	s_int16 __y lineInBuf[2*BLOCKSIZE]; // line-in audio buffer
	s_int16 __y *bufptr = delayBuffer;
	s_int16 temp[BLOCKSIZE];
	s_int16 temp1[BLOCKSIZE];
	s_int16 temp2[BLOCKSIZE];
	u_int16 mode=0;
	u_int16 first = 1;
	// initialize delay line with silence
	memsetY(delayBuffer, 0, DELAY_BUF_SZ);
	// disable interrupts during initialization
	Disable();
	// basic initialization phase
	InitAudioExample(MY_SAMPLERATE,USE_MIC,CORE_CLOCK_3X);
	// adjust output samplerate
	SetHardware(1/*stereo output*/,MY_SAMPLERATE/*DA sample rate*/);
	//
	USEX(INT_ENABLE)|=(1<<INT_EN_MODU)/*AD*/|(1<<INT_EN_DAC);
	// initialize audio_buffer read and write pointers
#ifdef VS1063
	audioPtr.rd = audio_buffer;
	audioPtr.wr = audio_buffer + 2*BLOCKSIZE;
#else
	audio_rd_pointer = audio_buffer;
	audio_wr_pointer = audio_buffer + 2*BLOCKSIZE;
#endif
	// clear audio buffer (avoid unwanted noise in the beginning)
	memsetY(audio_buffer,0,AUDIO_BUFFER_SZ);
	// set up GPIO
	CONFIGURE_LED_1;
	CONFIGURE_BUTTON_1;
	// set max volume
	USEX(SCI_VOL) = 0x0101;
	// enable interrupts
	Enable();	
//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Main loop
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
	while (1) {
		//
		// wait until there is enough audio data to process
		//
		if (StreamDiff() > BLOCKSIZE*2) {
			u_int16 scale;
			u_int16 i;
			__y s_int16 *lp = lineInBuf;
			s_int16 *sp = auxBuffer;
			s_int16 *te = temp;
			//
			// read input samples (stereo, hence 2 x block size)
			//
			StreamBufferReadData(lineInBuf, 2*BLOCKSIZE);
			//
			if(BUTTON_1_PRESSED && first==1) {
				mode=(mode+1)%8;
				first = 0;
			} else if(!BUTTON_1_PRESSED){
				SET_LED_1_ON;
				first = 1;
			} else{
				SET_LED_1_OFF;;
			}
			//
			// process BLOCKSIZE samples at one go
			//

			for(i=0;i<BLOCKSIZE;i++) {
				//retrieves input
				*te = *lp;

				//executes PWM algorthm if effect is on
				if(mode>=1){
				if(*te>=250)
					*te=1000*(i%2);
				else if (*te<=-250)
					*te=-1000*((i/mode)%3);
				else
					*te=0;
				}
				//advances buffers
				lp += 2;
				te++;
			}
			//
			// ouput sample pairs
			//
			te=temp;
			for(i=0; i<BLOCKSIZE; i++){
				*sp=*te;
				sp++;
				*sp=*te;
				sp++;
				te++;
			}
			
			AudioOutputSamples(auxBuffer, BLOCKSIZE);
			
		}
	}
	return 0;
}

