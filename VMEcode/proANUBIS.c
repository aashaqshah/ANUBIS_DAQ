#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "unistd.h"
#include <sys/time.h>
#include "CAENVMElib.h"
#include "CAENVMEtypes.h"
#include "CAENVMEoslib.h"
#include <stdarg.h>
#include <stdbool.h>
#include <time.h>
#include <iostream>
//#include <chrono>

#define ENABLE_LOG  1
#define store_data  0
#define valid_data_log  0
#define DATATYPE_FILLER  0xf8600001
#define not_valid_data 0xf8600000
#define EOB_mask 0xF8200000 //Also used to prevent some issue
#define valid_data_mask 0x100000 
#define header_mask 0x400000 
#define EOB_word 0xF8200001 //this is actual end of block word if there are only one hit in the event.
#define First_evnt_Header 0xF8400000
#define TWO_BITS_MASK 0x000000C0
#define tdc_err_mask 0x07200001
#define MAX_BLT_SIZE  (256*1024)
#define ulong  unsigned long
#define ushort unsigned short
#define llong long long
#define uchar  unsigned char

//===========
//Run Setting'

//No. of TDC's in VME
int nTDC = 1;

//V767 TDC's Base addresses
ulong base_addr[1] = {0xEE010000};
bool  Continuous = false; 	//default is trigger which is set to true
bool ethernet_bridge = false;
bool usb_bridge = true;
bool test = true;

//acquisation time in seconds
const int set_run_acq_time = 60; //time is in seconds

//Expected channel
int channel_no = 0;
//==========

//Some global variables
int byte_cnt[3]= {0,0,0};
uint32_t buffer[MAX_BLT_SIZE/4] ={0};
CVErrorCodes ret_blck;

//few additional things
const llong int mult_fact = 1000; //factor to converst time from sec. to millisecond
long int acq_time = set_run_acq_time * mult_fact; // converts time to sec.
int op_handshake;
int op_reg;
int op_reg_time_rest = 0; 

//Event related
int header_counter = 0;
int ch_data_counter = 0;
int EOB_counter = 0;
int additional_EOBs_counter = 0;
int extra_EOBs_counter = 0;     //This counter is something similar to additional_EOBs_counter but used at different place.

//Some call counters
int Wcall_numb = 0;
int Rcall_numb = 0;

//initialisation
int tdc = 0;
int SetFreq;

// handle per V767/767 
long handle; 
int32_t BHandle;

int VMEerror = 0;
char ErrorString[100];
FILE *logfile;
FILE *datafile;

//function prototypes
ushort read_reg(ushort reg_addr);

/*******************************************************************************/
/*            Helper: Hex to Binary  and data read                             */
/*******************************************************************************/
void HexToBin(uint32_t hexNumber) 
{ 

const char *hexDigitToBinary[16] = {"0000", "0001", "0010", "0011", "0100", "0101",
                                    "0110", "0111", "1000", "1001", "1010", "1011",
                                    "1100", "1101", "1110", "1111"};

const char hexDigits[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8',
      '9', 'A', 'B', 'C', 'D', 'E', 'F'};

//char *hexDigitToBinary[16] = {"0000", "0001", "0010", "0011", "0100", "0101",
//                                    "0110", "0111", "1000", "1001", "1010", "1011",
//                                    "1100", "1101", "1110", "1111"};



//char hexDigits[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8',
//      '9', 'A', 'B', 'C', 'D', 'E', 'F'};

    char hexadecimal[33] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    char binaryNumber[33]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};   
    int i = 0, j;  
    sprintf(hexadecimal, "%08X", hexNumber);


    /* 
     * Find he hexadecimal digit in hexDigits and the substitute it
     * with corresponding value in hexDigitToBinary
     */ 
    for(i=0; hexadecimal[i] != '\0'; i++)  {  
        for(j = 0; j < 16; j++){
            if(hexadecimal[i] == hexDigits[j]){
                strcat(binaryNumber, hexDigitToBinary[j]);

            }
        }
    }  


    		//printf("Info: Buffer: %08X; Byte Stream: %s;\n", hexNumber, binaryNumber); 
    		if (binaryNumber[9] == '1' && binaryNumber[10] == '1')
			{
			if (valid_data_log){
    				printf("Reading Buffer: %08X; Byte Stream: %s; Not a valid DATUM \n", hexNumber, binaryNumber);  
					}
			}
    		//if (binaryNumber[9] == '1' && binaryNumber[10] == '0')
    		else if (binaryNumber[9] == '1' && binaryNumber[10] == '0')
			{

			char subbuff[13];
			int sum=0;
			memcpy(subbuff, &binaryNumber[20], 12 );
			subbuff[12] = '\0';
			for(int s = strlen(subbuff); s > 0; s-- )
			    {
            			if(subbuff[s-1] == '1')
           			 {
          	  		  sum = sum + pow(2, 12-s);
            			}
    			    }
			int EvtN = sum;
    			printf("Reading Buffer: %08X; Byte Stream: %s; Header with Event No.: %d;\n", hexNumber, binaryNumber, EvtN); 
		        header_counter++;	
			if (store_data){
                		fprintf(datafile, "TDC: %d; Event No.: %d\n",tdc, EvtN);
				}
			}
    		//if (binaryNumber[9] == '0' && binaryNumber[10] == '0')
    		else if (binaryNumber[9] == '0' && binaryNumber[10] == '0')
			{
			if (binaryNumber[0] == '0' && binaryNumber[11] == '1'){

			double time_sum = 0.0;
			int ch_sum=0;
                        char timebuff[21], chbuff[8];
                        memcpy(timebuff, &binaryNumber[12], 20 );
                        memcpy(chbuff, &binaryNumber[1], 7 );
                        timebuff[20] = '\0';
                        chbuff[7] = '\0';
			//time info
                        for(int t = strlen(timebuff); t > 0; t-- )
                            {   
                                if(timebuff[t-1] == '1')
                                 {
                                  time_sum = time_sum + pow(2, 20-t)*0.8;
                                }
                            }
                        double timeMeasure = time_sum;

			//Channel info
                        for(int ch = strlen(chbuff); ch > 0; ch-- )
                            {
                                if(chbuff[ch-1] == '1')
                                //if(chbuff[ch] == '1')
                                 {
                                  ch_sum = ch_sum + pow(2, 7-ch);
                                }
                            }
                        int chN = ch_sum;
    			printf("Reading Buffer: %08X; Byte Stream: %s; Channel No.: %d; Time Measurement: %.2f ns;\n", hexNumber, binaryNumber,128*tdc+chN, timeMeasure); 
			ch_data_counter++;
			if (store_data){
                		fprintf(datafile, "TDC: %d; ChN: %d; TMeasure: %.2f\n", tdc, 128*tdc+chN, timeMeasure );
					}

			}
			
			}
    		else if ((binaryNumber[9] == '0' && binaryNumber[10] == '1') || (binaryNumber[0] == '1' && binaryNumber[1] == '1' && binaryNumber[2] == '1'  && binaryNumber[3] == '1'  && binaryNumber[4] == '1'   && binaryNumber[9] == '0' && binaryNumber[10] == '1'))
			{
			EOB_counter++;
			if (header_counter != EOB_counter) {	
					// printf("Byte Stream structure for 'unwanted EOB: %s\n", binaryNumber); 
    					//printf("Removing last additional (EOB) from counting \n"); 
					EOB_counter--;
					additional_EOBs_counter++;
				}
			else
    				printf("Reading Buffer: %08X; Byte Stream: %s; End of Block (EOB) \n", hexNumber, binaryNumber); 
				
					//Terminate run here if Headers != EOB's, it is temp. as of now
					//if ((header_counter != ch_data_counter) && (Continuous == false)   ) {
					if ((header_counter != EOB_counter) && (Continuous == false)   ) {
					if (header_counter > EOB_counter) header_counter--;
					if (header_counter < EOB_counter) EOB_counter--;	
					printf("Error: Garbage data - empty event or more than one hits per event detected, terminating run...\n\n");
					exit(1);
					}
			}
		else
			{
    			printf("Output Buffer: %08X, Byte Stream is : %s; Unknown data format\n", hexNumber, binaryNumber);  
			}
    //return binaryNumber;  
};
/*******************************************************************************/
/*                               READ_REG                                      */
/*******************************************************************************/
ushort read_reg(ushort reg_addr)
{
	ushort data=0;
	CVErrorCodes ret;
	ret = CAENVME_ReadCycle(handle, base_addr[tdc] + reg_addr, &data, cvA32_U_DATA, cvD16);
	if(ret != cvSuccess) {
		sprintf(ErrorString, "Cannot read at address %08X\n", (uint32_t)(base_addr[tdc] + reg_addr));
		VMEerror = 1;
        }
	if (ENABLE_LOG){
		fprintf(logfile, "Reading register at address %08X; data=%04X; ret=%d\n", (uint32_t)(base_addr[tdc] + reg_addr), data, (int)ret);
		;
		//return data;
		}
		return data;
}

/*******************************************************************************/
/*                                WRITE_REG                                    */
/*******************************************************************************/
void write_reg(ushort reg_addr, ushort data)
{
	CVErrorCodes ret;
	ret = CAENVME_WriteCycle(handle, base_addr[tdc] + reg_addr, &data, cvA32_U_DATA, cvD16);
	if(ret != cvSuccess) {
		sprintf(ErrorString, "Cannot write at address %08X\n", (uint32_t)(base_addr[tdc] + reg_addr));
		VMEerror = 1;
    }
	if (ENABLE_LOG){
		fprintf(logfile, "Writing register at address %08X; data=%04X; ret=%d\n", (uint32_t)(base_addr[tdc] + reg_addr), data, (int)ret);
		;
	}
}

/*******************************************************************************/
/*               function to check global tcd Status.                           */
/*******************************************************************************/
bool status_one;
bool tdc_status(ushort reg_addr)
 {
    status_one = false;
    ushort read_data;
    unsigned int global_data_ready_mask = 0x2;
    read_data = read_reg(reg_addr);
    sleep(1);
        if((read_data & global_data_ready_mask) != 0 ){
          printf("OK: TDCs are currently OK.\n");
          fprintf(logfile, "OK: TDCs are currently OK.\n");
          status_one = true;
        }
    return status_one;
 }
/*******************************************************************************/
/*               function to read Status1 reg.                           */
/*******************************************************************************/
ushort read_status1(ushort reg_addr)
 {
    status_one = false;	 
    ushort read_data;
    unsigned int data_ready_mask = 0x1;
    unsigned int global_data_ready_mask = 0x2;
    unsigned int module_busy_mask = 0x8;
    read_data = read_reg(reg_addr);
    sleep(1);
	if((read_data & global_data_ready_mask) != 0 ){
	  printf("OK: initial check complete.\n");
	  fprintf(logfile, "OK: initial check complete.\n");
	}
     
       if((read_data & module_busy_mask) != 0){		       
       		printf("Status: TDC module busy.\n");
       		fprintf(logfile, "Status: TDC module busy.\n");
	}

       while((read_data & module_busy_mask) != 0){
	       	write_reg(0x0054, 0);	
	       	sleep(3);
		read_data = read_reg(reg_addr);
       		sleep(1);
		if (read_data & global_data_ready_mask){
			printf("Status: atleast moudle has a data ready!\n");
			fprintf(logfile, "Status: atleast moudle has a data ready!\n");
		}
		if ( (read_data & data_ready_mask) == 0) {
			printf("Status: Output buffer has a data!\n");
			fprintf(logfile, "Status: Output buffer has a data!\n");
			break;
		}
       }
    return read_data;
 }

/*******************************************************************************/
/*               function to read Status2 reg.                           */
/*******************************************************************************/
 ushort status2_recovery(ushort reg_addr)
 {
    ushort read_data;
    int times = 0;
    unsigned int tdc_error_mask = 0x8;
    unsigned int chip3_error_mask = 0x8000;
    unsigned int chip2_error_mask = 0x4000;
    unsigned int chip1_error_mask = 0x2000;
    unsigned int chip0_error_mask = 0x1000;
    unsigned int empty_buffer_mask = 0x1;
    read_data = read_reg(reg_addr);
    sleep(1);
    printf("Check for next stage in progress.\n");
    fprintf(logfile, "Check for next stage in progress.\n");
    if ((read_data & tdc_error_mask) != 0) {
        	printf("Error: TDC module in error.\n");
        	fprintf(logfile, "Error: TDC module in error.\n");
		printf("Status: Reading register at address %08X; data= %X;\n", (uint32_t)(base_addr[0]+reg_addr), read_data);
		if((read_data & chip0_error_mask) != 0) printf("Spec. TDC Chip 0 is in error. \n");
		if((read_data & chip1_error_mask) != 0) printf("Spec. TDC Chip 1 is in error. \n");
		if((read_data & chip2_error_mask) != 0) printf("Spec. TDC Chip 2 is in error. \n");
		if((read_data & chip3_error_mask) != 0) printf("Spec. TDC Chip 3 is in error. \n");
		//try recovery during next stage
		printf("TDC recovery in progress.\n");
		fprintf(logfile, "STDC recovery in progress.\n");
      }
      while((read_data & tdc_error_mask) != 0){
	      //resetting VME/module
	      fprintf(logfile, "Performing module FE (single shot) reset ...\n");
	      write_reg(0x0018, 0);
	      sleep(3);
	      //fprintf(logfile, "Performing TDC clear...\n");
	      //write_reg(0x0054, 0);
	      //sleep(2);
	      read_data = read_reg(reg_addr);
	      sleep(1);
	      printf("Status: Still working...\n");
	      fprintf(logfile, "Status: Still working...\n");
	      if ((read_data & tdc_error_mask) == 0) {
		      printf("SUCCESS: TDC recovered successfully\n");
		      fprintf(logfile, "SUCCESS: TDC recovered successfully\n");
	      }
	      /*if ((read_data & empty_buffer_mask) != 0) {
		      printf("Empty buffer\n");
		      fprintf(logfile, "Empty buffer\n");
		      exit(1);
	      }*/
	      times++;
	      //check if it did not recover
	      if (( (read_data & tdc_error_mask) && (times <16)) != 0){
		      printf("FAIL: TDC recovery failed after %d attempts\n", times);
		      fprintf(logfile, "FAIL: TDC recovery failed after %d attempts\n", times);
	      		if (times == 15){
				printf("Terminating run.\n");
				exit(1);
			}
	      }
      }

    return read_data;
 }

/*******************************************************************************/
/*               function to read control reg.                           */
/*******************************************************************************/
 ushort read_control2(ushort reg_addr)
 {
    ushort read_data;
    unsigned int contains_event_mask = 0x3;
    read_data = read_reg(reg_addr);
    sleep(1);
    if ((read_data & contains_event_mask) == 0) {
                printf("Buffer has an event.\n");
      }
    else {
    	printf("Terminating run: Buffer doesn't seem to have an event.\n");
    	exit(1);
    }

    return read_data;
 }

/*******************************************************************************/
/*               function to read an opcode                            */
/*******************************************************************************/
 ushort read_opc(ushort handshake)
 {
    ushort read_data;
    int times = 0; 
    fprintf(logfile, "handshake read call %d - reading reg. address: 0x00%x\n",Rcall_numb, handshake);
    Rcall_numb++;
    do
    {
       read_data = read_reg(handshake);
       sleep(0.2);
       times++;
    //if (read_data == 0x01) break;
    if (read_data == 0x02) {
    	    fprintf(logfile, "OP Code ready for writting.\n");
    	    printf("OP Code ready for writting.\n");
    	    break;
    	    }
    }
    while((read_data!= 0x01) && (times < 150));
    if((read_data!= 0x01) && (times == 150))
    {
       fprintf(logfile, "Error: read handshake timeout!\n");
       printf("Error: read handshake timeout!\n");
       exit(1);
       //return -1;
    }
    return read_data;
 }
 /*******************************************************************************/
 /*             function to write an opcode                              */
 /*******************************************************************************/
 ushort write_opc(ushort op_reg_address, ushort command)
 { 
    fprintf(logfile, "handshake write call %d - reg. address: 0x00%x, command: 0x%x\n",Wcall_numb, op_reg_address, command);
    Wcall_numb++;
    ushort read_data, handshake;
    handshake = 0x50;
    int times = 0;
    do
    {  
       read_data = read_reg(handshake);
       sleep(0.2); 
       times++;
       //if (read_data == 0x02) break;
       if (read_data == 0x01) {
	       fprintf(logfile, "Error: OP Code can't be written.\n");
	       printf("Error: OP Code can't be written.\n");
	       //break;
       }
       else if (read_data == 0x02){
       		fprintf(logfile, "OP code ready to be written.\n");
       		break;
       } 
       else {
       		fprintf(logfile, "Still checking if OP code is readable.\n");
       }

    }
    while((read_data!= 0x02) && (times < 150));
    if((read_data != 0x02) && (times == 150))
    {  
       fprintf(logfile, "Error: write handshake timeout!\n");
       printf("Error: write handshake timeout!\n");
       //return -1;
       exit(1);
    }
    write_reg(op_reg_address, command);
    sleep(1);
    return read_data;
 }

int main(int argc,char *argv[])
{
	short device=0;
	//short usb_link=0; 
	void* usb_link = 0;
	const void*  ethernet_link = "10.11.31.10";

	CVErrorCodes	usb_init;
	CVErrorCodes	eth_init;
	
	//usb_init = CAENVME_Init(cvV1718, usb_link, device, &BHandle); //CAENVME_Init, works for old libraries
	std::cout << "Usb init.\n";

	//usb_init = CAENVME_Init2(cvV1718, usb_link, device, &BHandle);
	uint32_t link = 0;
	usb_init = CAENVME_Init2(cvV1718, &link,0, &BHandle);

	//eth_init = CAENVME_Init2(cvETH_V4718, const_cast<void*>(ethernet_link), 0, &BHandle);
	
	printf("START: attempting to open controller!\n");

	if (ethernet_bridge == true){
		if(eth_init == cvSuccess )
			{	
			 	printf("Success:: Connected via v4718 bridge!\n");
			}
		else {
	 		printf("\tError: Can't open ethernet bridge! \n");
	 		return 1;
			}
	} else if (usb_bridge == true) {
		 if (usb_init == cvSuccess )
		{
			printf("Success: Connected via v1718  bridge!\n");
		}

		else if(usb_init != cvSuccess )
			{
                	 printf("\tError: Can't open V1718 bridge \n");
			switch (usb_init)
				{
				case cvSuccess   : printf("\tCycle completed normally\n");							    
						   break ;
				case cvBusError	 : printf("\tBus Error !!! \n");
						   break ;				   
				case cvCommError : printf("\tCommunication Error !!!\n");
							   break ;
				default          : printf("\tUnknown Error !!!\n");
						   break ;
	        		}

                	 return 1;
			}
	}

	//Parse arguments	
	if (argc > 1) {
	    for (int j = 1; j < argc; j++) {
        	printf("Set Frequency: %s\n", argv[j]);
        	SetFreq = atoi(argv[j]);
   		 }
		} else {
    		printf("Error: Please provide at frequency as a command-line argument!\n");
    		return 0;
		}

	if (ENABLE_LOG) {
		printf("Log file is enabled\n");
		logfile = fopen("run_summary.log","w");
	}
	//Open a file to write data
	if (store_data) {
		datafile = fopen("V767_data","w");
	}

	// ------------------------------------------------------------------------------------
	// Setup loop
	// ------------------------------------------------------------------------------------
	try {
        	printf("Initializing (re-initializing)...\n");
        	fprintf(logfile, "Initializing (re-initializing)...\n");

        	// Load default config
        	write_opc(0x52, 0x1500);
		sleep(1);
       
		// Module FE reset
	       	fprintf(logfile, "Performing Module FE (single shot) reset.\n");	
        	write_reg(0x0018, 0);
        	sleep(2); //only 2 seconds needed.

		//Check status
		//read_status1(0x0E);
		//sleep(0.01);
		
		//TDC reset
	       	fprintf(logfile, "Performing TDC reset.\n");	
        	write_reg(0x0054, 0);
        	sleep(1);

	       	fprintf(logfile, "Checking status 1\n");	
        	//read_status1(0x000E);//Check TDC status 1
        	sleep(0.2);
        	
	       	fprintf(logfile, "Checking Status 2\n");	
		status2_recovery(0x0048); //Check TDC status 2
        	sleep(0.2);


        	// Print success message
        	printf("Initialization complete.\n");

    	} catch (...) {
                //std::exception_ptr ex = std::current_exception();
        	// Print error message and handle the exception
        	std::cerr << "Error in resetting.\n";
        	// Optionally exit the program if there's a critical error
         	exit(EXIT_FAILURE);
   	 }


	////Opcode handshake and register
	read_opc(0x50);
	sleep(op_reg_time_rest);
	
	tdc_status(0x000E);

      	/*load default config */
      	fprintf(logfile, "Loading default configuration - one step op.\n");
      	printf("Loading default configuration - one op.\n");
      	//read_opc(0x50);
      	//sleep(op_reg_time_rest);
      	write_opc(0x52, 0x1500);
      	sleep(op_reg_time_rest);	
	tdc_status(0x000E);
	//	/* disenable ch. 0 only */
	//	fprintf(logfile, "Disabling channel 0 - one step op.\n");
	//	printf("Diabling channel 0 - one step op.\n");
	//	read_opc(0x50);
	//	sleep(op_reg_time_rest);
	//	write_opc(0x52, 0x21);
	//	sleep(op_reg_time_rest);

	if (Continuous) {
		printf("Running in 'Continuous' mode!!\n");
		
		/*set Continuous mode*/
		write_opc(0x52, 0x1300);	
        	sleep(op_reg_time_rest);
		tdc_status(0x000E);

		read_opc(0x50);
		sleep(op_reg_time_rest);
               	tdc_status(0x000E);
		
		/*set D.R. = buffer almost full */
                write_opc(0x52, 0x7100);
                sleep(op_reg_time_rest);
		tdc_status(0x000E);
                
		/*set D.R. = buffer not empty  */
                //write_opc(0x52, 0x7200);
                //sleep(op_reg_time_rest);

	} else {
		
	        
		/* set Stop Trigger Matching mode */
	        printf("Setting 'stop trigger matching' mode - one step op.\n");	
	        fprintf(logfile, "Setting 'stop trigger matching' mode - one step op.\n");	
		read_opc(0x50);
		sleep(op_reg_time_rest);	
		write_opc(0x52, 0x1000);
		sleep(op_reg_time_rest);
		tdc_status(0x000E);
	
			
		////read acq. mode
	        //fprintf(logfile, "Sending write op. 0x1400 instruction to OP code register.\n");	
	        //printf("Sending write op. 0x1400 instruction to OP code register.\n");	
		//read_opc(0x50);
		//sleep(op_reg_time_rest);
		//write_opc(0x52, 0x1400);
		//sleep(op_reg_time_rest);
		
	        //fprintf(logfile, "Reading acquisation mode status.\n");	
	        //printf("Reading acquisation mode status.\n");	
		//read_opc(0x50);
		//sleep(op_reg_time_rest);
		//read_opc(0x1400);
		//sleep(op_reg_time_rest);

		
		//short win_offs, win_width;
		short win_offs, win_width;
		win_offs = -20;
		win_width = 40;
	
		/* set window width */
	        fprintf(logfile, "Setting window width - two steps op.\n");	
	        printf("Setting window width - two steps op.\n");	
		read_opc(0x50);
		sleep(op_reg_time_rest);
		write_opc(0x52, 0x3000);
		sleep(op_reg_time_rest);
		read_opc(0x50);
		sleep(op_reg_time_rest);
		write_reg(0x52, win_width);
		sleep(op_reg_time_rest);
		tdc_status(0x000E);
		
	//	/* read window width */
	//        fprintf(logfile, "Reading window width.\n");	
	//	read_opc(0x3100);
	//	sleep(1);
	
		/* set window offset */
	        fprintf(logfile, "Setting window offset - two steps op.\n");	
	        printf("Setting window offset - two steps op.\n");	
		read_opc(0x50);
		sleep(op_reg_time_rest);
		write_opc(0x52, 0x3200);
		sleep(op_reg_time_rest);
		read_opc(0x50);
		sleep(op_reg_time_rest);
		write_reg(0x52, win_offs);
		sleep(op_reg_time_rest);
		tdc_status(0x000E);

		/* read window offset */
	       	//fprintf(logfile, "Reading window offset.\n");	
	       	//printf("Reading window offset.\n");	
		//read_opc(0x50);
		//sleep(op_reg_time_rest);
		//write_opc(0x52, 0x3300);
		//sleep(op_reg_time_rest);
		//read_opc(0x3300);
		//sleep(op_reg_time_rest);


		/* set Trigger latency*/
		//write_opc(0x52, 0x3400);
		//write_reg(0x52, tr_latency);
		//sleep(0.1);
		
		/*set Data_Ready_Mode = Event_Ready */
		fprintf(logfile, "Setting data ready mode.\n");	
		printf("Setting data ready mode.\n");	
		read_opc(0x50);
		sleep(op_reg_time_rest);
		write_opc(0x52, 0x7000);
		sleep(op_reg_time_rest);
		tdc_status(0x000E);
	
	}

	      /* enable all channels */
		fprintf(logfile, "Enabling all channels - one step op.\n");
		printf("Enabling all channels - one step op.\n");
		read_opc(0x50);
		sleep(op_reg_time_rest);
		write_opc(0x52, 0x2300);
		sleep(op_reg_time_rest);
		tdc_status(0x000E);
			
	//	/* enable ch. 0 only */
	//	fprintf(logfile, "Enabling only one channel 0 - one step op.\n");
	//	printf("Enabling only one channel 0 - one step op.\n");
	//	read_opc(0x50);
	//	sleep(op_reg_time_rest);
	//	write_opc(0x52, 0x2000);
	//	sleep(op_reg_time_rest);
		
		/* disabling ch. 0 only */
		fprintf(logfile, "Disabling channel 0 - one step op.\n");
		printf("Diabling channel 0 - one step op.\n");
		read_opc(0x50);
		sleep(op_reg_time_rest);
		write_opc(0x52, 0x2100);
		sleep(op_reg_time_rest);
		tdc_status(0x000E);

	// ------------------------------------------------------------------------------------
	// Acquisition loop
	// ------------------------------------------------------------------------------------
	printf("Preparing for data acquisition from V767 modules\n");
	struct timeval t1, t2;
        double elapsedTime = 0;

	//double timpe_compansate = 2000.0; //This time (ms) after TDC reset is called

	while(tdc < nTDC) {
	
	//check TDC current status
	tdc_status(0x000E);
	 
	
	/*tdc clear*/
	printf("Clearing TDCs before data run.\n");
	fprintf(logfile, "Clearing TDCs before data run.\n");
	write_reg(0x0054, 0);
	sleep(1);
	
	//read_reg(0x004E); //Additionally clear the counter if needed
	
	//Check current status again 
	//tdc_status(0x000E);
	sleep(1);

	if (!status_one) {
		 printf("No Data Ready: terminating run.\n");
		 fprintf(logfile, "No Data Ready: terminating run.\n");
		 exit(1);
	 }

	
	//Check if buffer has an event
	if (Continuous == false) read_control2(0x004A);

	/*this part needs to be implemented...*/
	//do {
		//printf("Perfoming a small test Run...\n");
		//printf("Test completed successfully\n");
	//}

	
	printf("START: Beging data taking from TDC %d ...\n", tdc);
	fprintf(logfile, "START: Beging data taking from TDC %d ...\n", tdc);
	//time initilisation
	gettimeofday(&t1, NULL);
	do {
	//read_control2(0x004A);
	ret_blck = CAENVME_MBLTReadCycle(handle, base_addr[tdc], (uchar *)buffer, MAX_BLT_SIZE, cvA32_U_MBLT, &byte_cnt[tdc]);
	gettimeofday(&t2, NULL);
	//status2_recovery(0x0048); //status 2
	// compute elapsed time in millisec
        elapsedTime = (t2.tv_sec - t1.tv_sec)* mult_fact;     // Convert seconds to milliseconds or whatever value of mult_fact is 
        elapsedTime += (t2.tv_usec - t1.tv_usec)/mult_fact;   //Similarly, convert microseconds to milliseconds
	//printf("elapsed time:  %f\n", elapsedTime);	
	//sleep(0.01);
	if (ret_blck){
		if (ENABLE_LOG && (byte_cnt[tdc]>0)) {
		//if (byte_cnt[tdc]>0) {
				  //if (ENABLE_LOG) fprintf(logfile, " TDC: %d, Read Data Block from output buffer: size = %d bytes\n", tdc, byte_cnt[tdc]);
					for(int b=0; b<(byte_cnt[tdc]/4); b++){
					//if (ENABLE_LOG) fprintf(logfile, "TDC: %d; Reading output buffer: %08X\n", tdc, buffer[b]);
						//printf("EOB before, Byte stream %x:\n", buffer[b]);
						
						/*Some check here to see if the data makes sense before processing further*/
									
						//if ((buffer[b] & tdc_err_mask) ){
						if (((buffer[b] & tdc_err_mask) && (buffer[b] == EOB_mask)) != 0 ){
							HexToBin(buffer[b]);
							printf("Error: Garbage data - EOB Byte stream %x, terminating run.\n", buffer[b]);
							//exit(1);
						}
						
					//	if ((buffer[b] & TWO_BITS_MASK) == TWO_BITS_MASK){
					//	   	;
					//		//printf("Byte stream %x, not a Valid data.\n", buffer[b]);	
					//	}

						//check first event
						if ((buffer[0] == First_evnt_Header) && (buffer[1] == EOB_mask)){
							//printf("Error: First event with problem in EOB stream, terminating run.\n");
							printf("Error: First event as empty, terminating run.\n\n");
							exit(1);
						}

						//if ((buffer[b] == EOB_mask) && (b == 1)){
						//	HexToBin(buffer[b]);
						//	printf("Error: empty event, terminating run.\n");
						//	exit(1);
						//}
						if ((buffer[b] == EOB_word) && (buffer[b]==buffer[b-1]) ) {
							//printf("Info: Two back-to-back EOBs detected, ignoring last one\n");
							extra_EOBs_counter++;
							continue;
						}
						
						HexToBin(buffer[b]);
                                                }
				
					}
		}
	}
	while (elapsedTime < acq_time); 
	//while ((elapsedTime + timpe_compansate) < acq_time); 

	tdc++;
	} //end of tdc loop
	
	printf("\n\n========================\n");
	printf("Run Summary... \n");
	printf("========================\n");
	printf("Expected frequency: %d Hz.\n", SetFreq);
	printf("Acq. period: %.1f s.\n\n", elapsedTime/mult_fact);
	if (Continuous != true){
		printf("No. of Headers: %d \n", header_counter);
		printf("No. of EOB's: %d \n", EOB_counter);
		printf("Trig. rate (H's): %.2f Hz\n", (header_counter/elapsedTime* mult_fact));
		printf("Trig. rate (EoB's): %.2f Hz\n", (EOB_counter/elapsedTime* mult_fact));
	}
	printf("Ch. %d hits: %d \n", channel_no, ch_data_counter);
	//printf("Trig. rate (H's): %.2f Hz\n", (header_counter/(elapsedTime+timpe_compansate)* mult_fact));
	//printf("Trig. rate (EoB's): %.2f Hz\n", (EOB_counter/(elapsedTime+timpe_compansate)* mult_fact));
	//printf("====== Ch. hits ===========\n");
	//printf("Ch. %d hits: %d \n", channel_no, ch_data_counter);
	//printf("Ch. %d hit rate: %.2f Hz\n", channel_no, (ch_data_counter/(elapsedTime+timpe_compansate)* mult_fact));
	printf("Event rate, Ch. %d: %.2f Hz\n", channel_no, (ch_data_counter/elapsedTime* mult_fact));
	//printf("Warning: check additional EOB's: %d \n", additional_EOBs_counter);
	//printf("Warning: check extra Headers's: %d \n", extra_header_counter);
	printf("Warning: check extra EOB's: %d \n", extra_EOBs_counter);
	printf("========================\n");
	printf("End of Run\n");
	printf("========================\n");

	printf("Please wait: resetting VME\n");
        /*leave TDC in default config */
        //write_opc(0x52, 0x1500);
	//sleep(0.1);
	//leave VME (invoke VME reset) also in default state
	//write_reg(0x0018, 0xA);
        //sleep(2);
	//close handle
	CAENVME_End(handle);
       return 0;
}
