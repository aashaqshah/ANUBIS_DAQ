#include "stdio.h" 
#include <vector>
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "unistd.h"
#include "tdcConstants.h"
#include <sys/time.h>
#include "CAENVMElib.h"
#include "CAENVMEtypes.h"
#include "CAENVMEoslib.h"
#define ulong  unsigned long
#define ushort unsigned short
#define uchar  unsigned char
#include <stdarg.h>
#define MAX_BLT_SIZE  (256*1024)
#include <stdbool.h>
#include <time.h>
#include <iostream>
#include <thread>
#include <chrono>
using std::this_thread::sleep_for;

#define ENABLE_LOG  1
#define store_data  1
#define valid_data_log  0

//number of TDCs
int nTDCs = 5; 
int tdc = 0;

using namespace TDCconstants;

//TDC V767 Base addresses
ulong base_addr[5] = {0xEE000000, 0xEE010000, 0xEE020000, 0xEE030000, 0xEE040000};
//ulong base_addr[4] = {0xEE010000, 0xEE020000, 0xEE030000, 0xEE040000};

std::vector<std::vector<int>> badChannels = {{0,32,95,127},{0,64,96},{32,64},{0,31,32,127},{0}};

//ulong base_addr[3] = {0xEE040000, 0xEE030000, 0xEE020000};
//ulong base_addr[1] = {0xEE000000};

// handle per V767 
//ulong handle; 

ulong handle;
int32_t BHandle;

int VMEerror = 0;
char ErrorString[100];
FILE *logfile;
FILE *datafile;

/*******************************************************************************/
/*            Helper: Hex to Binary  and data read                             */
/*******************************************************************************/


bool HexToBin(uint32_t hexNumber)
{
    ulong EOBmask = 0x200000;
    ulong headerMask = 0x400000;

	if ( (hexNumber&headerMask) & (hexNumber&EOBmask))
		{
		return false;	
		if (valid_data_log){
			printf("Reading Buffer: %08X; Not a valid DATUM \n", hexNumber);
		}
	}
	else if (hexNumber&headerMask)
	{
        	uint32_t EvtN = hexNumber&0xfff;
		printf("TDC: %i, Reading Buffer: %08X; Header with Event No.: %d;\n", tdc, hexNumber, EvtN);
		if (store_data){
			fprintf(datafile, "TDC: %d; Event No.: %d\n",tdc, EvtN);
			}
	}
	else if (!(hexNumber&EOBmask))
	{
        	uint32_t time_meas = hexNumber&0xfffff;
        	uint32_t channel = (hexNumber>>24)&0x7f;
        	printf("Data word: %08X. Time meas. is %d, Channel is %d.\n", hexNumber, time_meas, channel);
	}
	else
	{
		printf("Reading Buffer: %08X; End of Block (EOB) \n", hexNumber);
	}
	return true;
};

/*******************************************************************************/
/*                               READ_REG                                      */
/*******************************************************************************/
ulong read_reg(ushort reg_addr, double dSize = 16)
{
	ulong data=0;
	CVErrorCodes ret;
    if(dSize==16){
	   ret = CAENVME_ReadCycle(handle, base_addr[tdc] + reg_addr, &data, cvA32_U_DATA, cvD16);
    }
    else{
	   ret = CAENVME_ReadCycle(handle, base_addr[tdc] + reg_addr, &data, cvA32_U_DATA, cvD32);
    }
	if(ret != cvSuccess) {
		sprintf(ErrorString, "Cannot read at address %08X\n", (uint32_t)(base_addr[tdc] + reg_addr));
		VMEerror = 1;
	}
	return(data);
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
	if (ENABLE_LOG)
		fprintf(logfile, " Writing register at address %08X; data=%04X; ret=%d\n", (uint32_t)(base_addr[tdc] + reg_addr), data, (int)ret);
}

/*******************************************************************************/
/*                                WRITE_OPCODE                                    */
/*******************************************************************************/
int write_opcode(ushort code)
{
   ushort rdata;
   int attempts=0;
   do
   {
      rdata=read_reg(op_handshake);
      attempts++;
      sleep_for(std::chrono::milliseconds(10));
   }
   while((rdata!= 0x02)&&(attempts<50));
   if(attempts>50)
   {
      printf("Handshake timeout!\n");
      return -1;
   }
   sleep_for(std::chrono::milliseconds(10));
   write_reg(op_reg, code);
   return 0;
}

/*******************************************************************************/
/*                                READ_OPCODE                                    */
/*******************************************************************************/
int read_opcode()
{
   ushort rdata;
   int attempts=0;
   do
   {
      rdata=read_reg(op_handshake);
      attempts++;
      sleep_for(std::chrono::milliseconds(100));
   }
   while((rdata!= 0x01)&&(attempts<50));
   if(attempts>50)
   {
      printf("Handshake timeout!\n");
      return -1;
   }
   sleep_for(std::chrono::milliseconds(10));
   rdata = read_reg(op_reg);
   return rdata;
}
/*******************************************************************************/
/*                                Interpret_TDCError                                */
/*******************************************************************************/
bool interpret_tdcError(ushort chip)
{  
   write_opcode(0x8000+chip);
   ushort errStat = read_opcode();
   return errStat;
}

/*******************************************************************************/
/*                                READ_TDCError                                */
/*******************************************************************************/
bool read_tdcError()
{
   ushort status = read_reg(stat_reg_2);
   ushort tdcErrorMask = 0x8;
   ushort chip3mask = 0x8000;
   ushort chip2mask = 0x4000;
   ushort chip1mask = 0x2000;
   ushort chip0mask = 0x1000;
   bool tdcError = false;
   if(status&tdcErrorMask){tdcError = true;}
   if(status&chip0mask)
   {
      tdcError = true;
      printf("TDC Chip A is in error, has %08X.\n", interpret_tdcError(0));
   }
   if(status&chip1mask)
   {
      tdcError = true;
      printf("TDC Chip B is in error, has %08X.\n", interpret_tdcError(1));

   }
   if(status&chip2mask)
   {
      tdcError = true;
      printf("TDC Chip C is in error, has %08X.\n", interpret_tdcError(2));
   }
   if(status&chip3mask)
   {
      tdcError = true;
      printf("TDC Chip D is in error, has %08X.\n", interpret_tdcError(3));
   }
   if(!tdcError){printf("No TDC Chips in Error.\n");}
   return tdcError;
}

/******************************************************************************/
/*                                   MAIN                                     */
/******************************************************************************/
int main(int argc,char *argv[])
{
	printf("Triggered: %s | Duration: %s", argv[1], argv[2]);

 	//ushort manufact_id;
	printf("Log file = %d\n", ENABLE_LOG);
	if (ENABLE_LOG) {
		printf("Log file is enabled\n");
		logfile = fopen("V767_log","w");
	}
	//Open a file to write data
	if (store_data) {
		//printf("Opening a file for data writing...\n");
		datafile = fopen("V767_data","w");
    }

    //const void* ethernet_link = "10.11.31.10";
    uint32_t usb_link = 0;
    //if(CAENVME_Init(cvV1718, 0, 0, &BHandle) == cvSuccess)
    if(CAENVME_Init2(cvV1718, &usb_link, 0, &BHandle)==cvSuccess)
    {
        printf("Connected to the device.\n");
    }
    else{
		printf("\tError in opening V1718 bridge \n");
		return 1;
	}

    

	// Read FW revision
    //ushort manufact_id = read_reg(0x102E);
    if (VMEerror) {
	printf(ErrorString);
		//getchar();
	CAENVME_End(handle);
        return 1;
    }
    ushort manId_0  = read_reg(man_id_0);
    ushort manId_1  = read_reg(man_id_1);
    ushort manId_2  = read_reg(man_id_2);
    ushort boardId_0  = read_reg(board_id_0);
    ushort boardId_1  = read_reg(board_id_1);
    ushort boardId_2  = read_reg(board_id_2);
    ushort boardId_3  = read_reg(board_id_3);

    printf("Manufacturer's ID = %02X,%02X,%02X\n", manId_0,manId_1,manId_2);
    printf("Board ID = %02X,%02X,%02X,%02X\n", boardId_0,boardId_1,boardId_2,boardId_3);
    //read_tdcError();

	// ------------------------------------------------------------------------------------
	// Acquisition loop
	// ------------------------------------------------------------------------------------
	printf("Starting Acquisition from V767 modules\n");
	//write_reg(soft_trig, 0x0011); //TDC reset
	short win_offs=-10;
	short win_width=50;  
    for(int i=0;i<nTDCs;i++){
        printf("Setting up TDC %i.\n",tdc);
        bool tdcError = read_tdcError();
	    do {
            write_reg(vme_reset,0x0011);
            sleep(2);   
            write_opcode(0x1500); //Load default config
            write_opcode(0x2400);//Disable all channels
		    if (atol(argv[1])==0){
		    	write_opcode(0x1300);//sets Continuous mode
		    }
		    else{
		    	write_opcode(0x1000);//sets StopTriggerMatching mode
		    }
		    write_opcode(0x3000);//sets window width
		    write_opcode(win_width);
//            write_opcode(0x2300);//enable all channels
		    write_opcode(0x3200);//sets window offset
		    write_opcode(win_offs);
		    write_opcode(0x7200);//sets Data_Ready_Mode=Buffer not empty
            for(uint16_t chan=0; chan<128; chan++) //Manually re-enable the 'good' channels
            {
                bool good=true;
                for(uint16_t badChan=0; badChan<badChannels[tdc].size(); badChan++)
                {
                    if(badChannels[tdc][badChan]==chan){good=false;}
                }
                if(good){write_opcode(0x2000+chan);}
            }

            sleep(1);
            tdcError = read_tdcError();
        }while(tdcError);
        tdc++;
    }
    tdc=0;
    for(int board=0; board<nTDCs; board++)
    {
	    write_reg(tdc_clear, 0x0011);
        tdc++;
    }
    tdc=0;
    for(int board=0; board<nTDCs; board++)
    {
	    bool boolErrCheck = read_tdcError();
        if(boolErrCheck){printf("TDC %i is in Error!.\n",tdc);}
        else{printf("TDC %i is OK.\n",tdc);}
        tdc++;
    }
    tdc=0;

    //wait for input
    //int var;
    //printf("Waiting for input.\n");
    //std::cin >> var;
	time_t endwait;
	time_t start = time(NULL);
	time_t seconds = atol(argv[2]); //end loop after this elapsed time
	endwait = start+seconds;

	do{
       ushort rstatus;
	   bool dataReady;
       do{
           rstatus = read_reg(stat_reg_1);
       }
       while(!(rstatus&0x01)&(time(NULL)<endwait));
       do{
           if(time(NULL)<endwait){
              HexToBin(read_reg(out_buffer, 32));
              dataReady = read_reg(stat_reg_1)&0x01;
           }
	   }
	   while(dataReady&(time(NULL)<endwait));
       tdc++;
       if(tdc>=nTDCs){tdc=0;}
	}
    while(time(NULL)<endwait);
	time_t endTime = time(NULL);
	printf("End time is %s", ctime(&endTime));
    tdc = 0;
	for(int chip=0; chip<nTDCs; chip++)
    {
        printf("Checking TDC %i status.\n",tdc);
        read_tdcError();
        tdc++;
    }

	CAENVME_End(handle);
    return 0;
}

