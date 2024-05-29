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
#define MAX_BLT_SIZE  (4*1024)
#include <stdbool.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <ctime>
#include <sstream>
#include <filesystem>
using std::this_thread::sleep_for;

#define ENABLE_LOG  1
#define valid_data_log  0

//number of TDCs
int nTDCs = 6; 
int tdc = 0;

using namespace TDCconstants;

//TDC V767 Base addresses
ulong base_addr[6] = {0xEE000000, 0xEE010000, 0xEE020000, 0xEE030000, 0xEE040000, 0xEE050000};

//std::vector<std::vector<int>> badChannels = {{0,32,95,127},{64,96},{32,64},{0,31,32,127},{0}};

//ulong base_addr[3] = {0xEE040000, 0xEE030000, 0xEE020000};
//ulong base_addr[1] = {0xEE000000};

// handle per V767 
//ulong handle; 

ulong handle;
int32_t BHandle;

int VMEerror = 0;
char ErrorString[100];
FILE *logfile;

//Hacky way to stop the program after "deamonizing" it. Reads a cfg file which should only contain a zero or one, stops if it doesn't see a one.
bool checkRunFlag()
{
    std::ifstream flagFile;
    flagFile.open("cfg/runFlag.txt");
    int state = 0; 
    flagFile >> state;
    if(state==1){return true;}
    return false;
}

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
        if(hexNumber&EOBmask){return true;}
        uint32_t EvtN = hexNumber&0xfff;
		printf("TDC: %i, Reading Buffer: %08X; Header with Event No.: %d;\n", tdc, hexNumber, EvtN);
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
/*                               READ_BLK                                      */
/*******************************************************************************/
int read_blk(uint32_t* buffer)
{
    int byte_cnt = 0;
    CVErrorCodes ret_blck = CAENVME_BLTReadCycle(handle, base_addr[tdc], (uchar *)buffer, MAX_BLT_SIZE, cvA32_U_MBLT, cvD64, &byte_cnt);
	if(ret_blck != cvSuccess) {
		sprintf(ErrorString, "Block transfer failed for TDC %i.", tdc);
		VMEerror = 1;
	}
	return byte_cnt/4;
}

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
   write_opcode(0x8000+(chip>>2));
   ushort errStat = read_opcode();
   return errStat;
}

//Get the current time
std::pair<uint32_t,uint32_t> getTime()
{
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch());
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(nanoseconds);
    nanoseconds-=seconds;
    std::pair<uint32_t,uint32_t> outTime;
    outTime.first = seconds.count();
    outTime.second = nanoseconds.count();
    return outTime;
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
      printf("TDC Chip A is in error.\n");
   }
   if(status&chip1mask)
   {
      tdcError = true;
      printf("TDC Chip B is in error.\n");
   }
   if(status&chip2mask)
   {
      tdcError = true;
      printf("TDC Chip C is in error.\n");
   }
   if(status&chip3mask)
   {
      tdcError = true;
      printf("TDC Chip D is in error.\n");
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

	printf("Log file = %d\n", ENABLE_LOG);
	if (ENABLE_LOG) {
		printf("Log file is enabled\n");
		logfile = fopen("V767_log","w");
	}

    uint32_t usb_link = 0;
    if(CAENVME_Init2(cvV1718, &usb_link, 0, &BHandle)==cvSuccess)
    {
        printf("Connected to the device.\n");
    }
    else{
		printf("\tError in opening V1718 bridge \n");
		return 1;
	}

	// Read FW revision
    if (VMEerror) {
	printf(ErrorString);
	CAENVME_End(handle);
        return 1;
    }

	// ------------------------------------------------------------------------------------
	// Acquisition loop
	// ------------------------------------------------------------------------------------
	printf("Starting Acquisition from V767 modules\n");
	short win_offs=-10;
	short win_width=50;  

    for(;tdc<nTDCs;tdc++){
        printf("Setting up TDC %i.\n",tdc);
        bool tdcError;
	    do {
            if(tdc<5) //Set up TDCs in triggered mode
            {
                write_reg(vme_reset,0x0011);
                sleep(2);   
                write_opcode(0x1500); //Load default config
                write_opcode(0xB800); //Set mask for evnt overflow
                write_opcode(0xB);
                write_opcode(0x2400);//Disable all channels
		        write_opcode(0x1000);//sets StopTriggerMatching mode
		        write_opcode(0x3000);//sets window width
		        write_opcode(win_width);
		        write_opcode(0x3200);//sets window offset
		        write_opcode(win_offs);
		        write_opcode(0x7000);//sets Data_Ready_Mode=Event Ready
                write_opcode(0x2300);//Enable all channels
            }
            else //Set up TDC 6 in continuous readout.
            {
                write_reg(vme_reset,0x0011);
                sleep(2);

                write_opcode(0x2100);
                //write_opcode(0x2102);
                write_opcode(0x1300);//Set continuous mode
		        write_opcode(0x7200);//sets Data_Ready_Mode=Buffer not empty
                //write_opcode(0x1600); //Set this as the default mode
                //write_opcode(0x1800); //Enable auto-load
                //write_reg(cont_reg_1, 0x2B); //Need to enable BERR_VME for block transfers
            }
            sleep(1);
            tdcError = read_tdcError();
        }while(tdcError);
    }
    
    std::pair<uint32_t,uint32_t> thisTime;
    uint32_t thisWord;
    uint32_t fillWord;
    uint32_t buffer[MAX_BLT_SIZE] = {0};
    std::vector<uint32_t> dataBuffer;

    while(true){
        //Save the chrono start time
        thisTime = getTime();
      
    	time_t endwait;
    	time_t start = time(NULL);
    	time_t seconds = atol(argv[2]); //end loop after this elapsed time
        time_t shortwaittime = 1800;
        time_t shortwait = start+shortwaittime;
    	endwait = start+seconds;
        //enum states {Waiting, Ready, Invalid};
        //states state=Waiting;
        
        std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::ostringstream oss;
        oss << std::put_time(std::localtime(&now), "%y%m%d_%H%M");
        std::ostringstream date;
        date << std::put_time(std::localtime(&now), "%y_%m");
        std::ostringstream day;
        day << std::put_time(std::localtime(&now), "%y_%m_%d");
        std::string ofName = "data/"+date.str()+"/"+day.str()+"/proAnubis_"+oss.str()+".raw";
        std::filesystem::create_directory("data/"+date.str());
        std::filesystem::create_directory("data/"+date.str()+"/"+day.str());
 
        std::ofstream thisFile;
        thisFile.open(ofName.c_str(), std::ios::binary | std::ios::out);
        //Write the start time into the TFile
        thisFile.write(reinterpret_cast< char* >(&thisTime.first),sizeof(uint32_t)); //(char*) &a, sizeof(a));
        thisFile.write(reinterpret_cast< char* >(&thisTime.second),sizeof(uint32_t));
        std::ofstream monFile;
        monFile.open("tempMonFile.raw", std::ios::binary | std::ios::out);
        monFile.write(reinterpret_cast< char* >(&thisTime.first),sizeof(uint32_t)); //(char*) &a, sizeof(a));
        monFile.write(reinterpret_cast< char* >(&thisTime.second),sizeof(uint32_t));

        ulong EOBMask = 0x200000;
        ulong headerMask = 0x400000;

        tdc=0;
        for(;tdc<nTDCs;tdc++)
        {
    	    write_reg(tdc_clear, 0x0011);
        }
        tdc=0;

    	do{
           int wordCount = read_blk(buffer);
           thisTime = getTime();
           bool lastWasBad = false;
           for(int b=0; b<wordCount; b++)
           {
              thisWord = buffer[b];
              if(thisWord&headerMask){
                 if(thisWord&EOBMask)
                 {
                    if(lastWasBad){break;}
                    else
                    {
                        lastWasBad=true;
                        fillWord = thisWord;
                    }
                 }
                 else{dataBuffer.push_back(thisWord);}
              }
              //HexToBin(thisWord);
              else
              { 
                if(lastWasBad)
                {
                    lastWasBad=false;
                    dataBuffer.push_back(fillWord);
                }
                dataBuffer.push_back(thisWord);
              }
           }
           if(dataBuffer.size()>0)
           {
              thisFile.write(reinterpret_cast< char* >(&thisTime.first),sizeof(uint32_t)); //(char*) &a, sizeof(a));
              thisFile.write(reinterpret_cast< char* >(&thisTime.second),sizeof(uint32_t));
              uint32_t headerWord = (tdc<<24) + dataBuffer.size();
              thisFile.write((char*)&headerWord,sizeof(uint32_t));
              thisFile.write((char*)&dataBuffer[0], dataBuffer.size() * sizeof(uint32_t));
              monFile.write(reinterpret_cast< char* >(&thisTime.first),sizeof(uint32_t)); //(char*) &a, sizeof(a));
              monFile.write(reinterpret_cast< char* >(&thisTime.second),sizeof(uint32_t));
              monFile.write((char*)&headerWord,sizeof(uint32_t));
              monFile.write((char*)&dataBuffer[0], dataBuffer.size() * sizeof(uint32_t));
              dataBuffer.clear();
           }
           
           tdc++;
           if(tdc>=nTDCs)
           {
              tdc=0;
              if(!checkRunFlag()){
                  thisTime = getTime();
                  thisFile.write(reinterpret_cast< char* >(&thisTime.first),sizeof(uint32_t)); //(char*) &a, sizeof(a));
                  thisFile.write(reinterpret_cast< char* >(&thisTime.second),sizeof(uint32_t));
                  thisFile.close();
                  CAENVME_End(handle);
                  return 0;
              }    
              if(time(NULL)>shortwait)
              {
                 thisTime = getTime();
                 monFile.write(reinterpret_cast< char* >(&thisTime.first),sizeof(uint32_t)); //(char*) &a, sizeof(a));
                 monFile.write(reinterpret_cast< char* >(&thisTime.second),sizeof(uint32_t));
                 monFile.close();
                 std::rename("tempMonFile.raw", "data/monitor/proANUBISmonitor.raw");
                 monFile.open("tempMonFile.raw", std::ios::binary | std::ios::out);
                 monFile.write(reinterpret_cast< char* >(&thisTime.first),sizeof(uint32_t)); //(char*) &a, sizeof(a));
                 monFile.write(reinterpret_cast< char* >(&thisTime.second),sizeof(uint32_t));

                 shortwait = shortwait+shortwaittime;
              }
           }
    	}
        while(time(NULL)<endwait);
        thisTime=getTime();
        thisFile.write(reinterpret_cast< char* >(&thisTime.first),sizeof(uint32_t)); //(char*) &a, sizeof(a));
        thisFile.write(reinterpret_cast< char* >(&thisTime.second),sizeof(uint32_t));
        thisFile.close();
    }
	CAENVME_End(handle);
    return 0;
}

