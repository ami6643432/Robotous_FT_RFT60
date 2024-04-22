/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>

#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"

#define EC_TIMEOUTMON 500

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
volatile int rtcnt;
boolean inOP;
uint8 currentgroup = 0;

// FOR RFTEC01-R4 & RFT SENSOR INTERFACE
// command types
#define CMD_NONE					(0)
#define CMD_GET_PRODUCT_NAME		(1)
#define CMD_GET_SERIAL_NUMBER		(2)
#define CMD_GET_FIRMWARE_VER		(3)
#define CMD_SET_ID					(4)	// Only CAN version
#define CMD_GET_ID					(5) // Only CAN version
#define CMD_SET_COMM_BAUDRATE		(6) // Only UART version, CAN : 1Mbps Fixed
#define CMD_GET_COMM_BAUDRATE		(7) 
#define CMD_SET_FT_FILTER			(8)
#define CMD_GET_FT_FILTER			(9)
#define CMD_FT_ONCE					(10) 
#define CMD_FT_CONT					(11) 
#define CMD_FT_CONT_STOP			(12)
#define CMD_RESERVED_1				(13)
#define CMD_RESERVED_2				(14)
#define CMD_SET_CONT_OUT_FRQ 		(15)	
#define CMD_GET_CONT_OUT_FRQ  		(16)	
#define CMD_SET_BIAS				(17)
#define CMD_GET_OVERLOAD_COUNT		(18)

// OUTPUT FRQ. DEFINITIONS
#define OUTPUT_FRQ_200Hz_DEFAULT	(0)
#define OUTPUT_FRQ_10Hz				(1)
#define OUTPUT_FRQ_20Hz				(2)
#define OUTPUT_FRQ_50Hz				(3)
#define OUTPUT_FRQ_100Hz			(4)
#define OUTPUT_FRQ_200Hz			(5)
#define OUTPUT_FRQ_300Hz			(6)
#define OUTPUT_FRQ_500Hz			(7)
#define OUTPUT_FRQ_1000Hz			(8)

// FILTER SET VALUE DEFINITIONS
#define FILTER_NONE			(0) // NONE
#define FILTER_1ST_ORDER_LP	(1) // 1st order low-pass filter

// Cutoff Frq.
#define CUTOFF_1Hz		(1)
#define CUTOFF_2Hz		(2)
#define CUTOFF_3Hz		(3)
#define CUTOFF_5Hz		(4)
#define CUTOFF_10Hz		(5)
#define CUTOFF_20Hz		(6)
#define CUTOFF_30Hz		(7)
#define CUTOFF_40Hz		(8)
#define CUTOFF_50Hz		(9)
#define CUTOFF_100Hz	(10)
#define CUTOFF_150Hz	(11)
#define CUTOFF_200Hz	(12)
#define CUTOFF_300Hz	(13)
#define CUTOFF_500Hz	(14)

// PROCESSING DATA MAPPING FOR RFTEC01 R4
// RFT data field index in RFTEC01 R4 EtherCAT
#define IDX_D1 (4) 
#define IDX_D2 (5)
#define IDX_D3 (6)
#define IDX_D4 (7)
#define IDX_D5 (8)
#define IDX_D6 (9)
#define IDX_D7 (10)
#define IDX_D8 (11)
#define IDX_D9 (16) 
#define IDX_D10 (17)
#define IDX_D11 (18)
#define IDX_D12 (19)
#define IDX_D13 (20)
#define IDX_D14 (21)
#define IDX_D15 (22)
#define IDX_D16 (23)

#define IDX_RAW_FT1 (24) // 2 BYTES
#define IDX_RAW_FT2 (26) // 2 BYTES
#define IDX_RAW_FT3 (28) // 2 BYTES
#define IDX_RAW_FT4 (30) // 2 BYTES
#define IDX_RAW_FT5 (32) // 2 BYTES
#define IDX_RAW_FT6 (34) // 2 BYTES

#define IDX_OVERLOAD 	(36)
#define IDX_ERRLR_FLAG  (37)

unsigned char RFT_DATA_FIELD[17] = { 0 };
unsigned char RFT_OverloadStatus = 0;
unsigned char RFT_ErrorFlag = 0;
short RFT_FT_RAW[6] = { 0 };
float force_divider = 50.0f; 	// refer to the RFT sensor manual for sensor specific value
float torque_divider = 2000.0f; // refer to the RFT sensor manual for sensor specific value
float RFT_FT[6] = {0.0f};

// For Output Processing Data... configParam (4Bytes)
unsigned char configCmdType = CMD_NONE;
unsigned char configParam[3] = { 0 };

#define RFTEC01_R4_SLAVE_ID 1 // make sure the id of slave index

OSAL_THREAD_HANDLE thread2; // rft sensor data display threads.
OSAL_THREAD_HANDLE thread3;

int isRunRFTDisplayThread = 1;
int isRunRT_Thread = 1;


void cvtBufferToInt16( short *value, unsigned char *buff )
{
	unsigned char *temp = (unsigned char *)value;
	temp[0] = buff[0];
	temp[1] = buff[1];
}

void changeRFTEC01_ConfigParam( unsigned char param1, unsigned char param2, unsigned char param3, unsigned char param4 )
{
	uint8 *data_ptr;

   data_ptr = ec_slave[RFTEC01_R4_SLAVE_ID].outputs;
   
   data_ptr[0] = param1; // LSB
   data_ptr[1] = param2;
   data_ptr[2] = param3;
   data_ptr[3] = param4; // MSB
  //printf("config param : %02x %02x %02x %02x \n", data_ptr[3], data_ptr[2], data_ptr[1], data_ptr[0] );
}

void updateRFT_Data( void )
{
	int i;

	for( i = 0; i < 8; i++ )
		RFT_DATA_FIELD[i] = ec_slave[RFTEC01_R4_SLAVE_ID].inputs[IDX_D1 + i];
	for( i = 0; i < 8; i++ )
		RFT_DATA_FIELD[i + 8] = ec_slave[RFTEC01_R4_SLAVE_ID].inputs[IDX_D9 + i];
	
	for( i = 0; i < 6; i++ )
	{
		cvtBufferToInt16( RFT_FT_RAW + i, ec_slave[RFTEC01_R4_SLAVE_ID].inputs + IDX_RAW_FT1 + i * 2 );
	}
	
	RFT_OverloadStatus = ec_slave[RFTEC01_R4_SLAVE_ID].inputs[IDX_OVERLOAD];
	RFT_ErrorFlag = ec_slave[RFTEC01_R4_SLAVE_ID].inputs[IDX_ERRLR_FLAG];	
	
	RFT_FT[0] = RFT_FT_RAW[0] / force_divider; // fx
	RFT_FT[1] = RFT_FT_RAW[1] / force_divider; // fy
	RFT_FT[2] = RFT_FT_RAW[2] / force_divider; // fz
	RFT_FT[3] = RFT_FT_RAW[3] / torque_divider; // tx
	RFT_FT[4] = RFT_FT_RAW[4] / torque_divider; // ty
	RFT_FT[5] = RFT_FT_RAW[5] / torque_divider; // tz
}


OSAL_THREAD_FUNC RFT_DATA_DISPLAY_THREAD(void *lpParam) 
{
	printf("RFT DATA DISPLAY THREAD STARTED..\n");
	while(isRunRFTDisplayThread)
    {
		switch( RFT_DATA_FIELD[0] ) // CHECK RFT SENSOR DATA TYPE
		{
			case CMD_GET_PRODUCT_NAME: //CMD_GET_FIRMWARE_VER, CMD_GET_PRODUCT_NAME
				RFT_DATA_FIELD[16] = 0;
				printf("%s\r", &RFT_DATA_FIELD[1] );
			break;

			case CMD_GET_FIRMWARE_VER: //CMD_GET_FIRMWARE_VER, CMD_GET_PRODUCT_NAME
				RFT_DATA_FIELD[16] = 0;
				printf("%s\r", &RFT_DATA_FIELD[1] );
			break;	

			case CMD_FT_CONT:
				printf("%.03f, %.03f, %.03f, %.03f, %.03f, %.03f\n", 
				RFT_FT[0], RFT_FT[1], RFT_FT[2], RFT_FT[3], RFT_FT[4], RFT_FT[5] );
			break;

			default:
			break;
		}
		osal_usleep(100000);
	}
	
	printf("RFT DATA DISPLAY THREAD FINISHED..\n");
}

void update_RFTEC01_R4_OutputProcessData(void)
{
	
	if( configCmdType == CMD_NONE )
	{
		changeRFTEC01_ConfigParam( 0, 0, 0, 0 ); 
		configParam[0] = configParam[1] = configParam[2] = 0;
	}
	else
	{
		changeRFTEC01_ConfigParam( configCmdType, configParam[0], configParam[1], configParam[2] ); 
	}	
}

/* most basic RT thread for process data, just does IO transfer */
OSAL_THREAD_FUNC RFT_TEST_OP_THREAD(void *lpParam)
{
	char cmd = 'u';
	int  isRun = 1;
	
	printf("RFT_TEST_OP_THREAD STARTED..\n");
	
	while(isRun)
	{

		cmd = getchar(); // type command character & enter key...

		if( (cmd == 'E') || (cmd == 'e') ) // exit
		{
			isRun = 0;
			isRunRFTDisplayThread = 0;
			isRunRT_Thread = 0;
			printf("BYE RFTEC01 R4 SIMPLE TEST\n");
		}
		else if( (cmd == 'N') || (cmd == 'n') ) // get RFT SENSOR name
		{
			printf("GET PRODUCT NAME:\n");					
			configCmdType = CMD_GET_PRODUCT_NAME;
			configParam[0] = configParam[1] = configParam[2] = 0;
		}
		else if( (cmd == 'V') || (cmd == 'v') ) // get Firmware version
		{
			printf("GET FW VERSION:\n");
			configCmdType = CMD_GET_FIRMWARE_VER;
			configParam[0] = configParam[1] = configParam[2] = 0;
		}
		else if( cmd == 'M' || cmd == 'm' ) // measure
		{
			printf("START MEASUREMENT:\n");
			configCmdType = CMD_FT_CONT;
			configParam[0] = configParam[1] = configParam[2] = 0;
		}
		else if( cmd == 'S' || cmd == 's' ) // stop
		{
			printf("STOP MEASUREMENT:\n");
			configCmdType = CMD_FT_CONT_STOP;
			configParam[0] = configParam[1] = configParam[2] = 0;
		}
		else if( cmd == 'B' || cmd == 'b' ) // bias
		{
			printf("SET BIAS:\n");
			configCmdType = CMD_SET_BIAS;
			configParam[0] = 1;
			configParam[1] = configParam[2] = 0;
		}
		else if( cmd == 'U' || cmd == 'u' ) // un-bias
		{
			printf("UN BIAS:\n");
			configCmdType = CMD_SET_BIAS;
			configParam[0] = configParam[1] = configParam[2] = 0;
		}
		else
		{
			if( cmd != '\r' && cmd != '\n' )
				printf("UNKNOWN COMMAND\n");
		}
		
		osal_usleep(100000);
	}
}

void RFTEC01_R4_TEST(char *ifname)
{
    int i, j, oloop, iloop, wkc_count, chk, slc;
	
    needlf = FALSE;
    inOP = FALSE;

   printf("Starting simple test\n");
   
   /* initialise SOEM, bind socket to ifname */
   if (ec_init(ifname))
   {   
      printf("ec_init on %s succeeded.\n",ifname);
      /* find and auto-config slaves */


       if ( ec_config_init(FALSE) > 0 )
       {
         printf("%d slaves found and configured.\n",ec_slavecount);

         ec_config_map(&IOmap);

         ec_configdc();

         printf("Slaves mapped, state to SAFE_OP.\n");
         /* wait for all slaves to reach SAFE_OP state */
         ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

	 
         printf("Request operational state for all slaves\n");
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);
         ec_slave[0].state = EC_STATE_OPERATIONAL;
         
		 /* send one valid process data to make outputs in slaves happy*/  		 
		 ec_send_processdata();
         ec_receive_processdata(EC_TIMEOUTRET);

         /* request OP state for all slaves */
         ec_writestate(0);
         chk = 40;
         /* wait for all slaves to reach OP state */
         do
         {
			ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
         }
         while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
		 
		 
         if (ec_slave[0].state == EC_STATE_OPERATIONAL )
         {
            printf("Operational state reached for all slaves.\n");

			/* start test operation thread */
			osal_thread_create(&thread3, 0L, &RFT_TEST_OP_THREAD, (void*) &ctime);

			/* start sensor display thread */
			osal_thread_create(&thread2, 0L, &RFT_DATA_DISPLAY_THREAD, (void*) &ctime);

            wkc_count = 0;
            inOP = TRUE;

			while(isRunRT_Thread)
			{
				update_RFTEC01_R4_OutputProcessData();

				ec_send_processdata();
				wkc = ec_receive_processdata(EC_TIMEOUTRET);
				rtcnt++;
				
				/* do RT control stuff here */
				updateRFT_Data();	
		
				usleep(1000);
			}
			
            inOP = FALSE;
         }
         else
         {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for(i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
         }           

         printf("\nRequest init state for all slaves\n");
         ec_slave[0].state = EC_STATE_INIT;
         /* request INIT state for all slaves */
         ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End test software, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }   
}   



OSAL_THREAD_FUNC ecatcheck(void *lpParam) 
{
    int slave;

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
               needlf = FALSE;
               printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                     printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);                              
                  }
                  else if(ec_slave[slave].state > 0)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);                           
                     }
                  } 
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (!ec_slave[slave].state)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);                           
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(!ec_slave[slave].state)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);                           
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);                           
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }   

    //return 0;
}  

int main(int argc, char *argv[])
{
 
   printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

   if (argc > 1)
   {      
      /* create thread to handle slave error handling in OP */
      osal_thread_create(&thread1, 128000, &ecatcheck, (void*) &ctime);
	  		 
      /* start cyclic part */
      RFTEC01_R4_TEST(argv[1]);
   }
   else
   {
      printf("Usage: execurtion_file_name ifname1\nifname = eth0 for example\n");
   }   
   
   printf("End program\n");
   return (0);
}
