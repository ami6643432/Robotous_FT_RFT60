/*
	CAN DEVICE: IXXAT CAN DEVICE 사용
*/

/*
	Rev0.0, 2017.12.15
		- initial creation
		- IXXAT CAN Controller
*/

#ifndef __RT_RFT_CAN_IF_H__
#define __RT_RFT_CAN_IF_H__

#include "RFT_IF_PACKET_Rev1.2.h"	//

#include "vciguid.h"	// for IXXAT CAN
#include <vcinpl.h>		// for IXXAT CAN 

#include <mutex>
using namespace std;

#define CAN_STANDARD	(0)
#define CAN_EXTENDED	(1)

#define CAN_BAUD_10KBPS		(0)
#define CAN_BAUD_20KBPS		(1)
#define CAN_BAUD_50KBPS		(2)
#define CAN_BAUD_100KBPS	(3)
#define CAN_BAUD_125KBPS	(4)
#define CAN_BAUD_250KBPS	(5)
#define CAN_BAUD_500KBPS	(6)
#define CAN_BAUD_800KBPS	(7)
#define CAN_BAUD_1000KBPS	(8)

#define CAN_DATA_LENGTH (8)

#define RFT_RCVD_DATA_BUFF_MAX_SIZE (2048)

// for callback.... 
typedef void(*RFT_CAN_IF_CALLBACK) (void *);

class CRT_RFT_CAN_IF
{
public:
	// constructor and destructor
	CRT_RFT_CAN_IF();
	~CRT_RFT_CAN_IF();

public:
	
	// opMode : STANDARD, EXTENDED, baudrate: buad rate type 참조
	bool run(int opMode, int baudrate, bool callback_enable, float forceDivider, float torqueDivider, int txId = 100, int rxId_01 = 1, int rxId_02 = 2);
	bool stop(void);

	// for thread
	HANDLE m_hCommReadThread;         // Handle of Thread
	DWORD  m_idCommReadThread;        // ID of Thread
	bool   m_bCommReadThreadFlag;
	static void commReadThread(CRT_RFT_CAN_IF *pThis);
	void   readWorker(void);

	// for callback function
	bool m_bIsEnabled_Callback;
	RFT_CAN_IF_CALLBACK m_pCallbackFunc;
	void *m_pCallbackParam;
	void setCallback(RFT_CAN_IF_CALLBACK pCallbackFunc, void *callbackParam);

	CRT_RFT_IF_PACKET m_RFT_IF_PACKET;				// data field & uart packet handling class


	bool rqst_ProductName(int txId);				  // read product name
	bool rqst_SerialNumber(int txId);				  // read serial number
	bool rqst_Firmwareverion(int txId);				  // read firmware version
	bool rqst_GetID(int txId);						  // read CAN message ID
	bool rqst_FT_Filter_Type(int txId);				  // read filter type
	bool rqst_FT(int txId);							  // read force/torque (once)
	bool rqst_FT_Continuous(int txId);				  // start force/torque continuous output
	bool rqst_FT_Stop(int txId);					  // stop force/torque continuous output
	bool rqst_FT_Cont_Interval(int txId);			  // read force/torque output frq.
	bool rqst_FT_OverloadCnt(int txId);				  // read overload count

	bool set_ID(int txId, int ftTxID, int ftRxID_01, int ftRxID_02);	// set CAN message ID
	bool set_FT_Filter_Type(int txId, int filter_type, int sub_type);	// set filter
	bool set_FT_Cont_Interval(int txId, int interval);					// set force/torque output frq.
	bool set_FT_Bias(int txId, int is_on);								// set bias


	// 제어 전송 명령 형태, 0 이면 현재 제어 전송 명령이 없음....
	int m_nTxId;									// the transmit ID < the receive ID of RFT sensor>
	int m_nRxId_01;									// the receive ID#1 < the transmit ID of RFT sensor>
	int m_nRxId_02;									// the receive ID#2 < the transmit ID of RFT sensor>
	int m_nCurrMode;								// current operation mode or command type
	int m_nRcvdPktCnt_AtCurrMode;					// received can message count 
	int m_nRcvdData_ValidLength;					// data length of received paket from RFT sensor
	bool m_bIsRcvd_Response_Pkt;					// receive flag for response packet of current command

	void RFT_Data_Handler(CANMSG msg);				// receive packet handler


	void rcvdCAN_ProduectName(CANMSG msg);			// check function for response of read product name command
	void rcvdCAN_SerialNumber(CANMSG msg);			// check function for response of read serial number command
	void rcvdCAN_Firmwareversion(CANMSG msg);		// check function for response of read firmware version command
	void rcvdCAN_ID(CANMSG msg);					// check function for response of read CAN ID command
	void rcvdCAN_FT_Filter_Type(CANMSG msg);		// check function for response of read filter type command
	void rcvdCAN_FT(CANMSG msg);					// check function for response of read force/torque command
	void rcvdCAN_FT_Cont_Interval(CANMSG msg);		// check function for response of read force/torque output frq. command
	void rcvdCAN_FT_Overload_Count(CANMSG msg);		// check function for response of read overload count command

	void rcvdCAN_Response_Set_ID(CANMSG msg);		// check function for response of set CAN ID command
	void rcvdCAN_Response_Set_FT_Filter_Type(CANMSG msg);	// check function for response of set filter type command
	void rcvdCAN_Response_Set_FT_Cont_Interval(CANMSG msg);	// check function for response of set force/torque output frq. command

	//////////////////////////////////////////////////////////////////////////////////
	// for log file
	bool startLogging(CString logfileName = "");
	bool stopLogging(void);

protected:
	// 
	void initialize_variables(void);

	bool sendData(int txId, unsigned char *dataBuff, int length);
	bool sendMultiPkt(int txId, unsigned char *dataBuff, int length);

	unsigned char m_rcvdDataBuff[RFT_RCVD_DATA_BUFF_MAX_SIZE];

	// for IXXAT CAN
	HANDLE hDevice;       // device handle
	LONG   lCtrlNo;       // controller number
	HANDLE hCanCtl;       // controller handle 
	HANDLE hCanChn;       // channel handle
	UINT8 m_nCAN_OpMode;  // operation mode of the IXXAT CAN Controller

	HRESULT SelectDevice(void);			// 
	void DisplayError(HRESULT hResult);	// 
	HRESULT InitSocket(UINT32 dwCanNo, UINT8  bMode, UINT8  bBtr0, UINT8  bBtr1); // 
	void getBaudParam(int baudrate, UINT8 *btr0, UINT8 *btr1);

	// for log file
	mutex m_logfile_mutex;
	bool m_bIsStartLogging;
	unsigned long m_nlogDataCount;
	FILE *m_pLogFile;
};

#endif//__RT_RFT_CAN_IF_H__

// END OF FILE
