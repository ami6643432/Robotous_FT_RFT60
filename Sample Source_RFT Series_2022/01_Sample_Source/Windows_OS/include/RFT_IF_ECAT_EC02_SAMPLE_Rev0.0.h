/*
	EtherCAT, SOEM Open Source Lib. 
*/

/*
	Rev0.0, 2018.02.23
		- initial creation
*/

#ifndef __RT_RFT_CAN_IF_H__
#define __RT_RFT_CAN_IF_H__

#include "RFT_IF_PACKET_Rev1.2.h"	//

#include <vector>
#include <mutex>
using namespace std;

//#include <timeapi.h>
#include <stdio.h>
#include <string.h>
#include <Mmsystem.h>

#include "osal.h"
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"

#define EC_TIMEOUTMON (500)

// for callback.... 
typedef void(*RFT_ECAT_IF_CALLBACK) (void *);

class CRT_RFT_ECAT_IF
{
public:
	// constructor and destructor
	CRT_RFT_ECAT_IF();
	~CRT_RFT_ECAT_IF();

public:
	
	bool run(CString NIC_Name, int slaveId, bool callback_enable, float forceDivider, float torqueDivider);
	bool stop(void);

	// for thread
	HANDLE m_hEcheckThread;
	DWORD  m_dwEcheckThreadId;
	bool   m_bEcatcheckThreadFlag;
	static void ecatcheckThread(CRT_RFT_ECAT_IF *pThis);
	void   ecatcheckWorker(void);

	static void CALLBACK RTthread(UINT uTimerID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2);
	void RTthreadWorker(void);

	HANDLE m_hOpCheckThread;
	DWORD  m_dwOpCheckThreadId;
	bool   m_bOpCheckheckThreadFlag;
	static void opCheckThread(CRT_RFT_ECAT_IF *pThis);
	void   opCheckWorker(void);

	// for callback function
	bool m_bIsEnabled_Callback;
	RFT_ECAT_IF_CALLBACK m_pCallbackFunc;
	void *m_pCallbackParam;
	void setCallback(RFT_ECAT_IF_CALLBACK pCallbackFunc, void *callbackParam);

	CRT_RFT_IF_PACKET m_RFT_IF_PACKET;			  // data field & uart packet handling class

	
	bool rqst_ProductName(void);				  // read product name
	bool rqst_SerialNumber(void);				  // read serial number
	bool rqst_Firmwareverion(void);				  // read firmware version
	bool rqst_GetID(void);						  // read CAN message ID
	bool rqst_FT_Filter_Type(void);				  // read filter type
	bool rqst_FT(void);							  // read force/torque (once)
	bool rqst_FT_Continuous(void);				  // start force/torque continuous output
	bool rqst_FT_Stop(void);					  // stop force/torque continuous output
	bool rqst_FT_Cont_Interval(void);			  // read force/torque output frq.
	bool rqst_FT_OverloadCnt(void);				  // read overload count

	//bool set_ID(void);	// set CAN message ID - not supported for ECAT
	bool set_FT_Filter_Type( int filter_type, int sub_type);	// set filter
	bool set_FT_Cont_Interval(int interval);					// set force/torque output frq.
	bool set_FT_Bias(int is_on);								// set bias


	// 제어 전송 명령 형태, 0 이면 현재 제어 전송 명령이 없음....
	int m_nCurrMode;								// current operation mode or command type
	bool m_bIsRcvd_Response_Pkt;					// receive flag for response packet of current command

	//////////////////////////////////////////////////////////////////////////////////
	// for log file
	bool startLogging(CString logfileName = "");
	bool stopLogging(void);

	// for SOEM
	vector<CString> m_vecNIC_Desc;
	vector<CString> m_vecNic_Name;

protected:
	// 
	void initialize_variables(void);

	// for SOEM
	bool m_bInOP;
	int m_nExpectedWKC;
	bool m_bNeedlf;
	int m_nSlaveID;
	volatile int m_nWKC;
	volatile int m_nRT_Cnt;
	char IOmap[4096];
	unsigned char m_ucCurrentGroup;
	UINT m_mmResult; // timer event id

	unsigned char configCmdType;
	unsigned char configParam[3];
	unsigned char RFT_DATA_FIELD[17];
	unsigned char RFT_ErrorFlag;
	short RFT_FT_RAW[6];
	void cvtBufferToInt16(short *value, unsigned char *buff);
	void changeRFTEC01_ConfigParam(unsigned char param1, unsigned char param2, unsigned char param3, unsigned char param4);
	void updateRFT_Data(void);
	void update_RFTEC01_R4_OutputProcessData(void);

	// for log file
	mutex m_logfile_mutex;
	bool m_bIsStartLogging;
	unsigned long m_nlogDataCount;
	FILE *m_pLogFile;
};

#endif//__RT_RFT_CAN_IF_H__

// END OF FILE
