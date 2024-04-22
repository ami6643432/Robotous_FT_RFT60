/*
	EtherCAT, SOEM Open Source Lib. 
*/

/*
	Rev0.0, 2018.01.29
		- initial creation
*/

#include "RT_Console_Rev0.1.h"

#include "RFT_IF_ECAT_EC02_SAMPLE_Rev0.0.h"

// PROCESSING DATA MAPPING FOR RFT EC02
// RFT data field index in RFT EC02 EtherCAT
#define IDX_D1 (0) 
#define IDX_D2 (1)
#define IDX_D3 (2)
#define IDX_D4 (3)
#define IDX_D5 (4)
#define IDX_D6 (5)
#define IDX_D7 (6)
#define IDX_D8 (7)
#define IDX_D9 (8) 
#define IDX_D10 (9)
#define IDX_D11 (10)
#define IDX_D12 (11)
#define IDX_D13 (12)
#define IDX_D14 (13)
#define IDX_D15 (14)
#define IDX_D16 (15)

#define IDX_RAW_FT1 (16) // 2 BYTES
#define IDX_RAW_FT2 (18) // 2 BYTES
#define IDX_RAW_FT3 (20) // 2 BYTES
#define IDX_RAW_FT4 (22) // 2 BYTES
#define IDX_RAW_FT5 (24) // 2 BYTES
#define IDX_RAW_FT6 (26) // 2 BYTES

#define IDX_OVERLOAD 	(28)
#define IDX_ERRLR_FLAG  (29)

// constructor and destructor
CRT_RFT_ECAT_IF::CRT_RFT_ECAT_IF()
{
	initialize_variables();

	ec_adapter *adapter = NULL;
	ec_adapter *adapter_old = NULL;

	adapter = ec_find_adapters();
	while (adapter != NULL)
	{
		CONSOLE_S("NIC Description : %s, Device to use for wpcap: \n --> %s\n", adapter->desc, adapter->name);
		
		CString desc = adapter->desc;
		CString name = adapter->name;
		m_vecNIC_Desc.push_back(desc);
		m_vecNic_Name.push_back(name);

		adapter_old = adapter;
		adapter = adapter->next;
		delete adapter_old;
	}

	delete adapter;



}

CRT_RFT_ECAT_IF::~CRT_RFT_ECAT_IF()
{
	stop();
}

bool CRT_RFT_ECAT_IF::stop(void)
{
	if (m_bEcatcheckThreadFlag)
	{
		m_bEcatcheckThreadFlag = false;
		m_hEcheckThread = NULL;
	}

	if (m_bOpCheckheckThreadFlag)
	{
		m_bOpCheckheckThreadFlag = false;
		m_hOpCheckThread = NULL;
	}

	/* stop RT thread */
	if (m_mmResult)
	{
		timeKillEvent(m_mmResult);
	}

	if (m_bInOP)
	{
		m_bInOP = false;

		CONSOLE_S("\nRequest init state for all slaves\n");
		ec_slave[0].state = EC_STATE_INIT; // 0 is master

		/* request INIT state for all slaves */
		ec_writestate(0);

		/* stop SOEM, close socket */
		ec_close();
	}

	return true;
}

bool CRT_RFT_ECAT_IF::run(CString NIC_Name, int slaveId, bool callback_enable, float forceDivider, float torqueDivider)
{
	bool result = false;

	int chk;

	m_bIsEnabled_Callback = callback_enable;

	m_RFT_IF_PACKET.setDivider(forceDivider, torqueDivider);

	m_nSlaveID = slaveId;

	
	if (ec_init(NIC_Name.GetBuffer()))
	{
		if (ec_config_init(FALSE) > 0)
		{
			// EHTERCAT CHECK THREAD
			m_bEcatcheckThreadFlag = true;
			m_hEcheckThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)ecatcheckThread, (LPVOID)this, 0, &m_dwEcheckThreadId);

			CONSOLE_S("%d slaves found and configured.\n", ec_slavecount);
			ec_config_map(&IOmap);

			ec_configdc();

			CONSOLE_S("Slaves mapped, state to SAFE_OP.\n");
			
			/* wait for all slaves to reach SAFE_OP state */
			ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);


			CONSOLE_S("Request operational state for all slaves\n");
			m_nExpectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
			CONSOLE_S("Calculated workcounter %d\n", m_nExpectedWKC);
			ec_slave[0].state = EC_STATE_OPERATIONAL;

			/* send one valid process data to make outputs in slaves happy*/
			ec_send_processdata();
			ec_receive_processdata(EC_TIMEOUTRET);

			/* start RT thread as periodic MM timer */
			m_mmResult = timeSetEvent(1, 0, RTthread, (DWORD_PTR)this, TIME_PERIODIC);

			/* request OP state for all slaves */
			ec_writestate(0);
			chk = 40;
			/* wait for all slaves to reach OP state */
			do
			{
				ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
				//Sleep(10);
			} while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));


			if (ec_slave[0].state == EC_STATE_OPERATIONAL)
			{
				CONSOLE_S("Operational state reached for all slaves.\n");
				
				// Sensor Operation CHECK THREAD
				m_bOpCheckheckThreadFlag = true;
				m_hOpCheckThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)opCheckThread, (LPVOID)this, 0, &m_dwOpCheckThreadId);

				m_bInOP = true;
				result = true;
			}
			else
			{
				//CONSOLE_S("Not all slaves reached operational state.\n");
				ec_readstate();
				for (int i = 1; i <= ec_slavecount; i++)
				{
					if (ec_slave[i].state != EC_STATE_OPERATIONAL)
					{
						CONSOLE_S("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
							i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
					}
				}

				CString message;
				message = "Not all slaves reached operational state.";
				::AfxMessageBox(message);

				stop();
			}
		}
		else
		{
			ec_close();
			CString message;
			message = "There is no ECAT slave";
			::AfxMessageBox(message);
		}
	}
	else
	{
		CString message;
		message.Format("No socket connection on %s", NIC_Name.GetBuffer());
		::AfxMessageBox(message);
	}

	return result;
}


void CRT_RFT_ECAT_IF::ecatcheckThread(CRT_RFT_ECAT_IF *pThis)
{
	pThis->ecatcheckWorker();
}

void CRT_RFT_ECAT_IF::ecatcheckWorker(void)
{
	CONSOLE_S("Start... ecatcheck\n");
	int slave;

	while (m_bEcatcheckThreadFlag)
	{

		if (m_bInOP && ((m_nWKC < m_nExpectedWKC) || ec_group[m_ucCurrentGroup].docheckstate))
		{
			if (m_bNeedlf)
			{
				m_bNeedlf = FALSE;
				CONSOLE_S("\n");
			}
			
			/* one ore more slaves are not responding */
			ec_group[m_ucCurrentGroup].docheckstate = FALSE;
			ec_readstate();

			for (slave = 1; slave <= ec_slavecount; slave++)
			{
				if ((ec_slave[slave].group == m_ucCurrentGroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
				{
					ec_group[m_ucCurrentGroup].docheckstate = TRUE;
					if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
					{
						CONSOLE_S("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
						ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
						ec_writestate(slave);
					}
					else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
					{
						CONSOLE_S("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
						ec_slave[slave].state = EC_STATE_OPERATIONAL;
						ec_writestate(slave);
					}
					else if (ec_slave[slave].state > 0)
					{
						if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
						{
							ec_slave[slave].islost = FALSE;
							CONSOLE_S("MESSAGE : slave %d reconfigured\n", slave);
						}
					}
					else if (!ec_slave[slave].islost)
					{
						/* re-check state */
						ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
						if (!ec_slave[slave].state)
						{
							ec_slave[slave].islost = TRUE;
							CONSOLE_S("ERROR : slave %d lost\n", slave);
						}
					}
				}
				if (ec_slave[slave].islost)
				{
					if (!ec_slave[slave].state)
					{
						if (ec_recover_slave(slave, EC_TIMEOUTMON))
						{
							ec_slave[slave].islost = FALSE;
							CONSOLE_S("MESSAGE : slave %d recovered\n", slave);
						}
					}
					else
					{
						ec_slave[slave].islost = FALSE;
						CONSOLE_S("MESSAGE : slave %d found\n", slave);
					}
				}
			}
			
			if (!ec_group[m_ucCurrentGroup].docheckstate)
				CONSOLE_S("OK : all slaves resumed OPERATIONAL.\n");
		}

		SleepEx(10, FALSE );
		//osal_usleep(10000); // [us]
	}

	CONSOLE_S("Stop... ecatcheck\n");
}


/* most basic RT thread for process data, just does IO transfer */
void CALLBACK CRT_RFT_ECAT_IF::RTthread(UINT uTimerID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2)
{
	CRT_RFT_ECAT_IF *pThis = (CRT_RFT_ECAT_IF*)dwUser;
	pThis->RTthreadWorker();
}

void CRT_RFT_ECAT_IF::RTthreadWorker(void)
{
	//CONSOLE_S("j");
	//IOmap[0]++;

	//
	update_RFTEC01_R4_OutputProcessData();

	ec_send_processdata();

	m_nWKC = ec_receive_processdata(EC_TIMEOUTRET);

	m_nRT_Cnt++;

	//CONSOLE_S("%d\n", m_gRT_Cnt);

	/* do RT control stuff here */
	updateRFT_Data();
}

void CRT_RFT_ECAT_IF::setCallback(RFT_ECAT_IF_CALLBACK pCallbackFunc, void *callbackParam)
{
	m_pCallbackFunc = pCallbackFunc;
	m_pCallbackParam = callbackParam;

	CONSOLE_S("SETTING.... RFT CAN I/F CALL BACK FUNCTION\n");
}



void CRT_RFT_ECAT_IF::initialize_variables(void)
{

	m_hEcheckThread = NULL;         // Handle of Thread
	m_dwEcheckThreadId = 0;
	m_bEcatcheckThreadFlag = false;
	
	m_hOpCheckThread = NULL;
	m_dwOpCheckThreadId = 0;
	m_bOpCheckheckThreadFlag = false;

	m_pCallbackFunc = NULL;
	m_pCallbackParam = NULL;

	// for logging
	m_pLogFile = NULL;
	m_nlogDataCount = 0;

	m_bInOP = false;
	m_bNeedlf = false;
	m_nExpectedWKC = 0;
	m_ucCurrentGroup = 0;
	m_nSlaveID = 1;
	m_nWKC = 0;
	m_nRT_Cnt = 0;

	m_mmResult = 0;

	configCmdType = CMD_NONE;
	memset(configParam, 0, sizeof(configParam));
	memset(RFT_DATA_FIELD, 0, sizeof(RFT_DATA_FIELD));
	RFT_ErrorFlag = 0;
	memset(RFT_FT_RAW, 0, sizeof(RFT_FT_RAW));
}

void CRT_RFT_ECAT_IF::cvtBufferToInt16(short *value, unsigned char *buff)
{
	unsigned char *temp = (unsigned char *)value;
	temp[0] = buff[0];
	temp[1] = buff[1];
}

void CRT_RFT_ECAT_IF::changeRFTEC01_ConfigParam(unsigned char param1, unsigned char param2, unsigned char param3, unsigned char param4)
{
	uint8 *data_ptr;

	data_ptr = ec_slave[m_nSlaveID].outputs;

	data_ptr[0] = param1; // LSB
	data_ptr[1] = param2;
	data_ptr[2] = param3;
	data_ptr[3] = param4; // MSB
	//CONOLE_S("config param : %02x %02x %02x %02x \n", data_ptr[3], data_ptr[2], data_ptr[1], data_ptr[0] );

	data_ptr[7] = param1; // LSB
	data_ptr[6] = param2;
	data_ptr[5] = param3;
	data_ptr[4] = param4; // MSB
}

void CRT_RFT_ECAT_IF::updateRFT_Data(void)
{
	int i;

	for (i = 0; i < 8; i++)
		RFT_DATA_FIELD[i] = ec_slave[m_nSlaveID].inputs[IDX_D1 + i];
	for (i = 0; i < 8; i++)
		RFT_DATA_FIELD[i + 8] = ec_slave[m_nSlaveID].inputs[IDX_D9 + i];

	for (i = 0; i < 6; i++)
	{
		cvtBufferToInt16(RFT_FT_RAW + i, ec_slave[m_nSlaveID].inputs + IDX_RAW_FT1 + i * 2);
	}

	m_RFT_IF_PACKET.m_rcvdForceStatus = ec_slave[m_nSlaveID].inputs[IDX_OVERLOAD];
	RFT_ErrorFlag = ec_slave[m_nSlaveID].inputs[IDX_ERRLR_FLAG];

	m_RFT_IF_PACKET.m_rcvdForce[0] = RFT_FT_RAW[0] / m_RFT_IF_PACKET.m_fDividerForce; // fx
	m_RFT_IF_PACKET.m_rcvdForce[1] = RFT_FT_RAW[1] / m_RFT_IF_PACKET.m_fDividerForce; // fy
	m_RFT_IF_PACKET.m_rcvdForce[2] = RFT_FT_RAW[2] / m_RFT_IF_PACKET.m_fDividerForce; // fz
	m_RFT_IF_PACKET.m_rcvdForce[3] = RFT_FT_RAW[3] / m_RFT_IF_PACKET.m_fDividerTorque; // tx
	m_RFT_IF_PACKET.m_rcvdForce[4] = RFT_FT_RAW[4] / m_RFT_IF_PACKET.m_fDividerTorque; // ty
	m_RFT_IF_PACKET.m_rcvdForce[5] = RFT_FT_RAW[5] / m_RFT_IF_PACKET.m_fDividerTorque; // tz

	if (RFT_DATA_FIELD[0] == CMD_FT_CONT)
	{
		if (m_bIsEnabled_Callback)
		{
			// callback function...
			m_pCallbackFunc(m_pCallbackParam);
		}

		// data logging
		m_logfile_mutex.lock();
		if (m_bIsStartLogging)
		{
			m_nlogDataCount++;
			fprintf(m_pLogFile, "%f %f %f %f %f %f\n",
				m_RFT_IF_PACKET.m_rcvdForce[0], m_RFT_IF_PACKET.m_rcvdForce[1], m_RFT_IF_PACKET.m_rcvdForce[2],
				m_RFT_IF_PACKET.m_rcvdForce[3], m_RFT_IF_PACKET.m_rcvdForce[4], m_RFT_IF_PACKET.m_rcvdForce[5]);
		}
		m_logfile_mutex.unlock();
	}
}

void CRT_RFT_ECAT_IF::update_RFTEC01_R4_OutputProcessData(void)
{

	if (configCmdType == CMD_NONE)
	{
		changeRFTEC01_ConfigParam(0, 0, 0, 0);
		configParam[0] = configParam[1] = configParam[2] = 0;
	}
	else
	{
		changeRFTEC01_ConfigParam(configCmdType, configParam[0], configParam[1], configParam[2]);
	}

	if (configCmdType != CMD_SET_BIAS)
		m_nCurrMode = configCmdType;
}

void CRT_RFT_ECAT_IF::opCheckThread(CRT_RFT_ECAT_IF *pThis)
{
	pThis->opCheckWorker();
}

void CRT_RFT_ECAT_IF::opCheckWorker(void)
{
	CONSOLE_S("Start... RFT Sensor Status Checking");

	while (m_bOpCheckheckThreadFlag)
	{
		switch (m_nCurrMode)
		{
		case CMD_GET_PRODUCT_NAME:
			if (RFT_DATA_FIELD[0] == m_nCurrMode)
			{
				m_bIsRcvd_Response_Pkt = true;
			}

			break;

		case CMD_GET_SERIAL_NUMBER:
			if (RFT_DATA_FIELD[0] == m_nCurrMode)
			{
				m_bIsRcvd_Response_Pkt = true;
			}
			break;

		case CMD_GET_FIRMWARE_VER:
			if (RFT_DATA_FIELD[0] == m_nCurrMode)
			{
				m_bIsRcvd_Response_Pkt = true;
			}
			break;

			//case CMD_SET_ID: not supported at ethercat type
			//	break;

		case CMD_GET_ID:
			if (RFT_DATA_FIELD[0] == m_nCurrMode)
			{
				m_bIsRcvd_Response_Pkt = true;
			}
			break;

		case CMD_SET_FT_FILTER:
			if (RFT_DATA_FIELD[0] == m_nCurrMode)
			{
				m_bIsRcvd_Response_Pkt = true;
			}
			break;

		case CMD_GET_FT_FILTER:
			if (RFT_DATA_FIELD[0] == m_nCurrMode)
			{
				m_bIsRcvd_Response_Pkt = true;
			}
			break;

		case CMD_FT_ONCE:
			if (RFT_DATA_FIELD[0] == m_nCurrMode)
			{
				m_bIsRcvd_Response_Pkt = true;
			}
			break;

		case CMD_FT_CONT:
			if (RFT_DATA_FIELD[0] == m_nCurrMode)
			{
				m_bIsRcvd_Response_Pkt = true;
			}
			break;

		case CMD_FT_CONT_STOP:
			// there is no response packet
			break;

		case CMD_SET_CONT_OUT_FRQ:
			if (RFT_DATA_FIELD[0] == m_nCurrMode)
			{
				m_bIsRcvd_Response_Pkt = true;
			}
			break;

		case CMD_GET_CONT_OUT_FRQ:
			if (RFT_DATA_FIELD[0] == m_nCurrMode)
			{
				m_bIsRcvd_Response_Pkt = true;
			}
			break;

		case CMD_GET_OVERLOAD_COUNT:
			if (RFT_DATA_FIELD[0] == m_nCurrMode)
			{
				m_bIsRcvd_Response_Pkt = true;
			}
			break;

		default: break;
		}

		SleepEx(10, FALSE);
	}

	CONSOLE_S("Finish... RFT Sensor Status Checking");
}


// read product name
bool CRT_RFT_ECAT_IF::rqst_ProductName(void)				  
{
	m_bIsRcvd_Response_Pkt = true;
	configParam[0] = 0;
	configParam[1] = 0;
	configParam[2] = 0;
	configCmdType = CMD_GET_PRODUCT_NAME;
	return true;
}

// read serial number
bool CRT_RFT_ECAT_IF::rqst_SerialNumber(void)				  
{
	m_bIsRcvd_Response_Pkt = true;
	configParam[0] = 0;
	configParam[1] = 0;
	configParam[2] = 0;
	configCmdType = CMD_GET_SERIAL_NUMBER;
	return true;
}

// read firmware version
bool CRT_RFT_ECAT_IF::rqst_Firmwareverion(void)				  
{
	m_bIsRcvd_Response_Pkt = true;
	configParam[0] = 0;
	configParam[1] = 0;
	configParam[2] = 0;
	configCmdType = CMD_GET_FIRMWARE_VER;
	return true;
}

// read CAN message ID
bool CRT_RFT_ECAT_IF::rqst_GetID(void)						  
{
	m_bIsRcvd_Response_Pkt = true;
	configParam[0] = 0;
	configParam[1] = 0;
	configParam[2] = 0;
	configCmdType = CMD_GET_ID;
	return true;
}

// read filter type
bool CRT_RFT_ECAT_IF::rqst_FT_Filter_Type(void)				  
{
	m_bIsRcvd_Response_Pkt = true;
	configParam[0] = 0;
	configParam[1] = 0;
	configParam[2] = 0;
	configCmdType = CMD_GET_FT_FILTER;
	return true;
}

// read force/torque (once)
bool CRT_RFT_ECAT_IF::rqst_FT(void)							  
{
	m_bIsRcvd_Response_Pkt = true;
	configParam[0] = 0;
	configParam[1] = 0;
	configParam[2] = 0;
	configCmdType = CMD_FT_ONCE;
	return true;
}

// start force/torque continuous outpu
bool CRT_RFT_ECAT_IF::rqst_FT_Continuous(void)				  
{
	m_bIsRcvd_Response_Pkt = true; 
	configParam[0] = 0;
	configParam[1] = 0;
	configParam[2] = 0;
	configCmdType = CMD_FT_CONT;
	return true;
}

// stop force/torque continuous outpu
bool CRT_RFT_ECAT_IF::rqst_FT_Stop(void)					  
{
	m_bIsRcvd_Response_Pkt = true; // there is no response packet.
	configParam[0] = 0;
	configParam[1] = 0;
	configParam[2] = 0;
	configCmdType = CMD_FT_CONT_STOP;
	return true;
}

// read force/torque output frq
bool CRT_RFT_ECAT_IF::rqst_FT_Cont_Interval(void)			  
{
	m_bIsRcvd_Response_Pkt = false;
	configParam[0] = 0;
	configParam[1] = 0;
	configParam[2] = 0;
	configCmdType = CMD_GET_CONT_OUT_FRQ;
	return true;
}

// read overload count
bool CRT_RFT_ECAT_IF::rqst_FT_OverloadCnt(void)				  
{
	m_bIsRcvd_Response_Pkt = false;
	configParam[0] = 0;
	configParam[1] = 0;
	configParam[2] = 0;
	configCmdType = CMD_GET_OVERLOAD_COUNT;
	return true;
}

//bool set_ID(void);	// set CAN message ID - not supported for ECAT

// set filter
bool CRT_RFT_ECAT_IF::set_FT_Filter_Type(int filter_type, int sub_type)	
{
	m_bIsRcvd_Response_Pkt = false;
	configParam[0] = filter_type;
	configParam[1] = sub_type;
	configParam[2] = 0;
	configCmdType = CMD_SET_FT_FILTER;
	return true;
}

// set force/torque output frq.
bool CRT_RFT_ECAT_IF::set_FT_Cont_Interval(int interval)					
{
	m_bIsRcvd_Response_Pkt = false;
	configParam[0] = interval;
	configParam[1] = 0;
	configParam[2] = 0;
	configCmdType = CMD_SET_CONT_OUT_FRQ;
	return true;
}

// set bias
bool CRT_RFT_ECAT_IF::set_FT_Bias(int is_on)								
{
	m_bIsRcvd_Response_Pkt = true; // there is no response packet
	configParam[0] = is_on;
	configParam[1] = 0;
	configParam[2] = 0;
	configCmdType = CMD_SET_BIAS;
	return true;
}

bool CRT_RFT_ECAT_IF::startLogging(CString logfileName)
{
	bool result = true;

	stopLogging(); // if the log file was opened then close the file.

	m_logfile_mutex.lock();

	SYSTEMTIME	sysTime;
	::GetLocalTime(&sysTime);

	CString time;
	time.Format("_%04d%02d%02d_%02d%02d%02d", sysTime.wYear, sysTime.wMonth, sysTime.wDay,
		sysTime.wHour, sysTime.wMinute, sysTime.wSecond);

	logfileName += "_log" + time + ".txt";

	fopen_s(&m_pLogFile, logfileName.GetBuffer(), "w");
	if (m_pLogFile == NULL)
	{
		result = false;
	}
	else
	{
		m_nlogDataCount = 0;
		m_bIsStartLogging = true;
	}
	m_logfile_mutex.unlock();

	return result;
}


bool CRT_RFT_ECAT_IF::stopLogging(void)
{
	m_logfile_mutex.lock();
	if (m_pLogFile != NULL)
	{
		fclose(m_pLogFile);

		m_bIsStartLogging = false;
	}
	m_logfile_mutex.unlock();

	return true;
}
// END OF FILE

