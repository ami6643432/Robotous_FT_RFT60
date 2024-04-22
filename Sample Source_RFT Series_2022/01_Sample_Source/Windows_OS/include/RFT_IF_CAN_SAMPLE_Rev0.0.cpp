/*
CAN DEVICE: IXXAT CAN DEVICE 사용
*/


#include "RT_Console_Rev0.1.h"

#include "RFT_IF_CAN_SAMPLE_Rev0.0.h"


// constructor and destructor
CRT_RFT_CAN_IF::CRT_RFT_CAN_IF()
{
	initialize_variables();
}

CRT_RFT_CAN_IF::~CRT_RFT_CAN_IF()
{
	stop();
}

bool CRT_RFT_CAN_IF::sendData(int txId, unsigned char *dataBuff, int length)
{
	HRESULT hResult;
	CANMSG  sCanMsg;
	int   i, j;

	UINT8 frameFormat = 0;
	if (m_nCAN_OpMode != CAN_OPMODE_STANDARD)
		frameFormat = CAN_MSGFLAGS_EXT;

	sCanMsg.dwTime = 0;
	sCanMsg.dwMsgId = txId;    // CAN message identifier

	sCanMsg.uMsgInfo.Bytes.bType = CAN_MSGTYPE_DATA;
	sCanMsg.uMsgInfo.Bytes.bFlags = CAN_MAKE_MSGFLAGS(CAN_DATA_LENGTH, 0, 0, 0, frameFormat);
	//sCanMsg.uMsgInfo.Bits.srr = 1;	// self reception request....

	int txPktCnt = length / CAN_DATA_LENGTH;
	int remain = length % CAN_DATA_LENGTH;

	for (i = 0; i < txPktCnt; i++)
	{
		for (j = 0; j < CAN_DATA_LENGTH; j++)
		{
			sCanMsg.abData[j] = dataBuff[i*CAN_DATA_LENGTH + j];
		}

		// write the CAN message into the transmit FIFO
		hResult = canChannelSendMessage(hCanChn, INFINITE, &sCanMsg);
		if (hResult != VCI_OK)
		{	// 에러....
			DisplayError(hResult);
			return false;
		}
		Sleep(1); // 수신측 부하를 줄이기 위해...
	}

	if (remain > 0)
	{
		for (j = 0; j < remain; j++)
		{
			sCanMsg.abData[j] = dataBuff[i*CAN_DATA_LENGTH + j]; // 
		}
		for (j = remain; j < CAN_DATA_LENGTH; j++)
		{
			sCanMsg.abData[j] = dataBuff[i*CAN_DATA_LENGTH + j];
		}
		// write the CAN message into the transmit FIFO
		hResult = canChannelSendMessage(hCanChn, INFINITE, &sCanMsg);
		if (hResult != VCI_OK)
		{	//error ....
			DisplayError(hResult);
			return false;
		}
		Sleep(1); // 수신측 부하를 줄이기 위해...
	}

	return true;
}

void CRT_RFT_CAN_IF::readWorker(void)
{
	CONSOLE_S("READ WORKER THREAD STARTED...");

	HRESULT hResult;
	CANMSG  sCanMsg;
	LARGE_INTEGER st, ed, freq;
	::QueryPerformanceFrequency(&freq);

	while (m_bCommReadThreadFlag)
	{
		// read a CAN message from the receive FIFO

		::QueryPerformanceCounter(&st); // get start time.
		hResult = canChannelReadMessage(hCanChn, 100, &sCanMsg);

		if (hResult == VCI_OK)
		{
			if (sCanMsg.uMsgInfo.Bytes.bType == CAN_MSGTYPE_DATA)
			{
				//
				// show data frames
				//
				if (sCanMsg.uMsgInfo.Bits.rtr == 0)
				{
					//UINT8 j;
					//
					//CONSOLE_S("\nTime: %10u  ID: %3X  DLC: %1u  Data:",
					//	sCanMsg.dwTime,
					//	sCanMsg.dwMsgId,
					//	sCanMsg.uMsgInfo.Bits.dlc);
					//for (j = 0; j < sCanMsg.uMsgInfo.Bits.dlc; j++)
					//{
					//	CONSOLE_S(" %.2X", sCanMsg.abData[j]);
					//}

					

					RFT_Data_Handler(sCanMsg);

					::QueryPerformanceCounter(&ed); // get end time
					double passedTime = ((double)ed.QuadPart - st.QuadPart) / ((double)freq.QuadPart);
					
					//CONSOLE_S("can rcv time: %f\n", passedTime);
					//if (passedTime > 0.00600)
					//{
					//	CONSOLE_S("can rcv ooooopsssss: %f\n", passedTime);
					//}
				}
				else
				{
					CONSOLE_S("\nTime: %10u ID: %3X  DLC: %1u  Remote Frame",
						sCanMsg.dwTime,
						sCanMsg.dwMsgId,
						sCanMsg.uMsgInfo.Bits.dlc);
				}
			}
			else if (sCanMsg.uMsgInfo.Bytes.bType == CAN_MSGTYPE_INFO)
			{
				//
				// show informational frames
				//
				switch (sCanMsg.abData[0])
				{
				case CAN_INFO_START: CONSOLE_S("\nCAN started..."); break;
				case CAN_INFO_STOP: CONSOLE_S("\nCAN stoped...");  break;
				case CAN_INFO_RESET: CONSOLE_S("\nCAN reseted..."); break;
				}
			}
			else if (sCanMsg.uMsgInfo.Bytes.bType == CAN_MSGTYPE_ERROR)
			{
				//
				// show error frames
				//
				switch (sCanMsg.abData[0])
				{
				case CAN_ERROR_STUFF: CONSOLE_S("\nstuff error...");          break;
				case CAN_ERROR_FORM: CONSOLE_S("\nform error...");           break;
				case CAN_ERROR_ACK: CONSOLE_S("\nacknowledgment error..."); break;
				case CAN_ERROR_BIT: CONSOLE_S("\nbit error...");            break;
				case CAN_ERROR_CRC: CONSOLE_S("\nCRC error...");            break;
				case CAN_ERROR_OTHER:
				default: printf("\nother error...");          break;
				}
			}
		}

	}

	CONSOLE_S("READ WORKER THREAD FINISHED...");
}


// opMode : STANDARD, EXTENDED, baudrate: buad rate type 참조
bool CRT_RFT_CAN_IF::run(int opMode, int baudrate, bool callback_enable, float forceDivider, float torqueDivider, int txId, int rxId_01, int rxId_02)
{
	bool result = false;

	m_nTxId = txId;
	m_nRxId_01 = rxId_01;
	m_nRxId_02 = rxId_02;

	m_bIsEnabled_Callback = callback_enable;

	m_RFT_IF_PACKET.setDivider(forceDivider, torqueDivider);

	HRESULT hResult = SelectDevice();

	if (hResult == VCI_OK)
	{
		//CAN_OPMODE_STANDARD | CAN_OPMODE_ERRFRAME, CAN_BT0_1000KB, CAN_BT1_1000KB
		UINT8 bMode, bBtr0, bBtr1;

		if (opMode == CAN_STANDARD)
		{
			bMode = CAN_OPMODE_STANDARD | CAN_OPMODE_ERRFRAME;
			m_nCAN_OpMode = CAN_OPMODE_STANDARD;
		}
		else
		{
			bMode = CAN_OPMODE_EXTENDED | CAN_OPMODE_ERRFRAME;
			m_nCAN_OpMode = CAN_OPMODE_EXTENDED;
		}

		bBtr0 = 0;
		bBtr1 = 0;
		getBaudParam(baudrate, &bBtr0, &bBtr1);

		hResult = InitSocket(lCtrlNo, bMode, bBtr0, bBtr1 );

		if (hResult == VCI_OK)
		{
			m_bCommReadThreadFlag = true;
			m_hCommReadThread = ::CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)commReadThread, (LPVOID)this, 0, &m_idCommReadThread);
			if (m_hCommReadThread)
			{
				::SetPriorityClass(m_hCommReadThread, REALTIME_PRIORITY_CLASS);
				::SetThreadPriority(m_hCommReadThread, THREAD_PRIORITY_TIME_CRITICAL);

				result = true;
			}
		}
	}

	return result;
}

void CRT_RFT_CAN_IF::commReadThread(CRT_RFT_CAN_IF *pThis)
{
	pThis->readWorker();
	//::ExitThread( 0 );
}

bool CRT_RFT_CAN_IF::stop(void)
{
	m_hCommReadThread = NULL;
	m_bCommReadThreadFlag = false;

	canControlReset(hCanCtl);
	canChannelClose(hCanChn);
	canControlClose(hCanCtl);

	vciDeviceClose(hDevice);

	return true;
}


void CRT_RFT_CAN_IF::setCallback(RFT_CAN_IF_CALLBACK pCallbackFunc, void *callbackParam)
{
	m_pCallbackFunc = pCallbackFunc;
	m_pCallbackParam = callbackParam;

	CONSOLE_S("SETTING.... RFT CAN I/F CALL BACK FUNCTION\n");
}


void CRT_RFT_CAN_IF::getBaudParam(int baudrate, UINT8 *btr0, UINT8 *btr1)
{
	switch (baudrate)
	{
	case CAN_BAUD_10KBPS:
		*btr0 = CAN_BT0_10KB;
		*btr1 = CAN_BT1_10KB;
		break;
	case CAN_BAUD_20KBPS:
		*btr0 = CAN_BT0_20KB;
		*btr1 = CAN_BT1_20KB;
		break;
	case CAN_BAUD_50KBPS:
		*btr0 = CAN_BT0_50KB;
		*btr1 = CAN_BT1_50KB;
		break;
	case CAN_BAUD_100KBPS:
		*btr0 = CAN_BT0_100KB;
		*btr1 = CAN_BT1_100KB;
		break;
	case CAN_BAUD_125KBPS:
		*btr0 = CAN_BT0_125KB;
		*btr1 = CAN_BT1_125KB;
		break;
	case CAN_BAUD_250KBPS:
		*btr0 = CAN_BT0_250KB;
		*btr1 = CAN_BT1_250KB;
		break;
	case CAN_BAUD_500KBPS:
		*btr0 = CAN_BT0_500KB;
		*btr1 = CAN_BT1_500KB;
		break;
	case CAN_BAUD_800KBPS:
		*btr0 = CAN_BT0_800KB;
		*btr1 = CAN_BT1_800KB;
		break;
	case CAN_BAUD_1000KBPS:
		*btr0 = CAN_BT0_1000KB;
		*btr1 = CAN_BT1_1000KB;
		break;

	default:
		*btr0 = CAN_BT0_1000KB;
		*btr1 = CAN_BT1_1000KB;
		break;
	}
}

HRESULT CRT_RFT_CAN_IF::SelectDevice(void)
{
	//
	HRESULT hResult; // error code
	HANDLE        hEnum;   // enumerator handle
	VCIDEVICEINFO sInfo;   // device info

	//
	// open the device list
	//
	hResult = vciEnumDeviceOpen(&hEnum);

	//
	// retrieve information about the first
	// device within the device list
	//
	if (hResult == VCI_OK)
	{
		hResult = vciEnumDeviceNext(hEnum, &sInfo);
	}

	//
	// close the device list (no longer needed)
	//
	vciEnumDeviceClose(hEnum);

	//
	// open the device
	//
	if (hResult == VCI_OK)
	{
		//hResult = vciDeviceOpen(&sInfo.VciObjectId, &hDevice);

		// REFVCIID가 C++에서는 pointer가 아니고, reference로 선언되었네..
		//#if defined(__cplusplus)
		//		typedef const VCIID& REFVCIID;
		//#else
		//		typedef const VCIID* const REFVCIID;
		//#endif
		hResult = vciDeviceOpen(sInfo.VciObjectId, &hDevice);
	}

	//
	// always select controller 0
	//
	lCtrlNo = 0;

	DisplayError(hResult);
	return hResult;
}

HRESULT CRT_RFT_CAN_IF::InitSocket(UINT32 dwCanNo, UINT8  bMode, UINT8  bBtr0, UINT8  bBtr1)
{
	HRESULT hResult;

	//
	// create a message channel
	//
	if (hDevice != NULL)
	{
		//
		// create and initialize a message channel
		//
		hResult = canChannelOpen(hDevice, dwCanNo, FALSE, &hCanChn);

		//
		// initialize the message channel
		//
		if (hResult == VCI_OK)
		{
			UINT16 wRxFifoSize = 1024;
			UINT16 wRxThreshold = 1;
			UINT16 wTxFifoSize = 128;
			UINT16 wTxThreshold = 1;

			hResult = canChannelInitialize(hCanChn,
				wRxFifoSize, wRxThreshold,
				wTxFifoSize, wTxThreshold);
		}

		//
		// activate the CAN channel
		//
		if (hResult == VCI_OK)
		{
			hResult = canChannelActivate(hCanChn, TRUE);
		}

		//
		// open the CAN controller
		//
		if (hResult == VCI_OK)
		{
			hResult = canControlOpen(hDevice, dwCanNo, &hCanCtl);
			// this function fails if the controller is in use
			// by another application.
		}

		//
		// initialize the CAN controller
		//
		if (hResult == VCI_OK)
		{
			//hResult = canControlInitialize(hCanCtl, CAN_OPMODE_STANDARD | CAN_OPMODE_ERRFRAME, CAN_BT0_1000KB, CAN_BT1_1000KB);
			hResult = canControlInitialize(hCanCtl, bMode, bBtr0, bBtr1);
		}

		//
		// set the acceptance filter
		//
		if (hResult == VCI_OK)
		{
			hResult = canControlSetAccFilter(hCanCtl, FALSE,
				CAN_ACC_CODE_ALL, CAN_ACC_MASK_ALL);
		}

		//
		// start the CAN controller
		//
		if (hResult == VCI_OK)
		{
			hResult = canControlStart(hCanCtl, TRUE);
		}
	}
	else
	{
		hResult = VCI_E_INVHANDLE;
	}

	DisplayError(hResult);
	return hResult;
}

void CRT_RFT_CAN_IF::DisplayError(HRESULT hResult)
{
	char szError[VCI_MAX_ERRSTRLEN];

	if (hResult != NO_ERROR)
	{
		if (hResult == -1)
			hResult = GetLastError();

		szError[0] = 0;
		vciFormatError(hResult, szError, sizeof(szError));
		MessageBox(NULL, szError, "RFT INTERFACE CAN", MB_OK | MB_ICONSTOP);
	}
}

void CRT_RFT_CAN_IF::initialize_variables(void)
{
	hDevice = NULL;       // device handle
	lCtrlNo = 0;       // controller number
	hCanCtl = NULL;       // controller handle 
	hCanChn = NULL;       // channel handle


	m_hCommReadThread = NULL;         // Handle of Thread
	m_idCommReadThread = 0;        // ID of Thread
	m_bCommReadThreadFlag = false;
	
	m_pCallbackFunc = NULL;
	m_pCallbackParam = NULL;

	m_nCAN_OpMode = CAN_OPMODE_STANDARD;

	m_nTxId = 100;
	m_nRxId_01 = 1;
	m_nRxId_02 = 2;

	m_nCurrMode = CMD_NONE;
	m_nRcvdPktCnt_AtCurrMode = 0; // 모드가 변경되므로 초기화...
	m_nRcvdData_ValidLength = 0;

	m_bIsRcvd_Response_Pkt = false;

	memset(m_rcvdDataBuff, 0x00, RFT_RCVD_DATA_BUFF_MAX_SIZE);

	// for logging
	m_pLogFile = NULL;
	m_nlogDataCount = 0;
}



void CRT_RFT_CAN_IF::rcvdCAN_ProduectName(CANMSG msg)
{
	for (int i = 0; i < CAN_DATA_LENGTH; i++)
	{
		m_rcvdDataBuff[m_nRcvdData_ValidLength] = msg.abData[i];
		m_nRcvdData_ValidLength++;
	}

	if (m_rcvdDataBuff[0] == CMD_GET_PRODUCT_NAME) 
	{
		m_nRcvdPktCnt_AtCurrMode++;
	}
	else
	{
		m_nRcvdData_ValidLength = 0;
		m_nRcvdPktCnt_AtCurrMode = 0;
	}

	if (m_nRcvdPktCnt_AtCurrMode == RESPONSE_CAN_PACKET_CNT)
	{
		m_rcvdDataBuff[m_nRcvdData_ValidLength] = 0x00; 

		m_bIsRcvd_Response_Pkt = m_RFT_IF_PACKET.rcvd_data_field_processing(m_rcvdDataBuff, CMD_GET_PRODUCT_NAME);

		m_nRcvdPktCnt_AtCurrMode = 0;
		m_nRcvdData_ValidLength = 0;
	}
}

void CRT_RFT_CAN_IF::rcvdCAN_SerialNumber(CANMSG msg)
{
	for (int i = 0; i < CAN_DATA_LENGTH; i++)
	{
		m_rcvdDataBuff[m_nRcvdData_ValidLength] = msg.abData[i];
		m_nRcvdData_ValidLength++;
	}

	if (m_rcvdDataBuff[0] == CMD_GET_SERIAL_NUMBER) 
	{
		m_nRcvdPktCnt_AtCurrMode++;
	}
	else
	{
		m_nRcvdData_ValidLength = 0;
		m_nRcvdPktCnt_AtCurrMode = 0;
	}

	if (m_nRcvdPktCnt_AtCurrMode == RESPONSE_CAN_PACKET_CNT)
	{
		m_rcvdDataBuff[m_nRcvdData_ValidLength] = 0x00; 

		m_bIsRcvd_Response_Pkt = m_RFT_IF_PACKET.rcvd_data_field_processing(m_rcvdDataBuff, CMD_GET_SERIAL_NUMBER);

		m_nRcvdPktCnt_AtCurrMode = 0;
		m_nRcvdData_ValidLength = 0;
	}
}

void CRT_RFT_CAN_IF::rcvdCAN_Firmwareversion(CANMSG msg)
{
	for (int i = 0; i < CAN_DATA_LENGTH; i++)
	{
		m_rcvdDataBuff[m_nRcvdData_ValidLength] = msg.abData[i];
		m_nRcvdData_ValidLength++;
	}

	if (m_rcvdDataBuff[0] == CMD_GET_FIRMWARE_VER) 
	{
		m_nRcvdPktCnt_AtCurrMode++;
	}
	else
	{
		m_nRcvdData_ValidLength = 0;
		m_nRcvdPktCnt_AtCurrMode = 0;
	}

	if (m_nRcvdPktCnt_AtCurrMode == RESPONSE_CAN_PACKET_CNT)
	{
		m_rcvdDataBuff[m_nRcvdData_ValidLength] = 0x00; 

		m_bIsRcvd_Response_Pkt = m_RFT_IF_PACKET.rcvd_data_field_processing(m_rcvdDataBuff, CMD_GET_FIRMWARE_VER);

		m_nRcvdPktCnt_AtCurrMode = 0;
		m_nRcvdData_ValidLength = 0;
	}
}

void CRT_RFT_CAN_IF::rcvdCAN_ID(CANMSG msg)
{
	for (int i = 0; i < CAN_DATA_LENGTH; i++)
	{
		m_rcvdDataBuff[m_nRcvdData_ValidLength] = msg.abData[i];
		m_nRcvdData_ValidLength++;
	}

	if (m_rcvdDataBuff[0] == CMD_GET_ID) 
	{
		m_nRcvdPktCnt_AtCurrMode++;
	}
	else
	{
		m_nRcvdData_ValidLength = 0;
		m_nRcvdPktCnt_AtCurrMode = 0;
	}

	if (m_nRcvdPktCnt_AtCurrMode == RESPONSE_CAN_PACKET_CNT)
	{
		m_rcvdDataBuff[m_nRcvdData_ValidLength] = 0x00;

		m_bIsRcvd_Response_Pkt = m_RFT_IF_PACKET.rcvd_data_field_processing(m_rcvdDataBuff, CMD_GET_ID);

		m_nRcvdPktCnt_AtCurrMode = 0;
		m_nRcvdData_ValidLength = 0;
	}
}


void CRT_RFT_CAN_IF::rcvdCAN_FT_Cont_Interval(CANMSG msg)
{
	for (int i = 0; i < CAN_DATA_LENGTH; i++)
	{
		m_rcvdDataBuff[m_nRcvdData_ValidLength] = msg.abData[i];
		m_nRcvdData_ValidLength++;
	}

	if (m_rcvdDataBuff[0] == CMD_GET_CONT_OUT_FRQ) 
	{
		m_nRcvdPktCnt_AtCurrMode++;
	}
	else
	{
		m_nRcvdData_ValidLength = 0;
		m_nRcvdPktCnt_AtCurrMode = 0;
	}

	if (m_nRcvdPktCnt_AtCurrMode == RESPONSE_CAN_PACKET_CNT)
	{
		m_rcvdDataBuff[m_nRcvdData_ValidLength] = 0x00;

		m_bIsRcvd_Response_Pkt = m_RFT_IF_PACKET.rcvd_data_field_processing(m_rcvdDataBuff, CMD_GET_CONT_OUT_FRQ);

		m_nRcvdPktCnt_AtCurrMode = 0;
		m_nRcvdData_ValidLength = 0;
	}
}

void CRT_RFT_CAN_IF::rcvdCAN_FT_Overload_Count(CANMSG msg)
{
	for (int i = 0; i < CAN_DATA_LENGTH; i++)
	{
		m_rcvdDataBuff[m_nRcvdData_ValidLength] = msg.abData[i];
		m_nRcvdData_ValidLength++;
	}

	if (m_rcvdDataBuff[0] == CMD_GET_OVERLOAD_COUNT) 
	{
		m_nRcvdPktCnt_AtCurrMode++;
	}
	else
	{
		m_nRcvdData_ValidLength = 0;
		m_nRcvdPktCnt_AtCurrMode = 0;
	}

	if (m_nRcvdPktCnt_AtCurrMode == RESPONSE_CAN_PACKET_CNT)
	{
		m_rcvdDataBuff[m_nRcvdData_ValidLength] = 0x00;

		m_bIsRcvd_Response_Pkt = m_RFT_IF_PACKET.rcvd_data_field_processing(m_rcvdDataBuff, CMD_GET_OVERLOAD_COUNT);

		m_nRcvdPktCnt_AtCurrMode = 0;
		m_nRcvdData_ValidLength = 0;
	}
}

void CRT_RFT_CAN_IF::rcvdCAN_FT_Filter_Type(CANMSG msg)
{
	for (int i = 0; i < CAN_DATA_LENGTH; i++)
	{
		m_rcvdDataBuff[m_nRcvdData_ValidLength] = msg.abData[i];
		m_nRcvdData_ValidLength++;
	}
	m_nRcvdPktCnt_AtCurrMode++;

	if (m_rcvdDataBuff[0] == CMD_GET_FT_FILTER) 
	{
		m_nRcvdPktCnt_AtCurrMode++;
	}
	else
	{
		m_nRcvdData_ValidLength = 0;
		m_nRcvdPktCnt_AtCurrMode = 0;
	}

	if (m_nRcvdPktCnt_AtCurrMode == RESPONSE_CAN_PACKET_CNT)
	{
		m_rcvdDataBuff[m_nRcvdData_ValidLength] = 0x00;

		m_bIsRcvd_Response_Pkt = m_RFT_IF_PACKET.rcvd_data_field_processing(m_rcvdDataBuff, CMD_GET_FT_FILTER);

		m_nRcvdPktCnt_AtCurrMode = 0;
		m_nRcvdData_ValidLength = 0;
	}
}

void CRT_RFT_CAN_IF::rcvdCAN_FT(CANMSG msg)
{
	
	if (((m_nRcvdPktCnt_AtCurrMode == 0) && (msg.dwMsgId != m_nRxId_01)) ||
		((m_nRcvdPktCnt_AtCurrMode == 1) && (msg.dwMsgId != m_nRxId_02)))
	{
		return;
	}


	for (int i = 0; i < CAN_DATA_LENGTH; i++)
	{
		m_rcvdDataBuff[m_nRcvdData_ValidLength] = msg.abData[i];
		m_nRcvdData_ValidLength++;
	}
	
	if ((m_rcvdDataBuff[0] == CMD_FT_ONCE) || (m_rcvdDataBuff[0] == CMD_FT_CONT)) 
	{
		m_nRcvdPktCnt_AtCurrMode++;
	}
	else
	{
		m_nRcvdData_ValidLength = 0;
		m_nRcvdPktCnt_AtCurrMode = 0;
	}


	if (m_nRcvdPktCnt_AtCurrMode == RESPONSE_CAN_PACKET_CNT)
	{
		m_rcvdDataBuff[m_nRcvdData_ValidLength] = 0x00;

		m_nRcvdPktCnt_AtCurrMode = 0;
		m_nRcvdData_ValidLength = 0;

		m_bIsRcvd_Response_Pkt = m_RFT_IF_PACKET.rcvd_data_field_processing(m_rcvdDataBuff, m_rcvdDataBuff[0]);

		if (m_bIsRcvd_Response_Pkt)
		{
			//....
			if (m_bIsEnabled_Callback)
				m_pCallbackFunc(m_pCallbackParam);

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
}



void CRT_RFT_CAN_IF::rcvdCAN_Response_Set_ID(CANMSG msg)
{
	for (int i = 0; i < CAN_DATA_LENGTH; i++)
	{
		m_rcvdDataBuff[m_nRcvdData_ValidLength] = msg.abData[i];
		m_nRcvdData_ValidLength++;
	}
	
	if (m_rcvdDataBuff[0] == CMD_SET_ID) 
	{
		m_nRcvdPktCnt_AtCurrMode++;
	}
	else
	{
		m_nRcvdData_ValidLength = 0;
		m_nRcvdPktCnt_AtCurrMode = 0;
	}

	if (m_nRcvdPktCnt_AtCurrMode == RESPONSE_CAN_PACKET_CNT)
	{
		m_rcvdDataBuff[m_nRcvdData_ValidLength] = 0x00;

		m_bIsRcvd_Response_Pkt = m_RFT_IF_PACKET.rcvd_data_field_processing(m_rcvdDataBuff, CMD_SET_ID);

		m_nRcvdPktCnt_AtCurrMode = 0;
		m_nRcvdData_ValidLength = 0;
	}
}

void CRT_RFT_CAN_IF::rcvdCAN_Response_Set_FT_Cont_Interval(CANMSG msg)
{
	for (int i = 0; i < CAN_DATA_LENGTH; i++)
	{
		m_rcvdDataBuff[m_nRcvdData_ValidLength] = msg.abData[i];
		m_nRcvdData_ValidLength++;
	}


	if (m_rcvdDataBuff[0] == CMD_SET_CONT_OUT_FRQ) 
	{
		m_nRcvdPktCnt_AtCurrMode++;
	}
	else
	{
		m_nRcvdData_ValidLength = 0;
		m_nRcvdPktCnt_AtCurrMode = 0;
	}

	if (m_nRcvdPktCnt_AtCurrMode == RESPONSE_CAN_PACKET_CNT)
	{
		m_rcvdDataBuff[m_nRcvdData_ValidLength] = 0x00;

		m_bIsRcvd_Response_Pkt = m_RFT_IF_PACKET.rcvd_data_field_processing(m_rcvdDataBuff, CMD_SET_CONT_OUT_FRQ);

		m_nRcvdPktCnt_AtCurrMode = 0;
		m_nRcvdData_ValidLength = 0;
	}
}

void CRT_RFT_CAN_IF::rcvdCAN_Response_Set_FT_Filter_Type(CANMSG msg)
{
	for (int i = 0; i < CAN_DATA_LENGTH; i++)
	{
		m_rcvdDataBuff[m_nRcvdData_ValidLength] = msg.abData[i];
		m_nRcvdData_ValidLength++;
	}


	if (m_rcvdDataBuff[0] == CMD_SET_FT_FILTER) 
	{
		m_nRcvdPktCnt_AtCurrMode++;
	}
	else
	{
		m_nRcvdData_ValidLength = 0;
		m_nRcvdPktCnt_AtCurrMode = 0;
	}

	if (m_nRcvdPktCnt_AtCurrMode == 2)
	{
		m_rcvdDataBuff[m_nRcvdData_ValidLength] = 0x00;

		m_bIsRcvd_Response_Pkt = m_RFT_IF_PACKET.rcvd_data_field_processing(m_rcvdDataBuff, CMD_SET_FT_FILTER);

		m_nRcvdPktCnt_AtCurrMode = 0;
		m_nRcvdData_ValidLength = 0;
	}
}


void CRT_RFT_CAN_IF::RFT_Data_Handler(CANMSG msg)
{
	// skip the CAN message of the tx id 
	if (msg.dwMsgId == m_nTxId)
		return;

	switch (m_nCurrMode)
	{
	case CMD_GET_PRODUCT_NAME:
		rcvdCAN_ProduectName(msg);
		break;

	case CMD_GET_SERIAL_NUMBER:
		rcvdCAN_SerialNumber(msg);
		break;

	case CMD_GET_FIRMWARE_VER:
		rcvdCAN_Firmwareversion(msg);
		break;

	case CMD_SET_ID:
		rcvdCAN_Response_Set_ID(msg);
		break;

	case CMD_GET_ID:
		rcvdCAN_ID(msg);
		break;

	case CMD_SET_FT_FILTER:
		rcvdCAN_Response_Set_FT_Filter_Type(msg);
		break;

	case CMD_GET_FT_FILTER:
		rcvdCAN_FT_Filter_Type(msg);
		break;

	case CMD_FT_ONCE:
		rcvdCAN_FT(msg);
		break;

	case CMD_FT_CONT:
		rcvdCAN_FT(msg);
		break;

	case CMD_FT_CONT_STOP:
		break;

	case CMD_SET_CONT_OUT_FRQ:
		rcvdCAN_Response_Set_FT_Cont_Interval(msg);
		break;

	case CMD_GET_CONT_OUT_FRQ:
		rcvdCAN_FT_Cont_Interval(msg);
		break;

	case CMD_GET_OVERLOAD_COUNT:
		rcvdCAN_FT_Overload_Count(msg);
		break;

	default: break;
	}
}

bool CRT_RFT_CAN_IF::rqst_ProductName(int txId)
{
	unsigned char buff[CAN_DATA_LENGTH];
	memset(buff, 0x00, CAN_DATA_LENGTH);

	m_RFT_IF_PACKET.DFG_read_product_name(buff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = buff[0];
	m_nRcvdPktCnt_AtCurrMode = 0; // 모드가 변경되므로 초기화...
	m_nRcvdData_ValidLength = 0;

	return sendData(txId, buff, CAN_DATA_LENGTH);
}

bool CRT_RFT_CAN_IF::rqst_SerialNumber(int txId)
{
	unsigned char buff[CAN_DATA_LENGTH];
	memset(buff, 0x00, CAN_DATA_LENGTH);

	m_RFT_IF_PACKET.DFG_read_serial_number(buff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = buff[0];
	m_nRcvdPktCnt_AtCurrMode = 0; 
	m_nRcvdData_ValidLength = 0;

	return sendData(txId, buff, CAN_DATA_LENGTH);
}

bool CRT_RFT_CAN_IF::rqst_Firmwareverion(int txId)
{
	unsigned char buff[CAN_DATA_LENGTH];
	memset(buff, 0x00, CAN_DATA_LENGTH);

	m_RFT_IF_PACKET.DFG_read_firmware_version(buff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = buff[0];
	m_nRcvdPktCnt_AtCurrMode = 0; 
	m_nRcvdData_ValidLength = 0;

	return sendData(txId, buff, CAN_DATA_LENGTH);
}

bool CRT_RFT_CAN_IF::rqst_GetID(int txId)
{
	unsigned char buff[CAN_DATA_LENGTH];
	memset(buff, 0x00, CAN_DATA_LENGTH);

	m_RFT_IF_PACKET.DFG_read_message_ID(buff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = buff[0];
	m_nRcvdPktCnt_AtCurrMode = 0; 
	m_nRcvdData_ValidLength = 0;

	return sendData(txId, buff, CAN_DATA_LENGTH);
}


bool CRT_RFT_CAN_IF::rqst_FT_Filter_Type(int txId)
{
	unsigned char buff[CAN_DATA_LENGTH];
	memset(buff, 0x00, CAN_DATA_LENGTH);

	m_RFT_IF_PACKET.DFG_read_filter_type(buff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = buff[0];
	m_nRcvdPktCnt_AtCurrMode = 0; 
	m_nRcvdData_ValidLength = 0;

	return sendData(txId, buff, CAN_DATA_LENGTH);
}

bool CRT_RFT_CAN_IF::rqst_FT(int txId)
{
	unsigned char buff[CAN_DATA_LENGTH];
	memset(buff, 0x00, CAN_DATA_LENGTH);

	m_RFT_IF_PACKET.DFG_read_force_once(buff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = buff[0];
	m_nRcvdPktCnt_AtCurrMode = 0; 
	m_nRcvdData_ValidLength = 0;

	return sendData(txId, buff, CAN_DATA_LENGTH);
}

bool CRT_RFT_CAN_IF::rqst_FT_Continuous(int txId)
{
	unsigned char buff[CAN_DATA_LENGTH];
	memset(buff, 0x00, CAN_DATA_LENGTH);

	m_RFT_IF_PACKET.DFG_read_force(buff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = buff[0];
	m_nRcvdPktCnt_AtCurrMode = 0; 
	m_nRcvdData_ValidLength = 0;

	return sendData(txId, buff, CAN_DATA_LENGTH);
}

bool CRT_RFT_CAN_IF::rqst_FT_Stop(int txId)
{
	unsigned char buff[CAN_DATA_LENGTH];
	memset(buff, 0x00, CAN_DATA_LENGTH);

	m_RFT_IF_PACKET.DFG_set_stop_force_out(buff);

	m_bIsRcvd_Response_Pkt = true; // there is no response packet
	m_nCurrMode = CMD_NONE;        
	m_nRcvdPktCnt_AtCurrMode = 0; 
	m_nRcvdData_ValidLength = 0;

	return sendData(txId, buff, CAN_DATA_LENGTH);
}

bool CRT_RFT_CAN_IF::rqst_FT_Cont_Interval(int txId)
{
	unsigned char buff[CAN_DATA_LENGTH];
	memset(buff, 0x00, CAN_DATA_LENGTH);

	m_RFT_IF_PACKET.DFG_read_output_frq(buff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = buff[0];
	m_nRcvdPktCnt_AtCurrMode = 0; 
	m_nRcvdData_ValidLength = 0;

	return sendData(txId, buff, CAN_DATA_LENGTH);
}


bool CRT_RFT_CAN_IF::rqst_FT_OverloadCnt(int txId)
{
	unsigned char buff[CAN_DATA_LENGTH];
	memset(buff, 0x00, CAN_DATA_LENGTH);

	m_RFT_IF_PACKET.DFG_read_overload_count(buff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = buff[0];
	m_nRcvdPktCnt_AtCurrMode = 0; 
	m_nRcvdData_ValidLength = 0;

	return sendData(txId, buff, CAN_DATA_LENGTH);
}


bool CRT_RFT_CAN_IF::set_ID(int txId, int ftTxID, int ftRxID_01, int ftRxID_02)
{
	unsigned char buff[CAN_DATA_LENGTH];
	memset(buff, 0x00, CAN_DATA_LENGTH);

	m_RFT_IF_PACKET.DFG_set_message_ID(buff, ftTxID, ftRxID_01, ftRxID_02);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = buff[0];
	m_nRcvdPktCnt_AtCurrMode = 0; 
	m_nRcvdData_ValidLength = 0;

	return sendData(txId, buff, CAN_DATA_LENGTH);
}


bool CRT_RFT_CAN_IF::set_FT_Filter_Type(int txId, int filter_type, int sub_type)
{
	unsigned char buff[CAN_DATA_LENGTH];
	memset(buff, 0x00, CAN_DATA_LENGTH);

	m_RFT_IF_PACKET.DFG_set_filter_type(buff, filter_type, sub_type);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = buff[0];
	m_nRcvdPktCnt_AtCurrMode = 0; 
	m_nRcvdData_ValidLength = 0;

	return sendData(txId, buff, CAN_DATA_LENGTH);
}

bool CRT_RFT_CAN_IF::set_FT_Cont_Interval(int txId, int interval)
{
	unsigned char buff[CAN_DATA_LENGTH];
	memset(buff, 0x00, CAN_DATA_LENGTH);

	m_RFT_IF_PACKET.DFG_set_output_frq(buff, interval);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = buff[0];
	m_nRcvdPktCnt_AtCurrMode = 0; 
	m_nRcvdData_ValidLength = 0;

	return sendData(txId, buff, CAN_DATA_LENGTH);
}

bool CRT_RFT_CAN_IF::set_FT_Bias(int txId, int is_on)
{
	unsigned char buff[CAN_DATA_LENGTH];
	memset(buff, 0x00, CAN_DATA_LENGTH);

	m_RFT_IF_PACKET.DFG_set_bias(buff, is_on);

	// Bias setting command don't have response packet
	//m_bIsRcvd_Response_Pkt = false;
	//m_nCurrMode = buff[0];
	//m_nRcvdPktCnt_AtCurrMode = 0;  
	//m_nRcvdData_ValidLength = 0;

	return sendData(txId, buff, CAN_DATA_LENGTH);
}


bool CRT_RFT_CAN_IF::sendMultiPkt(int txId, unsigned char *dataBuff, int length)
{
	unsigned char buff[CAN_DATA_LENGTH];
	memset(buff, 0x00, CAN_DATA_LENGTH);

	int txCnt = length / CAN_DATA_LENGTH;
	int remain = length % CAN_DATA_LENGTH;

	int i, j;
	for (i = 0; i < txCnt; i++)
	{
		memset(buff, 0x00, CAN_DATA_LENGTH);
		for (j = 0; j < CAN_DATA_LENGTH; j++)
		{
			buff[j] = dataBuff[i*CAN_DATA_LENGTH + j];
		}

		sendData(txId, buff, CAN_DATA_LENGTH);
		Sleep(1);
	}

	if (remain > 0)
	{
		memset(buff, 0x00, CAN_DATA_LENGTH);
		for (j = 0; j < remain; j++)
		{
			buff[j] = dataBuff[i*CAN_DATA_LENGTH + j]; 
		}
		for (j = remain; j < CAN_DATA_LENGTH; j++)
		{
			buff[j] = dataBuff[i*CAN_DATA_LENGTH + j];
		}

		sendData(txId, buff, CAN_DATA_LENGTH);
		Sleep(1);
	}

	return true;
}

bool CRT_RFT_CAN_IF::startLogging(CString logfileName)
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


bool CRT_RFT_CAN_IF::stopLogging(void)
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

