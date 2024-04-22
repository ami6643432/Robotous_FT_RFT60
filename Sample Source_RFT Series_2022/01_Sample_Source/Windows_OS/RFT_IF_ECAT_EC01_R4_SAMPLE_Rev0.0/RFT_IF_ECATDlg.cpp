
// RFT_IF_ECATDlg.cpp : ���� ����
//

#include "stdafx.h"
#include "RFT_IF_ECAT.h"
#include "RFT_IF_ECATDlg.h"
#include "afxdialogex.h"

#include "RT_Console_Rev0.1.h"
#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")

LARGE_INTEGER g_st, g_ed, g_freq;

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#define NUM_OF_SAMPLE_FOR_GRAPH (500)
#define WAIT_TIMEOUT_RFT	(100)	// for waiting response packet
#define WAIT_SLEEP_TIME_RFT (50)	// for waiting response packet


// ���� ���α׷� ������ ���Ǵ� CAboutDlg ��ȭ �����Դϴ�.

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// ��ȭ ���� �������Դϴ�.
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV �����Դϴ�.

// �����Դϴ�.
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CRFT_IF_ECATDlg ��ȭ ����



CRFT_IF_ECATDlg::CRFT_IF_ECATDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CRFT_IF_ECATDlg::IDD, pParent)
	, m_strRxData(_T(""))
	, m_fDivider_Force(50.0)
	, m_fDivider_Torque(2000.0)
	, m_nSlaveId(1)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CRFT_IF_ECATDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_COMBO_NIC_INFO, m_nic_info);
	DDX_Text(pDX, IDC_EDIT_RX_DATA, m_strRxData);
	DDX_Control(pDX, IDC_CUSTOM_CHART_FORCE, m_ChartCtrl_Force);
	DDX_Control(pDX, IDC_CUSTOM_CHART_MOMENT, m_ChartCtrl_Torque); //
	DDX_Control(pDX, IDC_COMBO_FILTER_TYPE, m_combo_Filter_Cutoff_Frq);
	DDX_Text(pDX, IDC_EDIT_DIVIDER_FORCE, m_fDivider_Force);
	DDX_Text(pDX, IDC_EDIT_DIVIDER_TORQUE, m_fDivider_Torque);
	DDX_Text(pDX, IDC_EDIT_TX_ID, m_nSlaveId);
}

BEGIN_MESSAGE_MAP(CRFT_IF_ECATDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_WM_DESTROY()
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_CHECK_INTERFACE_OPEN, &CRFT_IF_ECATDlg::OnBnClickedCheckInterfaceOpen)
	ON_BN_CLICKED(IDC_CHECK_BIAS, &CRFT_IF_ECATDlg::OnBnClickedCheckBias)
	ON_BN_CLICKED(IDC_CHECK_FT_OUT_CONT, &CRFT_IF_ECATDlg::OnBnClickedCheckFtOutCont)
	ON_BN_CLICKED(IDC_READ_OVERLOAD_COUNT, &CRFT_IF_ECATDlg::OnBnClickedReadOverloadCount)
	ON_BN_CLICKED(IDC_BUTTON_FILTER_SETTING, &CRFT_IF_ECATDlg::OnBnClickedButtonFilterSetting)
	ON_BN_CLICKED(IDC_CHECK_DATA_LOGGING, &CRFT_IF_ECATDlg::OnBnClickedCheckDataLogging)
END_MESSAGE_MAP()


// CRFT_IF_ECATDlg �޽��� ó����

BOOL CRFT_IF_ECATDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// �ý��� �޴��� "����..." �޴� �׸��� �߰��մϴ�.
	///////////////////////////////////////////////////////////////////////////////
	// Process priority setting
	if (::SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS)) // �� ���α׷��� �ְ� �����̾�Ƽ Ŭ������...
	{
		CONSOLE_S(CONSOLE_RED, "\n======== Priority setting OK.. =========\n");
	}
	// priority ������ �߸� �Ǿ�����... 
	timeBeginPeriod(1); // increase time resolution



	// IDM_ABOUTBOX�� �ý��� ��� ������ �־�� �մϴ�.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// �� ��ȭ ������ �������� �����մϴ�.  ���� ���α׷��� �� â�� ��ȭ ���ڰ� �ƴ� ��쿡��
	//  �����ӿ�ũ�� �� �۾��� �ڵ����� �����մϴ�.
	SetIcon(m_hIcon, TRUE);			// ū �������� �����մϴ�.
	SetIcon(m_hIcon, FALSE);		// ���� �������� �����մϴ�.

	// TODO: ���⿡ �߰� �ʱ�ȭ �۾��� �߰��մϴ�.
	int numOfNic = m_RFT_IF.m_vecNIC_Desc.size();
	for (int i = 0; i < numOfNic; i++)
	{
		CString data = m_RFT_IF.m_vecNIC_Desc[i] + " : " + m_RFT_IF.m_vecNic_Name[i];
		m_nic_info.InsertString(i, data.GetBuffer());
	}
	m_nic_info.SetCurSel(0);

	m_combo_Filter_Cutoff_Frq.InsertString(0, "No filtering");
	m_combo_Filter_Cutoff_Frq.InsertString(1, "500Hz");
	m_combo_Filter_Cutoff_Frq.InsertString(2, "300Hz");
	m_combo_Filter_Cutoff_Frq.InsertString(3, "200Hz");
	m_combo_Filter_Cutoff_Frq.InsertString(4, "150Hz");
	m_combo_Filter_Cutoff_Frq.InsertString(5, "100Hz");
	m_combo_Filter_Cutoff_Frq.InsertString(6, "50Hz");
	m_combo_Filter_Cutoff_Frq.InsertString(7, "40Hz");
	m_combo_Filter_Cutoff_Frq.InsertString(8, "30Hz");
	m_combo_Filter_Cutoff_Frq.InsertString(9, "20Hz");
	m_combo_Filter_Cutoff_Frq.InsertString(10, "10Hz");
	m_combo_Filter_Cutoff_Frq.InsertString(11, "5Hz");
	m_combo_Filter_Cutoff_Frq.InsertString(12, "3Hz");
	m_combo_Filter_Cutoff_Frq.InsertString(13, "2Hz");
	m_combo_Filter_Cutoff_Frq.InsertString(14, "1Hz");
	m_combo_Filter_Cutoff_Frq.SetCurSel(0);

	GetDlgItem(IDC_CHECK_FT_OUT_CONT)->EnableWindow(FALSE);
	GetDlgItem(IDC_CHECK_BIAS)->EnableWindow(FALSE);
	GetDlgItem(IDC_READ_OVERLOAD_COUNT)->EnableWindow(FALSE);
	GetDlgItem(IDC_BUTTON_FILTER_SETTING)->EnableWindow(FALSE);
	GetDlgItem(IDC_CHECK_DATA_LOGGING)->EnableWindow(FALSE);

	// callback function setting
	m_RFT_IF.setCallback(callback_RFT_Data_Receive, this);

	// for graph
	CChartStandardAxis* pBottomAxis = m_ChartCtrl_Force.CreateStandardAxis(CChartCtrl::BottomAxis);
	pBottomAxis->SetMinMax(0, NUM_OF_SAMPLE_FOR_GRAPH);
	pBottomAxis->SetAutomatic(true);
	CChartStandardAxis* pLeftAxis = m_ChartCtrl_Force.CreateStandardAxis(CChartCtrl::LeftAxis);
	pLeftAxis->SetMinMax(-10, 10);
	pLeftAxis->SetAutomatic(true);
	m_ChartCtrl_Force.SetBackColor(RGB(255,255,255)); // white back-ground color
	m_ChartCtrl_Force.GetLegend()->SetVisible(true);

	m_pGraph_F[0] = m_ChartCtrl_Force.CreateLineSerie(false, false);
	m_pGraph_F[0]->SetWidth(1);
	m_pGraph_F[0]->SetPenStyle(0);
	m_pGraph_F[0]->SetName("Fx[N]");
	m_pGraph_F[0]->SetColor(RGB(255, 0, 0));

	m_pGraph_F[1] = m_ChartCtrl_Force.CreateLineSerie(false, false);
	m_pGraph_F[1]->SetWidth(1);
	m_pGraph_F[1]->SetPenStyle(0);
	m_pGraph_F[1]->SetName("Fy[N]");
	m_pGraph_F[1]->SetColor(RGB(0, 255, 0));

	m_pGraph_F[2] = m_ChartCtrl_Force.CreateLineSerie(false, false);
	m_pGraph_F[2]->SetWidth(1);
	m_pGraph_F[2]->SetPenStyle(0);
	m_pGraph_F[2]->SetName("Fz[N]");
	m_pGraph_F[2]->SetColor(RGB(0, 0, 255));


	CChartStandardAxis* pBottomAxis2 = m_ChartCtrl_Torque.CreateStandardAxis(CChartCtrl::BottomAxis);
	pBottomAxis2->SetMinMax(0, NUM_OF_SAMPLE_FOR_GRAPH);
	pBottomAxis2->SetAutomatic(true);
	CChartStandardAxis* pLeftAxis2 = m_ChartCtrl_Torque.CreateStandardAxis(CChartCtrl::LeftAxis);
	pLeftAxis2->SetMinMax(-10, 10);
	pLeftAxis2->SetAutomatic(true);
	m_ChartCtrl_Torque.SetBackColor(RGB(255, 255, 255)); // white back-ground color
	m_ChartCtrl_Torque.GetLegend()->SetVisible(true);

	m_pGraph_F[3] = m_ChartCtrl_Torque.CreateLineSerie(false, false);
	m_pGraph_F[3]->SetWidth(1);
	m_pGraph_F[3]->SetPenStyle(0);
	m_pGraph_F[3]->SetName("Tx[Nm]");
	m_pGraph_F[3]->SetColor(RGB(255, 0, 0));

	m_pGraph_F[4] = m_ChartCtrl_Torque.CreateLineSerie(false, false);
	m_pGraph_F[4]->SetWidth(1);
	m_pGraph_F[4]->SetPenStyle(0);
	m_pGraph_F[4]->SetName("Ty[Nm]");
	m_pGraph_F[4]->SetColor(RGB(0, 255, 0));

	m_pGraph_F[5] = m_ChartCtrl_Torque.CreateLineSerie(false, false);
	m_pGraph_F[5]->SetWidth(1);
	m_pGraph_F[5]->SetPenStyle(0);
	m_pGraph_F[5]->SetName("Tz[Nm]");
	m_pGraph_F[5]->SetColor(RGB(0, 0, 255));

	::QueryPerformanceFrequency(&g_freq);

	SetTimer(0, 100, NULL);

	return TRUE;  // ��Ŀ���� ��Ʈ�ѿ� �������� ������ TRUE�� ��ȯ�մϴ�.
}

void CRFT_IF_ECATDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// ��ȭ ���ڿ� �ּ�ȭ ���߸� �߰��� ��� �������� �׸�����
//  �Ʒ� �ڵ尡 �ʿ��մϴ�.  ����/�� ���� ����ϴ� MFC ���� ���α׷��� ��쿡��
//  �����ӿ�ũ���� �� �۾��� �ڵ����� �����մϴ�.

void CRFT_IF_ECATDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // �׸��⸦ ���� ����̽� ���ؽ�Ʈ�Դϴ�.

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Ŭ���̾�Ʈ �簢������ �������� ����� ����ϴ�.
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// �������� �׸��ϴ�.
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// ����ڰ� �ּ�ȭ�� â�� ���� ���ȿ� Ŀ���� ǥ�õǵ��� �ý��ۿ���
//  �� �Լ��� ȣ���մϴ�.
HCURSOR CRFT_IF_ECATDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void CRFT_IF_ECATDlg::OnDestroy()
{
	CDialogEx::OnDestroy();

	// TODO: ���⿡ �޽��� ó���� �ڵ带 �߰��մϴ�.
}



void CRFT_IF_ECATDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: ���⿡ �޽��� ó���� �ڵ带 �߰� ��/�Ǵ� �⺻���� ȣ���մϴ�.

	::QueryPerformanceCounter(&g_st); // get start time.

	if (m_RFT_IF.m_nCurrMode == CMD_FT_CONT)
	{
		m_strRxData.Format("froce<%.03f %.03f %.03f> torque<%.03f %.03f %.03f> overload[0x%02X]",
			m_RFT_IF.m_RFT_IF_PACKET.m_rcvdForce[0], m_RFT_IF.m_RFT_IF_PACKET.m_rcvdForce[1], m_RFT_IF.m_RFT_IF_PACKET.m_rcvdForce[2],
			m_RFT_IF.m_RFT_IF_PACKET.m_rcvdForce[3], m_RFT_IF.m_RFT_IF_PACKET.m_rcvdForce[4], m_RFT_IF.m_RFT_IF_PACKET.m_rcvdForce[5],
			m_RFT_IF.m_RFT_IF_PACKET.m_rcvdForceStatus);

		//CONSOLE_S("%s\n", m_strRxData.GetBuffer());
		if (m_RFT_IF.m_bIsEnabled_Callback == false)
		{
			m_vGraphDatas_F[0].push_back(m_RFT_IF.m_RFT_IF_PACKET.m_rcvdForce[0]);
			m_vGraphDatas_F[1].push_back(m_RFT_IF.m_RFT_IF_PACKET.m_rcvdForce[1]);
			m_vGraphDatas_F[2].push_back(m_RFT_IF.m_RFT_IF_PACKET.m_rcvdForce[2]);
			m_vGraphDatas_F[3].push_back(m_RFT_IF.m_RFT_IF_PACKET.m_rcvdForce[3]);
			m_vGraphDatas_F[4].push_back(m_RFT_IF.m_RFT_IF_PACKET.m_rcvdForce[4]);
			m_vGraphDatas_F[5].push_back(m_RFT_IF.m_RFT_IF_PACKET.m_rcvdForce[5]);
		}

		GetDlgItem(IDC_EDIT_RX_DATA)->SetWindowTextA(m_strRxData);

		for (int i = 0; i < RFT_NUM_OF_FORCE; i++)
		{
			int size = m_vGraphDatas_F[i].size();
			double *x = new double[size];
			double *y = new double[size];

			for (int idx = 0; idx < size; idx++)
			{
				x[idx] = idx;
				y[idx] = m_vGraphDatas_F[i][idx];
			}

			m_pGraph_F[i]->SetPoints(x, y, size);

			delete[] x;
			delete[] y;

			if (size > NUM_OF_SAMPLE_FOR_GRAPH)
			{
				int del_size = (size - NUM_OF_SAMPLE_FOR_GRAPH);
				m_vGraphDatas_F[i].erase(m_vGraphDatas_F[i].begin(), m_vGraphDatas_F[i].begin() + del_size);
			}
		}

		m_ChartCtrl_Force.RefreshCtrl();
		m_ChartCtrl_Torque.RefreshCtrl();

	}

	::QueryPerformanceCounter(&g_ed); // get end time
	double passedTime = ((double)g_ed.QuadPart - g_st.QuadPart) / ((double)g_freq.QuadPart);
	//CONSOLE_S("on timer: %f\n", passedTime);

	CDialogEx::OnTimer(nIDEvent);
}


BOOL CRFT_IF_ECATDlg::PreTranslateMessage(MSG* pMsg)
{
	// TODO: ���⿡ Ư��ȭ�� �ڵ带 �߰� ��/�Ǵ� �⺻ Ŭ������ ȣ���մϴ�.
	int YesNo = 0;
	bool isForcedReturn = false;
	switch (pMsg->message)    /// ���� Ű���� �޼��� ����
	{
	case WM_KEYDOWN:
		if ((pMsg->wParam == VK_ESCAPE) | (pMsg->wParam == VK_RETURN))
			isForcedReturn = true;
		break;

	case WM_SYSKEYDOWN:    // Alt + F4 �޼��� ó��
		if (pMsg->wParam == VK_F4)
		{
			YesNo = AfxMessageBox("Do you want to exit?", MB_YESNO, NULL);

			if (YesNo == IDYES)
			{
				DestroyWindow();
			}

			isForcedReturn = true;
		}
		break;
	case WM_LBUTTONDOWN:
		//CONSOLE_S("MAIN DLG BN_CLICKED\n");
		break;
	default:
		break;
	}

	if (isForcedReturn)
		return TRUE;

	return CDialogEx::PreTranslateMessage(pMsg);
}



void CRFT_IF_ECATDlg::OnBnClickedCheckInterfaceOpen()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	UpdateData();

	bool isOn = (bool)((CButton*)GetDlgItem(IDC_CHECK_INTERFACE_OPEN))->GetCheck();
	if (isOn)
	{
		int selected_nic = m_nic_info.GetCurSel();
		bool enableCallback = true;

		CString nic_dev_name = m_RFT_IF.m_vecNic_Name[selected_nic];

		if (m_RFT_IF.run(nic_dev_name, m_nSlaveId, enableCallback, m_fDivider_Force, m_fDivider_Torque)) // true - enable call back...
		{
			((CButton*)GetDlgItem(IDC_CHECK_INTERFACE_OPEN))->SetWindowTextA("EtherCAT Stop");

			GetDlgItem(IDC_CHECK_FT_OUT_CONT)->EnableWindow(TRUE);
			GetDlgItem(IDC_CHECK_BIAS)->EnableWindow(TRUE);
			GetDlgItem(IDC_READ_OVERLOAD_COUNT)->EnableWindow(TRUE);
			GetDlgItem(IDC_BUTTON_FILTER_SETTING)->EnableWindow(TRUE);
			GetDlgItem(IDC_CHECK_DATA_LOGGING)->EnableWindow(TRUE);
		}
		else
		{
			((CButton*)GetDlgItem(IDC_CHECK_INTERFACE_OPEN))->SetCheck(0);
			((CButton*)GetDlgItem(IDC_CHECK_INTERFACE_OPEN))->SetWindowTextA("EtherCAT Start");

			GetDlgItem(IDC_CHECK_FT_OUT_CONT)->EnableWindow(FALSE);
			GetDlgItem(IDC_CHECK_BIAS)->EnableWindow(FALSE);
			GetDlgItem(IDC_READ_OVERLOAD_COUNT)->EnableWindow(FALSE);
			GetDlgItem(IDC_BUTTON_FILTER_SETTING)->EnableWindow(FALSE);
			GetDlgItem(IDC_CHECK_DATA_LOGGING)->EnableWindow(FALSE);

		}
	}
	else
	{
		((CButton*)GetDlgItem(IDC_CHECK_INTERFACE_OPEN))->SetWindowTextA("EtherCAT Start");

		GetDlgItem(IDC_CHECK_FT_OUT_CONT)->EnableWindow(FALSE);
		GetDlgItem(IDC_CHECK_BIAS)->EnableWindow(FALSE);
		GetDlgItem(IDC_READ_OVERLOAD_COUNT)->EnableWindow(FALSE);
		GetDlgItem(IDC_BUTTON_FILTER_SETTING)->EnableWindow(FALSE);
		GetDlgItem(IDC_CHECK_DATA_LOGGING)->EnableWindow(FALSE);
		m_RFT_IF.stop();
	}
}


void CRFT_IF_ECATDlg::callback_RFT_Data_Receive( void *callbackParam)
{

	CRFT_IF_ECATDlg *pThisClass = (CRFT_IF_ECATDlg*)callbackParam;

	static unsigned long cnt = 0;

	if (pThisClass->m_RFT_IF.m_nCurrMode == CMD_FT_CONT)
	{
		cnt++;
		//if ((cnt % 5) == 0) // 200Hz data gathering for graph rendering...
		{
			// for graph.... 
			pThisClass->m_vGraphDatas_F[0].push_back(pThisClass->m_RFT_IF.m_RFT_IF_PACKET.m_rcvdForce[0]);
			pThisClass->m_vGraphDatas_F[1].push_back(pThisClass->m_RFT_IF.m_RFT_IF_PACKET.m_rcvdForce[1]);
			pThisClass->m_vGraphDatas_F[2].push_back(pThisClass->m_RFT_IF.m_RFT_IF_PACKET.m_rcvdForce[2]);
			pThisClass->m_vGraphDatas_F[3].push_back(pThisClass->m_RFT_IF.m_RFT_IF_PACKET.m_rcvdForce[3]);
			pThisClass->m_vGraphDatas_F[4].push_back(pThisClass->m_RFT_IF.m_RFT_IF_PACKET.m_rcvdForce[4]);
			pThisClass->m_vGraphDatas_F[5].push_back(pThisClass->m_RFT_IF.m_RFT_IF_PACKET.m_rcvdForce[5]);
		}
	}
}


void CRFT_IF_ECATDlg::OnBnClickedCheckBias()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.

	if (((CButton*)GetDlgItem(IDC_CHECK_BIAS))->GetCheck())
	{
		m_RFT_IF.set_FT_Bias(1);
	}
	else
	{
		m_RFT_IF.set_FT_Bias(0);
	}
}

void CRFT_IF_ECATDlg::OnBnClickedCheckFtOutCont()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.

	if (((CButton*)GetDlgItem(IDC_CHECK_FT_OUT_CONT))->GetCheck())
	{
		m_RFT_IF.rqst_FT_Continuous();

		GetDlgItem(IDC_READ_OVERLOAD_COUNT)->EnableWindow(FALSE);
		GetDlgItem(IDC_BUTTON_FILTER_SETTING)->EnableWindow(FALSE);
	}
	else
	{
		m_RFT_IF.rqst_FT_Stop();
		Sleep(500);

		GetDlgItem(IDC_READ_OVERLOAD_COUNT)->EnableWindow(TRUE);
		GetDlgItem(IDC_BUTTON_FILTER_SETTING)->EnableWindow(TRUE);
	}
}


void CRFT_IF_ECATDlg::OnBnClickedReadOverloadCount()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	m_RFT_IF.rqst_FT_OverloadCnt();
	int waitTimeOutCnt = 0;

	do{
		Sleep(WAIT_SLEEP_TIME_RFT);
		waitTimeOutCnt++;
		if (waitTimeOutCnt >= WAIT_TIMEOUT_RFT)
			break;
	} while (m_RFT_IF.m_bIsRcvd_Response_Pkt == false);

	// ���⼭ ������ ó��.....
	if (m_RFT_IF.m_bIsRcvd_Response_Pkt)
	{
		m_strRxData.Format("Overload Count Fx[%d], Fy[%d] Fz[%d] Tx[%d] Ty[%d] Tz[%d]",
			m_RFT_IF.m_RFT_IF_PACKET.m_rcvdOverloadCnt[0],
			m_RFT_IF.m_RFT_IF_PACKET.m_rcvdOverloadCnt[1],
			m_RFT_IF.m_RFT_IF_PACKET.m_rcvdOverloadCnt[2],
			m_RFT_IF.m_RFT_IF_PACKET.m_rcvdOverloadCnt[3],
			m_RFT_IF.m_RFT_IF_PACKET.m_rcvdOverloadCnt[4],
			m_RFT_IF.m_RFT_IF_PACKET.m_rcvdOverloadCnt[5]);

		GetDlgItem(IDC_EDIT_RX_DATA)->SetWindowTextA(m_strRxData);
	}
	else
	{
		m_strRxData = "Time out - Receive response packet";
		GetDlgItem(IDC_EDIT_RX_DATA)->SetWindowTextA(m_strRxData);
	}
}

void CRFT_IF_ECATDlg::OnBnClickedButtonFilterSetting()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	UpdateData();

	int filterSel = m_combo_Filter_Cutoff_Frq.GetCurSel();

	unsigned char param[2] = { 0, 0 };

	if (filterSel != 0)
	{
		param[0] = 1;
		param[1] = filterSel;
	}
	else
	{
		param[0] = 0;
		param[1] = 0;
	}

	m_RFT_IF.set_FT_Filter_Type( param[0], param[1]);
	int waitTimeOutCnt = 0;

	do{
		Sleep(WAIT_SLEEP_TIME_RFT);
		waitTimeOutCnt++;
		if (waitTimeOutCnt >= WAIT_TIMEOUT_RFT)
			break;
	} while (m_RFT_IF.m_bIsRcvd_Response_Pkt == false);

	// ���⼭ ������ ó��.....
	if (m_RFT_IF.m_bIsRcvd_Response_Pkt)
	{
		if (m_RFT_IF.m_RFT_IF_PACKET.m_response_result == 1)
			m_strRxData.Format("F/T Filter Setting - Successed");
		else
			m_strRxData.Format("F/T Filter Setting Error[Error Code: %d]", m_RFT_IF.m_RFT_IF_PACKET.m_response_errcode);

		GetDlgItem(IDC_EDIT_RX_DATA)->SetWindowTextA(m_strRxData);
	}
	else
	{
		m_strRxData = "Time out - Receive response packet";
		GetDlgItem(IDC_EDIT_RX_DATA)->SetWindowTextA(m_strRxData);
	}
}


void CRFT_IF_ECATDlg::OnBnClickedCheckDataLogging()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	bool isTrue = ((CButton*)GetDlgItem(IDC_CHECK_DATA_LOGGING))->GetCheck();

	if (isTrue)
	{
		if (m_RFT_IF.startLogging("RFT"))
		{
			GetDlgItem(IDC_CHECK_DATA_LOGGING)->SetWindowTextA("Stop Data Logging");
		}
		else
		{
			GetDlgItem(IDC_CHECK_DATA_LOGGING)->SetWindowTextA("Start Data Logging");
			((CButton*)GetDlgItem(IDC_CHECK_DATA_LOGGING))->SetCheck(0);
		}
	}
	else
	{
		m_RFT_IF.stopLogging();
		GetDlgItem(IDC_CHECK_DATA_LOGGING)->SetWindowTextA("Start Data Logging");
	}
}
