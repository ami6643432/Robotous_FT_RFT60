
// RFT_IF_ECAT.h : PROJECT_NAME ���� ���α׷��� ���� �� ��� �����Դϴ�.
//

#pragma once

#ifndef __AFXWIN_H__
	#error "PCH�� ���� �� ������ �����ϱ� ���� 'stdafx.h'�� �����մϴ�."
#endif

#include "resource.h"		// �� ��ȣ�Դϴ�.


// CRFT_IF_ECATApp:
// �� Ŭ������ ������ ���ؼ��� RFT_IF_ECAT.cpp�� �����Ͻʽÿ�.
//

class CRFT_IF_ECATApp : public CWinApp
{
public:
	CRFT_IF_ECATApp();

// �������Դϴ�.
public:
	virtual BOOL InitInstance();

// �����Դϴ�.

	DECLARE_MESSAGE_MAP()
};

extern CRFT_IF_ECATApp theApp;