#pragma once

#ifndef _INPUT_H_
#define _INPUT_H_

#define DIRECTINPUT_VERSION 0x0800

#include <windows.h>
#include <dinput.h>

class gInput
{
public:
	 gInput();
	 ~gInput();

	void init( HWND handle );
	bool reset();
	void close();

	bool isKeyDown( int key );
	bool isKeyUp( int key );
	bool isKeyPressed( int key );

	int getMouseX();
	int getMouseY();
	int getMouseZ(); // scroll

	bool isMouseDown( int button );
	bool isMouseUp( int button );
	bool isMousePressed( int button );

	void update();

protected:
	HWND m_handle;
	LPDIRECTINPUT8 m_pDI;
	LPDIRECTINPUTDEVICE8A m_pKeyboard;
	LPDIRECTINPUTDEVICE8A m_pMouse;
	bool m_keyboardOld[256];
	DIMOUSESTATE m_mouseOld;
	bool m_keyboardOld2[256];
	DIMOUSESTATE m_mouseOld2;
};

#endif