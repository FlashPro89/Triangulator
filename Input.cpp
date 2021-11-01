#include "Input.h"

#pragma comment( lib , "dinput8.lib" )
#pragma comment( lib , "dxguid.lib" )

#pragma warning( disable : 4800 ) // 'BYTE' : forcing value to bool 'true' or 'false' (performance warning)

gInput::gInput()
{
	m_handle = 0;
	m_pDI = 0;
	m_pKeyboard = 0;
	m_pMouse = 0;

	ZeroMemory( m_keyboardOld, 256 );
	ZeroMemory( &m_mouseOld, sizeof( m_mouseOld ) );
}
gInput::~gInput()
{
	close();
}

void gInput::init(HWND handle)
{
	DirectInput8Create( GetModuleHandle( 0 ), DIRECTINPUT_VERSION, 
        IID_IDirectInput8, (void**)&m_pDI, 0 ); 
	if( !m_pDI )
	{
		MessageBox( 0, "Ошибка при создании главного интерфейса DirectInput8!", "Error!", MB_OK|MB_ICONERROR|MB_SYSTEMMODAL );
		PostQuitMessage( -1 );
	}

	
	m_pDI->CreateDevice( GUID_SysKeyboard, &m_pKeyboard, 0 ); 
	if( !m_pKeyboard )
	{
		MessageBox( 0, "Ошибка при создании интерфейса клавиатуры!", "Error!", MB_OK|MB_ICONERROR|MB_SYSTEMMODAL );
		PostQuitMessage( -1 );
	}

	m_pDI->CreateDevice( GUID_SysMouse, &m_pMouse, 0 ); 
	if( !m_pMouse )
	{
		MessageBox( 0, "Ошибка при создании интерфейса мыши!", "Error!", MB_OK | MB_ICONERROR | MB_SYSTEMMODAL );
		PostQuitMessage( -1 );
	}

	
	m_pKeyboard->SetDataFormat( &c_dfDIKeyboard );
	m_pMouse->SetDataFormat( &c_dfDIMouse );

	HRESULT hr;
	hr = m_pKeyboard->SetCooperativeLevel( m_handle, DISCL_BACKGROUND | DISCL_NONEXCLUSIVE );
	if( FAILED( hr ) )
	{
		MessageBox( 0, "Ошибка при установке уровня взаимодействия клавиатуры!", "Error!", MB_OK|MB_ICONERROR|MB_SYSTEMMODAL );
		PostQuitMessage( -1 );
	}

	hr = m_pMouse->SetCooperativeLevel( m_handle, DISCL_BACKGROUND | DISCL_NONEXCLUSIVE );
	if( FAILED( hr ) )
	{
		MessageBox( 0, "Ошибка при установке уровня взаимодействия мыши!", "Error!", MB_OK|MB_ICONERROR|MB_SYSTEMMODAL );
		PostQuitMessage( -1 );
	}

	hr = m_pKeyboard->Acquire();
	if( FAILED( hr ) )
		throw( 0 );
	hr = m_pMouse->Acquire();
}

bool gInput::reset()
{
	HRESULT hr;
	hr = m_pKeyboard->Unacquire();
	hr = m_pMouse->Unacquire();
	hr = m_pKeyboard->Acquire();
	if( FAILED( hr ) )
		throw( 0 );
	hr = m_pMouse->Acquire();
	return true;
}

void gInput::close()
{
	if( m_pMouse )
	{
		m_pMouse->Unacquire();
		m_pMouse->Release();
	}
	m_pMouse = 0;

	if( m_pKeyboard )
	{
		m_pKeyboard->Unacquire();
		m_pKeyboard->Release();
	}
	m_pKeyboard = 0;

	if( m_pDI )
		m_pDI->Release();
	m_pDI = 0;
}

bool gInput::isKeyDown( int key )
{
	return !m_keyboardOld2[ key ] && m_keyboardOld[ key ];
}

bool gInput::isKeyUp( int key )
{
	return m_keyboardOld2[ key ] && !m_keyboardOld[ key ];
}

bool gInput::isKeyPressed( int key )
{
	return m_keyboardOld[ key ];
}

int gInput::getMouseX()
{
	return m_mouseOld.lX;
}

int gInput::getMouseY()
{
	return m_mouseOld.lY;
}

int gInput::getMouseZ() // scroll
{
	return m_mouseOld.lZ;
}

bool gInput::isMouseDown( int button )
{
	return m_mouseOld.rgbButtons[ button ] && !m_mouseOld2.rgbButtons[ button ];
}

bool gInput::isMouseUp( int button )
{
	return !m_mouseOld.rgbButtons[ button ] && m_mouseOld2.rgbButtons[ button ];
}

bool gInput::isMousePressed( int button )
{
	return m_mouseOld.rgbButtons[ button ];
}

void gInput::update()
{
	memcpy( m_keyboardOld2, m_keyboardOld, sizeof( m_keyboardOld ) );
	memcpy( (void*)&m_mouseOld2, (const void*)&m_mouseOld, sizeof( m_mouseOld ) );
	HRESULT hr;
	hr = m_pKeyboard->GetDeviceState( sizeof( m_keyboardOld ), (void*)m_keyboardOld );
	hr = m_pMouse->GetDeviceState( sizeof( m_mouseOld ), (void*)&m_mouseOld );
}