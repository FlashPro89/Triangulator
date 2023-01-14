#include "util.h"

#pragma comment( lib , "d3d9.lib" )
#pragma comment( lib , "d3dx9.lib" )

#define WIND_STYLE (WS_OVERLAPPED | WS_SYSMENU | WS_MINIMIZEBOX | WS_CAPTION)

//vars
extern HWND hwnd = 0;
extern LPDIRECT3D9 pD3D9 = 0;
extern LPDIRECT3DDEVICE9 pD3DDev9 = 0;
D3DPRESENT_PARAMETERS presParams;

//local
int l_width = 0;
int l_height = 0;

bool (*frameMoveCallback)() = 0;
void (*frameRenderCallback)() = 0;
void (*cleanUpCallback)() = 0;

//wndproc
LRESULT WINAPI _wndProc( HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam )
{
    switch( msg )
    {
        case WM_DESTROY:
            PostQuitMessage( 0 );
            return 0;
    }
    return DefWindowProc( hWnd, msg, wParam, lParam );
}

void wnd_create(const char* title, int w, int h)
{
	hwnd = 0;

	WNDCLASSEX wc = { sizeof(WNDCLASSEX), CS_CLASSDC, _wndProc, 0L, 0L,
				  GetModuleHandle(NULL), NULL, LoadCursor(NULL, IDC_ARROW), NULL, NULL,
				  "UTIL_LIB_WND_CLS", NULL };

	if (!RegisterClassEx(&wc))
		throw("Can't register window class!");

	DWORD wStyle = WIND_STYLE;
	RECT rect;
	rect.left = 0;
	rect.top = 0;
	rect.right = w;
	rect.bottom = h;
	AdjustWindowRect(&rect, wStyle, false);

	hwnd = CreateWindowEx(0, "UTIL_LIB_WND_CLS", title, wStyle, 0, 0, rect.right - rect.left, rect.bottom - rect.top, 0, 0, 0, 0);
	if( ! hwnd )
		throw( "Can't create window!" );

	l_width = w;
	l_height = h;

	
}

void wnd_destroy()
{
	if( hwnd )
		DestroyWindow( hwnd );
	UnregisterClass( "UTIL_LIB_WND_CLS", GetModuleHandle( 0 ) );
}

void wnd_show()
{
	ShowWindow( hwnd, SW_SHOWDEFAULT );
}

void wnd_hide()
{
	ShowWindow( hwnd, SW_HIDE );
}

void wnd_update()
{
	UpdateWindow( hwnd );
}

void wnd_setTitle(const char* title)
{
	if (hwnd)
		SetWindowText(hwnd, title);
}

void wnd_setFrameMoveCallBack(bool (*callback)())
{
	frameMoveCallback = callback;
}

void wnd_setFrameRenderCallBack(void (*callback)())
{
	frameRenderCallback = callback;
}

void wnd_setCleanUpCallBack(void (*callback)())
{
	cleanUpCallback = callback;
}

void d3d9_init( bool fullscreen )
{
	HRESULT hr;
	
	pD3D9 = Direct3DCreate9( D3D_SDK_VERSION );
	if( !pD3D9 )
		throw( "Ошибка при создании главного интерфейса Direct3D9!" );

	UINT num = pD3D9->GetAdapterCount();
	D3DADAPTER_IDENTIFIER9 id;
	pD3D9->GetAdapterIdentifier(D3DADAPTER_DEFAULT, 0, &id);
	wnd_setTitle(id.Description);


	//D3DPRESENT_PARAMETERS p;
	ZeroMemory( &presParams, sizeof(presParams) );
	presParams.AutoDepthStencilFormat = D3DFMT_D24X8;
	presParams.EnableAutoDepthStencil = true;
	presParams.hDeviceWindow = hwnd;
	presParams.PresentationInterval = D3DPRESENT_INTERVAL_IMMEDIATE;
	presParams.SwapEffect = D3DSWAPEFFECT_DISCARD;
	presParams.Windowed = !fullscreen;
	presParams.BackBufferFormat = D3DFMT_X8R8G8B8;
	presParams.BackBufferWidth = l_width;
	presParams.BackBufferHeight = l_height;


#ifdef D3D9_SHADER_DEBUG
	hr = pD3D9->CreateDevice( D3DADAPTER_DEFAULT, D3DDEVTYPE_REF, hwnd, 
		D3DCREATE_SOFTWARE_VERTEXPROCESSING, &presParams, &pD3DDev9 );
#else
	hr = pD3D9->CreateDevice( D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, hwnd, 
		D3DCREATE_HARDWARE_VERTEXPROCESSING | D3DCREATE_PUREDEVICE, &presParams, &pD3DDev9 );
#endif

	if( FAILED( hr ) )
		throw( "Ошибка при инициализации устройства Direct3D9!" );

	pD3DDev9->SetRenderState( D3DRS_LIGHTING, false );
	pD3DDev9->SetRenderState( D3DRS_CULLMODE, D3DCULL_NONE );
	pD3DDev9->SetRenderState( D3DRS_ZENABLE, true );
	//pD3DDev9->SetRenderState( D3DRS_SLOPESCALEDEPTHBIAS, (DWORD)0.2f );
	//pD3DDev9->SetRenderState( D3DRS_FILLMODE, D3DFILL_SOLID );

	pD3DDev9->SetSamplerState( 0, D3DSAMP_ADDRESSU, D3DTADDRESS_MIRROR );
	pD3DDev9->SetSamplerState( 0, D3DSAMP_ADDRESSV, D3DTADDRESS_MIRROR );

	pD3DDev9->SetSamplerState( 0, D3DSAMP_MINFILTER, D3DTEXF_ANISOTROPIC );
	pD3DDev9->SetSamplerState( 0, D3DSAMP_MAGFILTER, D3DTEXF_ANISOTROPIC );
	pD3DDev9->SetSamplerState( 0, D3DSAMP_MIPFILTER, D3DTEXF_ANISOTROPIC );

	pD3DDev9->SetSamplerState( 1, D3DSAMP_MINFILTER, D3DTEXF_LINEAR );
	pD3DDev9->SetSamplerState( 1, D3DSAMP_MAGFILTER, D3DTEXF_LINEAR );
	pD3DDev9->SetSamplerState( 1, D3DSAMP_MIPFILTER, D3DTEXF_LINEAR );

	pD3DDev9->SetSamplerState( 2, D3DSAMP_MINFILTER, D3DTEXF_LINEAR );
	pD3DDev9->SetSamplerState( 2, D3DSAMP_MAGFILTER, D3DTEXF_LINEAR );
	pD3DDev9->SetSamplerState( 2, D3DSAMP_MIPFILTER, D3DTEXF_LINEAR );

	pD3DDev9->SetSamplerState( 3, D3DSAMP_MINFILTER, D3DTEXF_LINEAR );
	pD3DDev9->SetSamplerState( 3, D3DSAMP_MAGFILTER, D3DTEXF_LINEAR );
	pD3DDev9->SetSamplerState( 3, D3DSAMP_MIPFILTER, D3DTEXF_LINEAR );

	pD3DDev9->SetSamplerState( 4, D3DSAMP_MINFILTER, D3DTEXF_LINEAR );
	pD3DDev9->SetSamplerState( 4, D3DSAMP_MAGFILTER, D3DTEXF_LINEAR );
	pD3DDev9->SetSamplerState( 4, D3DSAMP_MIPFILTER, D3DTEXF_LINEAR );

	pD3DDev9->SetSamplerState( 5, D3DSAMP_MINFILTER, D3DTEXF_LINEAR );
	pD3DDev9->SetSamplerState( 5, D3DSAMP_MAGFILTER, D3DTEXF_LINEAR );
	pD3DDev9->SetSamplerState( 5, D3DSAMP_MIPFILTER, D3DTEXF_LINEAR );

	pD3DDev9->SetSamplerState( 6, D3DSAMP_MINFILTER, D3DTEXF_LINEAR );
	pD3DDev9->SetSamplerState( 6, D3DSAMP_MAGFILTER, D3DTEXF_LINEAR );
	pD3DDev9->SetSamplerState( 6, D3DSAMP_MIPFILTER, D3DTEXF_LINEAR );

	pD3DDev9->SetSamplerState( 7, D3DSAMP_MINFILTER, D3DTEXF_LINEAR );
	pD3DDev9->SetSamplerState( 7, D3DSAMP_MAGFILTER, D3DTEXF_LINEAR );
	pD3DDev9->SetSamplerState( 7, D3DSAMP_MIPFILTER, D3DTEXF_LINEAR );

	pD3DDev9->SetSamplerState( 1, D3DSAMP_ADDRESSU, D3DTADDRESS_MIRROR);
	pD3DDev9->SetSamplerState( 1, D3DSAMP_ADDRESSV, D3DTADDRESS_MIRROR);
}

void d3d9_destroy()
{
	if( pD3DDev9 )
		pD3DDev9->Release();
	pD3DDev9 = 0;
	
	if( pD3D9 )
		pD3D9->Release();
	pD3D9 = 0;
}

bool d3d9_reset()
{
	return(SUCCEEDED(pD3DDev9->Reset(&presParams)));
}

bool d3d9_isFullScreen()
{
	return !presParams.Windowed;
}

void d3d9_setFullScreen(bool fullscreen)
{
	presParams.Windowed = !fullscreen;
}

void wnd_setWH(unsigned short w, unsigned short h)
{
	presParams.BackBufferWidth = w;
	presParams.BackBufferHeight = h;

	if (presParams.Windowed)
	{
		DWORD wStyle = WIND_STYLE;
		RECT rect;
		rect.left = 0;
		rect.top = 0;
		rect.right = w;
		rect.bottom = h;
		AdjustWindowRect(&rect, wStyle, false);

		SetWindowPos(hwnd, hwnd, 0, 0, rect.right, rect.bottom, SWP_NOZORDER);
	}
	else
	{
		SetWindowPos(hwnd, hwnd, 0, 0, w, h, SWP_NOZORDER);
	}
}

unsigned int d3d9_GetBBWidth() 
{
	return presParams.BackBufferWidth;
}

unsigned int d3d9_GetBBHeight()
{
	return presParams.BackBufferHeight;
}