#pragma once

#ifndef _UTIL_H_
#define _UTIL_H_

#define D3D9_SHADER_DEBUG_NO

#ifdef D3D9_SHADER_DEBUG
#define D3D_DEBUG_INFO
#endif

#include <windows.h>
#include <d3d9.h>
#include <d3dx9.h>

//vars
extern HWND hwnd;
extern LPDIRECT3D9 pD3D9;
extern LPDIRECT3DDEVICE9 pD3DDev9;

//prototypes

//window util
void wnd_create( const char *title, int w, int h );
void wnd_destroy();
void wnd_show();
void wnd_hide();
void wnd_update();
void wnd_mode( bool fs );
void wnd_setTitle( const char* title );
void wnd_setFrameMoveCallBack(bool (*callback)());
void wnd_setFrameRenderCallBack(void (*callback)());
void wnd_setCleanUpCallBack(void (*callback)());

//d3d9
void d3d9_init( bool fuulscreen );
void d3d9_destroy();
bool d3d9_reset();
bool d3d9_isFullScreen();
void d3d9_setFullScreen(bool fullscreen);
void d3d9_setDisplayWH(unsigned short w, unsigned short h);
unsigned int d3d9_GetBBWidth();
unsigned int d3d9_GetBBHeight();

#endif