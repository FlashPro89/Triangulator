#include <Windows.h>
#include <stdlib.h>
#include "util.h"
#include "input.h"
#include <time.h>
#include <map>
#include <math.h>

#define FVF D3DFVF_XYZRHW | D3DFVF_DIFFUSE | D3DFVF_PSIZE

struct RHWVertex
{
	RHWVertex() { x = y = z = 0; rhw = 1.f; diffuse = 0xFFFFFFFF; psize = 1.f; }
	RHWVertex(const RHWVertex& other) { x = other.x; y = other.y; z = other.z; rhw = other.rhw; diffuse = other.diffuse; psize = other.psize; }
	RHWVertex(const D3DXVECTOR3& v, DWORD color, float _psize = 0.f ) { x = v.x; y = v.y; z = v.z; rhw = 1.0f; diffuse = color; psize = _psize; }

	float x, y, z, rhw;
	float psize;
	DWORD diffuse;
};

#define LINEBBUFFERSZ 2048
#define PT_LINELIST 0
#define PT_UNSELECTED -1
#define PT_POINTLIST 1

RHWVertex vvBuffer[ LINEBBUFFERSZ * 2 ];
unsigned int vvBufferPos = 0;
char pt = -1;
gInput input;

#define MAXPOINTS 1024
D3DXVECTOR2 points[MAXPOINTS];
unsigned int pointsNum = 0;

struct EDGEKEY
{
	EDGEKEY(unsigned short _v0, unsigned short _v1) { v0 = _v0; v1 = _v1; }
	union
	{
		unsigned short v0, v1;
		unsigned int key;
	};
};

#define KEYGEN(v0,v1) ( v0 | (v1 << 16) )
#define KEYGETLOW(k) ( k & 0xFFFF )
#define KEYGETHIGH(k) ( k >> 16 )

std::multimap< float, unsigned int > rawEdges;
std::multimap< float, unsigned int > finalEdges;

float edgeLenght( unsigned short v0, unsigned short v1 )
{
	D3DXVECTOR2 d = points[v1] - points[v0];
	return sqrtf( d.x * d.x + d.y * d.y );
}

void cleanUpTriangulation()
{
	rawEdges.clear();
	finalEdges.clear();
}

#define EPSILON 0.0001f

bool testIntersection( float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4 )
{

	float v0xmax = max( x1, x2 );
	float v0xmin = min( x1, x2 );

	float v0ymax = max( y1, y2 );
	float v0ymin = min( y1, y2 );

	if ( ( ( (x3 < v0xmin) && (x4 < v0xmin) ) || ( (x3 > v0xmax ) && (x4 > v0xmax) ) ) &&
		 ( ( (y3 < v0ymin) && (y4 < v0ymin) ) || ( (y3 > v0ymax ) && (y4 > v0ymax) ) ) )
		return false; // проекции на оси совсем не пересекаются

	float z = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
	float cta = (x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3);
	float ctb = (x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3);

	if (fabsf(z) < EPSILON)
	{
		if ((fabsf(cta) < EPSILON) && (fabsf(ctb) < EPSILON))
		{
			return false; //параллельны и на одной линии
		}
		else
			return false; // отрезки параллельны, возможно надо проверить расстояние между ними...
	}

	cta /= z; ctb /= z;

	if ((cta > 0) && (cta < 1.f) && (ctb > 0) && (ctb < 1.f))
		return true;
	else
		return false;
}


bool testIntersection(unsigned int e0, unsigned int e1)
{
	unsigned short v0 = KEYGETLOW(e0);
	unsigned short v1 = KEYGETHIGH(e0);
	unsigned short v2 = KEYGETLOW(e1);
	unsigned short v3 = KEYGETHIGH(e1);

	float x1 = points[v0].x; float y1 = points[v0].y;
	float x2 = points[v1].x; float y2 = points[v1].y;
	float x3 = points[v2].x; float y3 = points[v2].y;
	float x4 = points[v3].x; float y4 = points[v3].y;

	float v0xmax = max(x1, x2);
	float v0xmin = min(x1, x2);

	float v0ymax = max(y1, y2);
	float v0ymin = min(y1, y2);

	if ((((x3 < v0xmin) && (x4 < v0xmin)) || ((x3 > v0xmax) && (x4 > v0xmax))) &&
		(((y3 < v0ymin) && (y4 < v0ymin)) || ((y3 > v0ymax) && (y4 > v0ymax))))
		return false; // проекции на оси совсем не пересекаются

	float z = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
	float cta = (x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3);
	float ctb = (x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3);

	if (fabsf(z) < EPSILON)
	{
		/*
		if ((fabsf(cta) < EPSILON) && (fabsf(ctb) < EPSILON))
		{
			return false; //параллельны и на одной линии
		}
		else
			return false; // отрезки параллельны, возможно надо проверить расстояние между ними...
		*/
		return false;
	}

	cta /= z; ctb /= z;

	if ((cta > EPSILON) && (cta < 1.f - EPSILON ) && (ctb > EPSILON) && (ctb < 1.f- EPSILON))
		return true;
	else
		return false;
}

void runTriangulation()
{
	cleanUpTriangulation();

	if (pointsNum <= 3)
		return;

	for (unsigned int i = 0; i < pointsNum; i++)
	{
		for (unsigned int j = i + 1; j < pointsNum; j++)
		{
			rawEdges.insert( std::pair<float, unsigned int> ( edgeLenght(i, j), KEYGEN(i, j)) );
		}
	}

	auto it = rawEdges.begin();

	unsigned short p0, p1, p2, p3;

	while (it != rawEdges.end())
	{
		auto itf = finalEdges.begin();
		bool intersected = false;

		p0 = KEYGETLOW(it->second);
		p1 = KEYGETHIGH(it->second);

		while (itf != finalEdges.end())
		{
			p2 = KEYGETLOW(itf->second);
			p3 = KEYGETHIGH(itf->second);

			if ( testIntersection( points[p0].x, points[p0].y, points[p1].x, points[p1].y,
					points[p2].x, points[p2].y, points[p3].x, points[p3].y ) == true )
			{
				intersected = true;
				break;
			}
			itf++;
		}

		if (!intersected)
		{
			finalEdges.insert( std::pair< float, unsigned int >( it->first, it->second ) );
		//	it = rawEdges.erase( it );
		}
		//else
			it++;
	}
}

void batch_fire()
{
	if (vvBufferPos == 0)
		return;

	pD3DDev9->SetFVF(FVF);
	
	switch (pt)
	{
	case PT_UNSELECTED: break;
	case PT_LINELIST:
		pD3DDev9->DrawPrimitiveUP(D3DPT_LINELIST, vvBufferPos / 2, vvBuffer, sizeof(RHWVertex)); break;
	case PT_POINTLIST:
		pD3DDev9->DrawPrimitiveUP(D3DPT_POINTLIST, vvBufferPos, vvBuffer, sizeof(RHWVertex)); break;
	}

	vvBufferPos = 0;
	pt = PT_UNSELECTED;
}

void batch_line(const D3DXVECTOR2& v0, const D3DXVECTOR2& v1, DWORD color)
{
	if (vvBufferPos != 0)
		if (pt != PT_LINELIST)
			batch_fire();
	pt = PT_LINELIST;

	if (vvBufferPos >= LINEBBUFFERSZ * 2)
		batch_fire();

	vvBuffer[vvBufferPos++] = RHWVertex(D3DXVECTOR3(v0.x, 
		v0.y, 0.5f), color);

	vvBuffer[vvBufferPos++] = RHWVertex(D3DXVECTOR3(v1.x, 
		v1.y, 0.5f), color);

}

void batch_point( const D3DXVECTOR2& v, float size, DWORD color )
{
	if (vvBufferPos != 0)
		if ( pt != PT_POINTLIST )
			batch_fire();
	pt = PT_POINTLIST;

	vvBuffer[vvBufferPos++] = RHWVertex(D3DXVECTOR3(v.x,
		v.y, 0.5f), color, size);
}

float randomize(float min, float max)
{
	return ((float)rand() / RAND_MAX) * (max - min) + min;
}

void batch_circle(const D3DXVECTOR2& v, float r, DWORD color, unsigned char segments)
{
	//TODO: err prot
	float step = D3DX_PI * 2 / segments;
	D3DXVECTOR2 v1, v0 = v + D3DXVECTOR2(r, 0); // cos(0), sin(0)

	for (float i = step; i < D3DX_PI*2 + step; i += step)
	{
		v1 = v + D3DXVECTOR2(cosf(i) * r, sinf(i) * r);
		batch_line(v0, v1, color);
		v0 = v1;
	}
}

void addPoint(const D3DXVECTOR2& point)
{
	points[pointsNum++] = point;

	if (pointsNum >= MAXPOINTS)
		pointsNum = 0;
}

bool frame_move()
{
	input.update();

	if (input.isMouseDown(0))
	{
		POINT pt;
		GetCursorPos(&pt);
		ScreenToClient(hwnd, &pt);
		addPoint( D3DXVECTOR2((float)pt.x, (float)pt.y) );
	}

	if (input.isKeyDown(DIK_SPACE) )
	{
		runTriangulation();
	}

	if (input.isKeyDown(DIK_R))
	{
		pointsNum = 0;
		
		for (int i = 0; i < 800; i += 50)
			for (int j = 0; j < 600; j += 50)
				addPoint(D3DXVECTOR2(i + 50.f, j + 50.f));
		
		//for( int i = 0; i < 128; i++ )
		//	addPoint( D3DXVECTOR2( randomize(10.f, d3d9_GetBBWidth() - 10.f), randomize(10.f, d3d9_GetBBHeight() - 10.f) ) );
		
		runTriangulation();
	}

	return true;
}

void frame_render()
{
	HRESULT hr = S_OK;
	hr = pD3DDev9->BeginScene();
	if (FAILED(hr))
		return;
	
	pD3DDev9->Clear(0, 0, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0xFF7f7f7f, 1.0f, 0);

	if (input.isKeyPressed(DIK_E))
	{
		auto it = rawEdges.begin();
		while (it != rawEdges.end())
		{
			batch_line(points[KEYGETLOW(it->second)], points[KEYGETHIGH(it->second)], 0xFFFF0000);
			it++;
		}
	}
	else
	{
		auto it = finalEdges.begin();
		while (it != finalEdges.end())
		{
			batch_line(points[KEYGETLOW(it->second)], points[KEYGETHIGH(it->second)], 0xFFFF0000);
			it++;
		}
	}
	batch_fire();

	for (unsigned int i = 0; i < pointsNum; i++)
		batch_point(points[i], 10.f, 0xFF00FFFF);
	
	batch_fire();

	pD3DDev9->EndScene();
	pD3DDev9->Present(0, 0, 0, 0);
}

void cleanUp()
{
	cleanUpTriangulation();
	input.close();
}

int main()
{
	srand((unsigned int)time(0));

	testIntersection(0, 0, 100, 100, 400, 300, 600, 500);

	try
	{
		wnd_create("BSP LEVEL LOADER", 1024, 768);
		wnd_setFrameMoveCallBack(frame_move);
		wnd_setFrameRenderCallBack(frame_render);
		wnd_setCleanUpCallBack(cleanUp);
		d3d9_init( false );
		wnd_show();
		//pD3DDev9->Clear(0, 0, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0xFF7f7f7f, 1.0f, 0);
		wnd_update();
		input.init(hwnd);

		MSG msg = { 0 };
		while (true)
		{
			while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
			{
				if (msg.message == WM_QUIT)
				{
					cleanUp();
					return 0;
				};

				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
			if (frame_move())
				frame_render();
		}
	}
	catch (const char* msg)
	{
		wnd_hide();
		MessageBox(0, msg, "BSP Loader ",
			MB_OK | MB_ICONERROR | MB_SYSTEMMODAL);
	}
	cleanUp();
	return 0;
}