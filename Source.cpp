#include <Windows.h>
#include <stdlib.h>
#include "util.h"
#include "input.h"
#include <time.h>
#include <map>
#include <math.h>
#include "poly2tri/poly2tri.h"

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

#define LINEBBUFFERSZ 0xFFFFF
#define PT_LINELIST 0
#define PT_UNSELECTED -1
#define PT_POINTLIST 1

RHWVertex vvBuffer[ LINEBBUFFERSZ * 2 ];
unsigned int vvBufferPos = 0;
char pt = -1;
gInput input;

std::vector<D3DXVECTOR2> points;
std::vector<D3DXVECTOR2> stainer_points;

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
// for p2t lib:
p2t::CDT *c = nullptr;
std::vector<p2t::Triangle *> tris_vec;
std::vector<p2t::Point *> polyline;

float edgeLenght2(unsigned short v0, unsigned short v1, const std::vector<D3DXVECTOR2> &in_points)
{
	D3DXVECTOR2 d = in_points[v1] - in_points[v0];
	return D3DXVec2LengthSq(&d);
}

float edgeLenght( unsigned short v0, unsigned short v1 )
{
	D3DXVECTOR2 d = points[v1] - points[v0];
	return sqrtf( d.x * d.x + d.y * d.y );
}



void cleanUpTriangulation()
{
	rawEdges.clear();
	finalEdges.clear();
	tris_vec.clear();
	for (int i = 0; i < polyline.size(); i++)
		delete polyline[i];
	polyline.clear();
}

void clearPoints()
{
	points.clear();
	stainer_points.clear();
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
		return false; // �������� �� ��� ������ �� ������������

	float z = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
	float cta = (x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3);
	float ctb = (x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3);

	if (fabsf(z) < EPSILON)
	{
		if ((fabsf(cta) < EPSILON) && (fabsf(ctb) < EPSILON))
		{
			return false; //����������� � �� ����� �����
		}
		else
			return false; // ������� �����������, �������� ���� ��������� ���������� ����� ����...
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
		return false; // �������� �� ��� ������ �� ������������

	float z = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
	float cta = (x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3);
	float ctb = (x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3);

	if (fabsf(z) < EPSILON)
	{
		/*
		if ((fabsf(cta) < EPSILON) && (fabsf(ctb) < EPSILON))
		{
			return false; //����������� � �� ����� �����
		}
		else
			return false; // ������� �����������, �������� ���� ��������� ���������� ����� ����...
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

	if (points.size() + stainer_points.size() <= 3)
		return;

	std::vector<D3DXVECTOR2> all_points;
	all_points.insert(all_points.begin(), points.begin(), points.end());
	all_points.insert(all_points.end(), stainer_points.begin(), stainer_points.end());

	for (unsigned int i = 0; i < all_points.size(); i++)
	{
		for (unsigned int j = i + 1; j < all_points.size(); j++)
		{
			rawEdges.insert( std::pair<float, unsigned int> ( edgeLenght2(i, j, all_points), KEYGEN(i, j)) );
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

			if ( testIntersection( all_points[p0].x, all_points[p0].y, all_points[p1].x, all_points[p1].y,
				all_points[p2].x, all_points[p2].y, all_points[p3].x, all_points[p3].y ))
			{
				intersected = true;
				break;
			}
			itf++;
		}

		if (!intersected)
		{
			finalEdges.insert( std::pair< float, unsigned int >( it->first, it->second ) );
			it = rawEdges.erase( it );
		}
		else
			it++;
	}
}

void runTriangulationByLib()
{
	cleanUpTriangulation();

	if (points.size() + stainer_points.size() <= 3)
		return;

	polyline.reserve(points.size());
	for (int i = 0; i < points.size(); i++)
		polyline.push_back(new p2t::Point(points[i].x, points[i].y));

	if (c)
		delete c;
	c = new p2t::CDT(polyline);
	
	for (int i = 0; i < stainer_points.size(); i++)
		c->AddPoint(new p2t::Point(stainer_points[i].x, stainer_points[i].y));
	
	c->Triangulate();
	tris_vec = std::move(c->GetTriangles());
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

void batch_point(const D3DXVECTOR2 &v, float size, DWORD color)
{
	if (vvBufferPos != 0)
		if (pt != PT_POINTLIST)
			batch_fire();
	pt = PT_POINTLIST;

	vvBuffer[vvBufferPos++] = RHWVertex(D3DXVECTOR3(v.x,
		v.y, 0.5f), color, size);
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

void batch_line(const p2t::Point *p0, const p2t::Point *p1, DWORD color)
{
	batch_line(
		D3DXVECTOR2(static_cast<float>(p0->x), static_cast<float>(p0->y)), 
		D3DXVECTOR2(static_cast<float>(p1->x), static_cast<float>(p1->y)), color);
}

void batch_triangle(p2t::Triangle *triangle)
{
	auto pt0 = triangle->GetPoint(0);
	auto pt1 = triangle->GetPoint(1);
	auto pt2 = triangle->GetPoint(2);

	batch_line(pt0, pt1, 0xFFFFFFFF);
	batch_line(pt1, pt2, 0xFFFFFFFF);
	batch_line(pt2, pt0, 0xFFFFFFFF);
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

void addPoint(const D3DXVECTOR2 &point)
{
	points.push_back(point);
}

void addSteiner(const D3DXVECTOR2 &point)
{
	stainer_points.push_back(point);
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

	if (input.isMouseDown(1))
	{
		POINT pt;
		GetCursorPos(&pt);
		ScreenToClient(hwnd, &pt);
		addSteiner(D3DXVECTOR2((float)pt.x, (float)pt.y));
	}

	if (input.isKeyDown(DIK_C))
	{
		cleanUpTriangulation();
		clearPoints();
	}

	if (input.isKeyDown(DIK_F))
	{
		cleanUpTriangulation();
		clearPoints();

		for (int i = 0; i < 800; i += 50) 
		{
			for (int j = 0; j < 600; j += 50)
			{
				if (i == 0 || i == 799 || j == 0 || j == 599)
					addSteiner(D3DXVECTOR2(i + 50.f, j + 50.f));
				else
					addPoint(D3DXVECTOR2(i + 50.f, j + 50.f));
			}
		}
	}

	if (input.isKeyDown(DIK_R))
	{
		cleanUpTriangulation();
		clearPoints();

		addPoint(D3DXVECTOR2(10, 10));
		addPoint(D3DXVECTOR2(d3d9_GetBBWidth() - 10, 10));
		addPoint(D3DXVECTOR2(d3d9_GetBBWidth() - 10, d3d9_GetBBHeight() - 10));
		addPoint(D3DXVECTOR2(10, d3d9_GetBBHeight() - 10));

		for (int i = 0; i < 10000; i++)
			addSteiner(D3DXVECTOR2(randomize(20, d3d9_GetBBWidth() - 20), randomize(20, d3d9_GetBBHeight() - 20)));
	}

	if (input.isKeyDown(DIK_T))
		runTriangulationByLib();
	if (input.isKeyDown(DIK_SPACE))
		runTriangulation();

	return true;
}

void frame_render()
{
	HRESULT hr = S_OK;
	hr = pD3DDev9->BeginScene();
	if (FAILED(hr))
		return;
	
	pD3DDev9->Clear(0, 0, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0xFF7f7f7f, 1.0f, 0);

	if (tris_vec.size() > 0)
	{
		auto it = tris_vec.begin();
		while (it != tris_vec.end())
		{
			batch_triangle(*it);
			it++;
		}
	}

	if (input.isKeyPressed(DIK_E))
	{
		auto it = rawEdges.begin();
		while (it != rawEdges.end())
		{
			int i0 = KEYGETLOW(it->second);
			int i1 = KEYGETHIGH(it->second);
			batch_line(
				i0 < points.size() ? points[i0] : stainer_points[i0 - points.size()], 
				i1 < points.size() ? points[i1] : stainer_points[i1 - points.size()], 0xFFFF0000);
			it++;
		}
	}
	else
	{
		auto it = finalEdges.begin();
		while (it != finalEdges.end())
		{
			int i0 = KEYGETLOW(it->second);
			int i1 = KEYGETHIGH(it->second);
			batch_line(
				i0 < points.size() ? points[i0] : stainer_points[i0 - points.size()], 
				i1 < points.size() ? points[i1] : stainer_points[i1 - points.size()], 0xFFFF0000);
			it++;
		}
	}
	batch_fire();

	for (unsigned int i = 0; i < points.size(); i++)
		batch_point(points[i], 2.f, 0xFF00FFFF);
	for (unsigned int i = 0; i < stainer_points.size(); i++)
		batch_point(stainer_points[i], 2.f, 0xFFFF00FF);
	
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

	points.reserve(0xFFFF);
	stainer_points.reserve(0xFFFF);

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