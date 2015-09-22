
//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

// Source altered and distributed from https://github.com/AdrienHerubel/imgui

#ifndef IMGUI_H
#define IMGUI_H

static const int BUTTON_HEIGHT = 20;
static const int SLIDER_HEIGHT = 20;
static const int SLIDER_MARKER_WIDTH = 5;
static const int CHECK_SIZE = 10;
static const int DEFAULT_VERTICAL_SPACING = 7;
static const int DEFAULT_HORIZONTAL_SPACING = 14;
static const int TEXT_HEIGHT = 10;
static const int FONT_HEIGHT = 30;
static const int SCROll_AREA_TOP_PADDING = 5;
static const int SCROLL_AREA_PADDING = 5;
static const int INDENT_SIZE = 16;
static const int AREA_HEADER = 20;
static const int SEPARATOR_LINE_WIDTH = 2;

enum imguiMouseButton
{
        IMGUI_MBUT_LEFT = 0x01, 
        IMGUI_MBUT_RIGHT = 0x02, 
};

enum imguiTextAlign
{
        IMGUI_ALIGN_LEFT,
        IMGUI_ALIGN_CENTER,
        IMGUI_ALIGN_RIGHT,
};

inline unsigned int imguiRGBA(unsigned char r, unsigned char g, unsigned char b, unsigned char a=255)
{
        return (r) | (g << 8) | (b << 16) | (a << 24);
}

void imguiBeginFrame(int mx, int my, unsigned char mbut, int scroll);
void imguiEndFrame();

bool imguiBeginScrollArea(const char* name, int x, int y, int w, int h, int* scroll);
void imguiEndScrollArea();

void imguiStartLine();
void imguiEndLine();
void imguiIndent();
void imguiUnindent();
void imguiVerticalSpace(int spaceY);
void imguiHorizontalSpace(int spaceX);
void imguiSeparatorLine();

bool imguiButton(const char* text, bool enabled = true, int width = -1, int height = -1, float scaleX=1, float scaleY=1);
bool imguiItem(const char* text, bool enabled = true, float scaleX=1, float scaleY=1);
bool imguiCheck(const char* text, bool checked, bool enabled = true, float scaleX = 1, float scaleY = 1);
bool imguiCollapse(const char* text, const char* subtext, bool checked, bool enabled = true);
void imguiLabel(const char* text);
void imguiValue(const char* text);
bool imguiSlider(const char* text, float* val, float vmin, float vmax, float vinc, bool enabled = true, float scalingX = 1.0f, float scalingY = 1.0f);

void imguiDrawText(int x, int y, int align, const char* text, unsigned int color);
void imguiDrawLine(float x0, float y0, float x1, float y1, float r, unsigned int color);
void imguiDrawRoundedRect(float x, float y, float w, float h, float r, unsigned int color);
void imguiDrawRect(float x, float y, float w, float h, unsigned int color);

// Pull render interface.
enum imguiGfxCmdType
{
        IMGUI_GFXCMD_RECT,
        IMGUI_GFXCMD_TRIANGLE,
        IMGUI_GFXCMD_LINE,
        IMGUI_GFXCMD_TEXT,
        IMGUI_GFXCMD_SCISSOR,
};

struct imguiGfxRect
{
        short x,y,w,h,r;
};

struct imguiGfxText
{
        short x,y,align;
        const char* text;
};

struct imguiGfxLine
{
        short x0,y0,x1,y1,r;
};

struct imguiGfxCmd
{
        char type;
        char flags;
        char pad[2];
        unsigned int col;
        union
        {
                imguiGfxLine line;
                imguiGfxRect rect;
                imguiGfxText text;
        };
};

const imguiGfxCmd* imguiGetRenderQueue();
int imguiGetRenderQueueSize();


#endif // IMGUI_H
