/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2015 Daniel Chappuis                                       *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                               *
* Permission is granted to anyone to use this software for any purpose,         *
* including commercial applications, and to alter it and redistribute it        *
* freely, subject to the following restrictions:                                *
*                                                                               *
* 1. The origin of this software must not be misrepresented; you must not claim *
*    that you wrote the original software. If you use this software in a        *
*    product, an acknowledgment in the product documentation would be           *
*    appreciated but is not required.                                           *
*                                                                               *
* 2. Altered source versions must be plainly marked as such, and must not be    *
*    misrepresented as being the original software.                             *
*                                                                               *
* 3. This notice may not be removed or altered from any source distribution.    *
*                                                                               *
********************************************************************************/

#ifndef GUI_H
#define	GUI_H

// Libraries
#include <imgui.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

// Class Gui
class Gui {

    protected :

        // -------------------- Constants -------------------- //


        // -------------------- Attributes -------------------- //

        // TODO : Delete this
        static GLFWwindow* mWindow;

        static double       g_Time;
        static bool         g_MousePressed[3];
        static float        g_MouseWheel;
        static GLuint       g_FontTexture;
        static int          g_ShaderHandle, g_VertHandle, g_FragHandle;
        static int          g_AttribLocationTex, g_AttribLocationProjMtx;
        static int          g_AttribLocationPosition, g_AttribLocationUV, g_AttribLocationColor;
        static size_t       g_VboSize;
        static unsigned int g_VboHandle, g_VaoHandle;


        // -------------------- Methods -------------------- //

        void createDeviceObjects();

        void createFontTextures();

        void beginNewFrame();

        static void renderDrawLists(ImDrawList** const cmd_lists, int cmd_lists_count);

        static const char* getClipboardText();

        static void setClipboardText(const char* text);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        Gui();

        /// Destructor
        ~Gui();

        /// Create and return the singleton instance of this class
        static Gui& getInstance();

        /// Initialize the GUI
        void init();

        /// Display the GUI
        void render();

        static void setWindow(GLFWwindow* window);
};

inline void Gui::setWindow(GLFWwindow* window) {
    mWindow = window;
}

#endif
