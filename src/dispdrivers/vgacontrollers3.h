#pragma once

#include "displaycontroller.h"
#include "dispdrivers/vgabasecontroller.h"
#include "bitluni/VGA.h"

namespace fabgl 
{
    // 6 bit version, agon compatible
    static const PinConfig VGAControllerS3_PIN_AGON_LIGHT(-1,-1,-1,16,15,  -1,-1,-1,-1,7,6,  -1,-1,-1,5,4,  17,18);
    
    class VGAControllerS3  : public BitmappedDisplayController
    {
        public:
            VGAControllerS3 ();
        
            // inherited from BaseDisplayController
            virtual void setResolution(char const * modeline, int viewPortWidth = -1, int viewPortHeight = -1, bool doubleBuffered = false);
            virtual void begin();
            virtual inline DisplayControllerType controllerType() {return DisplayControllerType::Bitmapped;};
            virtual inline int colorsCount() {return m_colorCount;};
            inline int getScreenWidth () {return m_screenWidth;};
            inline int getScreenHeight () {return m_screenHeight;};
            inline int getViewPortWidth () {return m_viewPortWidth;};
            inline int getViewPortHeight () {return m_viewPortHeight;};

            // inherited from BitmappedDisplayController
            virtual NativePixelFormat nativePixelFormat();
            virtual void suspendBackgroundPrimitiveExecution();
            virtual void resumeBackgroundPrimitiveExecution();
            virtual void readScreen(Rect const & rect, RGB888 * destBuf);
            virtual void setPixelAt(PixelDesc const & pixelDesc, Rect & updateRect);
            virtual void absDrawLine(int X1, int Y1, int X2, int Y2, RGB888 color);
            virtual void rawFillRow(int y, int x1, int x2, RGB888 color);
            virtual void drawEllipse(Size const & size, Rect & updateRect);
            virtual void clear(Rect & updateRect);
            virtual void VScroll(int scroll, Rect & updateRect);
            virtual void HScroll(int scroll, Rect & updateRect);
            virtual void drawGlyph(Glyph const & glyph, GlyphOptions glyphOptions, RGB888 penColor, RGB888 brushColor, Rect & updateRect);
            virtual void invertRect(Rect const & rect, Rect & updateRect);
            virtual void swapFGBG(Rect const & rect, Rect & updateRect);
            virtual void copyRect(Rect const & source, Rect & updateRect);
            virtual void swapBuffers();
            virtual int  getBitmapSavePixelSize();
            virtual void rawDrawBitmap_Native(int destX, int destY, Bitmap const * bitmap, int X1, int Y1, int XCount, int YCount);
            virtual void rawDrawBitmap_Mask(int destX, int destY, Bitmap const * bitmap, void * saveBackground, int X1, int Y1, int XCount, int YCount);
            virtual void rawDrawBitmap_RGBA2222(int destX, int destY, Bitmap const * bitmap, void * saveBackground, int X1, int Y1, int XCount, int YCount);
            virtual void rawDrawBitmap_RGBA8888(int destX, int destY, Bitmap const * bitmap, void * saveBackground, int X1, int Y1, int XCount, int YCount);

            // own specialised methods
            virtual void setResolution(Mode mode);
            inline void rawSetPixelInRow (int y,int x,int color);
            uint8_t IRAM_ATTR preparePixel(RGB222 rgb);
            void end ();
            void begin(PinConfig set_pins);
            void rawCopyRow(int x1, int x2, int srcY, int dstY);
            
        private:
            int m_colorCount;
            VGATimings m_timings;
            volatile int m_primitiveProcessingSuspended;
            VGA vga;
            Mode mode;
            PinConfig pins;
            volatile int16_t m_maxVSyncISRTime; // Maximum us VSync interrupt routine can run
            TaskHandle_t redraw_task_handle;
            const UBaseType_t REDRAW_TASK_PRIORITY=2;

		    static void redraw_task(void *pArg);
    };
}