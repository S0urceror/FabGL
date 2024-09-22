#ifdef CONFIG_IDF_TARGET_ESP32S3

#pragma once

#include "displaycontroller.h"
#include "dispdrivers/vgabasecontroller.h"
#ifdef BITLUNI
#include "bitluni/VGA.h"
using namespace bitluni;
#else
#include "devdrivers/VGA.h"
#endif
namespace fabgl 
{
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
            #ifdef BITLUNI
            virtual void setResolution(Mode mode,bool double_buffer);
            #else
            //virtual void setResolution(int width,int height,int color_depth,bool double_buffer);
            void setResolution(VGATimings const& timings, int color_depth, bool doubleBuffered);
            #endif
            void begin(PinConfig set_pins);
            void end ();

            uint8_t IRAM_ATTR preparePixel(RGB222 rgb);
            
            void rawCopyRow(int x1, int x2, int srcY, int dstY);
            inline uint8_t rawGetPixelInRow (int y,int x);
            inline void rawSetPixelInRow (int y,int x,int color);
            bool convertModelineToTimings(char const * modeline, VGATimings * timings);
            
        private:
            int m_colorCount;
            volatile int m_primitiveProcessingSuspended;
            PinConfig _pins;
            static void redraw_task(void *pArg);
            volatile int16_t m_maxVSyncISRTime; // Maximum us VSync interrupt routine can run
            TaskHandle_t redraw_task_handle;
            const UBaseType_t REDRAW_TASK_PRIORITY=2;
            VGATimings m_timings;
            
            #ifdef BITLUNI
            VGA vga;
            Mode mode;
            #else
            VGA vga;
            #endif
            
    };

    // 6 bit version, agon compatible
    static const PinConfig PIN_AGON_LIGHT(-1,-1,-1,15,16,  -1,-1,-1,-1,6,7,  -1,-1,-1,4,5,  17,18);
}

#endif //CONFIG_IDF_TARGET_ESP32S3