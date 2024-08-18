#include "vgacontrollers3.h"
#include "fabutils.h"

namespace fabgl
{

VGAControllerS3::VGAControllerS3 ()
{
    m_screenWidth=640;
    m_screenHeight=480;
    m_viewPortWidth=640;
    m_viewPortHeight=480;
    m_colorCount=64;
}
void VGAControllerS3::setResolution(Mode new_mode) 
{
    mode = new_mode;
    log_d ("frequency:%d",mode.frequency);
    log_d ("hRes:%d",mode.hRes);
    log_d ("vRes:%d",mode.vRes);

    // just in case setResolution() was called before
    end();

    m_screenWidth = m_viewPortWidth = mode.hRes;
    m_screenHeight = m_viewPortHeight = mode.vRes;

    m_timings.frequency = mode.frequency;
    m_timings.HBackPorch = mode.hBack;
    m_timings.HFrontPorch = mode.hFront;
    m_timings.HVisibleArea=mode.hRes;
    m_timings.HSyncPulse = mode.hSync;
    m_timings.VBackPorch = mode.vBack;
    m_timings.VFrontPorch = mode.vFront;
    m_timings.VSyncPulse = mode.vSync;
    m_timings.VVisibleArea=mode.vRes;
    m_timings.scanCount=mode.vClones;
    int linesize = m_timings.HFrontPorch + m_timings.HSyncPulse + m_timings.HBackPorch + m_timings.HVisibleArea;
    int viewportrow = 0;

    // inform base class about screen size
    setScreenSize(m_timings.HVisibleArea, m_timings.VVisibleArea);
    // create primitive queues
    setDoubleBuffered(false);
    // set default paint state (brushes, cliprects, origin)
    resetPaintState();

    // number of microseconds usable in VSynch ISR
    m_maxVSyncISRTime = ceil(1000000.0 / m_timings.frequency * m_timings.scanCount * linesize * (m_timings.VSyncPulse + m_timings.VBackPorch + m_timings.VFrontPorch + viewportrow));
    log_i("max vsync ISR time: %d",m_maxVSyncISRTime);

    if(vga.init(pins, mode, 8)) 
	{
        vga.show();
	    vga.start();
    }
    else
    {
		log_e ("VGA resolution change not successful\r\n");
		end();
	}
}
void VGAControllerS3::setResolution(char const * modeline, int viewPortWidth, int viewPortHeight, bool doubleBuffered)
{
    log_e ("FabGL modeline not yet supported");
}
void VGAControllerS3::begin()
{
    log_d ("VGAControllerS3::begin()");
    begin (fabgl::VGAControllerS3_PIN_AGON_LIGHT);
}
void VGAControllerS3::begin(PinConfig set_pins) 
{
    log_d ("VGAControllerS3::begin(PinConfig set_pins)");
    m_primitiveProcessingSuspended=0;
    pins = set_pins;

    //Create redraw task on 2nd core
    xTaskCreatePinnedToCore(redraw_task,
                            "redraw",
                            4096,
                            (void *)this,
                            REDRAW_TASK_PRIORITY,
                            &redraw_task_handle,
                            1);
	//Give task handle to VGA controller
	//this will notify
	vga.setRedrawTask (redraw_task_handle);
};
void VGAControllerS3::end()
{
    log_d ("VGAControllerS3::end()");
    m_primitiveProcessingSuspended=0;
    vga.stopVSyncInterupt();
}

NativePixelFormat VGAControllerS3::nativePixelFormat()
{
    log_d ("returning nativePixelFormat()");
    return NativePixelFormat::SBGR2222;
}
void VGAControllerS3::suspendBackgroundPrimitiveExecution()
{
    //ets_printf ("suspendBackgroundPrimitiveExecution %d\r\n",m_primitiveProcessingSuspended);
    m_primitiveProcessingSuspended++;
    if (m_primitiveProcessingSuspended == 1) 
    {
        vga.stopVSyncInterupt();
    }
}
void VGAControllerS3::resumeBackgroundPrimitiveExecution()
{
    //ets_printf ("resumeBackgroundPrimitiveExecution %d\r\n",m_primitiveProcessingSuspended);
    m_primitiveProcessingSuspended = tmax(0, m_primitiveProcessingSuspended - 1);
    if (m_primitiveProcessingSuspended == 0) 
    {
        vga.startVSyncInterrupt();
    }
}
void VGAControllerS3::readScreen(Rect const & rect, RGB888 * destBuf)
{
    log_d ("readScreen");
}
void IRAM_ATTR VGAControllerS3::setPixelAt(PixelDesc const & pixelDesc, Rect & updateRect)
{
    ets_printf ("setPixelAt\r\n");
}
void IRAM_ATTR VGAControllerS3::absDrawLine(int X1, int Y1, int X2, int Y2, RGB888 color)
{
    ets_printf ("absDrawLine\r\n");
}
void IRAM_ATTR VGAControllerS3::rawCopyRow(int x1, int x2, int srcY, int dstY)
{
    //ets_printf ("rawCopyRow from: %d,%d-%d,%d - to: %d,%d-%d,%d\r\n",x1,srcY,x2,srcY,x1,dstY,x2,dstY);
    // from x1,srcY to x1,dstY, width of x2-x1, height of 1
    vga.move_rect (x1,srcY,x1,dstY,x2-x1,1);
}
void IRAM_ATTR VGAControllerS3::rawFillRow(int y, int x1, int x2, RGB888 color)
{
    //ets_printf ("rawFillRow %d,%d,%d - %d,%d,%d\r\n",y,x1,x2,color.R,color.G,color.B);
    vga.fill_rect (vga.rgb_to_bits(color.R,color.G,color.B),x1,y,x2,y);
}
void IRAM_ATTR VGAControllerS3::drawEllipse(Size const & size, Rect & updateRect)
{
    ets_printf ("drawEllipse\r\n");
}
void IRAM_ATTR VGAControllerS3::clear(Rect & updateRect)
{
    if (updateRect.width()==m_screenWidth && 
        updateRect.height()==m_screenHeight)
    {
        //ets_printf ("clear ()\r\n");
        vga.clear (0);
    }
    else
    {
        //ets_printf ("fill_rect (%d,%d,%d,%d)\r\n",updateRect.X1,updateRect.Y1,updateRect.X2,updateRect.Y2);
        vga.fill_rect (vga.rgb_to_bits(0,0,0),updateRect.X1,updateRect.Y1,updateRect.X2,updateRect.Y2);
    }
}
void IRAM_ATTR VGAControllerS3::VScroll(int scroll, Rect & updateRect)
{
    //ets_printf ("VScroll\r\n");
    hideSprites(updateRect);
    RGB888 color = getActualBrushColor();
    int Y1 = paintState().scrollingRegion.Y1;
    int Y2 = paintState().scrollingRegion.Y2;
    int X1 = paintState().scrollingRegion.X1;
    int X2 = paintState().scrollingRegion.X2;
    int height = Y2 - Y1 + 1;

    if (scroll < 0) {

      // scroll UP

      for (int i = 0; i < height + scroll; ++i) {
        // copy X1..X2 of (Y1 + i - scroll) to (Y1 + i)
        rawCopyRow(X1, X2, (Y1 + i - scroll), (Y1 + i));
      }
      // fill lower area with brush color
      for (int i = height + scroll; i < height; ++i)
        rawFillRow(Y1 + i, X1, X2, color);

    } else if (scroll > 0) {

      // scroll DOWN
      for (int i = height - scroll - 1; i >= 0; --i) {
        // copy X1..X2 of (Y1 + i) to (Y1 + i + scroll)
        rawCopyRow(X1, X2, (Y1 + i), (Y1 + i + scroll));
      }

      // fill upper area with brush color
      for (int i = 0; i < scroll; ++i)
        rawFillRow(Y1 + i, X1, X2, color);

    }
}
void IRAM_ATTR VGAControllerS3::HScroll(int scroll, Rect & updateRect)
{
    ets_printf ("HScroll\r\n");
}
void IRAM_ATTR VGAControllerS3::drawGlyph(Glyph const & glyph, GlyphOptions glyphOptions, RGB888 penColor, RGB888 brushColor, Rect & updateRect)
{
    // ets_printf ("drawGlyph");
    // ets_printf ("penColor=%d,%d,%d\r\n",penColor.R,penColor.G,penColor.B);
    // ets_printf ("brushColor=%d,%d,%d\r\n",brushColor.R,brushColor.G,brushColor.B);

    const int clipX1 = paintState().absClippingRect.X1;
    const int clipY1 = paintState().absClippingRect.Y1;
    const int clipX2 = paintState().absClippingRect.X2;
    const int clipY2 = paintState().absClippingRect.Y2;

    const int origX = paintState().origin.X;
    const int origY = paintState().origin.Y;

    const int glyphX = glyph.X + origX;
    const int glyphY = glyph.Y + origY;

    // ets_printf ("(x=%d,y=%d)\r\n",glyphX,glyphY);
    // ets_printf ("cliprect (%d,%d,%d,%d)\r\n",clipX1,clipY1,clipX2,clipY2);

    if (glyphX > clipX2 || glyphY > clipY2)
      return;

    int16_t glyphWidth        = glyph.width;
    int16_t glyphHeight       = glyph.height;
    uint8_t const * glyphData = glyph.data;
    int16_t glyphWidthByte    = (glyphWidth + 7) / 8;

    int16_t X1 = 0;
    int16_t XCount = glyphWidth;
    int16_t destX = glyphX;

    int16_t Y1 = 0;
    int16_t YCount = glyphHeight;
    int destY = glyphY;

    if (destX < clipX1) {
      X1 = clipX1 - destX;
      destX = clipX1;
    }
    if (X1 >= glyphWidth)
      return;

    if (destX + XCount > clipX2 + 1)
      XCount = clipX2 + 1 - destX;
    if (X1 + XCount > glyphWidth)
      XCount = glyphWidth - X1;

    if (destY < clipY1) {
      Y1 = clipY1 - destY;
      destY = clipY1;
    }
    if (Y1 >= glyphHeight)
      return;

    if (destY + YCount > clipY2 + 1)
      YCount = clipY2 + 1 - destY;
    if (Y1 + YCount > glyphHeight)
      YCount = glyphHeight - Y1;

    updateRect = updateRect.merge(Rect(destX, destY, destX + XCount - 1, destY + YCount - 1));
    hideSprites(updateRect);

    if (glyphOptions.invert ^ paintState().paintOptions.swapFGBG)
      tswap(penColor, brushColor);

    // a very simple and ugly reduce luminosity (faint) implementation!
    if (glyphOptions.reduceLuminosity) {
      if (penColor.R > 128) penColor.R = 128;
      if (penColor.G > 128) penColor.G = 128;
      if (penColor.B > 128) penColor.B = 128;
    }

    bool fillBackground = glyphOptions.fillBackground;

    auto penPattern   = preparePixel(penColor);
    auto brushPattern = preparePixel(brushColor);

    for (int y = Y1; y < Y1 + YCount; ++y, ++destY) {
      //auto dstrow = rawGetRow(destY);
      uint8_t const * srcrow = glyphData + y * glyphWidthByte;

      uint32_t src = (srcrow[0] << 24) | (srcrow[1] << 16) | (srcrow[2] << 8) | (srcrow[3]);
      src <<= X1;
      if (fillBackground) {
        // filled background
        for (int x = X1, adestX = destX; x < X1 + XCount; ++x, ++adestX, src <<= 1)
          rawSetPixelInRow(destY, adestX, src & 0x80000000 ? penPattern : brushPattern);
      } else {
        // transparent background
        for (int x = X1, adestX = destX; x < X1 + XCount; ++x, ++adestX, src <<= 1)
          if (src & 0x80000000)
            rawSetPixelInRow(destY, adestX, penPattern);
      }
    }
}

inline void VGAControllerS3::rawSetPixelInRow (int y,int x,int color)
{
    vga.set_pixel (x,y,color);
}

void IRAM_ATTR VGAControllerS3::invertRect(Rect const & rect, Rect & updateRect)
{
    ets_printf ("invertRect\r\n");
}
void IRAM_ATTR VGAControllerS3::swapFGBG(Rect const & rect, Rect & updateRect)
{
    // ets_printf ("swapFGBG (%d,%d,%d,%d)\r\n",rect.X1,rect.Y1,rect.X2,rect.Y2);
    auto pen = paintState().penColor;
    auto brush = paintState().brushColor;
    auto penPattern   = preparePixel(pen);
    auto brushPattern = preparePixel(brush);
    // ets_printf ("penColor=%d,%d,%d\r\n",pen.R,pen.G,pen.B);
    // ets_printf ("brushColor=%d,%d,%d\r\n",brush.R,brush.G,brush.B);
    // ets_printf ("pen=%x, brush=%x\r\n",penPattern,brushPattern);

    int origX = paintState().origin.X;
    int origY = paintState().origin.Y;

    const int clipX1 = paintState().absClippingRect.X1;
    const int clipY1 = paintState().absClippingRect.Y1;
    const int clipX2 = paintState().absClippingRect.X2;
    const int clipY2 = paintState().absClippingRect.Y2;

    const int x1 = iclamp(rect.X1 + origX, clipX1, clipX2);
    const int y1 = iclamp(rect.Y1 + origY, clipY1, clipY2);
    const int x2 = iclamp(rect.X2 + origX, clipX1, clipX2);
    const int y2 = iclamp(rect.Y2 + origY, clipY1, clipY2);

    updateRect = updateRect.merge(Rect(x1, y1, x2, y2));
    hideSprites(updateRect);

    for (int y = y1; y <= y2; ++y) 
    {
        for (int x = x1; x <= x2; ++x) 
        {
            auto px = vga.get_pixel(x,y);
            //ets_printf ("source: %02x, pen: %02x,brush: %02x\r\n",px, penPattern, brushPattern);
            if (px == penPattern)
                vga.set_pixel(x,y, brushPattern);
            else if (px == brushPattern)
                vga.set_pixel(x,y, penPattern);
        }
    }                                                  
}
void IRAM_ATTR VGAControllerS3::copyRect(Rect const & source, Rect & updateRect)
{
    ets_printf ("copyRect()\r\n");
}
void VGAControllerS3::swapBuffers()
{
    log_d ("swapBuffers()");
    vga.show ();
}
int VGAControllerS3::getBitmapSavePixelSize()
{
    log_d ("getBitmapSavePixelSize()");
    return 1;
}
void IRAM_ATTR VGAControllerS3::rawDrawBitmap_Native(int destX, int destY, Bitmap const * bitmap, int X1, int Y1, int XCount, int YCount)
{
    ets_printf ("rawDrawBitmap_Native()\r\n");
}
void IRAM_ATTR VGAControllerS3::rawDrawBitmap_Mask(int destX, int destY, Bitmap const * bitmap, void * saveBackground, int X1, int Y1, int XCount, int YCount)
{
    ets_printf ("rawDrawBitmap_Mask()\r\n");
}
void IRAM_ATTR VGAControllerS3::rawDrawBitmap_RGBA2222(int destX, int destY, Bitmap const * bitmap, void * saveBackground, int X1, int Y1, int XCount, int YCount)
{
    ets_printf ("rawDrawBitmap_RGBA2222()\r\n");
}
void IRAM_ATTR VGAControllerS3::rawDrawBitmap_RGBA8888(int destX, int destY, Bitmap const * bitmap, void * saveBackground, int X1, int Y1, int XCount, int YCount)
{
    ets_printf ("rawDrawBitmap_RGBA8888()\r\n");
}

uint8_t VGAControllerS3::preparePixel(RGB222 rgb) 
{ 
	uint8_t pixel = vga.rgb_to_bits (rgb.R<<6,rgb.G<<6,rgb.R<<6);
    return pixel; //m_HVSync | (rgb.B << VGA_BLUE_BIT) | (rgb.G << VGA_GREEN_BIT) | (rgb.R << VGA_RED_BIT); 
}

void VGAControllerS3::redraw_task(void *pArg)
{
	int x=0,y=0;
	int px=0,py=0;
	int xd=1;
	int yd=1;
	const int dx=50;
	const int dy=50;
	uint32_t ulNotificationValue;
	int color=0,backcolor;
	float framerate=0.0;
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 20 );

    #ifdef LOG_FRAMERATE_S3
	unsigned long timePreviousFrame = millis ();
	unsigned long timeCurrentFrame;
	int framerate_counter = 0;
    #endif

	log_i ("+Redraw task started\r\n");

    VGAControllerS3* controller = (VGAControllerS3*)pArg;
	VGA& vga = controller->vga;
	// wait till next vsync
	while (true)
	{
		ulNotificationValue = ulTaskNotifyTake( pdTRUE,
												xMaxBlockTime );
		if (ulNotificationValue>1)
				log_e ("-%d frames skipped\r\n",ulNotificationValue-1);
		if( ulNotificationValue == 1 )
		{
            #ifdef LOG_FRAMERATE_S3
            timeCurrentFrame = millis();
			framerate += 1.0/((timeCurrentFrame-timePreviousFrame)*0.001);
			timePreviousFrame = timeCurrentFrame;
			if (framerate_counter==0)
			{
				framerate_counter = 60; // every second update
				log_i ("+Framerate: %f\r\n",framerate/framerate_counter);
                framerate = 0;
			}
            else 
			    framerate_counter--;
            #endif

            // clear old rectangle
			if (vga.getBufferCount()==1)
			{
				// single buffer
                // do the painting
                int64_t startTime = controller->backgroundPrimitiveTimeoutEnabled() ? esp_timer_get_time() : 0;
                Rect updateRect = Rect(SHRT_MAX, SHRT_MAX, SHRT_MIN, SHRT_MIN);
                do 
                {
                    Primitive prim;
                    if (controller->getPrimitiveISR(&prim) == false)
                        break;

                    controller->execPrimitive(prim, updateRect, true);

                    if (controller->m_primitiveProcessingSuspended)
                        break;

                } while (!controller->backgroundPrimitiveTimeoutEnabled() || (startTime + controller->m_maxVSyncISRTime > esp_timer_get_time()));
                // show the result
                vga.show();
            }
			else
			{
                // double buffering
                //
                // show contents of previous frame written
				vga.show();
			}
		}
		// else
		// {
		// 	/* The call to ulTaskNotifyTake() timed out. */
		// 	log_e ("-Error, waited too long for VSync\r\n");
		// }
	}
}

} // namespace FabGL