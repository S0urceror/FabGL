#ifdef CONFIG_IDF_TARGET_ESP32S3

#include "vgacontrollers3.h"
#include "fabutils.h"
#include "esp32s3/rom/ets_sys.h"
namespace fabgl
{
#pragma GCC optimize ("O2")

VGAControllerS3::VGAControllerS3 ()
{
    m_screenWidth=640;
    m_screenHeight=480;
    m_viewPortWidth=640;
    m_viewPortHeight=480;
    m_colorCount=64;
    redraw_task_handle = NULL;
    m_initialized=false;
}


// Convert RGBA8888 Buffer to RGBA2222 
uint8_t IRAM_ATTR VGAControllerS3::convert_rgba8888_to_rgba2222(const RGBA8888 * value) {
    RGBA2222 rgba2222 = { 0, 0, 0, 0 };
    rgba2222.R = value->R/85;
    rgba2222.G = value->G/85;
    rgba2222.B = value->B/85;
    rgba2222.A = value->A/85;
    return *((uint8_t*)&rgba2222);
}

uint8_t IRAM_ATTR VGAControllerS3::convert_rgb888_to_rgba2222(const RGB888 * value) {
    RGBA2222 rgba2222 = { 0, 0, 0, 0};
    rgba2222.R = value->R/85;
    rgba2222.G = value->G/85;
    rgba2222.B = value->B/85;
    return *((uint8_t*)&rgba2222);
}

#ifdef BITLUNI
void VGAControllerS3::setResolution(Mode new_mode,bool double_buffer) 
{
    mode = new_mode;
    ets_printf ("frequency:%d\r\n",mode.frequency);
    ets_printf ("hRes:%d\r\n",mode.hRes);
    ets_printf ("vRes:%d\r\n",mode.vRes);

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
    setDoubleBuffered(double_buffer);
    // set default paint state (brushes, cliprects, origin)
    resetPaintState();

    // number of microseconds usable in VSynch ISR
    m_maxVSyncISRTime = ceil(1000000.0 / m_timings.frequency * m_timings.scanCount * linesize * (m_timings.VSyncPulse + m_timings.VBackPorch + m_timings.VFrontPorch + viewportrow));
    // ets_printf("max vsync ISR time: %d\r\n",m_maxVSyncISRTime);

    if(vga.init(_pins, mode, 8,double_buffer)) 
	{
        vga.show();
	    vga.start();
      m_initialized=true;
    }
    else
    {
		ets_printf ("VGA resolution change not successful\r\n");
		end();
	}
}
#else
//void VGAControllerS3::setResolution(int width,int height,int color_depth,bool double_buffer) 
void VGAControllerS3::setResolution(VGATimings const& timings, int color_depth, bool doubleBuffered)
{
    // just in case setResolution() was called before
    // end();
    m_timings = timings;
    m_screenWidth = m_viewPortWidth = timings.HVisibleArea;
    m_screenHeight = m_viewPortHeight = timings.VVisibleArea;

    int linesize = timings.HFrontPorch + timings.HSyncPulse + timings.HBackPorch + timings.HVisibleArea;
    int viewportrow = 0;
    
    // inform base class about screen size
    setScreenSize(m_screenWidth, m_screenHeight);
    // create primitive queues
    setDoubleBuffered(doubleBuffered);
    // setDoubleBuffered(false);
    // set default paint state (brushes, cliprects, origin)
    resetPaintState();

    // if (doubleBuffered)
    //   enableBackgroundPrimitiveTimeout(false);

    // number of microseconds usable in VSynch ISR
    m_maxVSyncISRTime = ceil(1000000.0 / timings.frequency * timings.scanCount * linesize * (timings.VSyncPulse + timings.VBackPorch + timings.VFrontPorch + viewportrow));
    //ets_printf("max vsync ISR time: %d\r\n",m_maxVSyncISRTime);

    if (vga.init(timings, color_depth, _pins, true, doubleBuffered, false))
      m_initialized=true;
    else
      end();
}
#endif

// modeline syntax:
//   "label" clock_mhz hdisp hsyncstart hsyncend htotal vdisp vsyncstart vsyncend vtotal [(+HSync | -HSync) (+VSync | -VSync)] [DoubleScan | QuadScan] [FrontPorchBegins | SyncBegins | BackPorchBegins | VisibleBegins] [MultiScanBlank]
bool VGAControllerS3::convertModelineToTimings(char const * modeline, VGATimings * timings)
{
  float freq;
  int hdisp, hsyncstart, hsyncend, htotal, vdisp, vsyncstart, vsyncend, vtotal;
  char HSyncPol = 0, VSyncPol = 0;
  int pos = 0;
// #define                                             VGA_640x480_60Hz "\"640x480@60Hz\" 25      640       656       752         800       480       490       492         525 -HSync -VSync"
  int count = sscanf(modeline, "\"%[^\"]\" %g %d %d %d %d %d %d %d %d %n", timings->label, &freq, &hdisp, &hsyncstart, &hsyncend, &htotal, &vdisp, &vsyncstart, &vsyncend, &vtotal, &pos);

  if (count == 10 && pos > 0) {

    timings->frequency      = freq * 1000000;
    timings->HVisibleArea   = hdisp;
    timings->HFrontPorch    = hsyncstart - hdisp;
    timings->HSyncPulse     = hsyncend - hsyncstart;
    timings->HBackPorch     = htotal - hsyncend;
    timings->VVisibleArea   = vdisp;
    timings->VFrontPorch    = vsyncstart - vdisp;
    timings->VSyncPulse     = vsyncend - vsyncstart;
    timings->VBackPorch     = vtotal - vsyncend;
    timings->HSyncLogic     = '-';
    timings->VSyncLogic     = '-';
    timings->scanCount      = 1;
    timings->multiScanBlack = 0;
    timings->HStartingBlock = VGAScanStart::VisibleArea;

    // actually this checks just the first character
    auto pc  = modeline + pos;
    auto end = pc + strlen(modeline);
    while (*pc && pc < end) {
      switch (*pc) {
        // parse [(+HSync | -HSync) (+VSync | -VSync)]
        case '+':
        case '-':
          if (!HSyncPol)
            timings->HSyncLogic = HSyncPol = *pc;
          else if (!VSyncPol)
            timings->VSyncLogic = VSyncPol = *pc;
          pc += 6;
          break;
        // parse [DoubleScan | QuadScan]
        // DoubleScan
        case 'D':
        case 'd':
          timings->scanCount = 2;
          pc += 10;
          break;
        // QuadScan
        case 'Q':
        case 'q':
          timings->scanCount = 4;
          pc += 8;
          break;
        // parse [FrontPorchBegins | SyncBegins | BackPorchBegins | VisibleBegins] [MultiScanBlank]
        // FrontPorchBegins
        case 'F':
        case 'f':
          timings->HStartingBlock = VGAScanStart::FrontPorch;
          pc += 16;
          break;
        // SyncBegins
        case 'S':
        case 's':
          timings->HStartingBlock = VGAScanStart::Sync;
          pc += 10;
          break;
        // BackPorchBegins
        case 'B':
        case 'b':
          timings->HStartingBlock = VGAScanStart::BackPorch;
          pc += 15;
          break;
        // VisibleBegins
        case 'V':
        case 'v':
          timings->HStartingBlock = VGAScanStart::VisibleArea;
          pc += 13;
          break;
        // MultiScanBlank
        case 'M':
        case 'm':
          timings->multiScanBlack = 1;
          pc += 14;
          break;
        case ' ':
          ++pc;
          break;
        default:
          return false;
      }
    }

    return true;

  }
  return false;
}
void VGAControllerS3::setResolution(char const * modeline, int viewPortWidth, int viewPortHeight, bool doubleBuffered)
{
  #ifndef BITLUNI
    VGATimings timings;
    if (convertModelineToTimings(modeline, &timings))
        setResolution(timings, 8, doubleBuffered);
  #else
    ets_printf ("modeline not yet implemented");
  #endif
}

void VGAControllerS3::begin()
{
    //ets_printf ("VGAControllerS3::begin()\r\n");
    begin (PIN_AGON_LIGHT);
}

void VGAControllerS3::begin(PinConfig set_pins) 
{
    //ets_printf ("VGAControllerS3::begin(PinConfig set_pins)\r\n");
    m_primitiveProcessingSuspended=0;
    _pins = set_pins;

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
#ifdef BITLUNI
void VGAControllerS3::end()
{
    ets_printf ("VGAControllerS3::end()\r\n");
    m_primitiveProcessingSuspended=1;
    vga.stopVSyncInterupt();
    m_initialized=false;
}
#else
void VGAControllerS3::end()
{
    //ets_printf ("VGAControllerS3::end()\r\n");
    if (redraw_task_handle)
    {
        vga.setRedrawTask (NULL);
        vTaskDelete (redraw_task_handle);
    }

    m_primitiveProcessingSuspended=1;
    vga.deinit();
    m_initialized=false;
}
#endif
NativePixelFormat VGAControllerS3::nativePixelFormat()
{
    //ets_printf ("returning nativePixelFormat()\r\n");
    return NativePixelFormat::SBGR2222;
}
#ifdef BITLUNI
void VGAControllerS3::suspendBackgroundPrimitiveExecution()
{
    ets_printf ("suspendBackgroundPrimitiveExecution %d\r\n",m_primitiveProcessingSuspended);
    m_primitiveProcessingSuspended++;
    if (m_primitiveProcessingSuspended == 1) 
    {
        vga.stopVSyncInterupt();
    }
}
void VGAControllerS3::resumeBackgroundPrimitiveExecution()
{
    ets_printf ("resumeBackgroundPrimitiveExecution %d\r\n",m_primitiveProcessingSuspended);
    m_primitiveProcessingSuspended = tmax(0, m_primitiveProcessingSuspended - 1);
    if (m_primitiveProcessingSuspended == 0) 
    {
        vga.startVSyncInterrupt();
    }
}
#else
void VGAControllerS3::suspendBackgroundPrimitiveExecution()
{
    //ets_printf ("suspendBackgroundPrimitiveExecution %d\r\n",m_primitiveProcessingSuspended);
    m_primitiveProcessingSuspended++;
}
void VGAControllerS3::resumeBackgroundPrimitiveExecution()
{
    //ets_printf ("resumeBackgroundPrimitiveExecution %d\r\n",m_primitiveProcessingSuspended);
    m_primitiveProcessingSuspended = tmax(0, m_primitiveProcessingSuspended - 1);
}
#endif
void VGAControllerS3::readScreen(Rect const & rect, RGB888 * destBuf)
{
    // ets_printf ("readScreen\r\n");
    vga.bits_to_rgb(rawGetPixelInRow(rect.Y1, rect.X1), destBuf->R, destBuf->G, destBuf->B);
}
void IRAM_ATTR VGAControllerS3::setPixelAt(PixelDesc const & pixelDesc, Rect & updateRect)
{
    const int x = pixelDesc.pos.X + paintState().origin.X;
    const int y = pixelDesc.pos.Y + paintState().origin.Y;

    const int clipX1 = paintState().absClippingRect.X1;
    const int clipY1 = paintState().absClippingRect.Y1;
    const int clipX2 = paintState().absClippingRect.X2;
    const int clipY2 = paintState().absClippingRect.Y2;

    if (x >= clipX1 && x <= clipX2 && y >= clipY1 && y <= clipY2) {
      updateRect = updateRect.merge(Rect(x, y, x, y));
      hideSprites(updateRect);
      vga.set_pixel (x,y,pixelDesc.color.R,pixelDesc.color.G,pixelDesc.color.B);
    } 
}
#define VGA8_INVERTPIXEL(y,x)        rawSetPixelInRow(y, x, rawGetPixelInRow(y, x) ^ 0b11111111) // 0b11111111 = 11111111b, invert RGB2222 pixel
#define VGA8_INVERTROW(y, x1, x2) \
  do { \
    for (int _x = (x1); _x <= (x2); ++_x) { \
      VGA8_INVERTPIXEL(y, _x); \
    } \
  } while (0)
#define VGA8_INVERTCOL(x, y1, y2) \
  do { \
    for (int _y = (y1); _y <= (y2); ++_y) { \
      VGA8_INVERTPIXEL(_y, x); \
    } \
  } while (0)
void IRAM_ATTR VGAControllerS3::absDrawLine(int X1, int Y1, int X2, int Y2, RGB888 color)
{
    if (paintState().penWidth > 1) {
      absDrawThickLine(X1, Y1, X2, Y2, paintState().penWidth, color);
      return;
    }
    auto pattern =preparePixel(color);
    if (Y1 == Y2) {
      // horizontal line
      if (Y1 < paintState().absClippingRect.Y1 || Y1 > paintState().absClippingRect.Y2)
        return;
      if (X1 > X2)
        tswap(X1, X2);
      if (X1 > paintState().absClippingRect.X2 || X2 < paintState().absClippingRect.X1)
        return;
      X1 = iclamp(X1, paintState().absClippingRect.X1, paintState().absClippingRect.X2);
      X2 = iclamp(X2, paintState().absClippingRect.X1, paintState().absClippingRect.X2);
      if (paintState().paintOptions.NOT){
        VGA8_INVERTROW(Y1, X1, X2);
        //rawInvertRow(Y1, X1, X2);
        // for (int x = X1; x <= X2; ++x){
          // rawSetPixelInRow(Y1, x, rawGetPixelInRow(Y1, x) ^ 0x3f); // 0x3f = 00111111b, invert RGB2222 pixel
        // }
      }
      else{
        // rawFillRow(Y1, X1, X2, pattern);
        for (int x = X1; x <= X2; ++x){
          rawSetPixelInRow(Y1, x, pattern); 
        }
      }
    } else if (X1 == X2) {
      // vertical line
      if (X1 < paintState().absClippingRect.X1 || X1 > paintState().absClippingRect.X2)
        return;
      if (Y1 > Y2)
        tswap(Y1, Y2);
      if (Y1 > paintState().absClippingRect.Y2 || Y2 < paintState().absClippingRect.Y1)
        return;
      Y1 = iclamp(Y1, paintState().absClippingRect.Y1, paintState().absClippingRect.Y2);
      Y2 = iclamp(Y2, paintState().absClippingRect.Y1, paintState().absClippingRect.Y2);
      if (paintState().paintOptions.NOT) {
        // log_d("absDrawLine: NOT paint option enabled, inverting pixels");
        VGA8_INVERTCOL(X1, Y1, Y2);
        // for (int y = Y1; y <= Y2; ++y)
        //   rawSetPixelInRow(y, X1, rawGetPixelInRow(y, X1) ^ 0x3f); // 0x3f = 00111111b, invert RGB2222 pixel

      } else {
        // log_d("absDrawLine: NOT paint option disabled, setting pixels to pattern");
        for (int y = Y1; y <= Y2; ++y)
          rawSetPixelInRow(y, X1, pattern);
      }
    } else {
      // other cases (Bresenham's algorithm)
      // TODO: to optimize
      //   Unfortunately here we cannot clip exactly using Sutherland-Cohen algorithm (as done before)
      //   because the starting line (got from clipping algorithm) may not be the same of Bresenham's
      //   line (think to continuing an existing line).
      //   Possible solutions:
      //      - "Yevgeny P. Kuzmin" algorithm:
      //               https://stackoverflow.com/questions/40884680/how-to-use-bresenhams-line-drawing-algorithm-with-clipping
      //               https://github.com/ktfh/ClippedLine/blob/master/clip.hpp
      // For now Sutherland-Cohen algorithm is only used to check the line is actually visible,
      // then test for every point inside the main Bresenham's loop.
      if (!clipLine(X1, Y1, X2, Y2, paintState().absClippingRect, true))  // true = do not change line coordinates!
        return;
      const int dx = abs(X2 - X1);
      const int dy = abs(Y2 - Y1);
      const int sx = X1 < X2 ? 1 : -1;
      const int sy = Y1 < Y2 ? 1 : -1;
      int err = (dx > dy ? dx : -dy) / 2;
      while (true) {
        if (paintState().absClippingRect.contains(X1, Y1)) {
          if (paintState().paintOptions.NOT)
            VGA8_INVERTPIXEL(Y1, X1);
            // rawSetPixelInRow(Y1, X1, rawGetPixelInRow(Y1, X1) ^ 0x3f); // 0x3f = 00111111b, invert RGB2222 pixel
          else
            rawSetPixelInRow(Y1, X1, pattern);
        }
        if (X1 == X2 && Y1 == Y2)
          break;
        int e2 = err;
        if (e2 > -dx) {
          err -= dy;
          X1 += sx;
        }
        if (e2 < dy) {
          err += dx;
          Y1 += sy;
        }
      }
    }
}
void IRAM_ATTR VGAControllerS3::rawCopyRow(int x1, int x2, int srcY, int dstY)
{
    //ets_printf ("rawCopyRow from: %d,%d-%d,%d - to: %d,%d-%d,%d\r\n",x1,srcY,x2,srcY,x1,dstY,x2,dstY);
    // from x1,srcY to x1,dstY, width of x2-x1+1, height of 1
    vga.move_rect (x1,srcY,x1,dstY,x2-x1+1,1);
}
void IRAM_ATTR VGAControllerS3::rawFillRow(int y, int x1, int x2, RGB888 color)
{
    //ets_printf ("rawFillRow %d,%d,%d - %d,%d,%d\r\n",y,x1,x2,color.R,color.G,color.B);
    vga.fill_rect (vga.rgb_to_bits(color.R,color.G,color.B),x1,y,x2,y);
}
void IRAM_ATTR VGAControllerS3::drawEllipse(Size const & size, Rect & updateRect)
{
    auto color = getActualPenColor();
    auto pattern = preparePixel( color );

    const int clipX1 = paintState().absClippingRect.X1;
    const int clipY1 = paintState().absClippingRect.Y1;
    const int clipX2 = paintState().absClippingRect.X2;
    const int clipY2 = paintState().absClippingRect.Y2;

    const int centerX = paintState().position.X;
    const int centerY = paintState().position.Y;

    const int halfWidth  = size.width / 2;
    const int halfHeight = size.height / 2;

    updateRect = updateRect.merge(Rect(centerX - halfWidth, centerY - halfHeight, centerX + halfWidth, centerY + halfHeight));
    hideSprites(updateRect);

    const int a2 = halfWidth * halfWidth;
    const int b2 = halfHeight * halfHeight;
    const int crit1 = -(a2 / 4 + halfWidth % 2 + b2);
    const int crit2 = -(b2 / 4 + halfHeight % 2 + a2);
    const int crit3 = -(b2 / 4 + halfHeight % 2);
    const int d2xt = 2 * b2;
    const int d2yt = 2 * a2;
    int x = 0;          // travels from 0 up to halfWidth
    int y = halfHeight; // travels from halfHeight down to 0
    int t = -a2 * y;
    int dxt = 2 * b2 * x;
    int dyt = -2 * a2 * y;

    while (y >= 0 && x <= halfWidth) {
      const int col1 = centerX - x;
      const int col2 = centerX + x;
      const int row1 = centerY - y;
      const int row2 = centerY + y;

      if (col1 >= clipX1 && col1 <= clipX2) {
        if (row1 >= clipY1 && row1 <= clipY2)
          rawSetPixelInRow(row1, col1, pattern);
        if (row2 >= clipY1 && row2 <= clipY2)
          rawSetPixelInRow(row2, col1, pattern);
      }
      if (col2 >= clipX1 && col2 <= clipX2) {
        if (row1 >= clipY1 && row1 <= clipY2)
          rawSetPixelInRow(row1, col2, pattern);
        if (row2 >= clipY1 && row2 <= clipY2)
          rawSetPixelInRow(row2, col2, pattern);
      }

      if (t + b2 * x <= crit1 || t + a2 * y <= crit3) {
        x++;
        dxt += d2xt;
        t += dxt;
      } else if (t - a2 * y > crit2) {
        y--;
        dyt += d2yt;
        t += dyt;
      } else {
        x++;
        dxt += d2xt;
        t += dxt;
        y--;
        dyt += d2yt;
        t += dyt;
      }
    }
}
void IRAM_ATTR VGAControllerS3::clear(Rect & updateRect)
{
    RGB888 rgb = getActualBrushColor();
    vga.clear (vga.rgb_to_bits (rgb.R,rgb.G,rgb.B));
    updateRect = updateRect.merge (Rect (0,0,m_screenWidth-1,m_screenHeight-1));
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
    // ets_printf ("drawGlyph \r\n");
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
    if (!glyphOptions.bold && !glyphOptions.italic && !glyphOptions.blank && !glyphOptions.underline && !glyphOptions.doubleWidth && glyph.width <= 32)
    { //drrawGlyph_light
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
    } else { //DrawGlyph_full
          int16_t glyphSize         = glyphHeight * glyphWidthByte;

    bool fillBackground = glyphOptions.fillBackground;
    bool bold           = glyphOptions.bold;
    bool italic         = glyphOptions.italic;
    bool blank          = glyphOptions.blank;
    bool underline      = glyphOptions.underline;
    int doubleWidth     = glyphOptions.doubleWidth;

    // modify glyph to handle top half and bottom half double height
    // doubleWidth = 1 is handled directly inside drawing routine
    if (doubleWidth > 1) {
      uint8_t * newGlyphData = (uint8_t*) alloca(glyphSize);
      // doubling top-half or doubling bottom-half?
      int offset = (doubleWidth == 2 ? 0 : (glyphHeight >> 1));
      for (int y = 0; y < glyphHeight ; ++y)
        for (int x = 0; x < glyphWidthByte; ++x)
          newGlyphData[x + y * glyphWidthByte] = glyphData[x + (offset + (y >> 1)) * glyphWidthByte];
      glyphData = newGlyphData;
    }

    // a very simple and ugly skew (italic) implementation!
    int skewAdder = 0, skewH1 = 0, skewH2 = 0;
    if (italic) {
      skewAdder = 2;
      skewH1 = glyphHeight / 3;
      skewH2 = skewH1 * 2;
    }

    int16_t X1 = 0;
    int16_t XCount = glyphWidth;
    int16_t destX = glyphX;

    if (destX < clipX1) {
      X1 = (clipX1 - destX) / (doubleWidth ? 2 : 1);
      destX = clipX1;
    }
    if (X1 >= glyphWidth)
      return;

    if (destX + XCount + skewAdder > clipX2 + 1)
      XCount = clipX2 + 1 - destX - skewAdder;
    if (X1 + XCount > glyphWidth)
      XCount = glyphWidth - X1;

    int16_t Y1 = 0;
    int16_t YCount = glyphHeight;
    int destY = glyphY;

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

    updateRect = updateRect.merge(Rect(destX, destY, destX + XCount + skewAdder - 1, destY + YCount - 1));
    hideSprites(updateRect);

    if (glyphOptions.invert ^ paintState().paintOptions.swapFGBG)
      tswap(penColor, brushColor);

    // a very simple and ugly reduce luminosity (faint) implementation!
    if (glyphOptions.reduceLuminosity) {
      if (penColor.R > 128) penColor.R = 128;
      if (penColor.G > 128) penColor.G = 128;
      if (penColor.B > 128) penColor.B = 128;
    }

    auto penPattern   = preparePixel(penColor);
    auto brushPattern = preparePixel(brushColor);
    auto boldPattern  = bold ? preparePixel(RGB888(penColor.R / 2 + 1,
                                                   penColor.G / 2 + 1,
                                                   penColor.B / 2 + 1))
                             : preparePixel(RGB888(0, 0, 0));

    for (int y = Y1; y < Y1 + YCount; ++y, ++destY) {

      // true if previous pixel has been set
      bool prevSet = false;

      // auto dstrow = rawGetRow(destY);
      auto srcrow = glyphData + y * glyphWidthByte;

      if (underline && y == glyphHeight - FABGLIB_UNDERLINE_POSITION - 1) {

        for (int x = X1, adestX = destX + skewAdder; x < X1 + XCount && adestX <= clipX2; ++x, ++adestX) {
          rawSetPixelInRow(destY, adestX, blank ? brushPattern : penPattern);
          if (doubleWidth) {
            ++adestX;
            if (adestX > clipX2)
              break;
            rawSetPixelInRow(destY, adestX, blank ? brushPattern : penPattern);
          }
        }

      } else {

        for (int x = X1, adestX = destX + skewAdder; x < X1 + XCount && adestX <= clipX2; ++x, ++adestX) {
          if ((srcrow[x >> 3] << (x & 7)) & 0x80 && !blank) {
            rawSetPixelInRow(destY, adestX, penPattern);
            prevSet = true;
          } else if (bold && prevSet) {
            rawSetPixelInRow(destY, adestX, boldPattern);
            prevSet = false;
          } else if (fillBackground) {
            rawSetPixelInRow(destY, adestX, brushPattern);
            prevSet = false;
          } else {
            prevSet = false;
          }
          if (doubleWidth) {
            ++adestX;
            if (adestX > clipX2)
              break;
            if (fillBackground)
              rawSetPixelInRow(destY, adestX, prevSet ? penPattern : brushPattern);
            else if (prevSet)
              rawSetPixelInRow(destY, adestX, penPattern);
          }
        }

      }

      if (italic && (y == skewH1 || y == skewH2))
        --skewAdder;

    }
  }
}

uint8_t VGAControllerS3::rawGetPixelInRow (int y,int x)
{
  return vga.get_pixel (x,y);
}

void VGAControllerS3::rawSetPixelInRow (int y,int x,int color)
{
 vga.set_pixel (x,y,color);
}

void IRAM_ATTR VGAControllerS3::invertRect(Rect const & rect, Rect & updateRect)
{
    // ets_printf ("invertRect\r\n");

    const int origX = paintState().origin.X;
    const int origY = paintState().origin.Y;

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
    for (int y = y1; y <= y2; ++y){
      VGA8_INVERTROW(y, x1, x2);
      // for (int x = x1; x <= x2; ++x){
      //   //rawSetPixelInRow(y, x, 255-rawGetPixelInRow(y, x));
      //   // rawSetPixelInRow(y, x, rawGetPixelInRow(y, x) ^ 0b11111111); // 0b11111111 = 11111111b, invert RGB2222 pixel
      // }
    }
}
void IRAM_ATTR VGAControllerS3::swapFGBG(Rect const & rect, Rect & updateRect)
{
    //ets_printf ("swapFGBG (%d,%d,%d,%d)\r\n",rect.X1,rect.Y1,rect.X2,rect.Y2);
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
    Rect rct (paintState().position.X,paintState().position.Y,paintState().position.X+source.width(),paintState().position.Y+source.height());
    updateRect = updateRect.merge(rct);
    vga.move_rect (source.X1,source.Y1,paintState().position.X,paintState().position.Y,source.width(),source.height());
    //ets_printf ("copyRect()\r\n");
}
void VGAControllerS3::swapBuffers()
{
    //ets_printf ("swapBuffers()\r\n");
    #ifndef BITLUNI
    vga.swapBuffers ();
    #endif
}
int VGAControllerS3::getBitmapSavePixelSize()
{
    //ets_printf ("getBitmapSavePixelSize()\r\n");
    return 1;
}
void IRAM_ATTR VGAControllerS3::rawDrawBitmap_Native(int destX, int destY, Bitmap const * bitmap, int X1, int Y1, int XCount, int YCount)
{
    // ets_printf ("rawDrawBitmap_Native()\r\n");
    const int yEnd = Y1 + YCount;
    const int xEnd = X1 + XCount;
    auto      data = (uint8_t*) bitmap->data;
    auto     width = bitmap->width;
    for (int y = Y1; y < yEnd; ++y, ++destY) {
      // auto dstrow = rawGetRow(destY);
      auto src = data + y * width + X1;
      for (int x = X1, adestX = destX; x < xEnd; ++x, ++adestX, ++src)
        rawSetPixelInRow(destY, adestX, *src);
    }
}
void IRAM_ATTR VGAControllerS3::rawDrawBitmap_Mask(int destX, int destY, Bitmap const * bitmap, void * saveBackground, int X1, int Y1, int XCount, int YCount)
{
    // ets_printf ("rawDrawBitmap_Mask()\r\n");
     const int width = bitmap->width;
    const int yEnd  = Y1 + YCount;
    const int xEnd  = X1 + XCount;
    auto data = bitmap->data;
    const int rowlen = (bitmap->width + 7) / 8;
    auto foregroundPattern = preparePixel(bitmap->foregroundColor);
    if (saveBackground) {
      // save background and draw the bitmap
      for (int y = Y1; y < yEnd; ++y, ++destY) {
        // auto dstrow = rawGetRow(destY);
        uint8_t* savePx = ((uint8_t*) saveBackground) + y * width + X1;
        auto src = data + y * rowlen;
        for (int x = X1, adestX = destX; x < xEnd; ++x, ++adestX, ++savePx) {
          *savePx = rawGetPixelInRow(destY, adestX);
          if ((src[x >> 3] << (x & 7)) & 0x80)
            rawSetPixelInRow(destY, adestX,foregroundPattern);
        }
      }

    } else {

      // just draw the bitmap
      for (int y = Y1; y < yEnd; ++y, ++destY) {
        // auto dstrow = rawGetRow(destY);
        auto src = data + y * rowlen;
        for (int x = X1, adestX = destX; x < xEnd; ++x, ++adestX) {
          if ((src[x >> 3] << (x & 7)) & 0x80)
            rawSetPixelInRow(destY, adestX,foregroundPattern);
        }
      }

    }
}
void IRAM_ATTR VGAControllerS3::rawDrawBitmap_RGBA2222(int destX, int destY, Bitmap const * bitmap, void * saveBackground, int X1, int Y1, int XCount, int YCount)
{
    const int width  = bitmap->width;
    const int yEnd   = Y1 + YCount;
    const int xEnd   = X1 + XCount;
    auto data = (RGBA2222 const *) bitmap->data;

    if (saveBackground) {

      // save background and draw the bitmap
      for (int y = Y1; y < yEnd; ++y, ++destY) {
        //auto dstrow = rawGetRow(destY);
        uint8_t* savePx = ((uint8_t*) saveBackground) + y * width + X1;
        auto src = data + y * width + X1;
        for (int x = X1, adestX = destX; x < xEnd; ++x, ++adestX, ++savePx, ++src) {
          *savePx = rawGetPixelInRow(destY, adestX);
          if (src ->A)//(*src & 0xc0)  // alpha > 0 ?
            rawSetPixelInRow(destY, adestX, preparePixel(*src));
        }
      }

    } else {

      // just draw the bitmap
      for (int y = Y1; y < yEnd; ++y, ++destY) {
        //auto dstrow = rawGetRow(destY);
        auto src = data + y * width + X1;
        for (int x = X1, adestX = destX; x < xEnd; ++x, ++adestX, ++src) {
          if (src ->A)//(*src & 0xc0)  // alpha > 0 ?
            rawSetPixelInRow(destY, adestX, preparePixel(*src));
        }
      }

    }
}
void IRAM_ATTR VGAControllerS3::rawDrawBitmap_RGBA8888(int destX, int destY, Bitmap const * bitmap, void * saveBackground, int X1, int Y1, int XCount, int YCount)
{
    // ets_printf ("rawDrawBitmap_RGBA8888()\r\n");
      const int width = bitmap->width;
    const int yEnd  = Y1 + YCount;
    const int xEnd  = X1 + XCount;
    auto data = (RGBA8888 const *) bitmap->data;

    if (saveBackground) {

      // save background and draw the bitmap
      for (int y = Y1; y < yEnd; ++y, ++destY) {
        // auto dstrow = rawGetRow(destY);
        uint8_t* savePx = ((uint8_t*) saveBackground) + y * width + X1;
        auto src = data + y * width + X1;
        for (int x = X1, adestX = destX; x < xEnd; ++x, ++adestX, ++savePx, ++src) {
          *savePx = rawGetPixelInRow(destY, adestX);
          if (src -> A)// alpha > 0 ?
            rawSetPixelInRow(destY, adestX, convert_rgba8888_to_rgba2222(src));
        }
      }

    } else {

      // just draw the bitmap
      for (int y = Y1; y < yEnd; ++y, ++destY) {
        // auto dstrow = rawGetRow(destY);
        auto src = data + y * width + X1;
        for (int x = X1, adestX = destX; x < xEnd; ++x, ++adestX, ++src) {
          if (src -> A)// alpha > 0 ?
            rawSetPixelInRow(destY, adestX, convert_rgba8888_to_rgba2222(src));
        }
      }

    }
}

uint8_t VGAControllerS3::preparePixel(RGB222 rgb) 
{ 
	  uint8_t pixel = vga.rgb_to_bits (rgb.R<<6,rgb.G<<6,rgb.B<<6);
    return pixel; //m_HVSync | (rgb.B << VGA_BLUE_BIT) | (rgb.G << VGA_GREEN_BIT) | (rgb.R << VGA_RED_BIT); 
}
uint8_t VGAControllerS3::preparePixel(RGB888 rgb) 
{ 
	  uint8_t pixel = vga.rgb_to_bits (rgb.R,rgb.G,rgb.B);
    return pixel; //m_HVSync | (rgb.B << VGA_BLUE_BIT) | (rgb.G << VGA_GREEN_BIT) | (rgb.R << VGA_RED_BIT); 
}
uint8_t VGAControllerS3::preparePixel(RGBA2222 rgb) 
{ 
	  uint8_t pixel = vga.rgb_to_bits (rgb.R<<6,rgb.G<<6,rgb.B<<6);
    return pixel; //m_HVSync | (rgb.B << VGA_BLUE_BIT) | (rgb.G << VGA_GREEN_BIT) | (rgb.R << VGA_RED_BIT); 
}

void VGAControllerS3::redraw_task(void *pArg)
{
    uint32_t ulNotificationValue;
    int color=0,backcolor;
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 20 );

    VGAControllerS3* controller = (VGAControllerS3*)pArg;
    VGA& vga = controller->vga;
    // wait till next vsync   
    while (true)
    {
          ulNotificationValue = ulTaskNotifyTake( pdTRUE,
                              xMaxBlockTime );

          if( ulNotificationValue == 1 )
          {
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
              #ifndef BITLUNI
              vga.show(updateRect);
              #else
              vga.show();
              #endif
              
          }
          else
          {
              /* The call to ulTaskNotifyTake() timed out. */
              // ets_printf ("-Error, waited too long for VSync\r\n");
              // ets_printf ("-%d frames skipped\r\n",ulNotificationValue-1);
          }
    }
}

} // namespace FabGL

#endif //CONFIG_IDF_TARGET_ESP32S3