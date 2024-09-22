// ESP32S3 VGA class by S0urceror (c) 2024 
// to make FabGL compatible with ESP32 S3

#ifdef CONFIG_IDF_TARGET_ESP32S3

#include "VGA.h"
#include <esp_log.h>
#include <esp_lcd_panel_ops.h>
#include <string.h>
#include <math.h>
#include "esp32s3/rom/ets_sys.h"

#ifndef BITLUNI // we do the ESPIDF official way

#ifndef min
#define min(a,b)((a)<(b)?(a):(b))
#endif
#ifndef max
#define max(a,b)((a)>(b)?(a):(b))
#endif

// default pin values chosen to avoid conflicting with spi, i2c, or serial.

#define VGA_PIN_NUM_HSYNC          17 //12
#define VGA_PIN_NUM_VSYNC          18 //13
#define VGA_PIN_NUM_DE             -1

// note: PCLK pin is not needed for VGA output.
// however, the current version of the esp lcd rgb driver requires this to be set
// to keep this pin unused and available for something else, you need a patched version
// of the driver (for now)
#if PATCHED_LCD_DRIVER
#define VGA_PIN_NUM_PCLK           -1
#else
#define VGA_PIN_NUM_PCLK           21
#endif
#define VGA_PIN_NUM_DISP_EN        -1

static const char *TAG = "vga";

namespace fabgl
{

VGA::VGA ()
{
    _dispBufferIndex = 0;
    _drawBufferIndex = 1;
	_screenWidth = 0;
	_screenHeight = 0;
	_colorBits = 8;
    _fbSize = 0;
    setRedrawTask (NULL);
    _panel_handle = NULL;
    _frameBuffers[0]=NULL;
    _frameBuffers[1]=NULL;
}

bool VGA::init(VGATimings const& timings, int bits, PinConfig pins, bool usePsram, bool doubleBuffering, bool bounceBuffering) 
{
	_screenWidth = timings.HVisibleArea;
	_screenHeight = timings.VVisibleArea;
    if (bits<=8)
        _fbSize = _screenHeight*_screenWidth;
    else if (bits <= 16)
        _fbSize = _screenHeight*_screenWidth*2;
    _colorBits = bits;

    ESP_LOGI(TAG, "Create semaphores");
    _sem_vsync_end = xSemaphoreCreateBinary();
    assert(_sem_vsync_end);
    _sem_gui_ready = xSemaphoreCreateBinary();
    assert(_sem_gui_ready);

	ESP_LOGI(TAG, "Install RGB LCD panel driver");
    esp_lcd_rgb_panel_config_t panel_config;
    memset(&panel_config, 0, sizeof(esp_lcd_rgb_panel_config_t));
    // general
    panel_config.data_width = 8;
    panel_config.psram_trans_align = 64;
    panel_config.clk_src = LCD_CLK_SRC_PLL240M; //LCD_CLK_SRC_DEFAULT;
    // pins
    panel_config.disp_gpio_num = -1;
    panel_config.pclk_gpio_num = VGA_PIN_NUM_PCLK;
    panel_config.vsync_gpio_num = pins.vSync;
    panel_config.hsync_gpio_num = pins.hSync;
    panel_config.de_gpio_num = VGA_PIN_NUM_DE;    
    panel_config.data_gpio_nums[0] = pins.r[2];
    panel_config.data_gpio_nums[1] = pins.r[3];
    panel_config.data_gpio_nums[2] = pins.r[4];
    panel_config.data_gpio_nums[3] = pins.g[3];
    panel_config.data_gpio_nums[4] = pins.g[4];
    panel_config.data_gpio_nums[5] = pins.g[5];
    panel_config.data_gpio_nums[6] = pins.b[3];
    panel_config.data_gpio_nums[7] = pins.b[4];
    panel_config.data_gpio_nums[8] = -1;
    panel_config.data_gpio_nums[9] = -1;
    panel_config.data_gpio_nums[10] = -1;
    panel_config.data_gpio_nums[11] = -1;
    panel_config.data_gpio_nums[12] = -1;
    panel_config.data_gpio_nums[13] = -1;
    panel_config.data_gpio_nums[14] = -1;
    panel_config.data_gpio_nums[15] = -1;
    // timings
    panel_config.timings.h_res = _screenWidth;
    panel_config.timings.v_res = _screenHeight*timings.scanCount;
    panel_config.timings.pclk_hz = timings.frequency;
    panel_config.timings.hsync_back_porch = timings.HBackPorch;
    panel_config.timings.hsync_front_porch = timings.HFrontPorch;
    panel_config.timings.hsync_pulse_width = timings.HSyncPulse;
    panel_config.timings.vsync_back_porch = timings.VBackPorch;
    panel_config.timings.vsync_front_porch = timings.VFrontPorch;
    panel_config.timings.vsync_pulse_width = timings.VSyncPulse;
    panel_config.timings.flags.pclk_active_neg = true;
    panel_config.timings.flags.hsync_idle_low = timings.HSyncLogic=='+';
    panel_config.timings.flags.vsync_idle_low = timings.VSyncLogic=='+';
    ESP_LOGI (TAG,"Timings:\r\nPixelclock: %d\r\nHSync back porch: %d\r\nHSync front porch: %d\r\nHSync pulse width: %d\r\nVSync back porch: %d\r\nVSync front porch: %d\r\nVSync pulse width: %d\r\n",timings.frequency,timings.HBackPorch,timings.HFrontPorch,timings.HSyncPulse,timings.VBackPorch,timings.VFrontPorch,timings.VSyncPulse);
    // framebuffer
    panel_config.flags.fb_in_psram = usePsram;
    panel_config.flags.double_fb = false;
    panel_config.flags.no_fb = false;
    if (doubleBuffering)
    {
        ESP_LOGI(TAG, "Double framebuffer");
        _fbCount = 2;
        _dispBufferIndex=0;
        _drawBufferIndex=1;
    }
    else
    {
        ESP_LOGI(TAG, "Single framebuffer");
        _fbCount = 1;
        _dispBufferIndex=0;
        _drawBufferIndex=0;
    }
    panel_config.num_fbs = _fbCount;
    if (bounceBuffering && !doubleBuffering)
    {
        ESP_LOGI(TAG, "Bounce buffer and single framebuffer");
        panel_config.bounce_buffer_size_px = 10 * _screenWidth;
    }

    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &_panel_handle));

    ESP_LOGI(TAG, "Register event callbacks");
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_vsync = vsyncEvent,
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(_panel_handle, &cbs, this));

    ESP_LOGI(TAG, "Initialize RGB LCD panel");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(_panel_handle));
    
    // retrieve initialized framebuffers
    if (_fbCount==1)
        ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(_panel_handle, _fbCount, &_frameBuffers[0]));
    else
        ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(_panel_handle, _fbCount, &_frameBuffers[0], &_frameBuffers[1]));
    return true;
}

bool VGA::deinit() {
    if (_panel_handle)
    {
        setRedrawTask (NULL);
        esp_err_t err = esp_lcd_panel_del(_panel_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "error deleting rgb lcd panel");
            return false;
        }
        _panel_handle = NULL;
        _dispBufferIndex = 0;
        _drawBufferIndex = 1;
        _frameBuffers[0]=NULL;
        _frameBuffers[1]=NULL;
        _screenWidth = 0;
        _screenHeight = 0;
        _colorBits = 8;
        _fbSize = 0;
    }

    return true;
}

void VGA::show() {
	// // get draw semaphore
    // xSemaphoreGive(_sem_gui_ready);
    // xSemaphoreTake(_sem_vsync_end, portMAX_DELAY);

    // instruct LCD device and DMA channel to update based on active frame buffer
    // ets_printf ("VGA::display () => %d\r\n",_dispBufferIndex);
    esp_lcd_panel_draw_bitmap(_panel_handle, 0, 0, _screenWidth-1, _screenHeight-1, _frameBuffers[_dispBufferIndex]);
}
void VGA::show(Rect& updaterect)
{
    int16_t temp;
    //ets_printf ("1-VGA::show (%d,%d,%d,%d)\r\n",updaterect.X1,updaterect.Y1,updaterect.X2, updaterect.Y2);
    if (updaterect.X1<0) updaterect.X1=0;
    if (updaterect.X2<0) updaterect.X2=0;
    if (updaterect.Y1<0) updaterect.Y1=0;
    if (updaterect.Y2<0) updaterect.Y2=0;
    if (updaterect.X1>_screenWidth-1) updaterect.X1=_screenWidth-1;
    if (updaterect.X2>_screenWidth-1) updaterect.X2=_screenWidth-1;
    if (updaterect.Y1>_screenHeight-1) updaterect.Y1=_screenHeight-1;
    if (updaterect.Y2>_screenHeight-1) updaterect.Y2=_screenHeight-1;
    if (updaterect.X1>updaterect.X2)
    {
        temp = updaterect.X1;
        updaterect.X1 = updaterect.X2;
        updaterect.X2 = temp;
    }
    if (updaterect.Y1>updaterect.Y2)
    {
        temp = updaterect.Y1;
        updaterect.Y1 = updaterect.Y2;
        updaterect.Y2 = temp;
    }
    //ets_printf ("VGA::show (%d,%d,%d,%d) => %d\r\n",updaterect.X1,updaterect.Y1,updaterect.X2, updaterect.Y2,_dispBufferIndex);
    esp_lcd_panel_draw_bitmap(_panel_handle, updaterect.X1, updaterect.Y1, updaterect.X2, updaterect.Y2, _frameBuffers[_dispBufferIndex]);
}
volatile int oldindex; 
uint8_t* VGA::getDrawBuffer() {
    if (_fbCount==1)
        return (uint8_t*)_frameBuffers[0];
    else 
    {
        if (oldindex!=_drawBufferIndex)
        {
            //ets_printf("draw: %d\r\n",_drawBufferIndex);
            oldindex = _drawBufferIndex;
        }
        // return the not active frame buffer
        return (uint8_t*)_frameBuffers[_drawBufferIndex];
    }
}

bool VGA::vsyncEvent(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx) {
    VGA* vga = (VGA*)user_ctx;
    if (vga==NULL)
        return false;
    BaseType_t high_task_awoken = pdFALSE;

    // notify drawing task to start drawing
    TaskHandle_t task = vga->getRedrawTask ();
    if (task!=NULL)
    {
        vTaskNotifyGiveFromISR( task,
                                &high_task_awoken );
    }

    // synchronise drawing to begin straight after vsync
    // if (xSemaphoreTakeFromISR(vga->_sem_gui_ready, &high_task_awoken) == pdTRUE) {
    //     xSemaphoreGiveFromISR(vga->_sem_vsync_end, &high_task_awoken);
    // }
    return high_task_awoken == pdTRUE;
}

void VGA::swapBuffers() {
    if (_fbCount==2)
    {
        if (_drawBufferIndex==0)
        {
            _drawBufferIndex = 1;
            _dispBufferIndex = 0;
        }
        else
        {
            _drawBufferIndex = 0;
            _dispBufferIndex = 1;
        }
    }
    //ets_printf ("swap db %d->%d\r\n",_dispBufferIndex,_drawBufferIndex);
}
void own_memset (uint8_t* address,uint8_t value,size_t size)
{
    while (size>0)
    {
        *address++=value;
        size--;
    }
}
void VGA::clear(uint8_t rgb)
{
    uint8_t* draw_buffer = getDrawBuffer ();
    own_memset (draw_buffer,rgb,_fbSize);
}

void VGA::fill_rect(uint8_t rgb,u_int x1,u_int y1,u_int x2,u_int y2)
{
    uint8_t* draw_buffer = getDrawBuffer ();
    int left = min (_screenWidth-1,x1);
    int right = min (_screenWidth-1,x2);
    if (right==left)
        return;
    if (right<left) 
    {
        tswap (left,right);
    }
    int top = min (_screenHeight-1,y1);
    int bottom = min (_screenHeight-1,y2);
    if (bottom<top)
    {
        tswap (top,bottom);
    }
	for(int y = top; y <= bottom; y++)
        own_memset (draw_buffer+y*_screenWidth+left,rgb,right-left);
}
void VGA::move_rect (u_int sx,u_int sy,u_int dx,u_int dy,u_int width,u_int height)
{
    uint8_t* draw_buffer = getDrawBuffer ();
    for (int y = 0;y < height;y++)
        memcpy (draw_buffer+((y+sy)*_screenWidth)+sx,
                draw_buffer+((y+dy)*_screenWidth)+dx,
                width);
}
void VGA::set_pixel(u_int x, u_int y, uint8_t r, uint8_t g, uint8_t b)
{
    if (x>_screenWidth)
        return;
    if (y>_screenHeight)
        return;
    uint8_t* draw_buffer = getDrawBuffer ();
    draw_buffer[x*_screenWidth+y]=rgb_to_bits(r,g,b);
}
void VGA::set_pixel(u_int x, u_int y, uint8_t rgb)
{
    if (x>_screenWidth)
        return;
    if (y>_screenHeight)
        return;
    uint8_t* draw_buffer = getDrawBuffer ();
    draw_buffer[y*_screenWidth+x]=rgb;
}
uint8_t  VGA::get_pixel(u_int x,u_int y)
{
    if (x>_screenWidth)
        return 0;
    if (y>_screenHeight)
        return 0;
    uint8_t* draw_buffer = getDrawBuffer ();
    return draw_buffer[y*_screenWidth+x];
}
uint8_t VGA::rgb_to_bits(uint8_t r, uint8_t g, uint8_t b)
{
    return (r >> 5) | ((g >> 5) << 3) | (b & 0b11000000);
}
void VGA::bits_to_rgb (uint8_t color,uint8_t& r, uint8_t& g, uint8_t& b)
{
    r = (color & 0b111) << 5;
    g = ((color & 0b111000)>>3) << 5;
    b = ((color & 0b11000000)>>6) << 5;
}
TaskHandle_t VGA::getRedrawTask () 
{
	return _redraw_task;
};
void VGA::setRedrawTask(TaskHandle_t to_notify_task)
{
	_redraw_task = to_notify_task;
};

} // namespace FabGL

#endif // BITLUNI

#endif // CONFIG_IDF_TARGET_ESP32S3