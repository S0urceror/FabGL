#ifdef BITLUNI

#include "VGA.h"
#include <esp_rom_gpio.h>
#include <esp_rom_sys.h>
#include "esp_log.h"
#include <hal/gpio_hal.h>
#include <hal/lcd_ll.h>
#include <driver/periph_ctrl.h>
#include <driver/gpio.h>
#include <soc/lcd_cam_struct.h>
#include <math.h>
#include <esp_private/gdma.h>
//#include <FreeRTOS.h>
#include <soc/lcd_periph.h>
#include <string.h>
#include "../dispdrivers/vgacontrollers3.h"

namespace bitluni {
#ifndef min
#define min(a,b)((a)<(b)?(a):(b))
#endif
#ifndef max
#define max(a,b)((a)>(b)?(a):(b))
#endif

#if CONFIG_LCD_RGB_ISR_IRAM_SAFE
#define LCD_RGB_INTR_ALLOC_FLAGS     (ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_INTRDISABLED)
#else
#define LCD_RGB_INTR_ALLOC_FLAGS     ESP_INTR_FLAG_INTRDISABLED
#endif

const char TAG[]="VGA";
volatile TaskHandle_t IRAM_ATTR VGA::redraw_task;

//borrowed from esp code
#define HAL_FORCE_MODIFY_U32_REG_FIELD(base_reg, reg_field, field_val)    \
{                                                           \
	uint32_t temp_val = base_reg.val;                       \
	typeof(base_reg) temp_reg;                              \
	temp_reg.val = temp_val;                                \
	temp_reg.reg_field = (field_val);                       \
	(base_reg).val = temp_reg.val;                          \
}

VGA::VGA()
{
	bufferCount = 1;
	dmaBuffer = NULL;
	usePsram = true;
	m_dmaChannel = 0;
	setRedrawTask (NULL);
}

VGA::~VGA()
{
	bits = 0;
	backBuffer = 0;
	setRedrawTask (NULL);
}

extern int Cache_WriteBack_Addr(uint32_t addr, uint32_t size);

void VGA::attachPinToSignal(int pin, int signal)
{
	esp_rom_gpio_connect_out_signal(pin, signal, false, false);
	gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[pin], PIN_FUNC_GPIO);
	gpio_set_drive_capability((gpio_num_t)pin, (gpio_drive_cap_t)3);
}

// bool IRAM_ATTR dma_callback(gdma_channel_handle_t dma_chan, gdma_event_data_t *event_data, void *user_data)
// {
// 	ets_printf ("eof\r\n");
// 	return true;
// }

unsigned long timeVSYNC;
int count=0;
static void IRAM_ATTR lcd_default_isr_handler(void *args)
{
	VGA* vga = (VGA*) args;
    bool need_yield = false;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    uint32_t intr_status = lcd_ll_get_interrupt_status(&LCD_CAM);
    lcd_ll_clear_interrupt_status(&LCD_CAM, intr_status);
    if (intr_status & LCD_LL_EVENT_VSYNC_END) 
	{
		// notify user specified redraw task
		TaskHandle_t task = vga->getRedrawTask ();
		if (task!=NULL)
		{
			// unsigned long newtime = millis();
			// if (count==0)
			// {
			// 	ets_printf ("-frametime: %ld msecs\r\n",newtime - timeVSYNC);
			// 	count=60;
			// }
			// timeVSYNC = newtime;
			// count--;

			vTaskNotifyGiveFromISR( task,
									&xHigherPriorityTaskWoken );
			need_yield = true;
		}
    }
    if (need_yield) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

bool VGA::init(const PinConfig pins, const Mode mode, int bits,bool double_buffer)
{
	this->pins = pins;
	this->mode = mode;
	this->bits = bits;
	if (double_buffer)
		this->bufferCount = 2;
	backBuffer = 0;

	//TODO check start

	dmaBuffer = new DMAVideoBuffer(mode.vRes, mode.hRes * (bits / 8), mode.vClones, true, usePsram, bufferCount);
	if(!dmaBuffer->isValid())
	{
		ESP_LOGI(TAG, "DMA video buffer allocation error");
		delete dmaBuffer;
		return false;
	}

	// setup LCD peripheral
	periph_module_enable(PERIPH_LCD_CAM_MODULE);
	periph_module_reset(PERIPH_LCD_CAM_MODULE);
	LCD_CAM.lcd_user.lcd_reset = 1;
	esp_rom_delay_us(100);

	// setup clocks
	//
	//f=240000000/(n+1)
	//n=240000000/f-1;
	int N = round(240000000.0/(double)mode.frequency);
	if(N < 2) N = 2;
	//clk = source / (N + b/a)
	LCD_CAM.lcd_clock.clk_en = 1;
	LCD_CAM.lcd_clock.lcd_clk_sel = 2;			// PLL240M
	// - For integer divider, LCD_CAM_LCD_CLKM_DIV_A and LCD_CAM_LCD_CLKM_DIV_B are cleared.
	// - For fractional divider, the value of LCD_CAM_LCD_CLKM_DIV_B should be less than the value of LCD_CAM_LCD_CLKM_DIV_A.
	LCD_CAM.lcd_clock.lcd_clkm_div_a = 0;
	LCD_CAM.lcd_clock.lcd_clkm_div_b = 0;
	LCD_CAM.lcd_clock.lcd_clkm_div_num = N; 	// 0 => 256; 1 => 2; 14 compfy
	LCD_CAM.lcd_clock.lcd_ck_out_edge = 0;		
	LCD_CAM.lcd_clock.lcd_ck_idle_edge = 0;
	LCD_CAM.lcd_clock.lcd_clk_equ_sysclk = 1;

	// setup operating mode
	LCD_CAM.lcd_ctrl.lcd_rgb_mode_en = 1;			// RGB mode
	LCD_CAM.lcd_user.lcd_2byte_en = (bits==8)?0:1;	// 8 bits or 16 bits
    LCD_CAM.lcd_user.lcd_cmd = 0;					// No command at LCD start
    LCD_CAM.lcd_user.lcd_dummy = 0;					// No dummy phase(s) @ LCD start
    LCD_CAM.lcd_user.lcd_dout = 1;					// Enable data out
    LCD_CAM.lcd_user.lcd_cmd_2_cycle_en = 0;		// 0 length command cycle
    LCD_CAM.lcd_user.lcd_dummy_cyclelen = 0;//-1;	// 0 length dummy cycle
    LCD_CAM.lcd_user.lcd_dout_cyclelen = 0;			// one dout cycle
	LCD_CAM.lcd_user.lcd_always_out_en = 1;			// Enable 'always out' mode
    LCD_CAM.lcd_ctrl2.lcd_hsync_idle_pol = mode.hPol ^ 1;
    LCD_CAM.lcd_ctrl2.lcd_vsync_idle_pol = mode.vPol ^ 1;
    LCD_CAM.lcd_ctrl2.lcd_de_idle_pol = 1;	

	LCD_CAM.lcd_misc.lcd_bk_en = 1;	
    LCD_CAM.lcd_misc.lcd_vfk_cyclelen = 0;
    LCD_CAM.lcd_misc.lcd_vbk_cyclelen = 0;

	// Set RGB LCD horizontal timing
	LCD_CAM.lcd_ctrl2.lcd_hsync_width = mode.hSync - 1;				//7 bit
    LCD_CAM.lcd_ctrl.lcd_hb_front = mode.blankHorizontal() - 1;		//11 bit
    LCD_CAM.lcd_ctrl1.lcd_ha_width = mode.hRes - 1;					//12 bit
    LCD_CAM.lcd_ctrl1.lcd_ht_width = mode.totalHorizontal();		//12 bit
	// Set RGB vertical timing
	LCD_CAM.lcd_ctrl2.lcd_vsync_width = mode.vSync - 1;				//7bit
    HAL_FORCE_MODIFY_U32_REG_FIELD(LCD_CAM.lcd_ctrl1, lcd_vb_front, mode.vSync + mode.vBack - 1);		//8bit
    LCD_CAM.lcd_ctrl.lcd_va_height = mode.vRes * mode.vClones - 1;					//10 bit
    LCD_CAM.lcd_ctrl.lcd_vt_height = mode.totalVertical() - 1;		//10 bit

	LCD_CAM.lcd_ctrl2.lcd_hs_blank_en = 1;							// hsync enable
	HAL_FORCE_MODIFY_U32_REG_FIELD(LCD_CAM.lcd_ctrl2, lcd_hsync_position, 0);//mode.hFront);

	LCD_CAM.lcd_misc.lcd_next_frame_en = 1; //?? limitation, auto frame enable

	if(bits == 8)
	{
		int pins[8] = {
			this->pins.r[2], this->pins.r[3], this->pins.r[4],
			this->pins.g[3], this->pins.g[4], this->pins.g[5],
			this->pins.b[3], this->pins.b[4]
		};
		for (int i = 0; i < bits; i++) 
			if (pins[i] >= 0) 
				attachPinToSignal(pins[i], LCD_DATA_OUT0_IDX + i);
	}
	else if(bits == 16)
	{
		int pins[16] = {
			this->pins.r[0], this->pins.r[1], this->pins.r[2], this->pins.r[3], this->pins.r[4],
			this->pins.g[0], this->pins.g[1], this->pins.g[2], this->pins.g[3], this->pins.g[4], this->pins.g[5],
			this->pins.b[0], this->pins.b[1], this->pins.b[2], this->pins.b[3], this->pins.b[4]
		};
		for (int i = 0; i < bits; i++) 
			if (pins[i] >= 0) 
				attachPinToSignal(pins[i], LCD_DATA_OUT0_IDX + i);
	}
	attachPinToSignal(this->pins.hSync, LCD_H_SYNC_IDX);
	attachPinToSignal(this->pins.vSync, LCD_V_SYNC_IDX);
  
	gdma_channel_alloc_config_t dma_chan_config = 
	{
		.direction = GDMA_CHANNEL_DIRECTION_TX,
	};
	// setup DMA channel
	gdma_channel_handle_t dma_chan;
	gdma_new_channel(&dma_chan_config, &dma_chan);
	m_dmaChannel = (int)dma_chan;
	// connect to LCD peripheral
	gdma_connect(dma_chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));
	gdma_transfer_ability_t ability = 
	{
        .sram_trans_align = 4,
        .psram_trans_align = 64,
    };
    gdma_set_transfer_ability(dma_chan, &ability);

	// // Enable DMA transfer callback
	// gdma_tx_event_callbacks_t tx_cbs = {
	// 	.on_trans_eof = dma_callback
	// };
	// if (ESP_OK!=gdma_register_tx_event_callbacks(dma_chan, &tx_cbs, NULL))
	// 	ESP_LOGE(TAG,"Unable to register DMA callback");
	//TODO check end

	// install interrupt service, (LCD peripheral shares the interrupt source with Camera by different mask)
    int isr_flags = LCD_RGB_INTR_ALLOC_FLAGS | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_LOWMED;
	esp_err_t ret;
    ret = esp_intr_alloc_intrstatus(lcd_periph_signals.panels[0].irq_id, isr_flags,
                                    (uint32_t)lcd_ll_get_interrupt_status_reg(&LCD_CAM),
                                    LCD_LL_EVENT_VSYNC_END, lcd_default_isr_handler, this, &m_vsync_int_handle);
    lcd_ll_enable_interrupt(&LCD_CAM, LCD_LL_EVENT_VSYNC_END, false); // disable all interrupts
    lcd_ll_clear_interrupt_status(&LCD_CAM, UINT32_MAX); // clear pending interrupt

	return true;
}

bool VGA::start()
{
	//TODO check start
	//very delicate... dma might be late for peripheral
	gdma_reset((gdma_channel_handle_t)m_dmaChannel);
    esp_rom_delay_us(1);	
	// stopping LCD peripheral
    LCD_CAM.lcd_user.lcd_start = 0;
    LCD_CAM.lcd_user.lcd_update = 1;
	esp_rom_delay_us(1);
	// Reset Async TX FIFO
	LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
    LCD_CAM.lcd_user.lcd_update = 1;
	gdma_start((gdma_channel_handle_t)m_dmaChannel, (intptr_t)dmaBuffer->getDescriptor());
    esp_rom_delay_us(1);
	// enable interrupts
    lcd_ll_enable_interrupt(&LCD_CAM, LCD_LL_EVENT_VSYNC_END, true);
	esp_intr_enable(m_vsync_int_handle);
	// starting LCD peripheral
    LCD_CAM.lcd_user.lcd_update = 1;
	LCD_CAM.lcd_user.lcd_start = 1;
	//TODO check end
	return true;
}

bool VGA::show()
{
	//TODO check start
	dmaBuffer->flush(backBuffer);
	if(bufferCount <= 1) 
		return true;
	dmaBuffer->attachBuffer(backBuffer);
	backBuffer = (backBuffer + 1) % bufferCount;
	//TODO check end
	return true;
}

void VGA::set_pixel(int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
	if(x >= mode.hRes || y >= mode.vRes) return;
	if(bits == 8)
		dmaBuffer->getLineAddr8(y, backBuffer)[x] = (r >> 5) | ((g >> 5) << 3) | (b & 0b11000000);
	else if(bits == 16)
		dmaBuffer->getLineAddr16(y, backBuffer)[x] = (r >> 3) | ((g >> 2) << 5) | ((b >> 3) << 11);

}

void VGA::set_pixel(int x, int y, int rgb)
{
	if(x >= mode.hRes || y >= mode.vRes) return;
	if(bits == 8)
		dmaBuffer->getLineAddr8(y, backBuffer)[x] = rgb;
	else if(bits == 16)
		dmaBuffer->getLineAddr16(y, backBuffer)[x] = rgb;
}
int VGA::get_pixel (int x,int y)
{
	if(x >= mode.hRes || y >= mode.vRes) return 0;
	if(bits == 8)
		return dmaBuffer->getLineAddr8(y, backBuffer)[x];
	else if(bits == 16)
		return dmaBuffer->getLineAddr16(y, backBuffer)[x];
	return 0;
}
// void VGA::dotdit(int x, int y, uint8_t r, uint8_t g, uint8_t b)
// {
// 	if(x >= mode.hRes || y >= mode.vRes) return;
// 	if(bits == 8)
// 	{
// 		r = min((rand() & 31) | (r & 0xe0), 255);
// 		g = min((rand() & 31) | (g & 0xe0), 255);
// 		b = min((rand() & 63) | (b & 0xc0), 255);
// 		dmaBuffer->getLineAddr8(y, backBuffer)[x] = (r >> 5) | ((g >> 5) << 3) | (b & 0b11000000);
// 	}
// 	else
// 	if(bits == 16)
// 	{
// 		r = min((rand() & 7) | (r & 0xf8), 255);
// 		g = min((rand() & 3) | (g & 0xfc), 255); 
// 		b = min((rand() & 7) | (b & 0xf8), 255);
// 		dmaBuffer->getLineAddr16(y, backBuffer)[x] = (r >> 3) | ((g >> 2) << 5) | ((b >> 3) << 11);
// 	}	
// }

// convert separate 8 bits RGB values to bits
// 8-bits pixels
// R = 0bRRR00000 => 0x00000RRR
// G = 0bGGG00000 => 0x00GGG000
// B = 0bBB000000 => 0xBB000000
//                   ==========
//                   0xBBGGGRRR
//
// 16-bits pixels
// R = 0bRRRRR000 => 0x00000000000RRRRR
// G = 0bGGGGG000 => 0x000000GGGGG00000
// B = 0bBBBB0000 => 0x00BBBB0000000000
//                   ==================
//                   0x00BBBBGGGGGRRRRR
int VGA::rgb_to_bits(uint8_t r, uint8_t g, uint8_t b)
{
	if(bits == 8)
		return (r >> 5) | ((g >> 5) << 3) | (b & 0b11000000);
	else if(bits == 16)
		return (r >> 3) | ((g >> 2) << 5) | ((b >> 3) << 11);
	return 0;
}
void VGA::bits_to_rgb (int color,uint8_t& r, uint8_t& g, uint8_t& b)
{
	if(bits == 8) {
		r = (color & 0b111) << 5;
		g = ((color & 0b111000)>>3) << 5;
		b = ((color & 0b11000000)>>6) << 5;
	}
	else if (bits == 16) {
		r = (color & 0b11111) << 3;
		g = ((color & 0b1111100000) >> 5) << 2;
		b = ((color & 0b11110000000000) >> 11) << 3;
	}
	return;
}
void VGA::clear(int rgb)
{
	uint8_t* pLine;
	if(bits == 8)
	{
		for(int y = 0; y < mode.vRes; y++)
		{
			pLine = dmaBuffer->getLineAddr8(y, backBuffer);
			memset (pLine,mode.hRes,rgb);
		}
	}
	else if(bits == 16)
	{
		for(int y = 0; y < mode.vRes; y++)
		{
			pLine = dmaBuffer->getLineAddr8(y, backBuffer);
			memset (pLine,mode.hRes*2,rgb);
		}
	}
}

void VGA::move_rect (int sx,int sy,int dx,int dy,int width,int height)
{
	uint8_t* pLineSrc;
	uint8_t* pLineDst;
	if(bits == 8)
	{
		for(int y = 0; y < height; y++)
		{
			pLineSrc = dmaBuffer->getLineAddr8(sy+y, backBuffer);
			pLineDst = dmaBuffer->getLineAddr8(dy+y, backBuffer);
			memcpy (pLineDst,pLineSrc,width);
		}
	}
	else if(bits == 16)
	{
		for(int y = 0; y < height; y++)
		{
			pLineSrc = dmaBuffer->getLineAddr8(sy+y, backBuffer);
			pLineDst = dmaBuffer->getLineAddr8(dy+y, backBuffer);
			memcpy (pLineDst,pLineSrc,width*2);
		}
	}
}
// fill / clear a rectangle with a color
// from [x1,y1] to [x2,y2]
void VGA::fill_rect(int rgb,int x1,int y1,int x2,int y2)
{
	for(int y = y1; y < min(mode.vRes,y2+1); y++)
		for(int x = x1; x < min(mode.hRes,x2+1); x++)
			set_pixel(x, y, rgb);
}

void VGA::startVSyncInterrupt()
{
	lcd_ll_enable_interrupt(&LCD_CAM, LCD_LL_EVENT_VSYNC_END, true);
}

void VGA::stopVSyncInterupt()
{
	lcd_ll_enable_interrupt(&LCD_CAM, LCD_LL_EVENT_VSYNC_END, false); // disable all interrupts
    lcd_ll_clear_interrupt_status(&LCD_CAM, UINT32_MAX); // clear pending interrupt
}

TaskHandle_t IRAM_ATTR VGA::getRedrawTask () 
{
	return redraw_task;
};
void VGA::setRedrawTask(TaskHandle_t to_notify_task)
{
	redraw_task = to_notify_task;
};
int VGA::getBufferCount () 
{
	return bufferCount;
};

} // namespace bitluni

#endif