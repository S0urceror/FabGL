#ifndef VGA_H
#define VGA_H

#include "../devdrivers/PinConfig.h"
#include "Mode.h"
#include "DMAVideoBuffer.h"
#include <esp_intr_alloc.h>
#include "FreeRTOS.h"
#include "task.h"

namespace bitluni {
class VGA
{
	public:
		VGA();
		~VGA();
		bool init(const PinConfig pins, const Mode mode, int bits,bool double_buffer);
		bool start();
		bool show();
		void clear(int rgb = 0);
		void fill_rect(int rgb,int x1,int y1,int x2,int y2);
		void move_rect (int sx,int sy,int dx,int dy,int width,int height);
		void set_pixel(int x, int y, uint8_t r, uint8_t g, uint8_t b);
		void set_pixel(int x, int y, int rgb);
		int  get_pixel(int x,int y);
		//void dotdit(int x, int y, uint8_t r, uint8_t g, uint8_t b);
		int rgb_to_bits(uint8_t r, uint8_t g, uint8_t b);
		void bits_to_rgb (int color,uint8_t& r, uint8_t& g, uint8_t& b);

		void setController (void* controller);
		void stopVSyncInterupt();
		void startVSyncInterrupt();

		TaskHandle_t getRedrawTask ();
		void setRedrawTask(TaskHandle_t to_notify_task);
		int getBufferCount ();

	protected:
		Mode mode;
		int bufferCount;
		int bits;
		PinConfig pins;
		int backBuffer;
		DMAVideoBuffer *dmaBuffer;
		bool usePsram;
		int m_dmaChannel;
		intr_handle_t m_vsync_int_handle;

		static volatile TaskHandle_t IRAM_ATTR redraw_task;
		
		void attachPinToSignal(int pin, int signal);
};

} // namespace bitluni

#endif //VGA_h