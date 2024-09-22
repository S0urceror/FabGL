// ESP32S3 VGA class by BitFixer (c) 2023
// adapted and changed by S0urceror (c) 2024
// to make FabGL compatible with ESP32 S3

#ifdef CONFIG_IDF_TARGET_ESP32S3

#pragma once
#include <esp_lcd_panel_rgb.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "PinConfig.h"
#include "../dispdrivers/vgabasecontroller.h"

namespace fabgl {

class VGA {
public:
	VGA();
	~VGA() {}

	bool init(VGATimings const& timings, int bits, PinConfig pins, bool usePsram, bool doubleBuffering, bool bounceBuffering);
	bool deinit();

	inline TaskHandle_t getRedrawTask ();
	inline void setRedrawTask(TaskHandle_t to_notify_task);

	// signal end of drawing and start of showing
	void show();
	void show(Rect& updaterect);
	// swap draw buffer and screen buffer
	void swapBuffers();

	// drawing primitives
	void clear(uint8_t rgb = 0);
	void fill_rect(uint8_t rgb,u_int x1,u_int y1,u_int x2,u_int y2);
	void move_rect (u_int sx,u_int sy,u_int dx,u_int dy,u_int width,u_int height);
	void set_pixel(u_int x, u_int y, uint8_t r, uint8_t g, uint8_t b);
	void set_pixel(u_int x, u_int y, uint8_t rgb);
	uint8_t get_pixel(u_int x,u_int y);
	uint8_t rgb_to_bits(uint8_t r, uint8_t g, uint8_t b);
	void bits_to_rgb (uint8_t color,uint8_t& r, uint8_t& g, uint8_t& b);

private:
	TaskHandle_t _redraw_task;
	SemaphoreHandle_t _sem_vsync_end;
	SemaphoreHandle_t _sem_gui_ready;
	esp_lcd_panel_handle_t _panel_handle;
	volatile int _drawBufferIndex;
	volatile int _dispBufferIndex;
	int _fbCount;
	void* _frameBuffers[2];
	int _screenWidth;
	int _screenHeight;
	int _colorBits;
	int _fbSize;


	uint8_t* getDrawBuffer();

	int colorBits() {
		return _colorBits;
	}

	int getBufferCount () {
		return _fbCount;
	}
	
	static bool vsyncEvent(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx);
	//static bool bounceEvent(esp_lcd_panel_handle_t panel, void* bounce_buf, int pos_px, int len_bytes, void* user_ctx);
};

}

#endif // CONFIG_IDF_TARGET_ESP32S3