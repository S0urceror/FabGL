// ESP32S3 VGA class by BitFixer (c) 2023
// adapted and changed by S0urceror (c) 2024
// to make FabGL compatible with ESP32 S3

#pragma once
#include <esp_lcd_panel_rgb.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

class VGA {
public:
	VGA() {}
	~VGA() {}

	bool initWithSize(int frameWidth, int frameHeight, int bits);
	bool init(int width, int height, int scale = 2, int hborder = 0, int yborder = 0, int bits = 8, int* pins = NULL, bool usePsram = false);
	bool deinit();

	void vsyncWait();
	uint8_t* getDrawBuffer();

	int frameWidth() {
		return _frameWidth;
	}

	int frameHeight() {
		return _frameHeight;
	}

	int colorBits() {
		return _colorBits;
	}

	// drawing primitives
	void clear(int rgb = 0);
	void fill_rect(int rgb,int x1,int y1,int x2,int y2);
	void move_rect (int sx,int sy,int dx,int dy,int width,int height);
	void set_pixel(int x, int y, uint8_t r, uint8_t g, uint8_t b);
	void set_pixel(int x, int y, int rgb);
	int  get_pixel(int x,int y);
	//void dotdit(int x, int y, uint8_t r, uint8_t g, uint8_t b);
	int rgb_to_bits(uint8_t r, uint8_t g, uint8_t b);
	void bits_to_rgb (int color,uint8_t& r, uint8_t& g, uint8_t& b);

protected:
	SemaphoreHandle_t _sem_vsync_end;
	SemaphoreHandle_t _sem_gui_ready;
	esp_lcd_panel_handle_t _panel_handle = NULL;
	uint8_t *_frameBuffers[2];
	int _frameBufferIndex = 0;
	int _frameWidth = 0;
	int _frameHeight = 0;
	int _screenWidth = 0;
	int _screenHeight = 0;
	int _frameScale = 2;
	int _colorBits = 8;
	int _bounceBufferLines = 0;
	int _hBorder = 0;
	int _vBorder = 0;
	int _lastBounceBufferPos = 0;
	bool _frameBuffersInPsram = false;

	bool validConfig(int width, int height, int scale = 2, int hborder = 0, int yborder = 0, int bits = 8, int* pins = NULL, bool usePsram = false);
	static bool vsyncEvent(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx);
	static bool bounceEvent(esp_lcd_panel_handle_t panel, void* bounce_buf, int pos_px, int len_bytes, void* user_ctx);
	void swapBuffers();
};