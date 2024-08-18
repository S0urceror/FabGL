#include <Arduino.h>
#include "fabgl.h"
 
fabgl::VGAControllerS3 displayController;
fabgl::Terminal        Terminal;

void setup() {
    neopixelWrite(RGB_BUILTIN,0,0,RGB_BRIGHTNESS/8); // Blue
    
    Serial.begin (115200);
    Serial.println ("ElectronHAL S3 edition");
    Serial.printf("esp idf version:%s \n\r",esp_get_idf_version());
    log_d("Total heap: %d", ESP.getHeapSize());
    log_d("Free heap: %d", ESP.getFreeHeap());
    log_d("Total PSRAM: %d", ESP.getPsramSize());
    log_d("Free PSRAM: %d", ESP.getFreePsram());
    
    displayController.begin(fabgl::VGAControllerS3_PIN_AGON_LIGHT);
    //displayController.setResolution(Mode::MODE_1024x768x60);
    //displayController.setResolution(Mode::MODE_800x600x60); 
    displayController.setResolution(Mode::MODE_640x480x60);
 
    Terminal.begin(&displayController);
    //Terminal.setLogStream(Serial);  // DEBUG ONLY
    Terminal.enableCursor(true);

    neopixelWrite(RGB_BUILTIN,0,RGB_BRIGHTNESS/8,0); // Green
}

void slowPrintf(const char * format, ...)
{
  va_list ap;
  va_start(ap, format);
  int size = vsnprintf(nullptr, 0, format, ap) + 1;
  if (size > 0) {
    va_end(ap);
    va_start(ap, format);
    char buf[size + 1];
    vsnprintf(buf, size, format, ap);
    for (int i = 0; i < size; ++i) {
      Terminal.write(buf[i]);
      delay(25);
    }
    // Terminal.write (buf);
    Serial.write (buf);
  }
  va_end(ap);
}
 
 
void demo1()
{
  Terminal.write("\e[44;37m"); // background: blue, foreground: gray
  Terminal.write("\e[2J");     // clear screen
  Terminal.write("\e[1;1H");   // move cursor to 1,1
  slowPrintf("* * * *  W E L C O M E   T O   F a b G L  * * * *\r\n");
  slowPrintf("2019-2022 by Fabrizio Di Vittorio - www.fabgl.com\r\n");
  slowPrintf("ESP-S3 port by S0urceror\r\n");
  slowPrintf("=================================================\r\n\n");
  slowPrintf("A Display Controller, PS2 Mouse and Keyboard Controller, Graphics Library, Audio Engine, Game Engine and ANSI/VT Terminal for the ESP32\r\n\n");
  slowPrintf("Current settings\r\n");
  slowPrintf("Screen Size   : %d x %d\r\n", displayController.getScreenWidth(), displayController.getScreenHeight());
  slowPrintf("Viewport Size : %d x %d\r\n", displayController.getViewPortWidth(), displayController.getViewPortHeight());
  slowPrintf("Terminal Size : %d x %d\r\n", Terminal.getColumns(), Terminal.getRows());
  //slowPrintf("Free Memory   : %d bytes\r\n", heap_caps_get_free_size(MALLOC_CAP_32BIT));
  slowPrintf("Total heap    : %d bytes\r\n", ESP.getHeapSize());
  slowPrintf("Free heap     : %d bytes\r\n", ESP.getFreeHeap());
  slowPrintf("Total PSRAM   : %d bytes\r\n", ESP.getPsramSize());
  slowPrintf("Free PSRAM    : %d bytes\r\n\n", ESP.getFreePsram());
}

void loop() {
    log_d ("in");
    //neopixelWrite(RGB_BUILTIN,0,0,0); // Off
    demo1();    
    //Terminal.canvas()->swapBuffers();
    delay (4000);
    
    //delay (500);
    //neopixelWrite(RGB_BUILTIN,0,RGB_BRIGHTNESS/8,0); // Green
    log_d ("out");
}
