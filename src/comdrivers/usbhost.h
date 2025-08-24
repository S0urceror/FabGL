#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "usb/usb_host.h"
#include "usbdriver.h"
#include <vector>

namespace fabgl
{
    class USBHost
    {
        private:
            TaskHandle_t daemon_task_hdl;
            std::vector <TaskHandle_t> class_driver_tasks;
            
        public:
            task_synchronisation_t task_synchronisation;

            USBHost ();
            ~USBHost ();
            void begin ();
            void add_driver (USBDriver& driver);
    };
} // namespace fabgl;