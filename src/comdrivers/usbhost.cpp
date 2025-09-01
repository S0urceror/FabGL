#ifdef CONFIG_IDF_TARGET_ESP32S3

#include "usbhost.h"
#define DAEMON_TASK_PRIORITY    2
#define CLASS_TASK_PRIORITY     3

static const char *TAG = "usb";

namespace fabgl
{

static void host_lib_daemon_task(void *pArg)
{
    USBHost* host = (USBHost*) pArg;
    SemaphoreHandle_t signaling_sem = host->task_synchronisation.signaling_sem;
    
    //ESP_LOGI(TAG, "Installing USB Host Library");
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .root_port_unpowered = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
        .enum_filter_cb = NULL,
        .fifo_settings_custom = {0, 0, 0}
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    //Signal to drivers that the host library is installed
    xSemaphoreGive(signaling_sem);
    vTaskDelay(10); //Short delay to let client task spin up
    
    bool has_clients = true;
    bool has_devices = true;
    while (has_clients || has_devices) {
        uint32_t event_flags;
        esp_err_t err = usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (err!=ESP_OK)
            ESP_LOGE (TAG,"usb host lib error: %d",err);

        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            has_clients = false;
            if ((err = usb_host_device_free_all ())!=ESP_OK)
                ESP_LOGE (TAG,"unable to free devices: %d",err);
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            has_devices = false;
        }
    }
    ESP_LOGI(TAG, "No more clients and devices");

    //Uninstall the USB Host Library
    ESP_ERROR_CHECK(usb_host_uninstall());

    //Wait to be deleted
    xSemaphoreGive(signaling_sem);
    vTaskSuspend(NULL);
}

USBHost::USBHost ()
{
    daemon_task_hdl=0;
}
USBHost::~USBHost ()
{
    //Wait for the tasks to complete
    //xSemaphoreTake(task_synchronisation.signaling_sem, portMAX_DELAY);

    //Delete the tasks
    std::vector <TaskHandle_t>::iterator it;
    for (it = class_driver_tasks.begin();it!=class_driver_tasks.end();it++)
        vTaskDelete(*it);
    vTaskDelete(daemon_task_hdl);
}

void USBHost::begin ()
{
    // init synchronisation objects
    task_synchronisation.signaling_sem = xSemaphoreCreateBinary();
    task_synchronisation.device_connected_sem = xSemaphoreCreateBinary ();

    //Create daemon task
    xTaskCreatePinnedToCore(host_lib_daemon_task,
                            "daemon",
                            4096,
                            (void *)this,
                            DAEMON_TASK_PRIORITY,
                            &daemon_task_hdl,
                            0);
}

void USBHost::add_driver (USBDriver& driver)
{
    // copy task synchronisation objects to driver
    driver.task_synchronisation=task_synchronisation;

    // multiple potential usbdriver tasks;
    TaskHandle_t task;
    xTaskCreatePinnedToCore((void(*) (void*)) driver.get_class_driver_task(),
                            "class",
                            4096,
                            (void *)&driver,
                            CLASS_TASK_PRIORITY,
                            &task,
                            0);
    class_driver_tasks.push_back (task);

    vTaskDelay(10);     //Add a short delay to let the tasks run
}

} // namespace fabgl;

#endif // CONFIG_IDF_TARGET_ESP32S3