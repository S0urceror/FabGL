#pragma once

namespace fabgl
{
    typedef struct {
        SemaphoreHandle_t signaling_sem;
        SemaphoreHandle_t device_connected_sem;
    } task_synchronisation_t;

    class USBDriver
    {
        private:
            
        public:
            task_synchronisation_t task_synchronisation;

            virtual void begin ()=0;
            virtual bool connected ()=0;
            virtual void* get_class_driver_task ()=0;
            
    };
} //namespace fabgl;