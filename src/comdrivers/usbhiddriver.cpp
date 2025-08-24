/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp32s3/rom/ets_sys.h"
#include "usbhiddriver.h"

static const char *TAG = "usb.hid";
    
// helper
// void print_bytes (uint8_t* start,int size)
// {
//     for (int i=0;i<size;i++)
//     {
//         if (i>0 && i%8==0)
//             ets_printf ("\r\n");
//         ets_printf ("0x%02x, ",start[i]);
//     }
//     ets_printf ("\r\n");
// }

namespace fabgl
{

void USBHIDDriver::hid_client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    USBHIDDriver* driver_obj = (USBHIDDriver *)arg;
    switch (event_msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        ESP_LOGV(TAG, "USB_HOST_CLIENT_EVENT_NEW_DEV: %d",driver_obj->dev_addr);
        if (driver_obj->dev_addr == 0) {
            driver_obj->hid_listening_input=false;
            driver_obj->dev_addr = event_msg->new_dev.address;
            //Open the device next
            driver_obj->actions = ACTION_OPEN_DEV;
            driver_obj->callback_req = CB_USB_HID_UNDEFINED;
        }
        break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        ESP_LOGV(TAG, "USB_HOST_CLIENT_EVENT_DEV_GONE");
        if (driver_obj->dev_hdl != NULL) {
            //Cancel any other actions and close the device next
            driver_obj->actions = ACTION_CLOSE_DEV;
        }
        break;
    default:
        //Should never occur
        abort();
    }
}

void USBHIDDriver::in_xfer_cb(usb_transfer_t *in_xfer)
{
    assert(in_xfer);
    assert(in_xfer->context);
    USBHIDDriver* driver_obj = (USBHIDDriver *) in_xfer->context;
    xSemaphoreGive (driver_obj->in_transfer_done);
}

void USBHIDDriver::hid_transfer_cb(usb_transfer_t *ctrl_xfer)
{
    assert(ctrl_xfer);
    assert(ctrl_xfer->context);
    USBHIDDriver* driver_obj = (USBHIDDriver *) ctrl_xfer->context;
    xSemaphoreGive (driver_obj->control_transfer_done);
}

bool USBHIDDriver::hid_interface_present (const usb_config_desc_t* config_desc,uint8_t req_interface_class,uint8_t req_interface_subclass,uint8_t req_interface_protocol)
{
    assert(config_desc);
    int offset = 0;
    int total_len = config_desc->wTotalLength;
    const usb_intf_desc_t *iface_desc = GET_NEXT_INTERFACE_DESC(config_desc, total_len, offset);
    while (iface_desc != NULL) 
    {
        if (iface_desc->bInterfaceClass==req_interface_class) 
        {
            switch (iface_desc->bInterfaceSubClass)
            {
                case 0x00:
                    //ESP_LOGI (TAG,"HID report");
                    break;
                case 0x01:
                    //ESP_LOGI (TAG,"HID boot");
                    break;
            }
            switch (iface_desc->bInterfaceProtocol)
            {
                case 0x00: 
                    //ESP_LOGI (TAG,"HID generic protocol: %d",iface_desc->iInterface);
                    break;
                case 0x01: 
                    //ESP_LOGI (TAG,"HID keyboard protocol: %d",iface_desc->iInterface);
                    break;
                case 0x02: 
                    //ESP_LOGI (TAG,"HID mouse protocol: %d",iface_desc->iInterface);
                    break;
            }
            if (iface_desc->bNumEndpoints>=1)
            {
                int hid_offset = offset;
                const hid_descriptor_t* hid_desc = (hid_descriptor_t*) usb_parse_next_descriptor_of_type ((const usb_standard_desc_t*)iface_desc,total_len,0x21,&hid_offset);
                if (hid_desc != NULL)
                {
                    //ESP_LOGI (TAG,"HID descriptor type: %d",hid_desc->descriptor_type);
                }
                int endpoint_offset = offset;
                const usb_ep_desc_t *endpoint = usb_parse_endpoint_descriptor_by_index (iface_desc,0,total_len,&endpoint_offset);
                if (endpoint != NULL) {
                    //ESP_LOGI (TAG,"HID endpoint: %d",endpoint->bEndpointAddress);
                }
                if (hid_desc && endpoint && iface_desc->bInterfaceSubClass==req_interface_subclass && iface_desc->bInterfaceProtocol==req_interface_protocol)
                {
                    hid_endpoint = endpoint->bEndpointAddress;
                    hid_interval = endpoint->bInterval;
                    hid_interface_no = iface_desc->bInterfaceNumber;
                    hid_reportsize = hid_desc->descriptor_length;
                    hid_packetsize = endpoint->wMaxPacketSize;
                    return true;
                }
            }
        }
        iface_desc = GET_NEXT_INTERFACE_DESC(iface_desc, total_len, offset);
    }
    return false;
}

void USBHIDDriver::hid_action_open_dev()
{
    assert(dev_addr != 0);
    //ESP_LOGI(TAG, "Opening device at address %d", dev_addr);
    ESP_ERROR_CHECK(usb_host_device_open(client_hdl, dev_addr, &dev_hdl));
    actions = ACTION_NONE;
}

void USBHIDDriver::hid_action_close_dev()
{
    //ESP_LOGI(TAG, "Closing device at address %d", dev_addr);
    
    if (hid_listening_input)
    {
        ESP_ERROR_CHECK(usb_host_interface_release(client_hdl, dev_hdl, hid_interface_no));
        hid_listening_input=false;
    }
    ESP_ERROR_CHECK(usb_host_device_close(client_hdl, dev_hdl));
    dev_hdl = NULL;
    dev_addr = 0;
    hid_device_connected=false;
    //We need to exit the event handler loop
    actions = ACTION_EXIT;
}

void USBHIDDriver::class_driver_task(void *pArg)
{
    USBHIDDriver* driver_obj = (USBHIDDriver*)pArg;
    SemaphoreHandle_t signaling_sem = driver_obj->task_synchronisation.signaling_sem;
    SemaphoreHandle_t connected_sem = driver_obj->task_synchronisation.device_connected_sem;

    //Wait until daemon task has installed USB Host Library
    xSemaphoreTake(signaling_sem, portMAX_DELAY);

    //ESP_LOGI(TAG, "Registering HID Client");
    usb_host_client_config_t client_config = {
        .is_synchronous = false,    //Synchronous clients currently not supported. Set this to false
        .max_num_event_msg = CLIENT_NUM_EVENT_MSG,
        .async = {
            .client_event_callback = hid_client_event_cb,
            .callback_arg = (void *) driver_obj,
        },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &driver_obj->client_hdl));

    while (1) {
        if (driver_obj->actions == 0) {
            usb_host_client_handle_events(driver_obj->client_hdl, portMAX_DELAY);
        } else {
            if (driver_obj->actions == ACTION_OPEN_DEV) {
                driver_obj->hid_action_open_dev();
                xSemaphoreGive(connected_sem);
            }
            if (driver_obj->actions == ACTION_CLOSE_DEV) {
                driver_obj->hid_action_close_dev();
            }
            if (driver_obj->actions == ACTION_EXIT) {
                // this client will continue to wait for the next device to be inserted
                driver_obj->actions = ACTION_NONE;
            }
        }
    }

    //ESP_LOGI(TAG, "Deregistering HID Client");
    ESP_ERROR_CHECK(usb_host_client_deregister(driver_obj->client_hdl));

    //Wait to be deleted
    xSemaphoreGive(signaling_sem);
    vTaskSuspend(NULL);
}

bool USBHIDDriver::getNextVirtualKey (VirtualKeyItem * item, int timeOutMS)
{
    usb_transfer_t*             in_transfer;
    uint8_t*                    report_ptr;
    uint8_t*                    end_ptr;
    std::vector<VirtualKeyItem> items;
    
    if (!hid_listening_input)
        return false;

    // get new keypresses if the buffer is empty
    if (keyboard_buffer.size()==0)
    {
        ESP_RETURN_ON_ERROR( usb_host_transfer_alloc (hid_packetsize, 0, &in_transfer),
                            TAG,"Unable to allocate transfer buffer for EP IN");

        in_transfer->device_handle = dev_hdl;
        in_transfer->callback = in_xfer_cb;
        in_transfer->context = this;
        in_transfer->timeout_ms = 5000;
        in_transfer->bEndpointAddress = hid_endpoint;
        in_transfer->num_bytes = hid_packetsize;

        ESP_RETURN_ON_ERROR( usb_host_transfer_submit (in_transfer),
                            TAG,"Cannot start IN transfer to interface %d EP %d",hid_interface_no,hid_endpoint);
        xSemaphoreTake (in_transfer_done,portMAX_DELAY);
        if (in_transfer->status==USB_TRANSFER_STATUS_COMPLETED) 
        {
            report_ptr = in_transfer->data_buffer;
            end_ptr = report_ptr + hid_packetsize;
            //print_bytes (report_ptr,hid_packetsize);
            while (report_ptr < end_ptr)
            {
                items = parser.parse_keyboard_input_report (report_ptr);
                for (int i=0;i<items.size();i++)
                    keyboard_buffer.push_back (items[i]);
            }
        }
        ESP_ERROR_CHECK( usb_host_transfer_free(in_transfer) );
    }
    // do we now have items in the buffer?
    if (keyboard_buffer.size()>0)
    {
        *item = keyboard_buffer[keyboard_buffer.size()-1];
        keyboard_buffer.pop_back();
        return true;
    }
    else
        return false;
}

void USBHIDDriver::begin ()
{
    usb_device_info_t           dev_info;
    const usb_device_desc_t*    dev_desc;
    const usb_config_desc_t*    config_desc;
    usb_transfer_t*             control_transfer;
    uint8_t*                    report_ptr;
    uint8_t*                    report_end_ptr;
        
    hid_device_connected = false;
    // clear hid application collection and dependent objects
    application_collection.initialize();
    
    // check if something is connected within 0,5 secs
    if (xSemaphoreTake (task_synchronisation.device_connected_sem,pdMS_TO_TICKS(500))!=pdTRUE)
    {
        ESP_LOGE (TAG,"No USB device connected");
        return;
    }
    // scan descriptors to determine if this is a HID device
    ESP_ERROR_CHECK(usb_host_device_info(dev_hdl, &dev_info));
    //ESP_LOGI(TAG, "USB device information");
    //ESP_LOGI(TAG, "\t%s speed", (dev_info.speed == USB_SPEED_LOW) ? "Low" : "Full");
    //ESP_LOGI(TAG, "\tbConfigurationValue %d", dev_info.bConfigurationValue);
    // device descriptor
    ESP_ERROR_CHECK(usb_host_get_device_descriptor(dev_hdl, &dev_desc));
    //usb_print_device_descriptor(dev_desc);
    // config descriptor
    ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(dev_hdl, &config_desc));
    //usb_print_config_descriptor(config_desc, NULL);
    // check if hid interface is present
    if (!hid_interface_present(config_desc,USB_CLASS_HID,0x01,0x01)) // BOOT_PROTOCOL, KEYBOARD_INTERFACE
    {
        ESP_LOGE (TAG, "No USB HID connected");
        return;
    }
    // get further device information
    if (dev_info.str_desc_manufacturer) 
    {
        //ESP_LOGI(TAG, "Getting Manufacturer string descriptor");
        //usb_print_string_descriptor(dev_info.str_desc_manufacturer);
    }
    if (dev_info.str_desc_product) 
    {
        //ESP_LOGI(TAG, "Getting Product string descriptor");
        //usb_print_string_descriptor(dev_info.str_desc_product);
    }
    if (dev_info.str_desc_serial_num) 
    {
        //ESP_LOGI(TAG, "Getting Serial Number string descriptor");
        //usb_print_string_descriptor(dev_info.str_desc_serial_num);
    }
    // HID report descriptor
    ESP_ERROR_CHECK(usb_host_transfer_alloc (sizeof (usb_setup_packet_t)+hid_reportsize,0,&control_transfer));
    control_transfer->device_handle = dev_hdl;
    control_transfer->callback = hid_transfer_cb;
    control_transfer->context = this;
    GET_HID_REPORT_DESCRIPTOR((usb_setup_packet_t *)control_transfer->data_buffer, hid_interface_no, hid_reportsize);
    control_transfer->num_bytes=sizeof (usb_setup_packet_t)+hid_reportsize;
    control_transfer->bEndpointAddress=0x00;
    ESP_ERROR_CHECK(usb_host_transfer_submit_control(client_hdl,control_transfer));
    xSemaphoreTake (control_transfer_done,portMAX_DELAY);
    if (control_transfer->status==USB_TRANSFER_STATUS_COMPLETED)
    {
        TEST_ASSERT_EQUAL(control_transfer->num_bytes, control_transfer->actual_num_bytes);
        //ESP_LOGI(TAG, "HID report descriptor returned:");
        report_ptr = control_transfer->data_buffer+sizeof(usb_setup_packet_t);
        report_end_ptr = report_ptr + hid_reportsize;
        //print_bytes (control_transfer->data_buffer+sizeof(usb_setup_packet_t),hid_reportsize);
        while (report_ptr < report_end_ptr)
            application_collection.parse_descriptor_item (report_ptr);
    }
    ESP_ERROR_CHECK( usb_host_transfer_free(control_transfer) );

    // set idle
    ESP_ERROR_CHECK(usb_host_transfer_alloc (sizeof (usb_setup_packet_t),0,&control_transfer));
    control_transfer->device_handle = dev_hdl;
    control_transfer->callback = hid_transfer_cb;
    control_transfer->context = this;
    SET_HID_IDLE ((usb_setup_packet_t *)control_transfer->data_buffer, hid_interface_no,0,0);
    control_transfer->num_bytes=sizeof (usb_setup_packet_t);
    control_transfer->bEndpointAddress=0x00;
    ESP_ERROR_CHECK(usb_host_transfer_submit_control(client_hdl,control_transfer));
    xSemaphoreTake (control_transfer_done,portMAX_DELAY);
    //if (control_transfer->status==USB_TRANSFER_STATUS_COMPLETED)
        //ESP_LOGI(TAG, "HID idle rate set");
    ESP_ERROR_CHECK( usb_host_transfer_free(control_transfer) );

    // claim the HID interface for further processing
    ESP_ERROR_CHECK( usb_host_interface_claim( client_hdl, dev_hdl, hid_interface_no,0) );
    hid_listening_input=true;

    // indicate we were succesful
    hid_device_connected = true;
}

USBHIDDriver::USBHIDDriver()
{
    control_transfer_done = xSemaphoreCreateBinary ();
    in_transfer_done = xSemaphoreCreateBinary ();
}

} // namespace fabgl;