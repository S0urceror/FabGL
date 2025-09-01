#pragma once
#ifdef CONFIG_IDF_TARGET_ESP32S3
#include "fabutils.h"
#include "usb/usb_host.h"
#include "usbhost.h"
#include "usbdriver.h"
#include "usbhidparser.h"

#define CLIENT_NUM_EVENT_MSG        5

#define ACTION_NONE                 0x00
#define ACTION_OPEN_DEV             0x01
#define ACTION_CLOSE_DEV            0x02
#define ACTION_EXIT                 0x03
// #define ACTION_GET_DEV_INFO         0x02
// #define ACTION_GET_DEV_DESC         0x04
// #define ACTION_GET_CONFIG_DESC      0x08
// #define ACTION_GET_REPORT_DESCRIPTOR 0x10
// #define ACTION_GET_STR_DESC         0x20
// #define ACTION_CLOSE_DEV            0x40
// #define ACTION_SET_IDLE             0x80
// #define ACTION_XFER_REPORTS         0x100

#define CB_USB_HID_UNDEFINED            0
#define CB_USB_HID_REPORT_DESCRIPTOR    1
#define CB_USB_HID_SET_IDLE             2

#define TEST_ASSERT_EQUAL_MESSAGE(a,b,str) if (a!=b) ESP_LOGE (TAG,str)
#define TEST_ASSERT_EQUAL(a,b) if (a!=b) {ESP_LOGE(TAG,"assertion error");abort();}

typedef enum {
    HID_CLASS_SPECIFIC_REQ_GET_REPORT = 0x01,
    HID_CLASS_SPECIFIC_REQ_GET_IDLE = 0x02,
    HID_CLASS_SPECIFIC_REQ_GET_PROTOCOL = 0x03,
    HID_CLASS_SPECIFIC_REQ_SET_REPORT = 0x09,
    HID_CLASS_SPECIFIC_REQ_SET_IDLE = 0x0A,
    HID_CLASS_SPECIFIC_REQ_SET_PROTOCOL = 0x0B
} __attribute__((packed)) hid_class_specific_req_t;

#define GET_HID_REPORT_DESCRIPTOR(setup_pkt_ptr, interface_no, descriptor_size) ({  \
    (setup_pkt_ptr)->bmRequestType = USB_BM_REQUEST_TYPE_DIR_IN | USB_BM_REQUEST_TYPE_TYPE_STANDARD | USB_BM_REQUEST_TYPE_RECIP_INTERFACE; \
    (setup_pkt_ptr)->bRequest = 0x06;               \
    (setup_pkt_ptr)->wValue = 0x2200;               \
    (setup_pkt_ptr)->wIndex = interface_no;         \
    (setup_pkt_ptr)->wLength = descriptor_size;     \
})

#define SET_HID_IDLE(setup_pkt_ptr,interface_no,duration,report_id) ({ \
    (setup_pkt_ptr)->bmRequestType = USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_CLASS | USB_BM_REQUEST_TYPE_RECIP_INTERFACE; \
    (setup_pkt_ptr)->bRequest = HID_CLASS_SPECIFIC_REQ_SET_IDLE;  \
    (setup_pkt_ptr)->wValue = (duration << 8) | report_id;        \
    (setup_pkt_ptr)->wIndex = interface_no;                       \
    (setup_pkt_ptr)->wLength = 0;                                 \
})

// USB Descriptor parsing helping macros
#define GET_NEXT_INTERFACE_DESC(p, max_len, offs)                                                \
    ((const usb_intf_desc_t *)usb_parse_next_descriptor_of_type((const usb_standard_desc_t *)p,  \
                                                                max_len,                         \
                                                                USB_B_DESCRIPTOR_TYPE_INTERFACE, \
                                                                &(offs)))

#define GET_NEXT_ENDPOINT_DESC(p, max_len, offs)                                                 \
    ((const usb_ep_desc_t *)usb_parse_next_descriptor_of_type((const usb_standard_desc_t *)p, \
                                                                max_len,                         \
                                                                USB_B_DESCRIPTOR_TYPE_ENDPOINT,  \
                                                                &(offs)))
                                                                
typedef struct __attribute__((packed)) _USB_HID_DESCRIPTOR {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t hid_version;
    uint8_t country_code;
    uint8_t num_descriptors;
    uint8_t descriptor_type;
    uint16_t descriptor_length;
} hid_descriptor_t;

namespace fabgl
{
    class USBHIDDriver : public USBDriver
    {
        protected:
            // root collection
            hid_collection              application_collection;
            hid_report_parser           parser;
            usb_host_client_handle_t    client_hdl;
            uint8_t                     dev_addr;
            usb_device_handle_t         dev_hdl;
            uint32_t                    actions;
            uint32_t                    callback_req;
            uint8_t                     hid_interface_no;
            uint8_t                     hid_endpoint;
            uint16_t                    hid_reportsize;
            uint16_t                    hid_packetsize;
            uint8_t                     hid_interval;
            bool                        hid_listening_input;
            SemaphoreHandle_t           control_transfer_done;
            SemaphoreHandle_t           in_transfer_done;
            bool                        hid_device_connected;
            std::vector<VirtualKeyItem> keyboard_buffer;

            // callbacks
            static void hid_client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg);
            static void in_xfer_cb(usb_transfer_t *in_xfer);
            static void hid_transfer_cb(usb_transfer_t *ctrl_xfer);
            
            void hid_action_open_dev();
            // void hid_action_get_info();
            // void hid_action_get_dev_desc();
            bool hid_interface_present (const usb_config_desc_t* config_desc,uint8_t req_interface_class,uint8_t req_interface_subclass,uint8_t req_interface_protocol);
            // void hid_action_get_config_desc();
            // void hid_action_get_str_desc();
            // esp_err_t hid_xfer_reports();
            // void hid_action_get_report_descriptor ();
            // void hid_action_set_idle ();
            void hid_action_close_dev();

        public:
            USBHIDDriver ();
            // class driver routines
            static void class_driver_task(void *pArg);
            void* get_class_driver_task () {return (void*) class_driver_task;};

            void begin ();
            inline bool connected () {return hid_device_connected;};
            bool getNextVirtualKey (VirtualKeyItem * item, int timeOutMS);
    };

} // fabgl;
#endif // CONFIG_IDF_TARGET_ESP32S3