#pragma once
#include "fabutils.h"
#include <vector>
#include <cstdint>

namespace fabgl 
{
    typedef enum  
    {
        MAIN = 0,
        GLOBAL = 1,
        LOCAL = 2,
        RESERVED = 3
    } hid_item_t;

    typedef enum
    {
        PHYSICAL = 0,
        APPLICATION = 1,
        LOGICAL = 2,
        REPORT = 3,
        NAMED_ARRAY = 4,
        USAGE_SWITCH = 5,
        USAGE_MODIFIER = 6
    } hid_collection_t;

    class item_bits
    {
        private:
            bool _constant;
            bool _data;
            bool _variable;
            bool _array;
            bool _relative;
            bool _absolute;
            bool _wrap;
            bool _nowrap;
            bool _nonlinear;
            bool _linear;
            bool _nonpreferred;
            bool _preferred;
            bool _nullstate;
            bool _nonnullposition;
            bool _reserved;
            bool _volatile;
            bool _nonvolatile;
            bool _buffered;
            bool _bitfield;
            std::vector<uint8_t> _bitdata;

        public:
            item_bits ();
            item_bits (std::vector<uint8_t>& data);
            std::vector <uint8_t> get_data ();

            inline bool is_constant() {return _constant;};
            inline bool is_data() {return _data;};
            inline bool is_variable() {return _variable;};
            inline bool is_array() {return _array;};
            inline bool is_relative() {return _relative;};
            inline bool is_absolute() {return _absolute;};
    };

    class hid_global_item
    {
        public:
            uint32_t    usage_page;
            int32_t     logical_minimum,logical_maximum;
            int32_t     physical_minimum,physical_maximum;
            int32_t     unit_exponent;
            uint16_t    unit;
            uint16_t    report_size;
            uint16_t    report_count;
            uint8_t     report_id;
        
            hid_global_item ();
    };

    class hid_main_item
    {
        public:
            item_bits               input_bits,output_bits,feature_bits;
            hid_collection_t        collection_type;
        
            hid_main_item ();
    };

    class hid_local_item
    {
        public:
            std::vector <uint16_t> usage_ids;
            uint16_t     usage_minimum;
            uint16_t     usage_maximum;
            uint16_t     designator_index;
            uint16_t     designator_minimum;
            uint16_t     designator_maximum;
            uint16_t     string_index;
            uint16_t     string_minimum;
            uint16_t     string_maximum;
            uint16_t     delimiter;

            hid_local_item ();
    };

    typedef enum 
    {
        REPORT_INPUT=0,
        REPORT_OUTPUT=1,
        REPORT_FEATURE=2
    } hid_report_t;

    class hid_report
    {
        private:
            uint16_t _usage;
            hid_report_t _type;
            hid_main_item _main_item;
            hid_global_item _global_item;
            hid_local_item _local_item;
            
        public:
            hid_report (hid_report_t type,uint16_t usage, hid_main_item& main_item,hid_global_item& global_item,hid_local_item& local_item) ;
            void print ();
            uint16_t get_bit_len ();
            bool is_input ();
            bool is_output ();
            bool is_keypad ();
            bool is_keypad_keys ();
            bool is_keypad_modifiers ();
            bool is_mouse_buttons();
            bool is_mouse_x();
            bool is_mouse_y();
            bool is_mouse_wheel();
            bool is_leds ();
            bool is_signed ();
            uint8_t get_report_id ();
            uint8_t get_report_count ();
            uint8_t get_report_size ();
            item_bits& get_input_bits();
            item_bits& get_output_bits();
            item_bits& get_feature_bits();
    };

    typedef struct 
    {
        fabgl::VirtualKey key_lower;
        uint8_t lower;
        fabgl::VirtualKey key_shifted;
        uint8_t shifted;
    } ascii_value_t;
        
    class hid_collection
    {
        private:
            std::vector <hid_collection>    collections;

            hid_global_item                 global_item;
            hid_main_item                   main_item;
            hid_local_item                  local_item;
            uint8_t                         item_tag;
            hid_item_t                      item_type;
            uint8_t                         item_size;
            std::vector <uint8_t>           item_data;
            uint8_t                         longitem_tag;
            
            uint32_t read_item_data (std::vector<uint8_t>& item_data);
            int32_t  read_item_data_signed (std::vector<uint8_t>& item_data);

        public:
            hid_collection ();
            hid_collection (hid_main_item main_item,hid_global_item global_item,hid_local_item local_item);
            void initialize ();
            void create_reports_per_usage (uint8_t item_tag);
            bool parse_descriptor_item (uint8_t*& report_ptr);         
    };

    class hid_report_parser
    {
        private:
            std::vector<VirtualKeyItem> prev_key_items;

            uint32_t extract_bits (uint8_t* report_ptr,uint8_t from,uint8_t size);

        public:
            void print_reports ();
            bool parse_input_reports (uint8_t*& report_ptr);
            std::vector<VirtualKeyItem> parse_keyboard_input_report (uint8_t*& report_ptr);
            void process_report_value (hid_report& report,uint8_t report_index,int32_t value);
    };

} // namespace fabgl;