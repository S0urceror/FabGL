#include "usbhidparser.h"
#include <iostream>
#include <iomanip>

#define HID_VERBOSE_PROCESS_VALUE       0
#define HID_VERBOSE_PARSE_REPORT        0
#define HID_VERBOSE_PRINT_REPORT        0
#define HID_VERBOSE_PARSE_DESCRIPTOR    0

namespace fabgl
{

std::vector <hid_report> reports;
std::vector <uint8_t> report_ids;

const char* hid_collection_type []={
    "PHYSICAL","APPLICATION","LOGICAL","REPORT","NAMED_ARRAY","USAGE_SWITCH","USAGE_MODIFIER"
};
const char* hid_item_type []={
    "MAIN","GLOBAL","LOCAL","RESERVED"
};
const char* hid_report_type []={
    "INPUT","OUTPUT","FEATURE"
};


ascii_value_t ascii_values[] = {
    {VK_NONE,0,VK_NONE,0},
    {VK_NONE,0,VK_NONE,0},
    {VK_NONE,0,VK_NONE,0},
    {VK_NONE,0,VK_NONE,0},
    {VK_a,'a',VK_A,'A'},
    {VK_b,'b',VK_B,'B'},
    {VK_c,'c',VK_C,'C'},
    {VK_d,'d',VK_D,'D'},
    {VK_e,'e',VK_E,'E'},
    {VK_f,'f',VK_F,'F'},
    {VK_g,'g',VK_G,'G'},
    {VK_h,'h',VK_H,'H'},
    {VK_i,'i',VK_I,'I'},
    {VK_j,'j',VK_J,'J'},
    {VK_k,'k',VK_K,'K'},
    {VK_l,'l',VK_L,'L'},
    {VK_m,'m',VK_M,'M'},
    {VK_n,'n',VK_N,'N'},
    {VK_o,'o',VK_O,'O'},
    {VK_p,'p',VK_P,'P'},
    {VK_q,'q',VK_Q,'Q'},
    {VK_r,'r',VK_R,'R'},
    {VK_s,'s',VK_S,'S'},
    {VK_t,'t',VK_T,'T'},
    {VK_u,'u',VK_U,'U'},
    {VK_v,'v',VK_V,'V'},
    {VK_w,'w',VK_W,'W'},
    {VK_x,'x',VK_X,'X'},
    {VK_y,'y',VK_Y,'Y'},
    {VK_z,'z',VK_Z,'Z'},
    {VK_1,'1',VK_EXCLAIM,'!'},
    {VK_2,'2',VK_AT,'@'},
    {VK_3,'3',VK_HASH,'#'},
    {VK_4,'4',VK_DOLLAR,'$'},
    {VK_5,'5',VK_PERCENT,'%'},
    {VK_6,'6',VK_CARET,'^'},
    {VK_7,'7',VK_AMPERSAND,'&'},
    {VK_8,'8',VK_ASTERISK,'*'},
    {VK_9,'9',VK_LEFTPAREN,'('},
    {VK_0,'0',VK_RIGHTPAREN,')'},
    {VK_RETURN,'\r',VK_NONE,0},
    {VK_ESCAPE,27,VK_NONE,0},
    {VK_BACKSPACE,8,VK_NONE,0},
    {VK_TAB,9,VK_NONE,0},
    {VK_SPACE,' ',VK_NONE,0},
    {VK_MINUS,'-',VK_MINUS,'_'},
    {VK_EQUALS,'=',VK_PLUS,'+'},
    {VK_LEFTBRACKET,'[',VK_LEFTBRACE,'{'},
    {VK_RIGHTBRACKET,']',VK_RIGHTBRACE,'}'},
    {VK_BACKSLASH,'\\',VK_VERTICALBAR,'|'},
    {VK_NONE,0,VK_NONE,0},
    {VK_SEMICOLON,';',VK_COLON,':'},
    {VK_ACUTEACCENT,'\'',VK_QUOTEDBL,'\"'},
    {VK_GRAVEACCENT,'`',VK_TILDE,'~'},
    {VK_COMMA,',',VK_LESS,'<'},
    {VK_PERIOD,'.',VK_GREATER,'>'},
    {VK_SLASH,'/',VK_QUESTION,'?'},
    {VK_CAPSLOCK,0,VK_NONE,0},
    {VK_F1,0,VK_NONE,0},
    {VK_F2,0,VK_NONE,0},
    {VK_F3,0,VK_NONE,0},
    {VK_F4,0,VK_NONE,0},
    {VK_F5,0,VK_NONE,0},
    {VK_F6,0,VK_NONE,0},
    {VK_F7,0,VK_NONE,0},
    {VK_F8,0,VK_NONE,0},
    {VK_F9,0,VK_NONE,0},
    {VK_F10,0,VK_NONE,0},
    {VK_F11,0,VK_NONE,0},
    {VK_F12,0,VK_NONE,0},
    {VK_PRINTSCREEN,0,VK_NONE,0},
    {VK_SCROLLLOCK,0,VK_NONE,0},
    {VK_PAUSE,0,VK_NONE,0},
    {VK_INSERT,0,VK_NONE,0},
    {VK_HOME,0,VK_NONE,0},
    {VK_PAGEUP,0,VK_NONE,0},
    {VK_DELETE,127,VK_NONE,0},
    {VK_END,0,VK_NONE,0},
    {VK_PAGEDOWN,0,VK_NONE,0},
    {VK_RIGHT,0,VK_NONE,0},
    {VK_LEFT,0,VK_NONE,0},
    {VK_DOWN,0,VK_NONE,0},
    {VK_UP,0,VK_NONE,0}
};

// HID input/output/feature bits
item_bits::item_bits () 
{
};

item_bits::item_bits (std::vector<uint8_t>& data)
{
    _bitdata=data;
    if (data.size()==0)
    {
        _constant = false;
        _data = !_constant;
        _variable = false;
        _array = !_variable;
        _relative = false;
        _absolute = !_relative;
        _wrap = false;
        _nowrap = !_wrap;
        _nonlinear = false;
        _linear = !_nonlinear;
        _nonpreferred = false;
        _preferred = !_nonpreferred;
        _nullstate = false;
        _nonnullposition = !_nullstate;
        _volatile = false;
        _nonvolatile = !_volatile;
        _reserved = false;
        _buffered = false;
        _bitfield = !_buffered;
    }
    if (data.size()>=1)
    {
        _constant = data[0] & 0b00000001;
        _data = !_constant;
        _variable = data[0] & 0b00000010;
        _array = !_variable;
        _relative = data[0] & 0b00000100;
        _absolute = !_relative;
        _wrap = data[0] & 0b00001000;
        _nowrap = !_wrap;
        _nonlinear = data[0] & 0b00010000;
        _linear = !_nonlinear;
        _nonpreferred = data[0] & 0b00100000;
        _preferred = !_nonpreferred;
        _nullstate = data[0] & 0b01000000;
        _nonnullposition = !_nullstate;
        _volatile = data[0] & 0b10000000;
        _nonvolatile = !_volatile;
        _reserved = data[0] & 0b10000000;
    }
    if (data.size()>=2)
    {
        _buffered = data[1] & 0b00000001;
        _bitfield = !_buffered;
    }
};
std::vector <uint8_t> item_bits::get_data () 
{
    return _bitdata;
};

// HID global item
hid_global_item::hid_global_item ()
{
    usage_page = 0;
    logical_minimum = 0;
    logical_maximum = 0;
    physical_maximum = 0;
    physical_minimum = 0;
    unit_exponent = 0;
    unit = 0;
    report_size = 0;
    report_count = 0;
    report_id = 0;
};

// HID main item
hid_main_item::hid_main_item ()
{
    collection_type = PHYSICAL;
};

// HID local item
hid_local_item::hid_local_item ()
{
    usage_maximum = 0;
    usage_minimum = 0;
    designator_index = 0;
    designator_maximum = 0;
    designator_minimum = 0;
    string_index = 0;
    string_maximum = 0;
    string_minimum = 0;
    delimiter = 0 ;
};

// HID report, contains information of the identified report
hid_report::hid_report (hid_report_t type,uint16_t usage, hid_main_item& main_item,hid_global_item& global_item,hid_local_item& local_item) 
{
    _type = type;
    _usage = usage;
    _main_item = main_item;
    _global_item = global_item;
    _local_item = local_item;

    // decided to create a report per usage, need to divide the number of bits per usage
    if ((_local_item.usage_ids.size()>1) && (_local_item.usage_ids.size()<=_global_item.report_count))
        _global_item.report_count = _global_item.report_count / _local_item.usage_ids.size();
};

void hid_report::print ()
{
    #if HID_VERBOSE_PRINT_REPORT

    std::cout << "Report";
    if (_global_item.usage_page==1)
    {
        std::cout << " - Pointer";
        std::vector <uint16_t>::iterator it;
        switch (_usage)
        {
            case 0x30 : std::cout << " - X"; break;
            case 0x31 : std::cout << " - Y"; break;
            case 0x32 : std::cout << " - Z"; break;
            case 0x38 : std::cout << " - Wheel"; break;
            default: std::cout << " - unknown";break;
        } 
    }
    if (_global_item.usage_page==7)
    {
        std::cout << " - Keyboard/keypad";
        if (get_input_bits().is_array())
        {
            std::cout << " - Keys";
        }
        if (get_input_bits().is_data() && get_report_size()==1)
        {
            std::cout << " - Modifiers";
        }
    }
    if (_global_item.usage_page==9)
    {
        std::cout << " - Buttons";
    }
    std::cout << std::endl;
    std::cout << "\t" << "bitlength:" << get_bit_len() << std::endl;
    std::cout << "\t" << "type:" << hid_report_type[_type] << std::endl;
    std::cout << "\t" << "usage: 0x" << std::hex << int(_usage) << std::dec << std::endl;
    std::cout << "\t" << "m.collection type:" << hid_collection_type[_main_item.collection_type] << std::endl;
    if (_main_item.input_bits.get_data().size()==1)
        std::cout << "\t" << "m.inputbits:" << int(_main_item.input_bits.get_data()[0]) << std::endl;
    if (_main_item.output_bits.get_data().size()==1)
        std::cout << "\t" << "m.outputbits:" << int(_main_item.output_bits.get_data()[0]) << std::endl;
    if (_main_item.feature_bits.get_data().size()==1)
        std::cout << "\t" << "m.featurebits:" << int(_main_item.feature_bits.get_data()[0]) << std::endl;
    std::cout << "\t" << "g.report_id:" << int(_global_item.report_id) << std::endl;
    std::cout << "\t" << "g.report_count:" << int(_global_item.report_count) << std::endl;
    std::cout << "\t" << "g.report_size:" << int(_global_item.report_size) << std::endl;
    std::cout << "\t" << "g.usage_page: 0x" << std::hex << int(_global_item.usage_page) << std::dec << std::endl;
    std::cout << "\t" << "g.logical_minimum:" << int(_global_item.logical_minimum) << std::endl;
    std::cout << "\t" << "g.logical_maximum:" << int(_global_item.logical_maximum) << std::endl;
    std::cout << "\t" << "g.physical_minimum:" << int(_global_item.physical_minimum) << std::endl;
    std::cout << "\t" << "g.physical_maximum:" << int(_global_item.physical_maximum) << std::endl;
    std::cout << "\t" << "l.usage_minimum:" << int(_local_item.usage_minimum) << std::endl;
    std::cout << "\t" << "l.usage_maximum:" << int(_local_item.usage_maximum) << std::endl;
    std::cout << "\t" << "l.delimiter:" << int(_local_item.delimiter) << std::endl;
    std::cout << "\t" << "l.string_index:" << int(_local_item.string_index) << std::endl;
    std::cout << "\t" << "l.string_minimum:" << int(_local_item.string_minimum) << std::endl;
    std::cout << "\t" << "l.string_maximum:" << int(_local_item.string_maximum) << std::endl;
    std::cout << "\t" << "l.designator_index:" << int(_local_item.designator_index) << std::endl;
    std::cout << "\t" << "l.designator_minimum:" << int(_local_item.designator_minimum) << std::endl;
    std::cout << "\t" << "l.designator_maximum:" << int(_local_item.designator_maximum) << std::endl;

    #endif // HID_VERBOSE_PRINT_REPORT
};

uint16_t hid_report::get_bit_len ()
{
    return _global_item.report_count * _global_item.report_size;
}
bool hid_report::is_input ()
{
    return _type==REPORT_INPUT;
}
bool hid_report::is_output ()
{
    return _type==REPORT_OUTPUT;
}
bool hid_report::is_keypad ()
{
    return _global_item.usage_page==7;
}
bool hid_report::is_keypad_keys ()
{
    return (_global_item.usage_page==7) && (get_input_bits().is_array());
}
bool hid_report::is_keypad_modifiers ()
{
    return (_global_item.usage_page==7) && (get_input_bits().is_data() && get_report_size()==1);
}
bool hid_report::is_mouse_buttons()
{
    return _global_item.usage_page==9;
}
bool hid_report::is_mouse_x()
{
    return _global_item.usage_page==0x01 && _usage==0x30;
}
bool hid_report::is_mouse_y()
{
    return _global_item.usage_page==0x01 && _usage==0x31;
}
bool hid_report::is_mouse_wheel()
{
    return _global_item.usage_page==0x01 && _usage==0x38;
}
bool hid_report::is_leds ()
{
    return _global_item.usage_page==8;
}
bool hid_report::is_signed ()
{
    return _global_item.logical_maximum<0 || _global_item.logical_minimum<0;
}
uint8_t hid_report::get_report_id ()
{
    return _global_item.report_id;
}
uint8_t hid_report::get_report_count ()
{
    return _global_item.report_count;
}
uint8_t hid_report::get_report_size ()
{
    return _global_item.report_size;
}
item_bits& hid_report::get_input_bits()
{
    return _main_item.input_bits;
}
item_bits& hid_report::get_output_bits()
{
    return _main_item.output_bits;
}
item_bits& hid_report::get_feature_bits()
{
    return _main_item.feature_bits;
}
// HID collection structure
hid_collection::hid_collection ()
{
    item_tag=0;
    item_type=MAIN;
    item_size=0;
    longitem_tag=0;
};
hid_collection::hid_collection (hid_main_item main,hid_global_item global,hid_local_item local)
{
    hid_collection();
    main_item = main;
    global_item = global;
    //local_item = local;
}
void hid_collection::initialize ()
{
    reports.clear();
    report_ids.clear();
}
uint32_t hid_collection::read_item_data (std::vector<uint8_t>& item_data)
{
    uint32_t value=0;
    if (item_data.size()==1)
        value=item_data[0];
    if (item_data.size()==2)
        value=(item_data[1]<<8) + item_data[0];
    if (item_data.size()==4)
        value=(item_data[3]<<24) + (item_data[2]<<16) + (item_data[1]<<8) + item_data[0];
    return value;
}
int32_t hid_collection::read_item_data_signed (std::vector<uint8_t>& item_data)
{
    if (item_data.size()==1)
    {
        uint8_t value;
        value=item_data[0];
        if (value & 0x80)
            value = -(~value+1);
        return int8_t(value);
    }
    if (item_data.size()==2)
    {
        uint16_t value;
        value=(item_data[1]<<8) + item_data[0];
        if (value & 0x8000)
            value = -(~value+1);
        return int16_t(value);
    }
    if (item_data.size()==4)
    {
        uint32_t value;
        value=(item_data[3]<<24) + (item_data[2]<<16) + (item_data[1]<<8) + item_data[0];
        if (value & 0x80000000)
            value = -(~value+1);
        return int32_t(value);
    }
    return 0;
}
void hid_collection::create_reports_per_usage (uint8_t item_tag)
{
    std::vector<uint16_t>::iterator it;
    
    if (local_item.usage_ids.size()==0)
        reports.push_back (hid_report ((hid_report_t)(item_tag&0b11),0,main_item,global_item,local_item));
    else
    {
        for (it=local_item.usage_ids.begin();it!=local_item.usage_ids.end();it++)
            reports.push_back (hid_report ((hid_report_t)(item_tag&0b11),*it,main_item,global_item,local_item));
    }
}
bool hid_collection::parse_descriptor_item (uint8_t*& report_ptr)
{
    uint16_t usage_id;
    // read short or long item
    //
    item_tag = (*report_ptr & 0b11110000) >> 4;
    item_type = (hid_item_t) ((*report_ptr & 0b00001100) >> 2);
    item_size = (*report_ptr & 0b00000011);
    report_ptr++;

    if (item_size==0b11)
        item_size=4;
    if (item_tag==0b1111)
    {
        item_size = *(report_ptr++);
        longitem_tag = *(report_ptr++);
    }
    while (item_size--)
        item_data.push_back (*(report_ptr++));
    
    // interpret item
    switch (item_type)
    {
        case MAIN: 
        {
            #if HID_VERBOSE_PARSE_DESCRIPTOR
            std::cout << "M.";
            #endif
            switch (item_tag)
            {
                case 0b1000:
                    #if HID_VERBOSE_PARSE_DESCRIPTOR
                    std::cout << "Input" << std::endl;
                    #endif
                    main_item.input_bits = item_bits (item_data);
                    create_reports_per_usage (item_tag&0b11);
                    local_item.usage_ids.clear();
                    main_item.input_bits = item_bits (); // clear input bits
                    // erase local items from state
                    local_item = hid_local_item ();
                    break;
                case 0b1001:
                    #if HID_VERBOSE_PARSE_DESCRIPTOR
                    std::cout << "Output" << std::endl;
                    #endif
                    main_item.output_bits = item_bits (item_data);
                    create_reports_per_usage (item_tag&0b11);
                    local_item.usage_ids.clear();
                    main_item.output_bits = item_bits (); // clear output bits
                    // erase local items from state
                    local_item = hid_local_item ();
                    break;
                case 0b1011:
                    #if HID_VERBOSE_PARSE_DESCRIPTOR
                    std::cout << "Feature" << std::endl;
                    #endif
                    main_item.feature_bits = item_bits (item_data);
                    create_reports_per_usage (item_tag&0b11);
                    local_item.usage_ids.clear();
                    main_item.feature_bits = item_bits (); // clear feature bits
                    // erase local items from state
                    local_item = hid_local_item ();
                    break;
                case 0b1010:
                    if (item_data.size()==1)
                    {
                        main_item.collection_type = (hid_collection_t) (item_data[0]);
                        #if HID_VERBOSE_PARSE_DESCRIPTOR
                        std::cout << "Collection: " << hid_collection_type[main_item.collection_type] << std::endl;
                        #endif
                        collections.push_back (hid_collection (main_item,global_item,local_item));
                        while (collections.back().parse_descriptor_item (report_ptr));
                    }
                    break;
                case 0b1100:
                    #if HID_VERBOSE_PARSE_DESCRIPTOR
                    std::cout << "End collection" << std::endl;
                    #endif
                    return false;
            }
            break;
        }
        case GLOBAL:
        {
            #if HID_VERBOSE_PARSE_DESCRIPTOR
            std::cout << "G.";
            #endif
            switch (item_tag)
            {
                case 0b0000:
                    global_item.usage_page=read_item_data(item_data);
                    #if HID_VERBOSE_PARSE_DESCRIPTOR
                    std::cout << "Usage Page: " << global_item.usage_page << std::endl;
                    #endif
                    break;
                case 0b0001:
                    global_item.logical_minimum=read_item_data_signed(item_data);
                    #if HID_VERBOSE_PARSE_DESCRIPTOR
                    std::cout << "Logical Minimum: " << global_item.logical_minimum << std::endl;
                    #endif
                    break;
                case 0b0010:
                    global_item.logical_maximum=read_item_data_signed(item_data);
                    #if HID_VERBOSE_PARSE_DESCRIPTOR
                    std::cout << "Logical Maximum: " << global_item.logical_maximum << std::endl;
                    #endif
                    break;
                case 0b0011:
                    global_item.physical_minimum=read_item_data_signed(item_data);
                    #if HID_VERBOSE_PARSE_DESCRIPTOR
                    std::cout << "Physical Minimum: " << global_item.physical_minimum << std::endl;
                    #endif
                    break;
                case 0b0100:
                    global_item.physical_maximum=read_item_data_signed(item_data);
                    #if HID_VERBOSE_PARSE_DESCRIPTOR
                    std::cout << "Physical Maximum: " << global_item.physical_maximum << std::endl;
                    #endif
                    break;
                case 0b0101:
                    global_item.unit_exponent=read_item_data_signed(item_data);
                    #if HID_VERBOSE_PARSE_DESCRIPTOR
                    std::cout << "Unit Exponent: " << global_item.unit_exponent << std::endl;
                    #endif
                    break;
                case 0b0110:
                    global_item.unit=read_item_data(item_data);
                    #if HID_VERBOSE_PARSE_DESCRIPTOR
                    std::cout << "Unit: " << global_item.unit << std::endl;
                    #endif
                    break;
                case 0b0111:
                    global_item.report_size=read_item_data(item_data);
                    #if HID_VERBOSE_PARSE_DESCRIPTOR
                    std::cout << "Report Size: " << global_item.report_size << std::endl;
                    #endif
                    break;
                case 0b1000:
                    global_item.report_id=read_item_data(item_data);
                    report_ids.push_back(global_item.report_id);
                    #if HID_VERBOSE_PARSE_DESCRIPTOR
                    std::cout << "Report ID: " << global_item.report_id << std::endl;
                    #endif
                    break;
                case 0b1001:
                    global_item.report_count=read_item_data(item_data);
                    #if HID_VERBOSE_PARSE_DESCRIPTOR
                    std::cout << "Report Count: " << global_item.report_count << std::endl;
                    #endif
                    break;
                case 0b1010:
                    #if HID_VERBOSE_PARSE_DESCRIPTOR
                    std::cout << "Push" << std::endl;
                    #endif
                    break;
                case 0b1011:
                    #if HID_VERBOSE_PARSE_DESCRIPTOR
                    std::cout << "Pop" << std::endl;
                    #endif
                    break;                        
            }
            break;
        }
        case LOCAL:
        {
            #if HID_VERBOSE_PARSE_DESCRIPTOR
            std::cout << "L.";
            #endif
            switch (item_tag)
            {
                case 0b0000:
                    usage_id=read_item_data(item_data);
                    local_item.usage_ids.push_back (usage_id);
                    #if HID_VERBOSE_PARSE_DESCRIPTOR
                    std::cout << "Usage:" << usage_id << std::endl;
                    #endif
                    break;
                case 0b0001:
                    local_item.usage_minimum=read_item_data(item_data);
                    #if HID_VERBOSE_PARSE_DESCRIPTOR
                    std::cout << "Usage Minimum:" << local_item.usage_minimum << std::endl;
                    #endif
                    break;
                case 0b0010:
                    local_item.usage_maximum=read_item_data(item_data);
                    #if HID_VERBOSE_PARSE_DESCRIPTOR
                    std::cout << "Usage Maximum:" << local_item.usage_maximum << std::endl;
                    #endif
                    break;
                case 0b0011:
                    local_item.designator_index=read_item_data(item_data);
                    #if HID_VERBOSE_PARSE_DESCRIPTOR
                    std::cout << "Designator Index:" << local_item.designator_index << std::endl;
                    #endif
                    break;
                case 0b0100:
                    local_item.designator_minimum=read_item_data(item_data);
                    #if HID_VERBOSE_PARSE_DESCRIPTOR
                    std::cout << "Designator Minimum:" << local_item.designator_minimum << std::endl;
                    #endif
                    break;
                case 0b0101:
                    local_item.designator_maximum=read_item_data(item_data);
                    #if HID_VERBOSE_PARSE_DESCRIPTOR
                    std::cout << "Designator Maximum:" << local_item.designator_maximum << std::endl;
                    #endif
                    break;
                case 0b0111:
                    local_item.string_index=read_item_data(item_data);
                    #if HID_VERBOSE_PARSE_DESCRIPTOR
                    std::cout << "String Index:" << local_item.string_index << std::endl;
                    #endif
                    break;
                case 0b1000:
                    local_item.string_minimum=read_item_data(item_data);
                    #if HID_VERBOSE_PARSE_DESCRIPTOR
                    std::cout << "String Minimum:" << local_item.string_minimum << std::endl;
                    #endif
                    break;
                case 0b1001:
                    local_item.string_maximum=read_item_data(item_data);
                    #if HID_VERBOSE_PARSE_DESCRIPTOR
                    std::cout << "String Maximum:" << local_item.string_maximum << std::endl;
                    #endif
                    break;
                case 0b1010:
                    local_item.delimiter=read_item_data(item_data);
                    #if HID_VERBOSE_PARSE_DESCRIPTOR
                    std::cout << "Delimiter:" << local_item.delimiter << std::endl;
                    #endif
                    break;
            }
            break;
        }
        case RESERVED:
        {
            break;
        }
    }
    item_data.clear();
    return true;
};    

void hid_report_parser::print_reports ()
{
    #if HID_VERBOSE_PRINT_REPORT

    std::vector <hid_report>::iterator report_it;
    std::vector <hid_collection>::iterator collection_it;
        
    for (report_it = reports.begin();report_it!=reports.end();report_it++)
    {
        // print the combined/inherited global, main and local contents of the HID report
        report_it->print();
    }

    #endif // HID_VERBOSE_PRINT_REPORT
};

uint32_t hid_report_parser::extract_bits (uint8_t* report_ptr,uint8_t from,uint8_t size)
{
    uint32_t result = 0;
    // which bytes
    uint8_t start_byte = from/8;
    uint8_t end_byte = (from+size-1)/8;
    uint8_t start_bit;
    uint8_t end_bit;

    for (int i=end_byte;i>=start_byte;i--)
    {
        // which bits within current byte
        start_bit = 0;
        if (i==start_byte)
            start_bit=from - (i*8);
        end_bit = 7;
        if (i==end_byte)
            end_bit=(from+size-1) - (i*8);
        for (int j=end_bit;j>=start_bit;j--)
        {
            uint8_t mask = 1 << j;
            uint8_t value = report_ptr[i] & mask;
            result = (result << 1);
            if (value)
                result=result | 1;
        }
    }
    return result;
}

void hid_report_parser::process_report_value (hid_report& report,uint8_t report_index,int32_t value)
{
    #if HID_VERBOSE_PROCESS_VALUE
    if (report.is_keypad_modifiers())
    {
        if (value)
        {
            switch (report_index)
            {
                case 0: 
                    std::cout << "L.CTRL ";
                    break;
                case 1: 
                    std::cout << "L.SHIFT ";
                    break;
                case 2: 
                    std::cout << "L.ALT ";
                    break;
                case 3: 
                    std::cout << "L.GUI ";
                    break;
                case 4: 
                    std::cout << "R.CTRL ";
                    break;
                case 5: 
                    std::cout << "R.SHIFT ";
                    break;
                case 6: 
                    std::cout << "R.ALT ";
                    break;
                case 7: 
                    std::cout << "R.GUI ";
                    break;
            }
        }
    }
    if (report.is_keypad_keys())
    {
        switch (value)
        {
            case 0x04: std::cout << "a "; break;
            case 0x05: std::cout << "b "; break;
            case 0x06: std::cout << "c "; break;
            case 0x07: std::cout << "d "; break;
            case 0x08: std::cout << "e "; break;
            case 0x09: std::cout << "f "; break;
            case 0x0a: std::cout << "g "; break;
            case 0x0b: std::cout << "h "; break;
            case 0x0c: std::cout << "i "; break;
            case 0x0d: std::cout << "j "; break;
            case 0x0e: std::cout << "k "; break;
            case 0x0f: std::cout << "l "; break;
            case 0x10: std::cout << "m "; break;
            case 0x11: std::cout << "n "; break;
            case 0x12: std::cout << "o "; break;
            case 0x13: std::cout << "p "; break;
            case 0x14: std::cout << "q "; break;
            case 0x15: std::cout << "r "; break;
            case 0x16: std::cout << "s "; break;
            case 0x17: std::cout << "t "; break;
            case 0x18: std::cout << "u "; break;
            case 0x19: std::cout << "v "; break;
            case 0x1a: std::cout << "w "; break;
            case 0x1b: std::cout << "x "; break;
            case 0x1c: std::cout << "y "; break;
            case 0x1d: std::cout << "z "; break;
            case 0x1e: std::cout << "1 "; break;
            case 0x1f: std::cout << "2 "; break;
            case 0x20: std::cout << "3 "; break;
            case 0x21: std::cout << "4 "; break;
            case 0x22: std::cout << "5 "; break;
            case 0x23: std::cout << "6 "; break;
            case 0x24: std::cout << "7 "; break;
            case 0x25: std::cout << "8 "; break;
            case 0x26: std::cout << "9 "; break;
            case 0x27: std::cout << "0 "; break;
            case 0x28: std::cout << "RETURN "; break;
        }
    }
    if (report.is_mouse_buttons())
    {
        if (value)
        {
            switch (report_index)
            {
                case 0: 
                    std::cout << "LEFT ";
                    break;
                case 1: 
                    std::cout << "RIGHT ";
                    break;
                case 2: 
                    std::cout << "THREE ";
                    break;
                case 3: 
                    std::cout << "FOUR ";
                    break;
            }
        }
    }
    if (report.is_mouse_x())
    {
        std::cout << "x=" << value << " ";
    }
    if (report.is_mouse_y())
    {
        std::cout << "y=" << value << " ";
    }
    if (report.is_mouse_wheel())
    {
        std::cout << "w=" << value << " ";
    }
    #endif
}

bool hid_report_parser::parse_input_reports (uint8_t*& report_ptr)
{
    uint8_t input_report_id;
    uint8_t report_bit_counter;
    int32_t value;
    std::vector <hid_report>::iterator report_it;
    std::vector <hid_collection>::iterator collection_it;
    
    #if HID_VERBOSE_PROCESS_VALUE
    std::cout << "# ";
    #endif
    bool expect_report_ids = report_ids.size()>0;

    if (expect_report_ids)
        input_report_id = *(report_ptr++);

    report_bit_counter = 0;
    for (report_it = reports.begin();report_it!=reports.end();report_it++)
    {
        // check the input reports
        if (report_it->is_input())
        {
            if (expect_report_ids && input_report_id!=report_it->get_report_id())
                continue;
            
            for (uint8_t i=0;i<report_it->get_report_count();i++)
            {
                value = extract_bits (report_ptr,report_bit_counter,report_it->get_report_size());
                // check sign
                if (report_it->is_signed())
                {
                    int32_t highbit_mask = 1<<(report_it->get_report_size()-1);
                    int32_t number_xor_mask = (1<<report_it->get_report_size()) - 1;
                    if (value & highbit_mask)
                        value = -((value % number_xor_mask)+1);
                }
                #if HID_VERBOSE_PARSE_REPORT
                std::cout << std::setw(4) << value;
                #endif
                process_report_value (*report_it,i,value);
                report_bit_counter += report_it->get_report_size();
            }
            #if HID_VERBOSE_PARSE_REPORT
            std::cout << " ";
            #endif
        }
    }
    report_ptr += report_bit_counter/8;
    #if HID_VERBOSE_PARSE_REPORT
    std::cout << std::endl;
    #endif
    #if HID_VERBOSE_PROCESS_VALUE
    std::cout << std::endl;
    #endif

    return true;
}

std::vector<VirtualKeyItem> hid_report_parser::parse_keyboard_input_report (uint8_t*& report_ptr)
{
    uint8_t input_report_id;
    uint8_t report_bit_counter;
    int32_t value;
    std::vector <hid_report>::iterator report_it;
    std::vector <hid_collection>::iterator collection_it;
    std::vector<VirtualKeyItem> items;
    VirtualKeyItem item;
    
    #if HID_VERBOSE_PROCESS_VALUE
    std::cout << "# ";
    #endif
    bool expect_report_ids = report_ids.size()>0;

    if (expect_report_ids)
        input_report_id = *(report_ptr++);

    report_bit_counter = 0;
    for (report_it = reports.begin();report_it!=reports.end();report_it++)
    {
        // check the input reports
        if (report_it->is_input())
        {
            if (expect_report_ids && input_report_id!=report_it->get_report_id())
                continue;
            
            for (uint8_t i=0;i<report_it->get_report_count();i++)
            {
                value = extract_bits (report_ptr,report_bit_counter,report_it->get_report_size());
                // check sign
                if (report_it->is_signed())
                {
                    int32_t highbit_mask = 1<<(report_it->get_report_size()-1);
                    int32_t number_xor_mask = (1<<report_it->get_report_size()) - 1;
                    if (value & highbit_mask)
                        value = -((value % number_xor_mask)+1);
                }
                #if HID_VERBOSE_PARSE_REPORT
                std::cout << std::setw(4) << value;
                #endif
                process_report_value (*report_it,i,value);
                if (report_it->is_keypad_modifiers())
                {
                    switch (i)
                    {
                        // left modifiers
                        case 0: item.CTRL=value;break;
                        case 1: item.SHIFT=value;break;
                        case 2: item.LALT=value;break;
                        case 3: item.GUI=value;break;
                        // right modifiers
                        case 4: item.CTRL|=value;break;
                        case 5: item.SHIFT|=value;break;
                        case 6: item.RALT=value;break;
                        case 7: item.GUI|=value;break;
                    }
                }
                if (report_it->is_keypad_keys())
                {
                    item.down = value!=0;
                    if (item.down && value<sizeof(ascii_values))
                    {
                        if (item.SHIFT)
                        {
                            item.vk = ascii_values[value].key_shifted;
                            item.ASCII = ascii_values[value].shifted;
                        }
                        else if (item.CTRL)
                        {
                            item.vk = ascii_values[value].key_lower;
                            item.ASCII = ascii_values[value].lower-'a'+1; // CTRL-A = 1, CTRL-Z = 26
                        }
                        else
                        {
                            item.vk = ascii_values[value].key_lower;
                            item.ASCII = ascii_values[value].lower;
                        }
                        if (item.vk!=VK_NONE)
                            items.push_back (item);
                    }
                }
                report_bit_counter += report_it->get_report_size();
            }
            #if HID_VERBOSE_PARSE_REPORT
            std::cout << " ";
            #endif
        }
    }
    report_ptr += report_bit_counter/8;
    #if HID_VERBOSE_PARSE_REPORT
    std::cout << std::endl;
    #endif
    #if HID_VERBOSE_PROCESS_VALUE
    std::cout << std::endl;
    #endif

    // check if we had prev_key_items that are not in items
    std::vector<VirtualKeyItem>::iterator prev_it,it;
    for (prev_it=prev_key_items.begin();prev_it!=prev_key_items.end();prev_it++)
    {
        for (it=items.begin();it!=items.end();it++)
        {
            // same key, still pressed?
            if (it->vk==prev_it->vk)
            {
                //items.erase(it);
                it->down=false;
                break;
            }
        }
        // if (it==items.end())
        // {
        //     // we had previously a key that is now not pressed
        //     // add it with the down = false
        //     prev_it->down = false;
        //     items.push_back(*prev_it);
        // }
    }
    prev_key_items=items;
    return items;
}

} // namespace fabgl;