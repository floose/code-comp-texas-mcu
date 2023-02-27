#ifndef _MANCHESTER_CONFIG
#define _MANCHESTER_CONFIG

struct Manchester_config
{
    unsigned char buffer_size; //configures buffer size
    unsigned char symbol_syze; //configures adc manchester symbol size
    unsigned short int threshold; //configures threshold value
    unsigned int conf_flag;      //checks if structure was already configured.
};


//configures the manchester structure manch_conf
void manchester_set_parameters(unsigned char b_size,
                               unsigned char s_size,
                               unsigned short int th);
//processes ADC samples. Only input scope function
unsigned char process_sample(unsigned short int sample);
//processes the bit, used only inside the scope
static unsigned char process_bit(unsigned char bit_value);
//gets the delivered bit
unsigned char get_data_byte();


#endif
