#ifndef _MANCHESTER_CONFIG
#define _MANCHESTER_CONFIG

#define FAILURE 0
#define SUCCESS 1


//configures the manchester structure manch_conf
void manchester_set_parameters(unsigned char b_size,
                               unsigned char s_size,
                               unsigned short int th);
//processes ADC samples. Only input scope function
unsigned char process_sample(unsigned short int sample);
//processes the bit, used only inside the scope
static void process_bit(unsigned char bit_value);
//gets the delivered bit
char get_data_byte();


#endif
