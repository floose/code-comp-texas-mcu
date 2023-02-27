/*
 * manchester.c
 *
 *  Created on: 27 de fev de 2023
 *      Author: felip
 */


#include "manchester.h"
#include <stdint.h>

static struct Manchester_config manch_conf; //this holds the configurations
static unsigned char delivered_byte = 0x0F; //this holds the byte processed

void manchester_set_parameters(unsigned char b_size,
                               unsigned char s_size,
                               unsigned short int th)
{
    //configures the parameters for the manchester algorithm
    manch_conf.buffer_size = b_size;
    manch_conf.symbol_syze = s_size;
    manch_conf.threshold = th;
    manch_conf.conf_flag = 1; //raises the flag of parameters configured
}

unsigned char process_sample(unsigned short int sample)
{

  if(manch_conf.conf_flag == 0)
  {
      return 0; //returns 0 if manchester is not configured
  }

  static unsigned short int last_sample = 0;
  static unsigned char loopcount = 0;
  static unsigned char bit_value = 0;
  loopcount++; //loop counter increment

  // Check for a transition from high to low or low to high
  if ((sample > manch_conf.threshold && last_sample < manch_conf.threshold))
  {
    //printf("Here I found one transition from low to high.\n");
    bit_value = 0;
  }
  else if((sample < manch_conf.threshold && last_sample > manch_conf.threshold))
  {
    //printf("Here I found one transition from high to low.\n");
    bit_value = 1;
  }

  if(loopcount % manch_conf.symbol_syze == 0)//checks if reached end of symbol
    {
      //printf(".............................Symbol ended. Processed bit is %d\n", bit_value);
      process_bit(bit_value);
    }
  last_sample = sample; //holds past samples

  return 1;
}

unsigned char process_bit(unsigned char bit_value)
{
  static unsigned char bits_received = 0;
  static unsigned char decoded_byte = 0;
  // checks if a full byte is received, then prints it on the console
  if (bit_value == 1)
  {
    decoded_byte = decoded_byte << 1; // left shift
    decoded_byte = decoded_byte | 0x01;
  }
  else if(bit_value == 0)
  {
    decoded_byte = (decoded_byte << 1); // Append a 0 to the byte
  }

  bits_received++;

  if (bits_received == 8) //if 8 bits were received, delivers the byte
  {
    //printf("Received byte: %c\n", decoded_byte);
    delivered_byte = decoded_byte;
    bits_received = 0;
    decoded_byte = 0;
  }

}

unsigned char get_data_byte()
{
    return delivered_byte;
}

