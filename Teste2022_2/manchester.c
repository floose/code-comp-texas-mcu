/*
 * manchester.c
 *
 *  Created on: 27 de fev de 2023
 *      Author: felip
 */


#include "manchester.h"


static unsigned char buffer_size = 0; //configures buffer size
static unsigned char symbol_syze = 0; //configures adc manchester symbol size
static unsigned short int threshold = 0; //configures threshold value
static unsigned int conf_flag = 0;      //checks if structure was already configured.
static char delivered_byte = 0x00; //this holds the byte processed

void manchester_set_parameters(unsigned char b_size,
                               unsigned char s_size,
                               unsigned short int th)
{
    //configures the parameters for the manchester algorithm
    buffer_size = b_size;
    symbol_syze = s_size;
    threshold = th;
    conf_flag = SUCCESS; //raises the flag of parameters configured
}

//
//processes the sample of the adc. When it reaches a byte, returns 1.
// While is still processing, returns a 0
//
unsigned char process_sample(unsigned short int sample)
{

  unsigned char flag = FAILURE;
  //check if manchester is configured
  if(conf_flag == 0)
  {
      flag = FAILURE;
      return flag; //returns 0 if manchester is not configured
  }

  //if it passes the confif check up, starts algorithm and static variables
  static unsigned short int last_sample = 0;
  static unsigned char loopcount = 0;
  static unsigned char bit_value = 0;
  loopcount++; //loop counter increment

  // Check for a transition from high to low or low to high
  if ((sample > threshold && last_sample < threshold))
  {
    //printf("Here I found one transition from low to high.\n");
    bit_value = 0;
  }
  else if((sample < threshold && last_sample > threshold))
  {
    //printf("Here I found one transition from high to low.\n");
    bit_value = 1;
  }

  if(loopcount % symbol_syze == 0)//checks if reached end of symbol
    {
      //printf(".............................Symbol ended. Processed bit is %d\n", bit_value);
      process_bit(bit_value);
      flag = SUCCESS; //if reaches symbol size, returns true
    }
  last_sample = sample; //holds past samples

  return flag;
}

static void process_bit(unsigned char bit_value)
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

char get_data_byte()
{
    char byte = delivered_byte;
    return byte;
}

