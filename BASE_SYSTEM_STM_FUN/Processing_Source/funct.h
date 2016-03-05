
#ifndef FUNCT_H_
#define FUNCT_H_

#include "stm32f10x.h"

typedef union {
	u16 u_short;
	u8 a_char[2];
} U_short_to_char;




#define SET_BIT_(X,Y) X|=(1<<Y);
#define CLEAR_BIT_(X,Y) X&=~(1<<Y);

//void Clrinbuf(struct Task_parameters *ptaskparameters,int MaxTime);
//void Clrinbuf_without_time(struct Task_parameters *ptaskparameters);
uint16_t CRC16(u8 *puchMsg, u16 usDatalen);
void modus_add_CRC(u8 *p_sent, u16 length_sent);
void ShortTooChar(unsigned short *a,unsigned char *buffer); 		// first Lo Second Hi
void IntTooChar(unsigned int *a,unsigned char *buffer);
void UintTooChar(unsigned int *a,unsigned char *buffer);
unsigned char IntTooOneUChar(int *a);
void FloatTooChar(float *a,unsigned char *buffer);
int CharTooInt(unsigned char *buffer);
unsigned int CharToouint(unsigned char *buffer);
float	CharToofloat(unsigned char *buffer);
unsigned short CharTooushort(unsigned char *buffer);
unsigned short CharTooshort(unsigned char CharHi,unsigned char CharLo); // first Hi second Lo
float TwoCharTooFloat(unsigned char Lo,unsigned char Hi,unsigned char type_ti);
int perevertish(unsigned char inbyte, unsigned char *poutbyte);
u8 get_array_bit(u8* data_in, u16 num_bit);

void Convert_ShortTooCharHiLo_(U_short_to_char my_short_to_char,u8 *buffer);
void Convert_ShortTooCharLoHi_(U_short_to_char *my_short_to_char,u8 *buffer);
void Convert_CharTooShortLoHi_(U_short_to_char *my_short_to_char,u16 *buffer);
void Convert_CharTooShortHiLo_(U_short_to_char *my_short_to_char,u16 *buffer);

#define Convert_ShortTooCharHiLo(X,Y) Convert_ShortTooCharHiLo_((U_short_to_char)X,Y)
#define Convert_ShortTooCharLoHi(X,Y) Convert_ShortTooCharLoHi_((U_short_to_char*)X,Y)
#define Convert_CharTooShortHiLo(X,Y) Convert_CharTooShortHiLo_((U_short_to_char*)X,Y)
#define Convert_CharTooShortLoHi(X,Y) Convert_CharTooShortLoHi_((U_short_to_char*)X,Y)


#endif
