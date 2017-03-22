/*
 * StringUtility.h
 *
 *  Created on: 30 oct. 2016
 *      Author: Julien
 */

#ifndef SOURCES_STRINGUTILITY_H_
#define SOURCES_STRINGUTILITY_H_


#ifdef __cplusplus
extern "C" {
#endif

void reverse(char* str, int length);

void itoa(int num, char* str, int base);

void ftoa(float num, char *str, int afterpoint);

#ifdef __cplusplus
}
#endif

#endif /* SOURCES_STRINGUTILITY_H_ */
