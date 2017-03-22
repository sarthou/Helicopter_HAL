/*
 * StringUtility.c
 *
 *  Created on: 30 oct. 2016
 *      Author: Julien
 */

#include "StringUtility.h"
#include <stdbool.h>
#include <math.h>

void reverse(char* str, int length)
{
    int start = 0;
    int end = length -1;
    char temp;
    while (start < end)
    {
    	temp = *(str+start);
    	*(str+start) = *(str+end);
    	*(str+end) = temp;
        start++;
        end--;
    }
}

void itoa(int num, char* str, int base)
{
    int i = 0;
    bool isNegative = false;

    /* Handle 0 explicitely, otherwise empty string is printed for 0 */
    if (num == 0)
    {
        str[i++] = '0';
        str[i] = '\0';
        return;
    }

    // In standard itoa(), negative numbers are handled only with
    // base 10. Otherwise numbers are considered unsigned.
    if (base == 10 && num < 0)
    {
        isNegative = true;
        num = -num;
    }

    // Process individual digits
    while (num != 0)
    {
        int rem = num % base;
        str[i++] = (rem > 9)? (rem-10) + 'a' : rem + '0';
        num = num/base;
    }

    // If number is negative, append '-'
    if (isNegative)
        str[i++] = '-';

    str[i] = '\0'; // Append string terminator

    // Reverse the string
    reverse(str, i);
}

// Converts a given integer x to string str[].  d is the number
// of digits required in output. If d is more than the number
// of digits in x, then 0s are added at the beginning.
static int ftoaUtility_itoa(int x, char str[], int d)
{
   int i = 0;
   while (x)
   {
       str[i++] = (x%10) + '0';
       x = x/10;
   }

   // If number of digits required is more, then
   // add 0s at the beginning
   while (i < d)
       str[i++] = '0';

   reverse(str, i);
   str[i] = '\0';
   return i;
}

void ftoa(float num, char *str, int afterpoint)
{
    // Extract integer part
    int ipart = (int)num;

    // Extract floating part
    float fpart = num - (float)ipart;

    // convert integer part to string
    int i = ftoaUtility_itoa(ipart, str, 0);

    // check for display option after point
    if (afterpoint != 0)
    {
        str[i] = '.';  // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        ftoaUtility_itoa((int)fpart, str + i + 1, afterpoint);
    }
}


