



#include "MyPrintf.h"


#define PRINT_SZ 128



/*
 * String manipulation functions
 */
uint32_t MyStrlen(const char *s)
{
    unsigned int length = 0;

    while (s[length] != '\0')
        length++;

    return length;
}

static uint32_t Myitoa(int32_t value, uint32_t radix, uint32_t uppercase,
                          char *buf, int32_t pad)
{
    char *pbuf = buf;
    bool negative = false;
    uint32_t i, length;

    // Check for unusual radixes
    if (radix > 16)
        return 0;

    if (value < 0 && radix == 10) {
        negative = true;
        value = -value;
    }

    // Build the string here starting from the end...
    do {
        int digit = value % radix;
        *(pbuf++) = (digit < 10 ? '0' + digit : (uppercase ? 'A' : 'a') + digit - 10);
        value /= radix;
    } while (value > 0);

    length = (pbuf - buf);
    for (i = length; i < pad; i++)
        *(pbuf++) = '0';

    // Add '-' sign if need be...
    if (negative)
        *(pbuf++) = '-';

    // Mark EOF
    *(pbuf) = '\0';

    // Reverse the string now...
    length = (pbuf - buf);
    for (i = 0; i < length / 2; i++)
    {
        char ch = buf[i];

        buf[i] = buf[length - i - 1];
        buf[length - i - 1] = ch;
    }

    return length;
}

static inline int MyPutc(char **wr_ptr, char *buf, uint32_t length, char ch)
{
    if ((uint32_t)((*wr_ptr - buf) + 1) >= length)
        return 0;

    **wr_ptr = ch;
    (*wr_ptr)++;
    **wr_ptr = '\0';

    return 1;
}


static inline int MyPuts(char **wr_ptr, char *buf, uint32_t length, char *s, uint32_t str_len)
{
    uint32_t i;

    if (length - (*wr_ptr - buf) - 1 < str_len)
        str_len = length - (*wr_ptr - buf) - 1;

    /* Copy to buffer */
    for (i = 0; i < str_len; i++)
    {
        **wr_ptr = s[i];
        (*wr_ptr)++;
    }
    **wr_ptr = '\0';

    return length;
}

static int32_t MyVsnprintf(char *buffer, uint32_t buffer_len, const char *fmt, va_list va)
{
    char *pbuffer = buffer;
    char bf[24];
    char ch;
    char qual = '\0';

    ch = *(fmt++);
    do {
        if ((unsigned int)((pbuffer - buffer) + 1) >= buffer_len)
            break;

        if (ch != '%')
        {
            MyPutc(&pbuffer, buffer, buffer_len, ch);
        }
        else
        {
            char zero_pad = 0;
            char *ptr;
            unsigned int len;

            ch =* (fmt++);

            /* Zero padding requested */
            if (ch == '0')
            {
                ch =* (fmt++);
                if (ch == '\0')
                    goto end;
                if (ch >= '0' && ch <= '9')
                    zero_pad = ch - '0';
                ch =* (fmt++);
            }

            if(ch == 'l' || ch == 'h')
            {
               qual = ch;
                ch =* (fmt++);
            }

            switch (ch) {
            case 0:
                goto end;

            case 'i':
            case 'd':
            {
                long val;

                if (qual == 'h')
                {
                    val = va_arg(va, int/*short*/);
                }
                else if (qual == 'l')
                {
                    val = va_arg(va, long);
                }
                else
                {
                    val = va_arg(va, int);
                }

                len = Myitoa(val, 10, 0, bf, zero_pad);
                MyPuts(&pbuffer, buffer, buffer_len, bf, len);
                break;
            }
            case 'u':
            case 'x':
            case 'X':
            {
                unsigned long val;

                if(qual == 'h')
                {
                    val = va_arg(va, unsigned int/*unsigned short*/);
                }
                else if(qual == 'l')
                {
                    val = va_arg(va, unsigned long);
                }
                else
                {
                    val = va_arg(va, unsigned int);
                }

                if (ch == 'u')
                    len = Myitoa(val, 10, 0, bf, zero_pad);
                else
                    len = Myitoa(val, 16, (ch=='X'), bf, zero_pad);

                MyPuts(&pbuffer, buffer, buffer_len, bf, len);
                break;
            }

            case 'c' :
                ch = va_arg(va, int32_t);
                MyPutc(&pbuffer, buffer, buffer_len, ch);
                break;

            case 's' :
                ptr = va_arg(va, char*);
                MyPuts(&pbuffer, buffer, buffer_len, ptr, MyStrlen(ptr));
                break;

            default:
                MyPuts(&pbuffer, buffer, buffer_len, "FATAL: unsupported printf character: ", 37);
                MyPutc(&pbuffer, buffer, buffer_len, ch);
                MyPuts(&pbuffer, buffer, buffer_len, ".\n", 2);
                break;
            }
        }
        ch = *(fmt++);
    } while(ch);
end:
    return pbuffer - buffer;
}

static int32_t MySnprintf(char* buffer, uint32_t buffer_len, const char *fmt, va_list va)
{
    int ret;

    ret = MyVsnprintf(buffer, buffer_len, fmt, va);

    return ret;
}

int MyVprintf(const char *fmt, va_list args)
{
    char my_buf[PRINT_SZ];

    int32_t written = MySnprintf(my_buf, sizeof(my_buf), fmt, args);
	
    if ((written > 0))
    {
//        my_buf[written] = '\0';
        uart_send(UART1, (uint8_t *)my_buf, MyStrlen(my_buf), UART_OP_BLOCKING);
    }

    return 1;
}

int MyPrintf(const char *fmt, ...)
{
	int res;
	if(0 == arch_get_sleep_mode())
	{
		va_list args;

	    va_start(args, fmt);
	    res = MyVprintf(fmt, args);
	    va_end(args);
	}
    

    return res;
}

#if defined (LOG)
int MyLog(const char *fmt, ...)
{
	int res;
	if(0 == arch_get_sleep_mode())
	{
		va_list args;

	    va_start(args, fmt);
	    res = MyVprintf(fmt, args);
	    va_end(args);
	}
	
    return res;
}

#endif


