#ifndef __RINGBUFFER_H__
#define __RINGBUFFER_H__


  void    Uart2BufferInit     (void);
  int16_t Uart2Getchar        (void);
  void    Uart2Putchar        (char c);
  void    Uart2_printf        (char * format, ...);
  void    Uart2_rx_printf     (char * format, ...);


#if 1

  #define Debug_printf(...)  { Uart4_printf(__VA_ARGS__);}

#else
   #define Debug_printf(...) {}
   #define Debug6_printf(...) {}

#endif


#endif // __RINGBUFFER_H__
