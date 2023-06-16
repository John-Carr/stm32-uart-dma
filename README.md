# stm32-uart-dma
**IMPORTANT**
The code cube mx generates is wrong the DMA init must come before the usart 2
init.
```c
MX_DMA_Init();
MX_USART2_UART_Init();
```
Newer versions of cube mx might have fixed this.
