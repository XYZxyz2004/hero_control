# 关于HAL_UARTEx_ReceiveToIdle_DMA()
## 关于DMA传输过半中断
![](https://i-blog.csdnimg.cn/blog_migrate/2b9cd048d66e39b0b5791eb0aa80e2b9.png)
我们看到，再使用这个函数之时，串口DMA传输完成一半时的回调函数赋了值
该函数最后还是进入了HAL_DMA_Start_IT()函数
![](https://i-blog.csdnimg.cn/blog_migrate/fc477c2588456ba976b84b0bb235e6c4.png)
我们看到，因为给串口DMA传输完成一半时的回调函数赋了值，所以HAL_DMA_Start_IT()中，DMA_IT_HT这个dma传输过半断被开启了
再关注这个中断
![](https://i-blog.csdnimg.cn/blog_migrate/5ad3a9553505b98ce009f48846954a0c.png)
串口DMA传输过半时，会产生中断
也就是说调用HAL_UARTEx_ReceiveToIdle_DMA就会默认开启DMA_IT_HT中断,即传输过半中断
导致的后果就是：理想效果是接收完整的一轮数据后执行一次HAL_UARTEx_RxEventCallback回调函数，而因为默认开启了DMA_IT_HT中断后，一轮数据接收意外的执行了两次HAL_UARTEx_RxEventCallback回调函数。
调用__HAL_DMA_DISABLE_IT();把这个中断手动关闭。

## 关于串口空闲中断
在普通无dma情况下，串口空闲中断存在但无回调函数，但在dma模式下，可以调用HAL_UARTEx_RxEventCallback()回调函数
这是因为，在HAL_UARTEx_ReceiveToIdle_DMA()中

```
status =  UART_Start_Receive_DMA(huart, pData, Size)，启用DMA将接收到的数据放在指针 pData 指向的位置。
```

在接受正常情况下，执行以下内容

```
__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_IDLEF);		// 清除UART_CLEAR_IDLEF标志位`
ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);	// 设置usart cr1寄存器的USART_CR1_IDLEIE标志位`
```

而HAL_UARTEx_ReceiveToIdle_DMA中剩下的部分会将当前的串口接收类型设置为HAL_UART_RECEPTION_TOIDLE，开启了IDLE中断
于是我们可以得知，调用HAL_UARTEx_ReceiveToIdle_DMA()函数后只要发生了串口空闲事件，就会产生串口空闲中断。
再查看DMA串口空闲中断相关的部分,发生串口空闲中断以后，进入到USART1_IRQHandler()，执行 HAL_UART_IRQHandler();

如下

```
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
        /*Call registered Rx Event callback*/
        huart->RxEventCallback(huart, (huart->RxXferSize - huart->RxXferCount));
#else
        /*Call legacy weak Rx Event callback*/
        HAL_UARTEx_RxEventCallback(huart, (huart->RxXferSize - huart->RxXferCount));
#endif /* (USE_HAL_UART_REGISTER_CALLBACKS) */
```

因而会开启DMA功能并调用回调函数HAL_UARTEx_RxEventCallback()。如果接收过程中发生错误，会调用HAL_UART_ErrorCallback()