# FreeRTOS_lvgl8
## 硬件
* STM32F429IGTx
* 野火800*480 7寸LCD屏
* W9812G6KH-6 TSOP-54 128Mbit SDRAM
## 软件
* FreeRTOS CMSIS_V2
* LVGL8.0.3-dev 2021.7.19 master

## 配置
### STM32	
* LCD-TFT clocks:  20MHZ
* 主频: 180MHZ
* DMA2D:  M2M RGB565
* LTDC : 1LAYER
### LVGL
* 16 depth
* DMA2D加速
* 触摸驱动移植野火bsp_touch

## 说明
**本项目是基于STM32F429IGTx与lvgl8.0.3的项目模板，硬件参考野火挑战者V2，freeRTOS仅一个lvgl任务，lvgl8.0.3移植模板，运行lvgl官方demo（lv_demo_widgets）**
