
<center>**<big>Quadcopter</big>** </center>

*本文使用Sublime Text 3编辑，**OmniMarkupPreviewer / markdown preview** 插件编译 (**ctrl+alt+o / alt+m**)  生成预览。*


<small>*
    如出现错误 "buffer_id(29) is not valid (closed or unsupported file format)"  
    Preferences → Package Settings → OmniMarkupPreviewer → Settings - User　复制和粘贴下面的代码。
    {
    "renderer_options-MarkdownRenderer": {
        "extensions": ["tables", "fenced_code", "codehilite"]
    }
    }
*</small>

目录

[TOC]

## 说明文档
### STcube生成文档

>相对路径地址：Quadcopter/MDK/RTE/Device/STM32F407VGTx/STCubeGenerated/STCubeGenerated.pdf

 [**Github**](https://github.com/yangdonghao/Quadcopter/blob/master/MDK/RTE/Device/STM32F407VGTx/STCubeGenerated/STCubeGenerated.pdf)

- 引脚配置
- 时钟树
>嘀嗒时钟始终为1000Hz

- 模块配置
- ...

##版本跟进
###V1
####LED灯状态指示

>颜色：

绿|橙|红|蓝
--|--|--|--
0 |1 | 2|3

![**原理图**](Readme/pic/1.png)
![**程序**](Readme/pic/2.png)

#####初始化:
- 红灯亮：MX初始化完成
- 蓝灯亮：MPU初始化完成
- ...

###无线串口传输(441M)
>发送接收空中延时约为0.2s

- 8字节传送周期

![8字节传送周期](Readme/pic/441M无线串口8字节传送周期.png)


- 32字节传送周期

![32字节传送周期](Readme/pic/441M无线串口32字节传送周期.png)


- 传输8字节时间

![传输8字节时间](Readme/pic/441M无线串口传输8字节时间.png)


- 传输32字节时间

![传输32字节时间](Readme/pic/441M无线串口传输32字节时间.png)

###串口DMA优化

直接在初始化赋值内存地址，程序运行中自动更新内存地址数据，无需用户操作

![串口DMA优化](Readme/pic/串口DMA优化.png)
