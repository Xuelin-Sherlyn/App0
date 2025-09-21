# Application0

## 需要注意的点

在创建同类应用的时候，需要将程序入口点设为QSPI Flash的地址(0x90000000),或者你的目标地址，并且将MPU关闭或者设置为 MPU_REGION_FULL_ACCESS。

另外，QSPI Flash不支持缓冲区和分享，但可以稍微缓存数据，因此MPU设为这个

    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
如果跑不起来可以调试一下AppLoader，看看执行到了哪个地址，然后看看./build/$Project Type/$Project Name.map，70%概率卡在\"HardFault_Handler\"或者\"MemManager_Handler\"这两个函数