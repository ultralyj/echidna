
#### 介绍
echidna 是本项目代号 目前版本号为2.4d2

#### 软件架构
软件架构说明
本项目以`rtthread`为系统，fatfs为辅助，适配英飞凌`aurix tc264d`芯片。
主要代码位于文件夹`Libraries/tjrc_library`
代码按照（片上外设）-> (片外模块) -> (软件算法) -> (线程) 的层次构建

* 目前`tjrc_hardware`和`tjrc_peripherals`两层以基本完成构建，目前已通过功能性验证。
* tjrc_algorithm目前构建了`kalman filter`和 `pid control`，后期还将加入以图像处理为主的若干算法。
* tjtc_thread目前尚未构建，由于线程之间的适配关系还没有完全确定，因此考虑后期功能成熟了再进行构建。
