
#### 介绍
echidna 是本项目代号 目前版本号为2.6a

#### 软件架构

##### 软件架构说明
本项目以`rtthread`为系统，fatfs为辅助，适配英飞凌`aurix tc264d`芯片。主要代码位于文件夹`Libraries/tjrc_library`代码按照（片上外设）-> (片外模块) -> (软件算法) -> (线程) 的层次构建

* 目前`tjrc_hardware`和`tjrc_peripherals`两层以基本完成构建，目前已通过功能性验证。
* tjrc_algorithm目前构建了`kalman filter`和 `pid control`，后期还将加入以图像处理为主的若干算法。
* tjtc_thread目前构建了各个线程，如下
  * `thread_life`生命线程，用于检测rtthread是否正常运行
  * `thread_key`用于扫描按键，发送信号量给其他线程
  * `thread_balance`用于控制车身平衡
  * `thread_camera`用于采集摄像头信息并且处理图像

#### 版本信息

* 动量轮后置
* 供电电压提高到12V
* 摄像头图像处理与运动学控制耦合
* 完善系统架构
