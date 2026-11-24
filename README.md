# RoundFoc 固件工程

## 打开与构建

用 IAR Embedded Workbench for Arm 打开 `iar/RoundFoc.eww`。在 Project → Batch Build 中选择 `Debug` 构建两个核。也可在本目录执行以下 PowerShell 命令。

```powershell
& 'D:/IAR/common/bin/IarBuild.exe' 'iar/boot_cm0plus.ewp' -build Debug
& 'D:/IAR/common/bin/IarBuild.exe' 'iar/round_foc_cm4.ewp' -build Debug
```

两个核分别生成 `build/Debug/cm0plus/Exe/boot_cm0plus.hex` 和 `build/Debug/cm4/Exe/round_foc_cm4.hex`。同目录的 `.out` 带调试符号。复位启动需要两个镜像分别位于各自的链接地址。

### 从 IAR 下载双核固件

1. 连接逐飞 CMSIS-DAP 下载器并给目标板供电，在 IAR 打开 `iar/RoundFoc.eww`。
2. 首次构建或修改源码后，按 **F8**，选择 **Debug** 并执行 **Make**，生成两个核的最新镜像。
3. 将 **boot_cm0plus – Debug** 设为活动工程，选择 **Project → Download → Download active application**。IAR 通过 SWD 写入 CM0+ 启动程序和 CM4 电机程序，并执行下载校验。调试时可在 **View → Messages → Debug Log** 查看目标识别信息和下载字节数。

双核下载选项保存在 `iar/boot_cm0plus.ewd`。附加的 CM4 镜像采用工程相对路径，地址偏移为 0。下载范围为两个程序的 Code Flash，校准记录所在的 Work Flash 不参与常规下载。

## 上板运行六步换相

适配圆形自制板、板载 AS5047P 和龙邱 L4225-6S420-A（12 槽、14 极、7 极对）。台式电源设为 16.8 V，母线限流 3.2 A。本版固件无相电流采样或软件过流保护。复位、校准结束和故障停机时，EG2134 的六路输入均为低电平。

1. 用 DAP 下载本次构建的 CM0+ 和 CM4 两个镜像。CM0+ 配置系统时钟并响应 Flash 系统调用，CM4 执行电机控制。板上 Y1 是 16 MHz 主晶振，32 kHz 晶振位未接入。
2. 打开下载接口引出的串口。VOFA+ 选择 **FireWater、115200、8N1**，命令用 ASCII 文本发送，末尾加回车或换行。上电先手转一圈，观察机械角在 0～359.9° 间连续变化。首次无有效参数时状态为 0。
3. 发 `cal`。保持转轴能自由跟随。固件从低占空比开始对齐，再沿六个电角度扇区各走一个电周并折返。约 2.2 s 后，方向、极对数对应的位移和回零一致性均通过才会保存。状态 1 表示已保存且处于待机。
4. 例如发 `run 3`，给定 **3% 占空比**。支持一位小数，如 `run 4.5`，也可带百分号。运行中可用同一命令调整给定。电机持续转动，发 `stop` 关断六路输入，`run 0` 同样停机。转速供观察。
5. 下次上电将加载最近的有效校准值，等待 `run`。更换电机、调换相线或调整磁铁安装位置后，先 `stop` 再 `cal`。校准过程中发 `stop` 可中止运动并保留之前的记录。排除故障后发 `stop` 回到待机或待校准。

### VOFA+ 四个通道

每 20 ms 输出一行 `机械角,机械转速,通电矢量编号,状态码`。机械角单位为度，随 AS5047P 原始读数增大。转速单位为 rpm，按 20 ms 位移估算。首次校准前转速符号随编码器，校准后按换相表正向运动为正。矢量编号 1～6，关桥时为 0。

| 状态码 | 含义 |
| --- | --- |
| 0 / 1 | 待首次校准 / 已校准待机 |
| 2 / 3 | 正在校准 / 正在运行 |
| -1 | 编码器 SPI 超时、帧校验、磁场诊断或运动中角度连续性异常 |
| -2 | 校准时转子未稳定、步进位移或往返电角度零位超差 |
| -3 | 运行时连续 500 ms 未累积到约 2.8° 正向机械位移 |
| -4 | Flash 系统调用、读取或保存校验失败 |
| -5 | 通电期间位置采样间隔超过两个控制周期 |
| -6 | 命令格式、给定范围或当前状态不允许，提示一帧 |
| -7 | 板级外设或控制时基初始化失败 |

### 板级连接与首次实验参数

| 功能 | MCU 连接或初始设置 | 上板观察 |
| --- | --- | --- |
| 三相桥 U / V / W | 高侧、低侧依次为 P14.0/P14.1、P18.4/P18.5、P18.6/P18.7 | 一相高侧 PWM，另一相低侧导通，第三相上下管均关闭 |
| 编码器 SCB3 | P13.0 MISO、P13.1 MOSI、P13.2 CLK、P13.3 CSn。SPI 模式 1，2 MHz | 每次采样都读磁场诊断和 14 位补偿角，检查偶校验和 EF |
| 调试串口 SCB5 | 接口 1 脚 P7.0 为 MCU RX，2 脚 P7.1 为 MCU TX | 串口接收命令并返回四列数值 |
| PWM 与换相间隔 | 20 kHz。换通电相或占空比时先全关至少 2 µs，首次通电为源相自举预充 30 µs | 示波器核对门极波形、上下管关断间隔和自举电压 |
| 位置采样与换相 | 4 kHz 定时读角。以 U+ V− 对齐点为电角度零，选择最接近转子前方 90° 的通电矢量 | 正转一机械圈依次换相 42 次 |
| 校准 | 对齐 400 ms，前 100 ms 逐步升至 3%。每个校准步保持 150 ms，取末 20 ms 的平均角度 | 单步约 8.57° 机械角，六步约 51.43°。返回初始位置，成功后关桥 |
| 运行占空比 | 首轮试车上限 15%，升幅 0.1 个百分点/2 ms，下调在下一次控制更新生效 | 观察是否平稳起转、转速符号和停机响应 |

PWM、换相间隔、自举时间、校准占空比和试车上限是待实物核对的实验初值。板级时序在 `app/board.c`，电机参数及校准流程在 `app/motor.h` 和 `app/motor.c`。尚无本版上板测量记录。编码器和驱动接口定义见 [AS5047P 数据手册](https://datasheet.lcsc.com/datasheet/pdf/f756e23b78f013a17a23aa4fb991cabd.pdf?productCode=C55112284) 和 [EG2134 产品资料](https://egmicro.com/products/detail/?name=EG2134)。

### 校准记录

使用单 Bank 模式下的片内 Work Flash。CM4 以 IAR `__no_init` 对象保留 `0x14018000` 和 `0x14018080` 两个独立的 128 字节小扇区。普通启动仅读取。记录含格式和电机配置标识、序号、编码器方向、电角度零位、CRC32 和提交标记。每次成功重校先关桥，将备用扇区擦除并写入，读回校验后最后写提交标记。复位时校验两份记录并取最近有效的一份。下载选项中的整片擦除若包含 Work Flash，将需要重新校准。

读取先通过 SROM 检查空白，再通过 M-DMA 通道 0 读取已编程的记录。掉电可能留下不可纠正的 ECC 错误，DMA 能把该记录作为无效数据处理并继续检查另一扇区。[英飞凌 Flash 访问说明](https://documentation.infineon.com/traveo/docs/grn1680597195267) · [CYT2BL Rev. A 勘误 137](https://www.infineon.com/assets/row/public/documents/10/52/infineon-cyt2bl-a-errata-rev2.3.pdf-erratasheet-en.pdf)

### 主机回归

在本目录用 GCC 运行。测试使用实际的换相、命令解析和参数保存代码，以模拟转子和内存扇区提供输入。覆盖两个编码器方向、机械角和计时回绕、每圈 42 次换相、停机与堵转、错误极对数、掉电及重校失败恢复。

```powershell
gcc -std=c11 -Wall -Wextra -Werror -O2 -Iapp app/motor.c app/cal_store.c app/console.c tests/six_step_test.c -o build/six_step_test.exe
& './build/six_step_test.exe'
```

## 器件和标准库

编译暂按 `CYT2BL3CAE`、`tviibe4m`、`CY_MCU_rev_a` 配置。板图仅标注 CYT2BL3，首次下载前请核对实物完整料号和封装。两个核共用 SDL 自带的 IAR 链接脚本，CM0+ 和 CM4 的向量表分别位于 `0x10000000` 和 `0x10080000`。

`vendor/sdl/8.7.0` 收录本工程使用的 Infineon TRAVEO T2G Sample Driver Library 子集。来源为本机归档的官方包，`provenance.json` 保存包来源和各文件 SHA-256，`License.txt` 随附。升级时同步核对双核器件宏、启动代码和链接布局。[英飞凌 SDL 说明](https://documentation.infineon.com/traveo/docs/lwy1680596932876)
