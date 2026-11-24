# RoundFoc 固件工程

## 打开与构建

用 IAR Embedded Workbench for Arm 打开 `iar/RoundFoc.eww`。在 Project → Batch Build 中选择 `Debug` 构建两个核。也可在本目录执行以下 PowerShell 命令。

```powershell
& 'D:/IAR/common/bin/IarBuild.exe' 'iar/boot_cm0plus.ewp' -build Debug
& 'D:/IAR/common/bin/IarBuild.exe' 'iar/round_foc_cm4.ewp' -build Debug
```

两个核分别生成 `build/Debug/cm0plus/Exe/boot_cm0plus.hex` 和 `build/Debug/cm4/Exe/round_foc_cm4.hex`。同目录的 `.out` 带调试符号。复位启动需要两个镜像分别位于各自的链接地址。

## 从哪里开始读

1. `app/boot_cm0plus.c` 是 CM0+ 的应用入口。当前学习工程在这里关闭 WDT，按 SDL 链接符号释放 CM4，然后等待中断。
2. `app/main_cm4.c` 是后续 FOC 实验入口。在这里设断点可以检查 CM4 是否已启动，并观察 SDL 的 `SystemCoreClock`。基线程序更新时钟变量后等待中断，PWM、栅极驱动和 ADC 均未配置。
3. `config/cy_project.h` 为 SDL 启动文件提供工程需要的声明。后续实验在应用侧配置实际板卡的引脚、极性和时序。

## 器件和标准库

编译暂按 `CYT2BL3CAE`、`tviibe4m`、`CY_MCU_rev_a` 配置。板图仅标注 CYT2BL3，首次下载前请核对实物完整料号和封装。两个核共用 SDL 自带的 IAR 链接脚本，CM0+ 和 CM4 的向量表分别位于 `0x10000000` 和 `0x10080000`。

`vendor/sdl/8.7.0` 收录本工程使用的 Infineon TRAVEO T2G Sample Driver Library 子集。来源为本机归档的官方包，`provenance.json` 保存包来源和各文件 SHA-256，`License.txt` 随附。升级时同步核对双核器件宏、启动代码和链接布局。[英飞凌 SDL 说明](https://documentation.infineon.com/traveo/docs/lwy1680596932876)
