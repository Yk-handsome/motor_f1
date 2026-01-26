# STM32 FOC 电机驱动

基于 STM32F103C8T6 的无刷直流电机 (BLDC) FOC 控制实现。

## ⚠️ 重要提醒

**编译时请至少开启 O2 优化等级！**

## 📚 文档与教程

理论讲解及代码详细分析：[CSDN系列文章](https://blog.csdn.net/qq570437459/category_12672491.html)

## 🛠️ 硬件配置

| 项目 | 配置 |
|------|------|
| 主控芯片 | STM32F103C8T6 |
| 电机极对数 | 7 |
| 编码器 | MT6701 磁编码器 (14位SPI) |
| PWM频率 | 40kHz |
| 电流采样 | 双ADC注入式 (INA826 + 0.02Ω采样电阻) |

**购买链接**: [配套驱动板](https://item.taobao.com/item.htm?ft=t&id=839302894720)

## 🔧 供电说明

- 板子**不提供5V输出**，防止损坏电脑USB口
- 供电方式：
  - DC 12V
  - 右侧5V接口（若搭配本项目DAPLink，插Type-C即可供电5V）

## 📁 项目结构

```
├── Core/                 # STM32 HAL外设配置
│   ├── Inc/              # 头文件
│   └── Src/              # 源文件
│       ├── main.c        # 主程序入口
│       ├── tim.c         # TIM1(PWM) + TIM3(速度计算)
│       ├── adc.c         # 电流采样与FOC控制环
│       └── spi.c         # MT6701编码器读取
├── Drivers/motor/        # FOC核心算法
│   ├── foc.h/c           # FOC控制核心
│   ├── conf.h            # 电机参数配置
│   └── motor_runtime_param.c/h
├── algorithm/            # 算法工具
│   └── filter.h/c        # 低通滤波、卡尔曼滤波
└── MDK-ARM/              # Keil工程文件
```

## 🎯 功能特性

- ✅ 位置/速度/力矩三环串级PID控制
- ✅ SVPWM空间矢量调制
- ✅ CMSIS-DSP库高精度计算
- ✅ 注入式ADC电流采样与PWM同步
- ✅ 多圈角度累计

## 🚀 快速开始

1. 使用 Keil MDK 打开 `MDK-ARM/stm32_foc_test.uvprojx`
2. 编译选项设置为 `-O2` 或更高优化等级
3. 下载并调试

## 🔄 更新日志

**2511更新**: 工程实践软件架构更新，更加合理高效的计算架构
- 最大转速提升20%
- 大幅降低旋转噪音
- 明显改善低速性能
- 纯速度环0.1RPM

新版本仓库: [https://gitee.com/best_pureer/stm32_foc_2](https://gitee.com/best_pureer/stm32_foc_2)

## 📖 学习建议

1. 先阅读 CSDN 系列教程理解 FOC 理论
2. 配合 `README_CN.md` 了解代码架构
3. 从里向外调：电流环 → 速度环 → 位置环
4. 使用示波器观察 PWM 波形进行调试
