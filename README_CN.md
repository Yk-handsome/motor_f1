# STM32 FOC 电机驱动项目分析

## 📋 项目概述

这是一个基于 **STM32F103C8T6** (ARM Cortex-M3) 的 **无刷直流电机 (BLDC)** 驱动的完整实现，使用 **FOC (Field Oriented Control，磁场定向控制)** 算法。

### 硬件配置
- **主控芯片**: STM32F103C8T6
- **驱动板**: 配套的FOC驱动板 (淘宝链接见原README)
- **电机**: 极对数 = 7
- **编码器**: MT6701 磁编码器 (14位分辨率，通过SPI读取)
- **电流采样**: 双ADC注入式同步采样 (INA826运放 + 0.02Ω采样电阻)

---

## 🏗️ 项目架构

```
stm32_foc-main/
├── Core/                          # STM32 HAL库标准外设配置
│   ├── Inc/                       # 头文件
│   │   ├── main.h
│   │   ├── tim.h                  # 定时器配置 (PWM + 速度计算)
│   │   ├── adc.h                  # ADC配置 (电流采样)
│   │   ├── spi.h                  # SPI配置 (编码器通信)
│   │   └── global_def.h           # 全局宏定义
│   └── Src/
│       ├── main.c                 # 主程序入口
│       ├── tim.c                  # TIM1( PWM ), TIM3( 速度计算 )
│       ├── adc.c                  # 电流采样与FOC控制环
│       ├── spi.c                  # MT6701磁编码器读取
│       └── gpio.c / usart.c
│
├── Drivers/motor/                 # 🎯 FOC核心算法
│   ├── foc.h / foc.c              # FOC控制核心
│   ├── conf.h                     # 电机参数配置
│   └── motor_runtime_param.c/h    # 运行时参数
│
├── algorithm/                     # 算法工具库
│   ├── filter.h / filter.c        # 低通滤波、卡尔曼滤波
│
└── MDK-ARM/                       # Keil工程文件
```

---

## ⚡ FOC算法核心流程

```
                    ┌─────────────┐
                    │  目标设定    │
                    │ (位置/速度/  │
                    │  力矩)       │
                    └──────┬──────┘
                           │
                           ▼
              ┌────────────────────────┐
              │  串级PID控制环          │
              │  位置环 → 速度环 →      │
              │  力矩环(d/q轴电流)      │
              └───────────┬────────────┘
                          │
                          ▼
         ┌────────────────────────────────┐
         │     Clarke变换 (3相→αβ)        │
         │   (Ia, Ib, Ic) → (Iα, Iβ)      │
         └────────────────┬───────────────┘
                          │
                          ▼
         ┌────────────────────────────────┐
         │     Park变换 (αβ→dq旋转坐标)    │
         │   (Iα, Iβ) → (Id, Iq)          │
         │   需要转子角度信息              │
         └────────────────┬───────────────┘
                          │
                          ▼
         ┌────────────────────────────────┐
         │    PID调节器 (Id, Iq)          │
         │   输出目标电压 (Vd, Vq)         │
         └────────────────┬───────────────┘
                          │
                          ▼
         ┌────────────────────────────────┐
         │   逆Park变换 (dq→αβ静止坐标)    │
         │   (Vd, Vq) → (Vα, Vβ)          │
         └────────────────┬───────────────┘
                          │
                          ▼
         ┌────────────────────────────────┐
         │        SVPWM调制               │
         │   生成3相PWM占空比 (Ua, Ub, Uc) │
         └────────────────┬───────────────┘
                          │
                          ▼
                    ┌─────────────┐
                    │   功率驱动   │
                    │   (H桥PWM)   │
                    └─────────────┘
```

---

## 🔑 核心代码解析

### 1. FOC控制入口 (`foc.c`)

```c
// 核心流程：电流采样 → Clarke → Park → PID → 逆Park → SVPWM
void foc_forward(float d, float q, float rotor_rad)
{
    float d_u = 0;
    float d_v = 0;
    float d_w = 0;
    svpwm(rotor_rad, d, q, &d_u, &d_v, &d_w);
    set_pwm_duty(d_u, d_v, d_w);
}
```

### 2. SVPWM实现

```c
// 空间矢量PWM：将dq轴电压转换为3相PWM占空比
static void svpwm(float phi, float d, float q, float *d_u, float *d_v, float *d_w)
{
    // 1. 逆Park变换
    arm_inv_park_f32(d, q, &alpha, &beta, sin_phi, cos_phi);
    
    // 2. 判断扇区 (6个扇区)
    bool A = beta > 0;
    bool B = fabs(beta) > SQRT3 * fabs(alpha);
    bool C = alpha > 0;
    int K = 4 * A + 2 * B + C;
    
    // 3. 计算各矢量作用时间
    // 4. 生成PWM占空比
}
```

### 3. 串级PID控制

```c
// 位置-速度-力矩 三环串级控制
void lib_position_speed_torque_control(float position_rad, float max_speed_rad, float max_torque_norm)
{
    float speed_rad = position_loop(position_rad);     // 位置环 → 速度
    speed_rad = min(fabs(speed_rad), max_speed_rad);
    lib_speed_torque_control(speed_rad, max_torque_norm); // 速度环 → 力矩
}
```

### 4. 电流采样与坐标变换 (`adc.c`)

```c
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    // 1. ADC采样 → 相电流 (Ia, Ib)
    motor_i_u = i_1;
    motor_i_v = i_2;
    
    // 2. Clarke变换 (3相→2相)
    arm_clarke_f32(motor_i_u, motor_i_v, &i_alpha, &i_beta);
    
    // 3. Park变换 (静止→旋转)
    arm_park_f32(i_alpha, i_beta, &_motor_i_d, &_motor_i_q, sin_value, cos_value);
    
    // 4. 调用控制环
    lib_position_control() / lib_speed_control() / lib_torque_control();
}
```

### 5. 编码器读取 (`spi.c`)

```c
// MT6701磁编码器，14位分辨率，支持多圈累计
encoder_angle = 2 * PI * angle_raw / (1 << 14);  // 原始角度
motor_logic_angle = cycle_diff(motor_logic_angle + diff_angle, position_cycle); // 多圈累计
```

---

## 📊 项目深度评价

### ⭐ 难度等级: **中高级**

| 方面 | 评价 | 说明 |
|------|------|------|
| **理论难度** | ⭐⭐⭐⭐ | 需要理解电机原理、坐标变换、PID控制 |
| **代码难度** | ⭐⭐⭐ | 结构清晰，使用CMSIS-DSP库简化计算 |
| **硬件要求** | ⭐⭐⭐ | 需要驱动板、电机、调试工具 |
| **完整性** | ⭐⭐⭐⭐ | 完整实现位置/速度/力矩三环控制 |

### ✅ 项目优点
1. **代码结构清晰** - FOC算法模块化，易于学习
2. **使用CMSIS-DSP** - ARM官方数学库，精度高、性能好
3. **完整的串级PID** - 位置-速度-力矩三环控制
4. **配套教程** - CSDN有详细的理论讲解系列文章
5. **工程实践优化** - 如注释所说，经过工程实践优化

### ⚠️ 需要改进的地方
1. **缺少速度环前馈** - 高速性能可进一步优化
2. **无观测器** - 没有使用观测器估计反电动势
3. **无弱磁控制** - 无法超过基速运行
4. **单电阻采样** - 如能用三电阻采样更全面

---

## 📚 学习路线建议

### 阶段一：基础知识储备 (2-3周)

**必看资源:**
1. **CSDN教程系列** (你找到的那个链接) - 理论+代码逐步讲解
2. **FOC入门视频** - B站搜索"磁匠FOC"、"不懂嵌入式的FEI"等
3. **数学基础**:
   - 三角函数与复数
   - 线性代数基础
   - 拉普拉斯变换与PID控制

**学习重点:**
- [ ] 理解为什么FOC需要坐标变换
- [ ] 理解Clarke/Park变换的物理意义
- [ ] 理解PID控制器原理及调参方法
- [ ] 理解SVPWM的空间矢量概念

### 阶段二：代码学习与调试 (3-4周)

**学习顺序 (建议):**

```
1. 🔧 硬件层
   └→ 理解PWM生成、定时器配置
   └→ 理解ADC采样时机 (注入式vs轮询)
   └→ 理解编码器读取

2. 📐 坐标变换
   └→ foc.c 中的 Clarke/Park/逆Park
   └→ 使用CMSIS-DSP库函数

3. 🎯 控制环
   └→ 理解串级PID结构
   └→ 从里向外调：电流环 → 速度环 → 位置环
   └→ PID参数整定方法

4. 🔄 完整流程
   └→ 画一个完整的数据流图
   └→ 理解每个变量的物理意义
```

**调试技巧:**
- 使用串口打印关键变量 (`printf("%.3f\n", motor_i_q);`)
- 示波器观察PWM波形、SVPWM调制效果
- 先调电流环，再调速度环，最后调位置环

### 阶段三：进阶优化 (持续)

**可以尝试的改进:**
1. 🔄 **更换控制算法**
   - 尝试无感FOC (观测器方案)
   - 实现弱磁控制 (扩展转速范围)

2. 📈 **性能优化**
   - 添加前馈控制
   - 使用滑模观测器
   - 实现自适应PID

3. 🛡️ **功能完善**
   - 添加堵转保护
   - 实现电流环限幅
   - 添加通信协议 (CAN/RS485)

---

## 🛠️ 实操建议

### 必备工具
1. **调试器**: ST-Link或DAPLink
2. **示波器**: 观察PWM波形、电流采样
3. **万用表**: 测量电压、电阻
4. **可调电源**: 限流供电，保护电路

### 推荐学习步骤

1. **先看懂代码** (1-2周)
   - 不要急于烧录
   - 画出数据流图
   - 理解每个函数的作用

2. **硬件验证** (1周)
   - 检查焊接是否正确
   - 用示波器验证PWM输出
   - 验证编码器通信

3. **软件调试** (2-3周)
   - 从力矩模式开始 (给定Iq)
   - 调通电流环 (观察Iq跟踪)
   - 逐步开启速度环、位置环

4. **参数整定** (1-2周)
   - 学习PID整定方法 (Ziegler-Nichols等)
   - 记录不同参数的效果

---

## 📖 推荐学习资源

### 理论类
| 资源 | 链接 | 推荐度 |
|------|------|--------|
| CSDN系列教程 | https://blog.csdn.net/qq570437459/category_12672491.html | ⭐⭐⭐⭐⭐ |
| 磁匠科技B站 | B站搜索"磁匠FOC" | ⭐⭐⭐⭐⭐ |
| TinyFOC | https://github.com/simplefoc/Arduino-FOC | ⭐⭐⭐⭐ |
| ST AN | "AN1095" "AN1086"等应用笔记 | ⭐⭐⭐⭐ |

### 实践类
| 资源 | 用途 |
|------|------|
| SimpleFOC库 | Arduino平台FOC参考 |
| ODrive | 开源高性能伺服驱动 |
| VESC | 电动滑板车开源方案 |

---

## 🚀 快速开始

### 编译项目
```bash
# 使用Keil MDK打开 MDK-ARM/stm32_foc_test.uvprojx
# 编译选项: Optimization Level -O2 或更高
```

### 硬件连接
```
电机 U相 → 驱动板 U
电机 V相 → 驱动板 V
电机 W相 → 驱动板 W
编码器 → SPI接口
12V电源 → 驱动板电源
ST-Link → SWD调试口
```

### 调试步骤
1. 先不接电机，检查PWM输出
2. 检查编码器能否正确读数
3. 力矩模式测试 (给定Iq)
4. 调通电流环
5. 开启速度环、位置环

---

## 💡 总结

这是一个 **质量较高** 的FOC入门项目：

✅ **适合人群**:
- 有一定嵌入式开发经验
- 了解PWM、ADC、中断等概念
- 对电机控制有兴趣

❌ **不太适合**:
- 完全没有嵌入式基础 (建议先学STM32基础)
- 需要无感FOC (本项目使用磁编码器)

**建议**: 结合CSDN教程学习，先理解理论，再看代码，最后动手调试。

---

## 📝 笔记

*本README由AI生成，用于项目分析学习。如有错误，欢迎指出。*

