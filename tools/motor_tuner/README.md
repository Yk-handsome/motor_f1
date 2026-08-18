# STM32 FOC 调参台

这是当前工程的浏览器上位机 MVP，直接兼容固件已有的 USART2 文本命令和 VOFA FireWater CSV 输出。

## 启动

在工程根目录启动任意静态 HTTP 服务，然后使用 Microsoft Edge 或 Google Chrome 打开：

`http://127.0.0.1:8010/tools/motor_tuner/`

Web Serial 需要在浏览器弹窗中选择实际的 USB 串口。页面不使用任何网络服务，也不会保存 PID 参数到 MCU Flash。

## 当前支持

- 连接 115200 串口并发送 `show`。
- 实时解析 `Iq目标,Iq实际,Id目标,Id实际` 四列 CSV 数据。
- 曲线显示、实时读数、通信错误统计。
- 修改 `mode`、`target`、`limit` 和四组 PID。
- `mode null` 急停，以及断开按钮的停机命令。
- 自动解析 `show` 返回值，并回填目标、限制和 PID 输入框。
- 结构化 JSON Lines 事件：停机、PID 应用/拒绝、模式切换、目标修改、DMA 状态和每秒心跳。

现有 MCU 接收端一次只能保存一条文本命令。上位机发送队列因此强制相邻命令间隔 35 ms，避免连续设置 Id、Iq、速度和位置时丢掉后续命令。

## 本轮边界

本版仍使用文本命令和 100 Hz CSV 遥测，适合操作、观察静态值和低频现象。电流环阶跃响应必须在下一阶段通过二进制协议和 MCU RAM 高速采样实现，不能用此页面的 100 Hz 曲线判断。

JSON Lines 是当前文本链路上的兼容诊断格式，事件行以 `{` 开始，包含 `type`、`event`、`tick`、`mode` 和当前电流快照。电流字段明确区分为 `_a`（安培）和 `_norm`（归一化值），例如 `iq_ref_a=0.100`、`iq_ref_norm=0.050`。下一阶段再增加独立的二进制帧：固定帧头、长度、消息类型、JSON payload 和 CRC16；不会把裸二进制直接混入现有 CSV/文本命令流。
