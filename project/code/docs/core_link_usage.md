# Core Link 使用说明

## 1. 模块作用

`core_link` 是一个基于 UART4 的轻量链路模块，用于两块单片机之间传输数据。

当前已经实现：

- UART4 接收中断
- FIFO 缓冲
- 通用帧格式收发
- 协议 ID 分发
- 文本协议
- ASCII 参数协议

当前示例层：

- `core_link_user.c`
- `main_cm7_0.c`

其中：

- `core_link.c/.h` 负责链路层
- `core_link_user.c/.h` 负责用户示例
- `main_cm7_0.c` 负责主循环与 IPS200 显示

## 2. 文件职责

### `core_link.h`

配置和对外接口文件。

主要内容：

- 串口配置
- 缓冲区大小
- 帧头定义
- 协议 ID 定义
- 初始化、任务处理、发送接口声明
- 收包回调声明

### `core_link.c`

链路核心实现文件。

主要内容：

- UART4 中断接收
- FIFO 入队
- 通用帧解析
- 协议分发
- 文本协议解析
- 参数协议解析
- 发包实现

### `core_link_user.h/.c`

用户示例文件。

当前示例实现：

- 周期发送文本 `"test_date is "`
- 周期发送参数 `T1=test_date`
- 在回调中保存接收到的：
  - `text_receive`
  - `test_date_receive`

### `main_cm7_0.c`

主循环入口。

当前作用：

- 初始化时钟和调试
- 初始化 `core_link`
- 初始化 IPS200
- 主循环里执行：
  - `core_link_task()`
  - `core_link_user_task()`
  - 屏幕刷新

## 3. 当前串口配置

在 `core_link.h` 中配置：

```c
#define CORE_LINK_UART_INDEX            (UART_4)
#define CORE_LINK_UART_BAUDRATE         (115200)
#define CORE_LINK_UART_TX_PIN           (UART4_TX_P14_1)
#define CORE_LINK_UART_RX_PIN           (UART4_RX_P14_0)
```

当前使用：

- UART4
- 波特率 115200
- TX: P14.1
- RX: P14.0

两块板连接方式：

- A 板 `P14.1(TX)` -> B 板 `P14.0(RX)`
- A 板 `P14.0(RX)` -> B 板 `P14.1(TX)`
- 两块板 `GND` 共地

## 4. 通用帧格式

当前链路使用统一帧格式：

```text
[SOF1][SOF2][PROTO_ID][LEN][PAYLOAD...][CHECKSUM]
```

字段说明：

- `SOF1`：固定帧头 1，当前为 `0xA5`
- `SOF2`：固定帧头 2，当前为 `0x5A`
- `PROTO_ID`：协议类型
- `LEN`：负载长度
- `PAYLOAD`：实际数据内容
- `CHECKSUM`：校验值

当前校验规则：

- `checksum = protocol_id ^ length ^ payload[0] ^ payload[1] ...`

## 5. 当前协议 ID

在 `core_link.h` 中定义：

```c
typedef enum
{
    CORE_LINK_PROTOCOL_TEXT        = 0x01,
    CORE_LINK_PROTOCOL_PARAM_ASCII = 0x02,
}core_link_protocol_enum;
```

### `0x01` 文本协议

负载直接是文本内容，例如：

```text
HELLO
test_date is
```

发送接口：

```c
core_link_send_text("HELLO");
```

### `0x02` 参数 ASCII 协议

负载格式：

```text
R1=1.234
T1=25.000
```

发送接口：

```c
core_link_send_param_ascii('R', '1', 1.234f);
core_link_send_param_ascii('T', '1', 25.0f);
```

其中：

- 第 1 个字符：`command`
- 第 2 个字符：`item`
- 后面是 `=value`

## 6. 如何接入主循环

### 初始化

在 `main` 中初始化：

```c
core_link_init();
core_link_user_init();
```

### 循环处理

在主循环中反复执行：

```c
core_link_task();
core_link_user_task();
```

说明：

- `core_link_task()` 负责收包、解包、协议分发
- `core_link_user_task()` 是你自己的示例任务

## 7. 中断接入方式

在 `cm7_0_isr.c` 的 `uart4_isr()` 中调用：

```c
core_link_uart4_rx_handler();
```

说明：

- 中断里不直接解析协议
- 中断里只负责把串口字节放进 FIFO
- 真正解包在 `core_link_task()` 中完成

这种方式更适合高频、多次接收场景

## 8. 如何发送数据

### 8.1 发送文本

```c
core_link_send_text("test_date is ");
```

### 8.2 发送参数

```c
core_link_send_param_ascii('T', '1', (float)test_date);
```

### 8.3 发送自定义协议帧

```c
uint8 payload[3] = {1, 2, 3};
core_link_send_frame(0x10, payload, 3);
```

适用于后续你自己扩展新的协议 ID。

## 9. 如何接收并处理数据

当前接收通过回调实现。

### 文本回调

```c
void core_link_on_text_received (const uint8 *text, uint8 length)
{
}
```

### 参数回调

```c
void core_link_on_param_received (const core_link_param_packet_struct *param)
{
}
```

### 通用回调

```c
void core_link_on_frame_received (uint8 protocol_id, const uint8 *payload, uint8 length)
{
}
```

当前建议：

- 普通文本用 `core_link_on_text_received`
- 参数用 `core_link_on_param_received`
- 新协议扩展用 `core_link_on_frame_received`

## 10. 如何扩展新的协议

推荐流程如下。

### 第一步：新增协议 ID

在 `core_link.h` 里增加，例如：

```c
CORE_LINK_PROTOCOL_SENSOR = 0x03,
CORE_LINK_PROTOCOL_POSE   = 0x04,
```

### 第二步：发送时使用新的协议 ID

例如：

```c
core_link_send_frame(CORE_LINK_PROTOCOL_SENSOR, payload, length);
```

### 第三步：在 `core_link.c` 中分发

在 `core_link_dispatch_frame()` 中增加新的 `case`。

例如：

```c
case CORE_LINK_PROTOCOL_SENSOR:
{
    // parse sensor payload
} break;
```

### 第四步：如果需要，增加新的解析函数

例如新增：

- `core_link_parse_sensor_packet(...)`
- `core_link_parse_pose_packet(...)`

## 11. 如何扩展新的参数

如果你只是想继续使用当前参数格式：

```text
X1=value
X2=value
Y1=value
```

通常不需要改链路层。

你只需要在：

- `core_link_on_param_received(...)`

里增加分支即可。

例如：

```c
void core_link_on_param_received (const core_link_param_packet_struct *param)
{
    if(('R' == param->command) && ('1' == param->item))
    {
        // handle R1
    }
    else if(('P' == param->command) && ('2' == param->item))
    {
        // handle P2
    }
}
```

也就是说：

- 增加参数种类，不一定要改 `core_link.c`
- 大多数情况下只改用户层回调就够了

## 12. 如何修改链路容量

在 `core_link.h` 中可改：

```c
#define CORE_LINK_RX_FIFO_SIZE      (256)
#define CORE_LINK_PAYLOAD_MAX_LEN   (64)
```

说明：

- `CORE_LINK_RX_FIFO_SIZE`：串口接收 FIFO 大小
- `CORE_LINK_PAYLOAD_MAX_LEN`：单帧最大负载长度

如果后续收发频率更高、单包更长，可以适当增大。

## 13. 当前示例说明

当前示例层行为：

- 每个示例周期发送一次文本：

```text
test_date is
```

- 每个示例周期发送一次参数：

```text
T1=test_date
```

- `test_date` 每周期自增 `1`

- 接收端显示：
  - `text_receive`
  - `test_date_receive`

## 14. 推荐扩展原则

建议后续按下面规则维护：

- `core_link.c` 只做链路层和协议层
- 业务逻辑尽量写在 `core_link_user.c` 或其他用户文件
- 显示逻辑尽量写在 `main_cm7_0.c` 或独立 UI 文件
- 扩协议优先新增 `protocol_id`
- 扩参数优先在参数回调里加分支

## 15. 后续推荐方向

如果后面继续做复杂扩展，建议按下面方向演进：

- 给不同协议单独拆文件
- 增加发送节流/定时发送机制
- 增加帧计数或时间戳
- 增加丢包统计
- 如需更高频控制，可考虑增加二进制参数协议

---

如果后续你继续迭代这份链路模块，建议优先保持：

- 串口中断只收字节
- 主循环负责解包
- 统一帧格式
- 协议 ID 分发

这样后面再扩很多功能时，代码结构不会乱。
