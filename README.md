# dipan 底盘工程 README

## 1. 工程简介

`dipan` 是基于 STM32F405RGTx + FreeRTOS 的 RM 底盘控制工程，核心链路为：

`DR16 遥控输入 -> 全向轮解算 -> 速度环 PID -> CAN 下发电机`

当前底盘支持 4 轮全向驱动，`defaultTask` 只负责调用任务层入口，实际控制任务由 `MyTask` 统一申请，BMI088 读取任务实现放在底盘模块层。

## 2. 代码入口

- FreeRTOS 线程入口：`Core/Src/freertos.c` 的 `StartDefaultTask()`
- 任务封装：`user_file/4_Task/MyTask.cpp`
- 底盘控制主逻辑：`user_file/3_Module/Chassis/Chassis.cpp`
- BMI088 周期读取任务：`user_file/3_Module/Chassis/Chassis.cpp` 的 `Chassis_BMI088_Task()`

执行顺序：

1. `MyTask_Init()`
2. `Chassis_Init()`（DR16、CAN、电机、PID、定时器初始化）
3. `MyTask_Run()` 申请 `Chassis_RunTask()` 和 `Chassis_BMI088_Task()`
4. `Chassis_RunTask()` 与 `Chassis_BMI088_Task()` 各自独立运行

## 3. 构建环境要求

- CMake >= 3.22
- Ninja
- `arm-none-eabi-gcc` 工具链（需要在 `PATH` 中）
- STM32CubeMX/CubeIDE（用于 `.ioc` 维护与代码生成）

说明：本工程的 CMake 预设在 `CMakePresets.json` 中，默认使用 `Ninja` + `cmake/gcc-arm-none-eabi.cmake`。

## 4. 编译命令

在 `E:\liantiao\dipan` 下执行：

```powershell
cmake --preset Debug
cmake --build build/Debug
```

产物：

- `build/Debug/dipan.elf`
- `build/Debug/dipan.map`

## 5. 烧录方式

### 方式 A：CubeIDE / CubeProgrammer

1. 选择工程 `dipan`
2. 使用 ST-Link或者 J-Link 连接
3. 烧录 `build/Debug/dipan.elf`

### 方式 B：命令行（可选）

```powershell
STM32_Programmer_CLI -c port=SWD -w build/Debug/dipan.elf -v -rst
```

## 6. 遥控控制说明（当前版本）

底盘旋转模式由 **左拨杆（S1）** 控制：

- `UP`：手动旋转模式
  - 右摇杆 `X` 控制旋转角速度
- `DOWN`：自动自旋模式
  - 持续自旋
  - 右摇杆 `X` 用于增减自旋速度
- `MIDDLE`：停止旋转
  - 旋转角速度置 0

平移控制：

- 左摇杆 `Y` 控制底盘 `vx`（前进/后退）
- 左摇杆 `X` 控制底盘 `vy`（左右平移）

## 7. 关键参数（调参入口）

文件：`user_file/3_Module/Chassis/Chassis.h`

- `CHASSIS_CTRL_DT_S`：控制计算 `dt`（与任务周期保持一致）
- `CHASSIS_MAX_LINEAR_SPEED_MPS`：最大平移速度
- `CHASSIS_MAX_ANGULAR_SPEED_RADPS`：最大旋转速度
- `CHASSIS_AUTO_SPIN_SPEED_RADPS`：自动自旋初始速度
- `CHASSIS_AUTO_SPIN_SPEED_MIN_RADPS`：自动自旋最小速度
- `CHASSIS_AUTO_SPIN_SPEED_MAX_RADPS`：自动自旋最大速度
- `CHASSIS_AUTO_SPIN_ADJUST_RADPS_PER_S`：自动自旋调速灵敏度
- `CHASSIS_AUTO_SPIN_ADJUST_DEADZONE`：右摇杆调速死区

## 8. USB CDC 接收链路（已替换）

文件：`USB_DEVICE/App/usbd_cdc_if.c`

当前 `CDC_Receive_FS()` 已改为：

- 不直接在此处做默认的 `SetRxBuffer/ReceivePacket` 组合
- 统一转发到 `USB_Rx_Callback(Buf, *Len)`
- 由 `user_file/1_middlewares/Driver/USB/drv_usb.c` 完成双缓冲切换、上层回调分发与下一包接收准备

## 9. 已知注意事项

1. 本工程运行依赖 FreeRTOS。
   - `Chassis_RunTask()` 使用了 `xTaskGetTickCount()` / `vTaskDelayUntil()`。
   - `Chassis_BMI088_Task()` 也使用 `vTaskDelayUntil()` 固定 `1ms` 轮询 BMI088。
2. 当前控制任务是 `1ms` 周期。
   - 在总线负载高时可能需要降到 `2ms` 以提高稳定性。
3. DR16 接收串口句柄以代码为准。
   - 当前 `Chassis_Init()` 中使用 `DR16_Init(&huart3)`，如硬件接到其他串口（例如 UART5），需要同步修改。
4. BMI088 最新数据可通过 `Chassis_GetBMI088ImuData()` / `Chassis_GetBMI088EulerData()` 获取。

## 10. 目录简表

- `Core/`：CubeMX 生成的 HAL / FreeRTOS 入口
- `USB_DEVICE/`：USB CDC 设备协议层
- `user_file/1_middlewares/`：驱动与算法
- `user_file/2_Device/`：设备层（电机、DR16、BMI088、VOFA）
- `user_file/3_Module/Chassis/`：底盘控制模块
- `user_file/4_Task/`：任务封装入口
- `dipan.ioc`：CubeMX 工程文件

---
