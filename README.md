# ZED X / N10 三相机与 IMU 采集

天准 N10 上的相机与 IMU 采集工具。保留原单 ZED + BMI088 驱动、联合采集程序和验证数据，新增两台 SHW5G 与一台 ZED X Mini 的并行采集、独立启停、文件记录和校验回传。

适用系统：N10 `B25EJ21_007 R1.1`，厂商内核 `6.8.12-rt-tegra`，L4T R38.2 / JetPack 7.0。需要已适配的厂商 BSP 和运行库，不是通用 Jetson 安装包。

## 当前相机配置

| 接口 | 相机 | 输出 |
| --- | --- | --- |
| CAM-3 V6 / V7 | SENSING SHW5G，ST VB1940 | 每台 2560×1984 RGB |
| CAM-3 V8 | Stereolabs ZED X Mini，双 AR0234，SN53004639 | 1920×1200 校正左右 RGB、官方 NEURAL 深度和置信度 |

ZED 使用官方 SDK 5.4.1 / L4T38.2，内置 V4 采集后端；SHW 使用型号对应的 NVIDIA Argus ISP。SHW 为单目，不各自生成深度。三路独立启停已验证，曝光同步和绝对距离精度尚未标定。

## 采集与回传

在开发电脑执行，使用已配置的 SSH 别名 `thor`：

```bash
python3 scripts/capture_remote_n10.py record --initialize --name new-session --frames 600 --save-every 60
```

重启 N10 后初始化一次，后续省略 `--initialize`。初始化在相机空闲时同步本机时间并加载混合驱动。每路预热 90 帧、接收 600 帧，每 60 帧保存一组；逐帧时间戳保留，`latest.jpg` 随保存更新。会话自动回传至本机 `ouputs/new-session/` 并进行 SHA256 校验。

续传已有会话：

```bash
python3 scripts/capture_remote_n10.py fetch new-session
```

在 Thor 上直接运行：

```bash
sudo python3 scripts/capture_three_n10.py "$PWD/ouputs/new-session" --frames 600 --save-every 60
```

用 `--cameras zed` 或 `--cameras shw_v6 shw_v7` 选择独立运行的相机集合，使用不同会话名。Ctrl+C 只停止该命令管理的相机。同一相机不能重复占用，全局初始化需停止全部采集。

## 目录与入口

```text
source/        采集程序、混合驱动、ISP 与 IMU 源码
scripts/       构建、初始化、采集、导出、分析和回传入口
driver/        原单 ZED + IMU 适配模块
device-tree/   N10 混合相机设备树
calibration/   序列号标定、型号 ISP 及来源校验
docs/          使用指南、依赖与最新验证报告
tests/         原 ZED RAW 处理及会话完整性回归测试
validation/    原仓库单 ZED + IMU 验证数据
ouputs/        当前三路采集与验证输出，Git 忽略
build/         N10 构建产物，Git 忽略
vendor/        厂商运行库，Git 忽略
evidence/      依赖、备份和诊断资料，Git 忽略
```

| 入口 | 功能 |
| --- | --- |
| [build_three_n10.sh](scripts/build_three_n10.sh) | 构建 SHW RGB 与 ZED 官方 RGB-D 采集程序 |
| [load_mixed_n10.sh](scripts/load_mixed_n10.sh) | 初始化当前三相机混合驱动 |
| [capture_three_n10.py](scripts/capture_three_n10.py) | 三路并行采集与记录 |
| [capture_remote_n10.py](scripts/capture_remote_n10.py) | PC 控制与校验回传 |
| [analyze_three_n10.py](scripts/analyze_three_n10.py) | 参考图颜色检查与拼接 |
| [validate_three_restart_n10.py](scripts/validate_three_restart_n10.py) | 逐路停止、重启验收 |
| [analyze_zed_svo_n10.sh](scripts/analyze_zed_svo_n10.sh) | 官方 SVO 离线深度检查 |
| [capture_joint_n10.sh](scripts/capture_joint_n10.sh) | 原单 ZED + IMU 联合采集 |
| [analyze_joint_n10.py](scripts/analyze_joint_n10.py) | 原图像与 IMU 软件时间配对分析 |

## 文档

- [三相机使用指南](docs/三相机使用指南.md)：型号、参数、构建、采集处理与输出。
- [三相机验证报告](docs/三相机验证报告.md)：最后一次实测结果、参考图和验证边界。
- [ZED 标定说明](docs/ZED标定说明.md)：序列号标定、来源与几何参数。
- [开发依赖与资料清单](docs/开发依赖与资料清单.md)：厂商 BSP、SDK、运行库和本地手册。
- [原单 ZED + IMU 使用指南](docs/使用指南.md)：原接线下的安装、数据字段、构建及回退。

原单 ZED + IMU 入口适用于其原接线和驱动配置，不在当前三相机接线上运行 `--full` 初始化。三相机入口不导出独立 IMU 数据流。原运行组件可用 `sha256sum -c RUNTIME-SHA256SUMS.txt` 校验。

深度 float32 数据单位为米，保留 NaN/Inf；显示图不能代替距离数据。校验通过只证明文件完整。SHW 保存时接收速度约 28 fps，未承诺无丢帧全帧录像。

禁止整体升级厂商系统或运行中直接断电；关机先执行 `sudo poweroff`。保留原启动项、备份及源码许可，凭据和受限厂商资料不提交仓库。
