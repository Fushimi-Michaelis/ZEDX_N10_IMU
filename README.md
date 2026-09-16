# ZED X 内置 IMU / N10 联合采集

ZED X 内置 BMI088 驱动适配、双目图像与 IMU 联合采集工具，以及实际设备验证数据。

**这是已完成图像驱动配置的 N10 的 IMU 补充包，不是通用 Jetson 驱动或完整刷机包。** 本仓库保留原交付文件和校验值；不包含设备登录密码、厂商下载账号或原始开发包中的其他厂商资料。

## 适用配置

- 天准 N10，板型 `B25EJ21_007`，原厂定制 `6.8.12-rt-tegra` / L4T R38.2 内核。
- ZED X 接 CAM-3/V7；双目 `/dev/video4`、`/dev/video5`。
- 内置 BMI088：I²C bus 7，陀螺仪 `0x3b`、加速度计 `0x3c`。
- 依赖此前已安装并验证的相机模块、设备树，以及 `/home/nvidia/load_zedx_n10.sh` 和 `/home/nvidia/zedx-shw5g-reference`。这些基础组件不在本 IMU 补充包中。
- 安装器检查原有模块哈希，不匹配时拒绝安装；不要通过删除检查来安装到其他系统。

## 验证结果

两轮各 45 秒联合采集，第二轮在完整重新初始化后执行：

| 项目 | 第一轮 | 第二轮 |
| --- | ---: | ---: |
| IMU 样本 | 8,993 | 8,996 |
| IMU 稳定段速率 | 约 199.925 Hz | 约 199.925 Hz |
| 图像 | 每目 1,212 帧，约 30 fps | 每目 1,212 帧，约 30 fps |
| 图像错误帧 / 帧序号断点 | 0 / 0 | 0 / 0 |
| IMU I²C 错误 / 环形缓冲区丢样 / FIFO 溢出观测 | 0 / 0 / 0 | 0 / 0 / 0 |

已验证完整重新加载、原模块回退、正常中断、再次采集，以及忙碌状态下拒绝第二消费者和拒绝重新初始化。

**验证边界：**通过的是采集链路及软件时间配对，不是曝光时刻、动态视觉—惯性时差或绝对 UTC 标定。图像时间是 VI SOF；IMU 时间是主机单调时钟下的软件估计。最近邻样本时间差约 2.76 ms 的上限不能解释为实际传感器同步精度。没有长期压力测试或开机自启验收。

## 使用

克隆仓库，普通 Git 即可，无需 Git LFS。仓库包含实际 PNG/RAW 和完整 IMU CSV，体积约 173 MiB（不含 Git 历史）。

```bash
git clone git@github.com:Ericonaldo/ZEDX_N10_IMU.git
cd ZEDX_N10_IMU
```

仅在上述已配置好的 N10 上安装：

```bash
bash scripts/install_imu_on_n10.sh --check
sudo bash scripts/install_imu_on_n10.sh --install
```

已安装的当前 N10 不需要重复安装。停止所有采集程序后，初始化并启动新一轮记录：

```bash
sudo bash /opt/zedx-n10-imu-20260916/scripts/load_zedx_imu_n10.sh --full
sudo bash /opt/zedx-n10-imu-20260916/scripts/capture_joint_n10.sh /home/nvidia/zedx-capture/joint-new-01
```

输出目录必须不存在；下一次采集更换目录名。采集完毕会停止 IMU 采样。原 NVIDIA 模块及启动文件不覆盖，没有新增自动启动服务。禁止执行系统整体升级或直接断电。

完整操作、数据字段、坐标轴与单位、构建及回退见 [使用指南](使用指南.md)；统计方法与边界见 [联合采集验证报告](联合采集验证报告.md)。

## 文件结构

```text
driver/                    两个适配后内核模块
source/                    内核模块及采集程序源码，保留原许可声明
scripts/                   安装、初始化、采集、导出与统计工具
validation/joint-01/       第一轮联合采集及 5 对图像/RAW
validation/joint-02/       重新初始化后的第二轮联合采集
validation/imu-*/          IMU 冒烟、中断及重新启动测试
validation/system-final.txt 最终驱动状态和校验记录
```

图像、原始帧与 IMU 可以通过眼别和帧序号追溯：`images/pairs.json` → `frame-imu-pairs.csv` → `imu/imu.csv`。RAW 使用校验值与偏移定位，不能用文件修改时间作拍摄时间。

`SHA256SUMS.txt` 对应原交付包的文件，`RUNTIME-SHA256SUMS.txt` 对应安装组件；新增的 Git README 和仓库配置不在原交付清单内。在仓库根目录检查：

```bash
shasum -a 256 -c SHA256SUMS.txt
shasum -a 256 -c RUNTIME-SHA256SUMS.txt
```

Linux 上可将 `shasum -a 256` 换成 `sha256sum`。请在验证数据副本上重新运行分析工具，以保留原始证据文件。
