# 当前 ZED 出厂标定

- 相机序列号：`53004639`，从当前设备的 `zed_eeprom_data` 读取。
- 型号：**ZED X Mini**，`53004639 // 10000000 = 5`，与 49.83 mm 基线一致；识别依据为 [官方序列号编码说明](https://docs.stereolabs.com/docs/development/zed-sdk/modules/camera/using-the-api)。735 像素焦距乘规格中的 3 μm 像元约为 2.21 mm，支持广角镜头判断，但未核验外壳完整 SKU/偏振片选项。
- 文件：[SN53004639.conf](../calibration/SN53004639.conf)。
- 来源：<https://calib.stereolabs.com/?SN=53004639>。
- 获取：用户明确批准向 Stereolabs 官方服务提交序列号后下载；本次工作日期 2026-10-30，Thor 系统时间未校准，不能用其文件时间确定下载日期。
- SHA256：`609267d81b45edea503d3a5353695ae05341d418c23aeb014dbf8ff730cca15e`。
- 使用 `LEFT_CAM_FHD1200` / `RIGHT_CAM_FHD1200`、对应八参数畸变、`STEREO` 中的平移和 Rodrigues 旋转。基线约 49.834557 mm，转换为米后计算深度。

官方 SDK 入口按序列号 53004639 打开相机，使用 SDK 返回的校正标定参数并保存 `calibration.json`；换相机不能沿用此文件。当前已验证的混合路由与 SDK V4 后端下，光学左目 RAW 对应 I²C `7-0028`、右目对应 `7-0020`。用左右对应特征的正视差和垂直残差检查方向，不能仅凭设备编号或节点名称判断。

[Stereolabs 标定文件说明](https://docs.stereolabs.com/docs/development/zed-sdk/modules/camera/camera-calibration)。官方标定只提供几何参数，不能代替当前装配后的实物距离精度验证。
