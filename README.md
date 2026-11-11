# Gento Luna 右臂 J7 运动测试

2026-11-11 开发交付：右臂 ARM1，第 7 轴先反向 10°定位，再正向 20°、反向 20°往复 **5 次**，速度和加速度比例均为 **20%**。

## 文件

- [开发文档](开发文档.md)：连接环境、开发过程、问题修复、接口、验证范围和备份位置。
- [运动程序](run_right_arm_5cycles.py)：最终独立 Python 程序。
- [Windows 启动文件](start_right_arm_test.cmd)：与运动程序放在同一文件夹，手动启动。
- [离线检查程序](verify_program_offline.py)：使用模拟对象检查逻辑，不连接机器人。
- [控制器回读记录](controller_20percent_verified.txt)及[配置基线](Gento_Luna_测试基线.json)。
- [完整 ZIP 交付包](Gento_Luna_右臂开发交付_20261111.zip)。

## 先检查，不运动

```powershell
python verify_program_offline.py
python run_right_arm_5cycles.py --preview
```

## 手动执行真实运动

**无参数运行以下程序会实际连接机器人并请求运动。** 请先阅读开发文档并确认现场条件。程序不会在仓库上传或离线检查时自动执行。

```powershell
python run_right_arm_5cycles.py
```

Windows 通过用户已配置的 `gentoluna` SSH 别名和密钥调用机器人 ROS 2 后端，不在 Windows 安装 ROS 包。仓库不包含 SSH 私钥、密码或商业授权文件；其他电脑需单独配置访问环境。

## 验证状态

右臂控制器速度/加速度已回读为 20%，错误码为 0；最终路径预检和离线检查通过。本次开发过程中未自动执行完整 20% 五次往复真机验收，需由用户手动完成。

本版本按控制器完成回执顺序衔接各段，不添加段间三次确认或额外等待；它不是一次上传的整条轨迹。控制器自身换向与完成判断仍保留。每次运行都相对当时位置重新计算，结束在起始位置减 10°处，不自动返回初始点。
