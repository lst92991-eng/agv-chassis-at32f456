# Ubuntu 最小操作

适用于2026-10-27重构后的下位机执行固件，已烧录并完成架空11米变速验证，记录见`hardware/execution_11m_raised_2026-10-27.md`。
仓库位于Ubuntu的`/home/st/agv-chassis-at32f456`。

1. USB-CAN FD连接到Ubuntu虚拟机；总线两端各120Ω，保持架空。
2. 配置接口，仲裁采样点必须使用本次已验证的75%：

   ```bash
   cd ~/agv-chassis-at32f456
   sudo ip link set can0 down
   sudo ip link set can0 type can bitrate 500000 sample-point 0.75 dbitrate 4000000 dsample-point 0.75 fd on
   sudo ip link set can0 up
   python3 tools/ubuntu_canfd_idle.py status
   ```

3. 在操作者准备好后选择一个测试：

   ```bash
   python3 tools/ubuntu_canfd_idle.py spin       # 50mm/s，3秒后STOP
   python3 tools/ubuntu_canfd_idle.py distance   # 目标11米，50→300→50mm/s，最长180秒
   ```

测试器直接发0x401，无前置ENABLE；按速度档位变化发送单帧并维护sequence。最后由测试器发送STOP。
这是上位机测试工具的有限运行策略，不是下位机的超时或保护。采集诊断会打印，不再把诊断字节等同于自动FAULT状态。

独立命令：

```bash
python3 tools/ubuntu_canfd_idle.py stop
python3 tools/ubuntu_canfd_idle.py reset   # 上位机明确解除ESTOP/DISABLE
```

每次测试默认从Reference序号1开始，因此应在板端重新上电后使用；不重启时必须通过
`--reference-start`给出上次已使用的序号，正常上位机应持续维护该计数。
STOP/ESTOP后旧序号不会被清除；新的上位机启动不得盲目复用旧序号。

本工具只依赖Python标准库和SocketCAN。原500k/4M默认87.5%采样点曾导致ERROR-PASSIVE；
改为75%后已正常收帧。旧USB适配器还出现过`failed to xmit URB: -EPIPE`，这属于PC/USB发送链路，
不是新固件的软件保护；若RX增长但TX计数不增长，应先恢复USB-CAN接口再测。
