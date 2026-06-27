# OpenArm ROS2 Control GUI

This project contains my OpenArm ROS2 Jazzy setup and custom Python GUI scripts for controlling the OpenArm v1.0 robotic arm through ROS2 action controllers.

## Features

- ROS2 Jazzy + OpenArm v1.0
- Classic CAN support for CANable / gs_usb
- Custom OpenArmHW modifications for MIT mode activation
- Left arm GUI control with Tkinter
- Joint delta control for joint1 to joint7
- Home and Emergency Home
- Gripper Open / Close
- Safe small-angle control with waypoint interpolation

## Important Safety Notes

Do not run:

- `openarm-can-demo`
- `openarm-can-zero-position-calibration`
- `set_zero`
- RViz `Plan & Execute` during first hardware tests

Use small joint deltas first, such as `0.01` or `0.02` rad.

# OpenArm 實體控制環境安裝與操作步驟

## 一、使用環境

建議使用：

```text
Windows + WSL2 Ubuntu 24.04 + ROS2 Jazzy
```

需要準備：

```text
1. OpenArm v1.0 實體手臂
2. CANable / MKS CANable V2.0
3. 手臂電源
4. CAN 線
5. Windows 電腦
6. WSL Ubuntu 24.04
7. ROS2 Jazzy
```

---

## 二、安裝必要工具

在 WSL Ubuntu terminal 執行：

```bash
sudo apt update
sudo apt install -y git python3-pip python3-tk can-utils net-tools
```

用途說明：

```text
git：下載 GitHub 專案
python3-tk：執行 Tkinter GUI
can-utils：使用 candump / cansend 檢查 CAN
net-tools：輔助檢查網路介面
```

確認 ROS2 Jazzy 已安裝後，每次開新 terminal 先執行：

```bash
source /opt/ros/jazzy/setup.bash
```

---

## 三、clone 專案

在 WSL terminal 執行：

```bash
cd ~
git clone https://github.com/411277002/openarm.git openarm_ros2_ws
cd ~/openarm_ros2_ws
```

---

## 四、build workspace

進入 workspace 後執行：

```bash
cd ~/openarm_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

如果 build 成功，之後每次開新的 terminal 都要先執行：

```bash
cd ~/openarm_ros2_ws
source install/setup.bash
```

---

## 五、連接 CANable 到 WSL

### 1. Windows PowerShell 操作

請用「系統管理員」開啟 PowerShell。

先查看 USB 裝置：

```powershell
usbipd list
```

找到 CANable / candleLight / gs_usb / STM32 類似名稱的裝置，記下它的 BUSID，例如：

```text
2-3
```

第一次使用時先 bind：

```powershell
usbipd bind --busid 2-3
```

再 attach 到 WSL：

```powershell
usbipd attach --wsl --busid 2-3
```

如果電腦有多個 WSL 發行版，也可以指定 Ubuntu：

```powershell
usbipd attach --wsl Ubuntu --busid 2-3
```

---

## 六、在 WSL 設定 CAN 介面

回到 WSL terminal，檢查 CAN 介面：

```bash
ip link
```

如果看到 `can0`，但本專案 launch 使用 `can1`，請改名成 `can1`：

```bash
sudo ip link set can0 down
sudo ip link set can0 name can1
```

設定 CAN bitrate：

```bash
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
```

確認狀態：

```bash
ip -details -statistics link show can1
```

正常應該看到：

```text
state ERROR-ACTIVE
bitrate 1000000
```

---

## 七、測試 CANable 是否正常

開一個 WSL terminal：

```bash
candump can1
```

再開另一個 WSL terminal：

```bash
cansend can1 7FF#11223344
```

如果第一個 terminal 有看到封包，代表 CANable、SocketCAN、`can1` 基本正常。

測完後在 `candump` terminal 按：

```text
Ctrl + C
```

---

## 八、啟動 OpenArm 實體控制

先打開 OpenArm 電源，手不要靠近關節與夾爪。

### 情況 A：控制左手臂

如果實體手臂接在左手 CAN 線，並使用 `can1`，執行：

```bash
cd ~/openarm_ros2_ws
source install/setup.bash

ros2 launch openarm_bimanual_moveit_config demo.launch.py \
  arm_type:=openarm_v1.0 \
  description_file:=openarm_v10.urdf.xacro \
  use_fake_hardware:=false \
  left_can_interface:=can1 \
  right_can_interface:=vcan0
```

意思是：

```text
left_can_interface:=can1   左手接實體 CAN
right_can_interface:=vcan0 右手用虛擬佔位
```

---

### 情況 B：控制右手臂

如果實體手臂接在右手 CAN 線，並使用 `can1`，執行：

```bash
cd ~/openarm_ros2_ws
source install/setup.bash

ros2 launch openarm_bimanual_moveit_config demo.launch.py \
  arm_type:=openarm_v1.0 \
  description_file:=openarm_v10.urdf.xacro \
  use_fake_hardware:=false \
  left_can_interface:=vcan0 \
  right_can_interface:=can1
```

意思是：

```text
left_can_interface:=vcan0  左手用虛擬佔位
right_can_interface:=can1  右手接實體 CAN
```

---

## 九、確認 controller 狀態

另開一個 WSL terminal：

```bash
cd ~/openarm_ros2_ws
source install/setup.bash
ros2 control list_controllers
```

如果控制左手，應該看到：

```text
joint_state_broadcaster           active
left_joint_trajectory_controller  active
left_gripper_controller           active
```

如果控制右手，應該看到：

```text
joint_state_broadcaster            active
right_joint_trajectory_controller  active
right_gripper_controller           active
```

---

## 十、確認 joint_states 有資料

```bash
ros2 topic echo /joint_states --once
```

如果是左手，應該能看到：

```text
openarm_left_joint1
openarm_left_joint2
...
openarm_left_joint7
openarm_left_finger_joint1
```

如果是右手，應該能看到：

```text
openarm_right_joint1
openarm_right_joint2
...
openarm_right_joint7
openarm_right_finger_joint1
```

---

## 十一、啟動 GUI

### 左手 GUI

如果要控制左手：

```bash
cd ~/openarm_ros2_ws
source install/setup.bash
python3 openarm_scripts/openarm_gui_panel.py
```

GUI 開啟後，可以使用：

```text
joint1 ~ joint7 delta 輸入框
Home
Emergency Home
Gripper Open
Gripper Close
更新目前狀態
```

第一次測試建議只輸入：

```text
joint4 delta = 0.01
```

其他欄位保持空白。

---

### 右手 GUI

如果要控制右手，需要使用右手版本 GUI，例如：

```bash
python3 openarm_scripts/openarm_right_gui_panel.py
```

右手 GUI 需要使用：

```text
openarm_right_joint1 ~ openarm_right_joint7
/right_joint_trajectory_controller/follow_joint_trajectory
openarm_right_finger_joint1
/right_gripper_controller/follow_joint_trajectory
```

如果目前專案還沒有右手 GUI，請先複製左手 GUI 並將 left 相關名稱改成 right。

---

## 十二、第一次實體測試建議

第一次不要直接按 Demo，也不要在 RViz 按 Plan & Execute。

建議順序：

```text
1. 開啟 launch
2. 確認 controller active
3. 開啟 GUI
4. 按「更新目前狀態」
5. 只輸入 joint4 delta = 0.01
6. 確認手臂有小幅動作
7. 檢查 joint4 狀態是否正常
```

左手 joint4 狀態檢查：

```bash
candump -tz can1,014:7FF > joint4_status_check.txt
```

等 2 秒後按：

```text
Ctrl + C
```

再執行：

```bash
grep "  014 " joint4_status_check.txt | awk '{print $5}' | sort | uniq -c
```

正常應該看到：

```text
14
```

如果看到 `C4`，請停止控制，關閉 launch，關閉手臂電源，等待約 30 秒後重新啟動。

---

## 十三、重要安全提醒

第一次實體控制時，請遵守以下規則：

```text
不要按 RViz 的 Plan & Execute
不要執行 openarm-can-demo
不要執行 openarm-can-zero-position-calibration
不要使用 set_zero
不要執行 zero calibration
不要一開始輸入 0.1、0.3、-0.5 這種大角度
第一次測試只用 0.01 rad
手不要靠近關節與夾爪
隨時準備關閉手臂電源
```

---

## 十四、常見問題

### 1. `can1` 不見了

可能是 CANable 沒有 attach 到 WSL。

請回 Windows PowerShell：

```powershell
usbipd list
usbipd attach --wsl --busid 你的BUSID
```

再回 WSL：

```bash
ip link
```

---

### 2. 出現 `can0` 不是 `can1`

執行：

```bash
sudo ip link set can0 down
sudo ip link set can0 name can1
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
```

---

### 3. GUI 打不開，出現 tkinter 錯誤

安裝：

```bash
sudo apt update
sudo apt install -y python3-tk
```

---

### 4. controller 不是 active

檢查 launch 是否正確，特別是：

```text
use_fake_hardware:=false
left_can_interface / right_can_interface 是否正確
can1 是否存在
手臂電源是否已開啟
```

---

### 5. 手臂有資料但不會動

請確認使用的是本專案修改後的 OpenArmHW，包含：

```text
CAN-FD 關閉
OpenArm v1.0 description
on_activate() 中有強制 arm 1~7 寫入 MIT mode
control_gains.yaml 中 joint4 kp 已調整
```

---
