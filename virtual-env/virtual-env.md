这份文档专门为你定制，基于 **Mac (Apple Silicon M1/M2/M3)** 环境，目标是完成 **TurtleBot 安全研究 Final Project**。

我们将采用 **UTM 虚拟机 + Ubuntu ARM64 + ROS 2** 的方案。这是目前在 Mac 上运行机器人仿真最稳定、最不容易"掉坑"的路线。

**支持的 Ubuntu 版本：**
- **Ubuntu 22.04 (Jammy Jellyfish)** → 使用 **ROS 2 Humble**
- **Ubuntu 24.04 (Noble Numbat)** → 使用 **ROS 2 Jazzy**

> 💡 **推荐**：Ubuntu 22.04 + ROS 2 Humble 更稳定，软件包更成熟。如果你已经安装了 Ubuntu 24.04，也可以使用 ROS 2 Jazzy。

-----

# 📘 Final Project 实战手册：环境搭建与基础测试

**目标**：从零搭建一个可以运行 TurtleBot3 仿真的虚拟环境，并验证能否控制它（为后续攻击实验做准备）。
**适用设备**：Mac (M1/M2/M3 芯片)
**总耗时预估**：1 - 2 小时

-----

## 阶段一：虚拟机搭建 (The Foundation)

我们需要一台“假”的 Linux 电脑来运行 ROS。

### Step 1: 下载并安装 UTM

UTM 是 Mac 上最好用的开源虚拟化软件（基于 QEMU）。

1.  前往 [UTM 官网](https://mac.getutm.app/) 下载并安装（免费版即可）。

### Step 2: 下载 Ubuntu ARM64 镜像

**选项 A：Ubuntu 22.04 LTS (Jammy Jellyfish) - 推荐**
1.  前往 Ubuntu 官网下载 [Ubuntu 22.04.3 LTS (Jammy Jellyfish)](https://ubuntu.com/download/server/arm).
      * **注意**：虽然这里写的是 Server，但它是 ARM64 架构的基础。或者你可以直接搜索下载 **Ubuntu 22.04 Desktop ARM64**（如果有桌面版 ISO 更好，省去配置界面的麻烦）。
      * *推荐链接*：找 Daily Build 或者直接用 UTM Gallery 里的 Ubuntu 22.04 预配置版（最省事）。

**选项 B：Ubuntu 24.04 LTS (Noble Numbat)**
1.  前往 Ubuntu 官网下载 [Ubuntu 24.04 LTS (Noble Numbat)](https://ubuntu.com/download/server/arm).
      * 同样选择 ARM64 架构版本。
      * 如果使用 Ubuntu 24.04，后续将安装 **ROS 2 Jazzy**（而不是 Humble）。

### Step 3: 配置虚拟机

1.  打开 UTM -\> **Create a New Virtual Machine**。
2.  选择 **Virtualize** (虚拟化) -\> **Linux**。
3.  点击 **Browse** 选择刚才下载的 `.iso` 镜像文件。
4.  **硬件配置（关键）**：
      * **Memory (内存)**：建议 **4096 MB (4GB)** 或更多。
      * **CPU Cores**：建议 **4 Cores**。
5.  **共享目录 (Optional)**：可以设置一个 Mac 文件夹共享给虚拟机，方便后续传实验报告截图。
6.  **启动并安装**：
      * 启动虚拟机，按照屏幕提示安装 Ubuntu。
      * *提示*：如果在安装过程中觉得图形界面非常卡，或者安装完黑屏，请在 UTM 设置中把 Display 的 **3D Acceleration** 关闭（或选为 `virtio-ramfb`）。

### Step 3.5: 配置 SSH 连接（推荐）

**为什么要用 SSH？** 从 Mac 主机通过 SSH 连接到虚拟机可以：
- 更方便地复制粘贴命令
- 使用 Mac 的终端工具（iTerm2, Terminal.app）
- 避免虚拟机图形界面可能的性能问题
- 可以同时打开多个终端会话

**在虚拟机内配置 SSH 服务器**：

1. 打开 Ubuntu 终端，安装 SSH 服务器：
```bash
sudo apt update
sudo apt install openssh-server -y
```

2. 启动 SSH 服务并设置开机自启：
```bash
sudo systemctl start ssh
sudo systemctl enable ssh
```

3. 查看虚拟机的 IP 地址：
```bash
ip addr show | grep "inet " | grep -v 127.0.0.1
```
   或者使用：
```bash
hostname -I
```
   记下显示的 IP 地址（通常是 `192.168.x.x` 或 `10.0.x.x`）

4. **在 UTM 中配置网络**（如果需要）：
   - 确保虚拟机的网络模式设置为 **Shared Network (NAT)** 或 **Bridged Network**
   - 如果使用 NAT，UTM 会自动分配 IP
   - 如果使用 Bridged，虚拟机会获得与 Mac 同一网段的 IP

**从 Mac 主机连接**：

1. 打开 Mac 的终端（Terminal.app 或 iTerm2）

2. 使用 SSH 连接（替换 `<IP地址>` 为步骤3中看到的IP）：
```bash
ssh <用户名>@<IP地址>
```
   例如：`ssh ubuntu@192.168.64.3`

3. 首次连接会提示确认主机密钥，输入 `yes`

4. 输入 Ubuntu 用户的密码

**连接成功后**，你就可以在 Mac 终端中直接操作虚拟机了！后续的所有配置命令都可以在这个 SSH 会话中执行。

**小贴士**：
- 如果 IP 地址经常变化，可以在虚拟机中设置静态 IP，或者使用 UTM 的端口转发功能
- 可以使用 SSH 密钥认证避免每次输入密码（可选）

-----

## 阶段二：ROS 2 Humble 环境部署 (The Software)

系统装好后，**在虚拟机终端或 SSH 会话中**，逐行执行以下命令。

### Step 4: 设置语言环境 (Locale)

ROS 2 对语言环境要求很严，必须支持 UTF-8。

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Step 5: 添加 ROS 2 软件源

告诉 Ubuntu 去哪里下载 ROS。

```bash
# 开启 universe 仓库
sudo apt install software-properties-common
sudo add-apt-repository universe

# 下载 GPG 密钥
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 添加源地址
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 6: 安装 ROS 2

这步需要一点时间，取决于你的网速。

**首先确认你的 Ubuntu 版本：**
```bash
lsb_release -cs
```

**根据 Ubuntu 版本选择对应的 ROS 2 版本：**

#### 如果是 Ubuntu 22.04 (Jammy) → 安装 ROS 2 Humble

```bash
sudo apt update
sudo apt upgrade
# 安装桌面完整版（包含 Gazebo 仿真器）
sudo apt install ros-humble-desktop
```

#### 如果是 Ubuntu 24.04 (Noble) → 安装 ROS 2 Jazzy

```bash
sudo apt update
sudo apt upgrade
# 安装桌面完整版（包含 Gazebo 仿真器）
sudo apt install ros-jazzy-desktop
```

> ⚠️ **重要**：确保 ROS 版本与 Ubuntu 版本匹配！Ubuntu 22.04 用 Humble，Ubuntu 24.04 用 Jazzy。

**⚠️ 如果安装失败，请按以下步骤排查：**

#### 故障排除 1: 检查软件源是否正确添加

首先确认 Step 5 是否成功执行：

```bash
# 检查 ROS 2 软件源是否存在
cat /etc/apt/sources.list.d/ros2.list

# 应该看到类似这样的输出：
# deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main
```

如果文件不存在或内容不对，重新执行 Step 5。

#### 故障排除 2: 检查 GPG 密钥

```bash
# 检查密钥文件是否存在
ls -la /usr/share/keyrings/ros-archive-keyring.gpg

# 如果不存在，重新下载：
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

#### 故障排除 3: 检查系统架构

UTM 虚拟机应该使用 ARM64 架构（因为 Mac M1/M2/M3 是 ARM 芯片）：

```bash
# 检查系统架构
dpkg --print-architecture
# 应该输出：arm64

# 检查 Ubuntu 版本
lsb_release -cs
# 应该输出：jammy (Ubuntu 22.04)
```

#### 故障排除 4: 更新软件包列表并查看具体错误

```bash
# 清理并更新
sudo apt clean
sudo apt update

# 查看详细错误信息
sudo apt install ros-humble-desktop 2>&1 | tee install_error.log
```

**常见错误及解决方案：**

1. **错误：`E: Unable to locate package ros-humble-desktop` 或 `E: Unable to locate package ros-jazzy-desktop`**
   - **原因 1**：软件源没有正确添加，或架构不匹配
   - **解决**：重新执行 Step 5，确保架构是 `arm64`
   - **原因 2**：ROS 版本与 Ubuntu 版本不匹配
   - **解决**：
     - Ubuntu 22.04 (Jammy) → 使用 `ros-humble-desktop`
     - Ubuntu 24.04 (Noble) → 使用 `ros-jazzy-desktop`
   - **检查方法**：
     ```bash
     # 查看 Ubuntu 版本
     lsb_release -cs
     # 如果是 "noble"，应该安装 ros-jazzy-desktop
     # 如果是 "jammy"，应该安装 ros-humble-desktop
     ```

2. **错误：`GPG error: ... NO_PUBKEY ...`**
   - **原因**：GPG 密钥问题
   - **解决**：
   ```bash
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   sudo apt update
   ```

3. **错误：`404 Not Found` 或网络连接问题**
   - **原因**：网络问题或镜像源问题
   - **解决**：检查网络连接，或尝试使用不同的镜像源

4. **错误：依赖冲突或包损坏**
   - **解决**：
   ```bash
   sudo apt --fix-broken install
   sudo apt update
   sudo apt upgrade
   ```

#### 故障排除 5: 如果上述都不行，尝试分步安装

如果完整版安装失败，可以尝试安装基础版本：

**ROS 2 Humble (Ubuntu 22.04):**
```bash
# 先安装基础版本
sudo apt install ros-humble-desktop-base

# 然后单独安装需要的组件
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-rviz2
```

**ROS 2 Jazzy (Ubuntu 24.04):**
```bash
# 先安装基础版本
sudo apt install ros-jazzy-desktop-base

# 然后单独安装需要的组件
sudo apt install ros-jazzy-gazebo-ros-pkgs
sudo apt install ros-jazzy-rviz2
```

#### 故障排除 6: 验证软件源配置（完整检查清单）

运行以下命令，确保所有配置都正确：

```bash
# 1. 检查架构
echo "架构: $(dpkg --print-architecture)"

# 2. 检查 Ubuntu 版本
echo "Ubuntu 版本: $(lsb_release -cs)"

# 3. 检查软件源文件
echo "软件源配置:"
cat /etc/apt/sources.list.d/ros2.list

# 4. 检查 GPG 密钥
echo "GPG 密钥:"
ls -la /usr/share/keyrings/ros-archive-keyring.gpg

# 5. 测试软件源连接
sudo apt update 2>&1 | grep -i "ros\|error\|failed"
```

如果所有检查都通过，但安装仍然失败，请把具体的错误信息发给我，我会帮你进一步诊断。

### Step 7: 环境初始化

把 ROS 命令加到你的启动脚本里，这样每次打开终端都能直接用。

**根据你安装的 ROS 版本选择对应的命令：**

#### 如果是 ROS 2 Humble (Ubuntu 22.04)
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 如果是 ROS 2 Jazzy (Ubuntu 24.04)
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

-----

## 阶段三：部署 TurtleBot3 仿真 (The Target)

我们要把你的攻击目标（TurtleBot）装进系统里。

### Step 8: 安装 TurtleBot3 相关包

**根据你安装的 ROS 版本选择对应的命令：**

#### ROS 2 Humble (Ubuntu 22.04)
```bash
sudo apt install ros-humble-gazebo-*
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-turtlebot3*
```

#### ROS 2 Jazzy (Ubuntu 24.04)
```bash
sudo apt install ros-jazzy-gazebo-*
sudo apt install ros-jazzy-cartographer
sudo apt install ros-jazzy-cartographer-ros
sudo apt install ros-jazzy-navigation2
sudo apt install ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-turtlebot3*
```

### Step 9: 设置机器人型号

TurtleBot 有几种型号，我们用最经典的 `burger`（汉堡包款）。

```bash
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
```

-----

## 阶段四：验证与初次运行 (The Test)

这一步决定了你是否可以开始做 Final Project 的核心部分。

### Step 10: 启动仿真世界

在终端输入：

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

  * **预期结果**：会弹出一个 Gazebo 窗口，里面有一个带有蓝色六角形障碍物的世界，地板中间停着一个小车（TurtleBot）。
  * *Mac 特有故障排除*：如果 Gazebo 打开后黑屏，或者闪退，回到 UTM 设置 -\> Display -\> 取消勾选 "3D Acceleration" (3D 加速)，然后重启虚拟机。

### Step 11: 尝试控制 (模拟攻击者的第一步)

保持 Step 10 的窗口开着，**打开一个新的终端窗口**，输入：

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

  * **操作**：按照屏幕提示，按键盘上的 `w` (前进), `a` (左转), `x` (后退), `d` (右转), `s` (停止)。
  * **观察**：看 Gazebo 里的机器人是否动了？
  * **意义**：如果你能控制它动，说明 `/cmd_vel` 这个控制接口是通的。
      * 你的 **Attack Project** 就是要写一个脚本，代替这个键盘程序，在后台悄悄发送 `w` (前进) 或 `s` (停止) 的指令，让机器人“失控”。

-----

## ✅ 下一步行动 Checkpoint

当你完成 **Step 11** 并成功让小车动起来后，请告诉我。

接下来我会教你如何进行第一项攻击实验：
**编写一个 Python 脚本，伪装成合法节点，向 TurtleBot 发送“死亡旋转”指令。**

**现在，请先去搞定 UTM 和 Ubuntu 吧！**