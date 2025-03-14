# rs_monitor

[README](readme.md) | [中文文档](readme_cn.md)

## 1. 简介

`rs_monitor` 用于监控 `ros2` 中关心的一些指标，例如: **内存/CPU/IO使用率**、**消息帧率**、**消息时间戳与当前系统时间的差值**等，监控结果会通过日志和 topic 输出

同时，该软件包还提供了将监控结果数据生成可视化报告的 `python` 脚本，您可以本地生成包含简易统计结果以及折线图的 html 格式报告

## 2. 前置依赖

此项目基于 `ros2 humble` + `python 3.8.10` 进行开发测试

其中 `rs_monitor` 节点依赖 `ros2` 以及 `package.xml` 中定义的依赖项，可视化报告生成脚本依赖 `requirements.txt` 中定义的第三方 `python` 模块

### 2.1 安装 ros2

根据您的操作系统选择 [官方教程](https://fishros.org/doc/ros2/humble/Installation.html) 中的指定内容进行执行

### 2.2 安装 python3 [可选]

您可以遵循您正在使用的平台的 Python 安装教程来安装 Python3，推荐使用 `3.8.10` 以上的版本

* **Windows:** 下载并执行 https://www.python.org/ftp/python/3.8.10/python-3.8.10-amd64.exe 即可完成安装
* **Ubuntu:** `sudo apt-get install -y python3 python3-pip`

## 3. 安装部署

### 3.1 代码拉取

您可以创建一个新的文件夹或进入您现有的 `ros2` 工作空间，执行以下命令将代码拉取到工作空间内

```bash
# ssh
git clone git@gitlab.robosense.cn:super_sensor_sdk/ros2_sdk/rs_monitor.git -b main
# http
git clone git@gitlab.robosense.cn:super_sensor_sdk/ros2_sdk/rs_monitor.git -b main
```

### 3.2 通过 rosdep 安装依赖

可以通过 `rosdep` 工具安装 `rs_monitor` 编译所需的依赖

```
rosdep install --from-paths rs_monitor --ignore-src -r -y
```

### 3.3 编译 rs_monitor

在您的工作空间下执行以下命令来编译安装 `rs_monitor`

```bash
colcon build --symlink-install --packages-select rs_monitor
```

编译安装完成后，推荐刷新一下工作空间的 `bash profile`，确保组件功能正常

```bash
source install/setup.bash
```

### 3.4 为可视化结果解析脚本安装依赖 [可选]

可视化脚本依赖了一些 `pip` 第三方库，可以通过执行以下命令来快捷对这些第三方库进行安装

```bash
ros2 run rs_monitor setup.py
```

如果提示只能安装 `Debian` 管理的 `python` 依赖，需要在当前目录下创建虚拟环境后再运行安装脚本

```bash
# 创建虚拟环境
python3 -m venv ./.venv
# 激活虚拟环境
source .venv/bin/activate
```

然后再次运行

```bash
ros2 run rs_monitor setup.py
```

或者

```bash
pip install -r rs_monitor/requirements.txt
```

## 4. 使用方式

### 4.1 运行监测节点

通过 `ros2 launch` 命令可以运行 `rs_monitor` 节点

```bash
ros2 launch rs_monitor rs_monitor.launch.py
```

可以通过参数 `config_file` 来指定需要加载的配置文件

```bash
ros2 launch rs_monitor rs_monitor.launch.py config_file:=path/to/your/config.yaml
```

### 4.2 录制监测数据

* 如果当前没有录制全量数据，可以单独启动 `ros2 bag` 命令来录制 `rs_monitor` 输出的 topic

* 默认配置下，**资源占用率**和**消息帧率**的监测会发布到 `/diagnostics` 话题，可以通过更改配置文件中的特定字段来修改

启动一个 `rosbag` 的录制进程来将监测数据录制到本地数据包内

```bash
# 默认配置下，录制 /diagnostics 话题即可
# -d 60 表示以[60秒]为单位分割 bag，可不加
ros2 bag record /diagnostics -d 60
```

如果通过修改配置使结果输出到了其他 topic，需要对应更改录制的 topic 参数

### 4.3 生成可视化报告

录制后的 `rosbag` 可通过 `ros2 run rs_monitor parser.py` 命令解析为 `html` 格式的可视化报告，内容包含**简略统计值**以及**折线图**

*注意: 需要执行上文提到的依赖安装脚本*

```bash
ros2 run rs_monitor parser.py --help
usage: parser.py [-h] [-d OUTPUT_DIRECTORY] [-f OUTPUT_FILENAME] [--resource_usage_topic RESOURCE_USAGE_TOPIC] [--frequency_topic FREQUENCY_TOPIC] [--split_duration SPLIT_DURATION] input_directory

位置参数:
  input_directory       本地 rosbag 的文件夹路径，需要内部包含 [metadata.yaml] 元信息文件

options:
  -h, --help            show this help message and exit
  -d OUTPUT_DIRECTORY, --output_directory OUTPUT_DIRECTORY 结果输出路径，默认为当前工作目录/output
  -f OUTPUT_FILENAME, --output_filename OUTPUT_FILENAME
                        输出的 html 结果文件名称，默认为 [result.html]
  --resource_usage_topic RESOURCE_USAGE_TOPIC
                        包含资源占用率信息的 topic 名称，默认为 [/diagnostics]
  --frequency_topic FREQUENCY_TOPIC
                        包含帧率信息的 topic 名称，默认为 [/diagnostics]
  --split_duration SPLIT_DURATION
                        控制每份可视化结果的最大时长的字符串, examples: 1h30m (一小时三十分) | 1d (一天) | 1m30s (一分三十秒)
                        默认为空，表示不进行分割
```

假设您在 `/root/rosbag2_2025_01_14-07_26_38` 下录制了一份 bag，可以通过以下命令在 `${PWD}/output/result.heml` 路径下输出可视化报告

```bash
ros2 run rs_monitor parser.py /root/rosbag2_2025_01_14-07_26_38
```

如果录制时间较长，可以通过 `--split_duration` 参数控制每份报告包含的数据时长，分割后的每份结果文件名格式为 `${name}_{index}.html`，如 `result_0.html`

```bash
# 按一小时分割
ros2 run rs_monitor parser.py /root/rosbag2_2025_01_14-07_26_38 --split_duration=1h
```

## 5. 功能说明

### 5.1 配置文件

| 编译安装得到的配置文件路径位于: [${workspace}/install/rs_monitor/share/rs_monitor/config/config.yaml]

配置文件中的设置项主要包括以下两部分:

1. **ros2 进程探测规则 (`ros2_process_manager`)**: 定义如何识别和管理 `ros2` 相关进程
2. **各监测模块的配置 (`*_monitor`)**: 包括资源监测、帧率监测和时间戳偏移监测等各模块的具体参数设置

以下是各部分配置项的详细说明:

#### 5.1.1 ros2_process_manager

- `refresh_interval_ms`: 刷新需要监测的进程列表的时间间隔，单位为毫秒。默认值为 `6000`，即每 6 秒刷新一次
- `ros2_process_identifiers`: 用于识别 `ros2` 进程的标识符列表
    - 在 `linux` 系统上会作为子串匹配各个进程的 `cmdline`
    - 和 `monitored_processes` 的区别在于 `monitored_processes` 更精确，且会指定监测结果中的进程名称
- `ros2_process_comm`: 与 `ros2` 相关的命令或可执行文件名称
    - 在 `linux` 系统上会匹配各个进程的 `comm`
- `monitored_processes`: 可选的监测进程列表，可以通过关键字匹配特定进程，在 `linux` 系统上会作为子串匹配进程的 `cmdline`，示例配置如下：
```yaml
monitored_processes:
- keyword: "ros2 echo /"    # cmdline 关键字
  name: "echo"              # 匹配到的进程在监测结果中的名称
```


#### 5.1.2 *_monitor

下面是一些监测模块的通用配置字段:

* `enable`: 布尔值，是否启用该监测模块
* `exec_interval_ms`: 监测结果的发布时间间隔，单位为*毫秒*，也会影响监测模块的内部计算频率
* `publish_topic_name`: 监测结果发布到的话题名称，为 `~` (None) 时将不会进行话题发布

#### 5.1.3 frequency_monitor

- `topics`: 需要监测帧率的 `topic` 列表，每个 `topic` 可以设置最小帧率限制。示例配置如下：
```yaml
frequency_monitor:
  topics:
  - name: "/diagnostics"
    min_freq: 1.0
  - name: "/rslidar_points_e1"
    min_freq: 0.0
```

#### 5.1.4 timestamp_monitor

- `topics`: 需要监测时间戳偏移的 `topic` 列表，每个 `topic` 可以设置最大时间差限制。示例配置如下：
```yaml
timestamp_monitor:
  topics:
  - name: "/diagnostics"
    max_difference_ms: 1000
```

### 5.2 资源监测

对于每一个需要监测的进程，`resource_monitor` 会统计下面三项资源占用率：

* CPU
    * 在 linux 系统下进程的 CPU 占用率包含 `utime + stime + cutime + cstime`，因为包含了子进程的占用统计，所以会在**子进程退出时出现较明显的增量**
* 内存
    * 在 linux 系统下采用 `RSS` 作为进程内存占用的统计值
* IO
    * 只包含直接对设备读写的字节数，不包括在缓存中读写的字节数

### 5.3 时间戳偏移监测

时间戳偏移的结果会输出到日志文件中，格式如下：

```bash
The difference between system timestamp and message timestamp exceeds the threshold [{}ms], topic: [{}]
```

### 5.4 可视化报告

**Tips:** 可视化报告中折线图的图例可以使用滚轮键进行翻页，同时可以按住 `shift` 键进行范围选取

## 6. FAQ

[Create New Issue](http://gitlab.robosense.cn/super_sensor_sdk/ros2_sdk/rs_monitor/-/issues/new)
