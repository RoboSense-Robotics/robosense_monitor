# rs_monitor

[README](readme.md) | [中文文档](readme_cn.md)

## 1. Introduction

`rs_monitor` is used to monitor specific metrics in `ros2`, such as **memory/CPU/IO usage**, **message frame rate**, **the difference between message timestamps and the current system time**, etc. The monitoring results are output through logs and topics.

Additionally, this package provides a `python` script for generating visual reports of monitoring results. You can generate HTML format reports locally, containing simple statistical results and line charts.

## 2. Prerequisites

This project is developed and tested based on `ros2 humble` + `python 3.8.10`.

The `rs_monitor` node depends on `ros2` and the dependencies defined in `package.xml`. The visualization report generation script depends on third-party `python` modules defined in `requirements.txt`.

### 2.1 Install ros2

Follow the [official tutorial](https://docs.ros.org/en/humble/Installation.html) for your operating system to install ros2.

### 2.2 Install python3 [Optional]

You can install Python3 by following the Python installation guide for your platform. It is recommended to use version `3.8.10` or above.

* **Windows:** Download and run https://www.python.org/ftp/python/3.8.10/python-3.8.10-amd64.exe to complete the installation.
* **Ubuntu:** `sudo apt-get install -y python3 python3-pip`

## 3. Installation

### 3.1 Pulling Source Code

You can create a new folder or go into your existing `ros2` workspace and execute the following commands to clone the code into the workspace.

```bash
# ssh
git clone git@github.com:RoboSense-Robotics/robosense_monitor.git -b main rs_monitor
# http
git clone https://github.com/RoboSense-Robotics/robosense_monitor.git -b main rs_monitor
```

### 3.2 Install Dependencies via rosdep

You can use the `rosdep` tool to install the dependencies required for compiling `rs_monitor`.

```
rosdep install --from-paths rs_monitor --ignore-src -r -y
```

### 3.3 Compile rs_monitor

Execute the following commands in your workspace to compile and install `rs_monitor`.

```bash
colcon build --symlink-install --packages-select rs_monitor
```

After compilation and installation, it is recommended to refresh the workspace's `bash profile` to ensure everything properly.

```bash
source install/setup.bash
```

### 3.4 Install Dependencies for Visualization Result Parsing Script [Optional]

The visualization script depends on some `pip` third-party libraries, which can be installed quickly by executing the following command:

```bash
ros2 run rs_monitor setup.py
```

If the prompt only allows installing python dependencies managed by Debian, you need to create a virtual environment in the current directory before running the installation script.

```bash
# Create a virtual environment
python3 -m venv ./.venv
# Activate the virtual environment
source .venv/bin/activate
```

Then run again:

```bash
ros2 run rs_monitor setup.py
```

or

```bash
pip install -r rs_monitor/requirements.txt
```

## 4. Usage

### 4.1 Run Monitoring Node

You can run the `rs_monitor` node using the `ros2 launch` command.

```bash
ros2 launch rs_monitor rs_monitor.launch.py
```

You can use your own configuration file by specifying the `config_file` parameter.

```bash
ros2 launch rs_monitor rs_monitor.launch.py config_file:=path/to/your/config.yaml
```

### 4.2 Record Monitoring Data

* If the full data is not currently being recorded, you can start the `ros2 bag` command separately to record the topics output by `rs_monitor`.

* By default, the monitoring of **resource usage** and **message frame rate** is published to the `/diagnostics` topic. You can modify this by changing specific fields in the configuration file.

Start a `rosbag` recording process to record the monitoring data into a local data package.

```bash
# By default, record the /diagnostics topic
# -d 60 means split the bag into [60 seconds] units, optional
ros2 bag record /diagnostics -d 60
```

If the results are output to other topics by modifying the configuration, the topic parameters for recording need to be changed accordingly.

### 4.3 Generate Visual Reports

The recorded `rosbag` can be parsed into an `html` format visual report using the `ros2 run rs_monitor parser.py` command. The report contains **brief statistical values** and **line charts**.

*Note: You need to execute the dependency installation script mentioned above.*

```bash
ros2 run rs_monitor parser.py --help
usage: parser.py [-h] [-d OUTPUT_DIRECTORY] [-f OUTPUT_FILENAME] [--resource_usage_topic RESOURCE_USAGE_TOPIC] [--frequency_topic FREQUENCY_TOPIC] [--split_duration SPLIT_DURATION] input_directory

positional arguments:
  input_directory       Path to the local rosbag folder, which needs to contain the [metadata.yaml] metadata file

options:
  -h, --help            show this help message and exit
  -d OUTPUT_DIRECTORY, --output_directory OUTPUT_DIRECTORY
                        Output path for the results, default is the current working directory/output
  -f OUTPUT_FILENAME, --output_filename OUTPUT_FILENAME
                        Name of the output html result file, default is [result.html]
  --resource_usage_topic RESOURCE_USAGE_TOPIC
                        Topic name containing resource usage information, default is [/diagnostics]
  --frequency_topic FREQUENCY_TOPIC
                        Topic name containing frame rate information, default is [/diagnostics]
  --split_duration SPLIT_DURATION
                        String controlling the maximum duration of each visual result, examples: 1h30m (one hour thirty minutes) | 1d (one day) | 1m30s (one minute thirty seconds)
                        Default is empty, indicating no splitting
```

Suppose you recorded a bag at `/root/rosbag2_2025_01_14-07_26_38`, you can output the visual report to `${PWD}/output/result.html` by executing the following command:

```bash
ros2 run rs_monitor parser.py /root/rosbag2_2025_01_14-07_26_38
```

If the recording time is long, you can control the duration of data included in each report through the `--split_duration` parameter. The split result file name format is `${name}_{index}.html`, such as `result_0.html`.

```bash
# Split by one hour
ros2 run rs_monitor parser.py /root/rosbag2_2025_01_14-07_26_38 --split_duration=1h
```

## 5. Function Description

### 5.1 Configuration File

| The path to the configuration file obtained after compilation and installation is located at: [${workspace}/install/rs_monitor/share/rs_monitor/config/config.yaml]

The settings in the configuration file mainly include the following two parts:

1. **ros2 process detection rules (`ros2_process_manager`)**: Defines how to identify and manage `ros2` related processes.
2. **Configuration of each monitoring module (`*_monitor`)**: Includes specific parameter settings for resource monitoring, frame rate monitoring, and timestamp offset monitoring modules.

Here are the detailed explanations of the configuration items for each part:

#### 5.1.1 ros2_process_manager

- `refresh_interval_ms`: The interval for refreshing the list of processes to be monitored, in milliseconds. The default value is `10000`, which refreshes every 10 seconds.
- `ros2_process_identifiers`: A list of identifiers used to recognize `ros2` processes.
    - In `linux` systems, it is used as a substring to match the `cmdline` of each process.
    - The difference with `monitored_processes` is that `monitored_processes` is more precise and specifies the process name in the monitoring results.
- `ros2_process_comm`: Names of commands or executables related to `ros2`.
    - In `linux` systems, it matches the `comm` of each process.
- `monitored_processes`: Optional list of processes to be monitored, which can match specific processes by keyword. In `linux` systems, it is used as a substring to match the `cmdline` of processes. Example configuration is as follows:
```yaml
monitored_processes:
- keyword: "ros2 echo /"    # cmdline keyword
  name: "echo"              # Name of the process in the monitoring results
```

#### 5.1.2 *_monitor

Below are some common configuration fields for monitoring modules:

* `enable`: Boolean value, whether to enable the monitoring module.
* `exec_interval_ms`: The interval for publishing monitoring results, in *milliseconds*, which also affects the internal calculation frequency of the monitoring module.
* `publish_topic_name`: The topic name to which the monitoring results are published. When set to `~` (None), topic publishing will not be performed.

#### 5.1.3 frequency_monitor

- `topics`: List of `topics` whose frame rates need to be monitored. Each `topic` can set a minimum frame rate limit. Example configuration is as follows:
```yaml
frequency_monitor:
  topics:
  - name: "/diagnostics"
    min_freq: 1.0
  - name: "/rslidar_points_e1"
    min_freq: 0.0
```

#### 5.1.4 timestamp_monitor

- `topics`: List of `topics` whose timestamp offsets need to be monitored. Each `topic` can set a maximum time difference limit. Example configuration is as follows:
```yaml
timestamp_monitor:
  topics:
  - name: "/diagnostics"
    max_difference_ms: 1000
```

### 5.2 Resource Monitoring

For each process that needs to be monitored, `resource_monitor` will count the following three resource usage rates:

* CPU
    * In linux systems, the CPU usage of the process includes `utime + stime + cutime + cstime`. Because it includes the statistics of sub-processes, a significant increase may appear when the **sub-process exits**.
* Memory
    * In linux systems, `RSS` is used as the statistical value of process memory usage.
* IO
    * Only includes the bytes directly read and written to the device, excluding the bytes read and written in the cache.

### 5.3 Timestamp Offset Monitoring

The results of timestamp offset monitoring are output to log files in the following format:

```bash
The difference between system timestamp and message timestamp exceeds the threshold [{}ms], topic: [{}]
```

### 5.4 Visual Reports

**Tips:** In the visual report, the legends of the line charts can be scrolled using the mouse wheel, and ranges can be selected by holding the `shift` key.

## 6. FAQ

[Contact with developer](mailto:william.ren@robosense.cn)
