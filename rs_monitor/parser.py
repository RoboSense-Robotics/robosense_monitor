# Copyright 2025 RoboSense Technology Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import os.path as osp
import re
from typing import Iterable, Tuple

from rs_monitor.common import IS_ROS1, IS_ROS2

if IS_ROS1:
    import rosbag
elif IS_ROS2:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

import yaml
from diagnostic_msgs.msg import DiagnosticArray

from rs_monitor.models import (
    DiagnosticsDataCollection,
    NodeUsage,
    SystemAndNodeResourceUsage,
)
from rs_monitor.painter import ChartPainter


class StatisticsResultParser(object):
    FREQUENCY_SUFFIX = "Topic Frequencies"
    NODE_USAGE_SUFFIX = "Node Resource Usage"
    SYSTEM_USAGE_SUFFIX = "System Resource Usage"

    def __init__(
        self,
        path: str,
        resource_usage_topic: str,
        frequency_topic: str,
        split_duration: str,
    ) -> None:
        self.bag_path: str = path
        self.frequency_topic: str = frequency_topic
        self.resource_usage_topic: str = resource_usage_topic
        self.split_duration_sec = (
            0
            if split_duration == ""
            else StatisticsResultParser.parse_duration(split_duration)
        )
        self.total_memory_mb: float = 0.0
        self.painter = ChartPainter()

    def parse_resource_usage_messages(self) -> Iterable[SystemAndNodeResourceUsage]:
        node_cpu_usage: DiagnosticsDataCollection = {}
        node_mem_usage: DiagnosticsDataCollection = {}
        system_cpu_usage: DiagnosticsDataCollection = {}
        system_mem_usage: DiagnosticsDataCollection = {}
        node_io_read: DiagnosticsDataCollection = {}
        node_io_write: DiagnosticsDataCollection = {}

        def process_message(
            msg: DiagnosticArray,
            node_cpu_usage: DiagnosticsDataCollection,
            node_mem_usage: DiagnosticsDataCollection,
            system_cpu_usage: DiagnosticsDataCollection,
            system_mem_usage: DiagnosticsDataCollection,
            node_io_read: DiagnosticsDataCollection,
            node_io_write: DiagnosticsDataCollection,
        ) -> Iterable[SystemAndNodeResourceUsage]:
            begin_timestamp_sec: float = 0.0

            # calculate timestamp
            if IS_ROS1:
                timestamp_sec = msg.header.stamp.secs + msg.header.stamp.nsecs / 1e9
            elif IS_ROS2:
                timestamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

            # split by duration
            if self.split_duration_sec > 0:
                if begin_timestamp_sec == 0.0:
                    begin_timestamp_sec = timestamp_sec
                else:
                    if timestamp_sec - begin_timestamp_sec > self.split_duration_sec:
                        begin_timestamp_sec = timestamp_sec
                        yield {
                            "node_cpu": node_cpu_usage,
                            "node_mem": node_mem_usage,
                            "system_cpu": system_cpu_usage,
                            "system_mem": system_mem_usage,
                            "node_io_read": node_io_read,
                            "node_io_write": node_io_write,
                        }
                        node_cpu_usage.clear()
                        node_mem_usage.clear()
                        system_cpu_usage.clear()
                        system_mem_usage.clear()
                        node_io_read.clear()
                        node_io_write.clear()

            # append resource usage data
            for status in msg.status:
                if status.name.endswith(self.NODE_USAGE_SUFFIX):
                    for item in status.values:
                        name = item.key
                        data = NodeUsage.parse(item.value)

                        # for sorting nodes
                        if data.nodes:
                            self.painter.add_explicit_node(name)

                        if name not in node_cpu_usage:
                            node_cpu_usage[name] = ([], [])
                            node_mem_usage[name] = ([], [])
                            node_io_read[name] = ([], [])
                            node_io_write[name] = ([], [])

                        if not math.isnan(data.cpu):
                            cpu_ref = node_cpu_usage[name]
                            cpu_ref[0].append(timestamp_sec)
                            cpu_ref[1].append(data.cpu)

                        if not math.isnan(data.mem):
                            mem_ref = node_mem_usage[name]
                            mem_ref[0].append(timestamp_sec)
                            mem_ref[1].append(data.mem)

                        if not math.isnan(data.io_read):
                            io_read_ref = node_io_read[name]
                            io_read_ref[0].append(timestamp_sec)
                            io_read_ref[1].append(data.io_read)

                        if not math.isnan(data.io_write):
                            io_write_ref = node_io_write[name]
                            io_write_ref[0].append(timestamp_sec)
                            io_write_ref[1].append(data.io_write)

                elif status.name.endswith(self.SYSTEM_USAGE_SUFFIX):
                    for item in status.values:
                        usage = 0.0
                        try:
                            if item.key == "Total CPU Usage":
                                name = "System"
                                target = system_cpu_usage
                                usage = float(item.value[:-1])

                            elif item.key == "Memory Usage":
                                name = "System"
                                target = system_mem_usage

                                left_bracket_idx = item.value.find("(")
                                percentage_idx = item.value.rfind("%")
                                if left_bracket_idx and percentage_idx:
                                    usage = float(
                                        item.value[
                                            left_bracket_idx + 1 : percentage_idx
                                        ]
                                    )

                                if not self.total_memory_mb:
                                    slash_idx = item.value.find("/")
                                    gb_idx = item.value.rfind("GB")
                                    if slash_idx and gb_idx:
                                        self.total_memory_mb = (
                                            float(
                                                item.value[
                                                    slash_idx + 1 : gb_idx
                                                ].strip()
                                            )
                                            * 1024  # GB to MB
                                        )

                            else:
                                # processors
                                name = "System {}".format(item.key)
                                target = system_cpu_usage
                                usage = float(item.value[:-1])

                            if math.isnan(usage):
                                continue

                            if name not in target:
                                target[name] = ([], [])

                            ref = target[name]
                            ref[0].append(timestamp_sec)
                            ref[1].append(usage)

                        except ValueError:
                            pass

        if IS_ROS2:
            reader = self.open_reader(self.resource_usage_topic)

            topic_types = reader.get_all_topics_and_types()

            # Create a map for quicker lookup
            type_map = {
                topic_types[i].name: topic_types[i].type
                for i in range(len(topic_types))
            }

            storage_filter = rosbag2_py.StorageFilter(
                topics=[self.resource_usage_topic]
            )
            reader.set_filter(storage_filter)

            while reader.has_next():
                (topic, data, _) = reader.read_next()
                msg_type = get_message(type_map[topic])
                if msg_type is not DiagnosticArray:
                    continue

                try:
                    msg = deserialize_message(data, msg_type)
                except Exception:
                    msg = None

                if not isinstance(msg, DiagnosticArray):
                    continue

                yield from process_message(
                    msg,
                    node_cpu_usage,
                    node_mem_usage,
                    system_cpu_usage,
                    system_mem_usage,
                    node_io_read,
                    node_io_write,
                )

        elif IS_ROS1:
            bag = rosbag.Bag(self.bag_path)
            for _, msg, __ in bag.read_messages(topics=[self.resource_usage_topic]):
                yield from process_message(
                    msg,
                    node_cpu_usage,
                    node_mem_usage,
                    system_cpu_usage,
                    system_mem_usage,
                    node_io_read,
                    node_io_write,
                )

        if any(
            [
                node_cpu_usage,
                node_mem_usage,
                system_cpu_usage,
                system_mem_usage,
                node_io_read,
                node_io_write,
            ]
        ):
            yield {
                "node_cpu": node_cpu_usage,
                "node_mem": node_mem_usage,
                "system_cpu": system_cpu_usage,
                "system_mem": system_mem_usage,
                "node_io_read": node_io_read,
                "node_io_write": node_io_write,
            }

    def parse_frequency_messages(self) -> Iterable[DiagnosticsDataCollection]:
        frequency_data: DiagnosticsDataCollection = {}

        def process_message(
            msg: DiagnosticArray,
            frequency_data: DiagnosticsDataCollection,
        ) -> Iterable[DiagnosticsDataCollection]:
            begin_timestamp_sec: float = 0.0

            # calculate timestamp
            if IS_ROS1:
                timestamp_sec = msg.header.stamp.secs + msg.haeder.stamp.nsecs / 1e9
            elif IS_ROS2:
                timestamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

            # split by duration
            if self.split_duration_sec > 0:
                if begin_timestamp_sec == 0.0:
                    begin_timestamp_sec = timestamp_sec
                else:
                    if timestamp_sec - begin_timestamp_sec > self.split_duration_sec:
                        begin_timestamp_sec = timestamp_sec
                        yield frequency_data
                        frequency_data.clear()

            for status in msg.status:
                if status.name.endswith(self.FREQUENCY_SUFFIX):
                    for item in status.values:
                        topic_name = item.key
                        frequency = 0.0
                        try:
                            SUFFIX = " Hz"
                            if item.value.endswith(SUFFIX):
                                frequency = float(item.value[: -len(SUFFIX)])
                        except ValueError:
                            pass

                        if topic_name not in frequency_data:
                            frequency_data[topic_name] = ([], [])

                        ref = frequency_data[topic_name]
                        ref[0].append(timestamp_sec)
                        ref[1].append(frequency)

        if IS_ROS2:
            reader = self.open_reader(self.frequency_topic)

            topic_types = reader.get_all_topics_and_types()

            # Create a map for quicker lookup
            type_map = {
                topic_types[i].name: topic_types[i].type
                for i in range(len(topic_types))
            }

            storage_filter = rosbag2_py.StorageFilter(topics=[self.frequency_topic])
            reader.set_filter(storage_filter)

            while reader.has_next():
                (topic, data, _) = reader.read_next()
                msg_type = get_message(type_map[topic])
                if msg_type is not DiagnosticArray:
                    continue

                try:
                    msg = deserialize_message(data, msg_type)
                except Exception:
                    msg = None

                if not isinstance(msg, DiagnosticArray):
                    continue

                yield from process_message(msg, frequency_data)

            if IS_ROS1:
                bag = rosbag.Bag(self.bag_path)
                for _, msg, __ in bag.read_messages(topics=[self.frequency_topic]):
                    yield from process_message(msg, frequency_data)

        if frequency_data:
            yield frequency_data

    def open_reader(self, topic: str) -> "rosbag2_py.SequentialReader":
        reader = rosbag2_py.SequentialReader()
        storage_options, converter_options = StatisticsResultParser.get_rosbag_options(
            self.bag_path,
            topic,
        )
        reader.open(storage_options, converter_options)

        return reader

    def parse(self, output_file: str) -> None:
        resource_usage_gen = self.parse_resource_usage_messages()
        frequency_gen = self.parse_frequency_messages()

        part_index = 0

        resource_usage = next(resource_usage_gen, None)  # type: ignore
        frequency = next(frequency_gen, None)  # type: ignore

        while resource_usage or frequency:
            charts = []
            titles = []

            if resource_usage is not None:
                cm_usage_chart = self.painter.plot_usage_chart(resource_usage)

                cpu_table = self.painter.plot_statistics_table(
                    "CPU Usage Statistics (%)",
                    "CPU (%)",
                    resource_usage["system_cpu"],
                    resource_usage["node_cpu"],
                )

                for usage in resource_usage["system_mem"].values():
                    for i in range(0, len(usage[1])):
                        usage[1][i] *= self.total_memory_mb / 100
                for usage in resource_usage["node_mem"].values():
                    for i in range(0, len(usage[1])):
                        usage[1][i] *= self.total_memory_mb / 100

                mem_table = self.painter.plot_statistics_table(
                    "Memory Usage Statistics (MB)",
                    "Memory (MB)",
                    resource_usage["system_mem"],
                    resource_usage["node_mem"],
                )

                merged_io_usage: DiagnosticsDataCollection = {}
                for key in resource_usage["node_io_read"]:
                    merged_io_usage[f"[Read] {key}"] = resource_usage["node_io_read"][
                        key
                    ]
                    if key in resource_usage["node_io_write"]:
                        merged_io_usage[f"[Write] {key}"] = resource_usage[
                            "node_io_write"
                        ][key]

                io_table = self.painter.plot_statistics_table(
                    "IO Usage Statistics (MB/sec)",
                    "Speed (MB/sec)",
                    merged_io_usage,
                )
                io_usage_chart = self.painter.plot_io_chart(merged_io_usage)

                charts.append(cpu_table)
                titles.append("CPU Usage Statistics")
                charts.append(mem_table)
                titles.append("Memory Usage Statistics")
                charts.append(cm_usage_chart)
                titles.append("CPU/Memory Usage")
                charts.append(io_table)
                titles.append("IO Usage Statistics")
                charts.append(io_usage_chart)
                titles.append("IO Usage")

            if frequency:
                freq_table = self.painter.plot_statistics_table(
                    "Frequency Statistics (Hz)",
                    "Frequency (Hz)",
                    frequency,
                )

                freq_chart = self.painter.plot_frequency_chart(frequency)

                charts.append(freq_table)
                titles.append("Frequency Statistics")
                charts.append(freq_chart)
                titles.append("Topic Frequency")

            # render html
            if self.split_duration_sec > 0:
                base, ext = osp.splitext(output_file)
                base += f"_{part_index}"
                self.painter.merge_charts(titles, charts).render(base + ext)
                part_index += 1

            else:
                self.painter.merge_charts(titles, charts).render(output_file)

            resource_usage = next(resource_usage_gen, None)  # type: ignore
            frequency = next(frequency_gen, None)  # type: ignore

    @staticmethod
    def parse_duration(duration: str) -> int:
        """Convert duration string to seconds."""
        duration = duration.strip()
        if not duration:
            return 0

        pattern = r"(\d+)\s*(d|h|m|s)"
        matches = re.findall(pattern, duration)

        time_factors = {"s": 1, "m": 60, "h": 3600, "d": 86400}

        result = 0
        for value, unit in matches:
            if unit in time_factors:
                result += int(value) * time_factors[unit]

        return result

    @staticmethod
    def get_rosbag_options(
        path: str,
        topic: str,
    ) -> Tuple["rosbag2_py.StorageOptions", "rosbag2_py.ConverterOptions"]:
        metadata_path = osp.join(path, "metadata.yaml")
        if not osp.exists(metadata_path):
            raise FileExistsError(metadata_path)

        try:
            with open(metadata_path, "r") as f:
                metadata = yaml.safe_load(f)

            information = metadata["rosbag2_bagfile_information"]
            storage_identifier = information["storage_identifier"]

            topics = information["topics_with_message_count"]
            serialization_format = ""

            for topic_info in topics:
                topic_metadata = topic_info["topic_metadata"]
                if topic_metadata["name"] == topic:
                    serialization_format = topic_metadata["serialization_format"]

            if not serialization_format:
                raise RuntimeError(
                    "Failed to find target topic [{}] in field [topics_with_message_count]!".format(
                        topic
                    )
                )

        except yaml.YAMLError as e:
            raise RuntimeError("Failed to parse metadata in yaml format: {}".format(e))

        except KeyError as e:
            raise RuntimeError("Missing required field: {}".format(e))

        storage_options = rosbag2_py.StorageOptions(
            uri=path, storage_id=storage_identifier
        )

        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format=serialization_format,
            output_serialization_format=serialization_format,
        )

        return storage_options, converter_options
