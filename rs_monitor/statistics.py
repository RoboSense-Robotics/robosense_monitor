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

from typing import TypedDict

from rs_monitor.models import DiagnosticsData


class Statistics(TypedDict):
    Mean: float
    Min: float
    Median: float
    P95: float
    P99: float
    Max: float


def calculate_statistics(data: DiagnosticsData, precision: int = 2) -> Statistics:
    _, values = data
    amount = len(values)

    sorted_values = sorted(values)

    max = sorted_values[-1] if amount else float("nan")
    min = sorted_values[0] if amount else float("nan")
    mean = (sum(sorted_values) / amount) if amount else float("nan")
    median = sorted_values[int(amount * 0.5)] if amount else float("nan")
    p99 = sorted_values[int(amount * 0.99)] if amount else float("nan")
    p95 = sorted_values[int(amount * 0.95)] if amount else float("nan")

    return Statistics(
        Mean=round(mean, precision),
        Median=round(median, precision),
        Min=round(min, precision),
        Max=round(max, precision),
        P95=round(p95, precision),
        P99=round(p99, precision),
    )
