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

import hashlib
from datetime import datetime
from typing import Dict, List, Set, Tuple

import pyecharts.options as opts
from loguru import logger
from pyecharts.charts import Grid, Line, Tab
from pyecharts.charts.base import Base
from pyecharts.components import Table

from rs_monitor.models import DiagnosticsDataCollection, SystemAndNodeResourceUsage
from rs_monitor.statistics import Statistics, calculate_statistics

StatisticsRow = Tuple[str, float, float, float, float, float, float]


class ChartPainter(object):
    def __init__(self) -> None:
        self.explicit_node_set: Set[str] = set()

    def add_explicit_node(self, node: str) -> None:
        self.explicit_node_set.add(node)

    @staticmethod
    def generate_hex_color_from_string(
        input_string: str, max_luminance: int = 200
    ) -> str:
        hash_object = hashlib.md5(input_string.encode())
        hash_hex = hash_object.hexdigest()

        # 将哈希值分割成 RGB 组件
        r = int(hash_hex[:2], 16)
        g = int(hash_hex[2:4], 16)
        b = int(hash_hex[4:6], 16)

        # 调整颜色使其亮度低于 max_luminance
        while 0.299 * r + 0.587 * g + 0.114 * b > max_luminance:
            r = int(r * 0.95)
            g = int(g * 0.95)
            b = int(b * 0.95)

        return f"#{r:02X}{g:02X}{b:02X}"

    @staticmethod
    def scroll_by_wheel_js_function(chart_id: str) -> str:
        return """
function createClearButton(chartId, buttonId) {
    var button = document.createElement("button");
    button.innerHTML = "Clear Selection";
    button.id = buttonId;
    button.style.position = "absolute";
    button.style.right = "10px";
    button.style.top = "-30px";
    button.style.zIndex = "1000";
    button.style.fontSize = "14px";
    button.style.color = "white";
    button.style.backgroundColor = "#00BFFF";
    button.style.border = "none";
    button.style.borderRadius = "4px";
    button.style.cursor = "pointer";
    button.style.boxShadow = "0 4px 6px rgba(0, 0, 0, 0.1)";
    button.style.transition = "background-color 0.3s ease";

    button.onmouseover = function() {
        button.style.backgroundColor = "#1E90FF";
    };
    button.onmouseout = function() {
        button.style.backgroundColor = "#00BFFF";
    };
    button.onclick = function() {
        var chart = echarts.getInstanceByDom(document.getElementById(chartId));
        chart.dispatchAction({
            type: "legendAllSelect"
        });
        chart.dispatchAction({
            type: "legendInverseSelect"
        });
    };
    document.getElementById(chartId).appendChild(button);
}
createClearButton("<raw_chart_id>", "<raw_chart_id>_clearBtn");

// window.addEventListener("resize", function() {
//     <chart_id>.resize();
// });
<chart_id>.getZr().on("mousewheel", function(event) {
    if (event.target) {
        return;
    }
    var legendComponent = null;
    for (var component in <chart_id>._componentsMap) {
        if (component.indexOf("legend.scroll") !== -1) {
            legendComponent = <chart_id>._componentsMap[component];
        }
    }
    if (legendComponent === null) {
        return;
    }
    const pageInfo = legendComponent._getPageInfo(legendComponent.__model);
    var nextIndex = null;

    if (event.wheelDelta > 0) {
        nextIndex = pageInfo.pagePrevDataIndex;
    } else if (event.wheelDelta < 0) {
        nextIndex = pageInfo.pageNextDataIndex;
    }
    if (nextIndex !== null) {
        <chart_id>.dispatchAction({
            type: "legendScroll",
            scrollDataIndex: nextIndex,
            legendId: legendComponent.__model.id,
        });
    }
});

var <chart_id>_startIndex = -1;
<chart_id>.on("legendselectchanged", function(params) {
    var legends = <chart_id>.getOption().legend[0].data
    if (params.selected[params.name] && event.shiftKey && <chart_id>_startIndex !== -1) {
        var endIndex = legends.indexOf(params.name);
        if (endIndex > <chart_id>_startIndex) {
            for (var i = <chart_id>_startIndex; i <= endIndex; i++) {
                <chart_id>.dispatchAction({
                    type: "legendSelect",
                    name: legends[i]
                });
            }
        } else {
            for (var i = endIndex; i <= <chart_id>_startIndex; i++) {
                <chart_id>.dispatchAction({
                    type: "legendSelect",
                    name: legends[i]
                });
            }
        }
    } else if (params.selected[params.name]) {
        <chart_id>_startIndex = legends.indexOf(params.name);
    } else {
        <chart_id>_startIndex = -1;
    }
});
""".replace(
            "<chart_id>", f"chart_{chart_id}"
        ).replace(
            "<raw_chart_id>", chart_id
        )

    @staticmethod
    def is_system_component(name: str) -> bool:
        return name == "System" or name.startswith("System ")

    def node_sort_key(self, node: str) -> Tuple[bool, bool, int, str]:
        if ChartPainter.is_system_component(node):
            return (False, True, len(node), node)

        else:
            return (
                True,
                node not in self.explicit_node_set,
                len(node),
                node,
            )

    def node_sort_key2(
        self, node: str, mean_value: float
    ) -> Tuple[bool, bool, float, int, str]:
        if ChartPainter.is_system_component(node):
            return (False, True, 0, len(node), node)

        else:
            return (
                True,
                node not in self.explicit_node_set,
                -mean_value,
                len(node),
                node,
            )

    def plot_cpu_usage_chart(
        self,
        system_cpu_usage: DiagnosticsDataCollection,
        node_cpu_usage: DiagnosticsDataCollection,
    ) -> Line:
        selected_map: Dict[str, bool] = {}
        for name in system_cpu_usage:
            if name == "System":
                selected_map[name] = True
            else:
                selected_map[name] = False
        for name in node_cpu_usage:
            selected_map[name] = False

        cpu_chart = Line(
            init_opts=opts.InitOpts(
                chart_id="usage",
                animation_opts=opts.AnimationOpts(
                    animation_duration=300,
                ),
            )
        ).set_global_opts(
            title_opts=opts.TitleOpts(
                title="CPU Usage (%)",
                pos_left="left",
                pos_top="2%",
            ),
            xaxis_opts=opts.AxisOpts(
                type_="time", axistick_opts=opts.AxisTickOpts(is_align_with_label=True)
            ),
            yaxis_opts=opts.AxisOpts(
                is_scale=True,
                axislabel_opts=opts.LabelOpts(formatter="{value}%"),
            ),
            tooltip_opts=opts.TooltipOpts(trigger="axis", axis_pointer_type="cross"),
            legend_opts=opts.LegendOpts(
                type_="scroll",
                selected_mode="multiple",
                orient="vertical",
                legend_icon="pin",
                selector=True,
                pos_right="right",
                item_height=16,
                textstyle_opts=opts.TextStyleOpts(
                    font_size=16,
                ),
                selected_map=selected_map,
                is_page_animation=True,
                page_animation_duration_update=300,
                page_button_position="end",
                page_icon_size=15,
                padding=0,
            ),
            datazoom_opts=[
                opts.DataZoomOpts(
                    type_="inside", range_start=0, range_end=100, xaxis_index=[0, 1]
                ),
            ],
        )

        for usage in [system_cpu_usage, node_cpu_usage]:
            for name in sorted(usage.keys(), key=lambda x: self.node_sort_key(x)):
                timestamps, values = usage[name]
                temp_chart = (
                    Line()
                    .add_xaxis([datetime.fromtimestamp(ts) for ts in timestamps])
                    .add_yaxis(
                        series_name=name,
                        y_axis=values,
                        is_connect_nones=False,
                        is_symbol_show=False,
                        markline_opts=opts.MarkLineOpts(
                            data=[
                                opts.MarkLineItem(
                                    type_="average", name="Mean CPU Usage"
                                )
                            ]
                        ),
                        color=ChartPainter.generate_hex_color_from_string(name),
                    )
                    .set_series_opts(label_opts=opts.LabelOpts(is_show=False))
                )
                cpu_chart.overlap(temp_chart)

        return cpu_chart

    def plot_mem_usage_chart(
        self,
        system_mem_usage: DiagnosticsDataCollection,
        node_mem_usage: DiagnosticsDataCollection,
    ) -> Line:
        selected_map: Dict[str, bool] = {}
        for name in system_mem_usage:
            selected_map[name] = True
        for name in node_mem_usage:
            selected_map[name] = False

        mem_chart = Line(
            init_opts=opts.InitOpts(
                animation_opts=opts.AnimationOpts(
                    animation_duration=300,
                )
            )
        ).set_global_opts(
            title_opts=opts.TitleOpts(
                title="Memory Usage (%)",
                pos_top="50%",
                pos_left="left",
            ),
            xaxis_opts=opts.AxisOpts(
                type_="time", axistick_opts=opts.AxisTickOpts(is_align_with_label=True)
            ),
            yaxis_opts=opts.AxisOpts(
                is_scale=True,
                axislabel_opts=opts.LabelOpts(formatter="{value}%"),
            ),
            tooltip_opts=opts.TooltipOpts(trigger="axis", axis_pointer_type="cross"),
            legend_opts=opts.LegendOpts(is_show=False),
        )

        for usage in [system_mem_usage, node_mem_usage]:
            for name in sorted(usage.keys(), key=lambda x: self.node_sort_key(x)):
                timestamps, values = usage[name]
                temp_chart = (
                    Line()
                    .add_xaxis([datetime.fromtimestamp(ts) for ts in timestamps])
                    .add_yaxis(
                        series_name=name,
                        y_axis=values,
                        is_connect_nones=False,
                        is_symbol_show=False,
                        markline_opts=opts.MarkLineOpts(
                            data=[
                                opts.MarkLineItem(
                                    type_="average", name="Mean Memory Usage"
                                )
                            ]
                        ),
                        color=ChartPainter.generate_hex_color_from_string(name),
                    )
                    .set_series_opts(label_opts=opts.LabelOpts(is_show=False))
                )
                mem_chart.overlap(temp_chart)

        return mem_chart

    def plot_io_chart(self, io_usage: DiagnosticsDataCollection) -> Grid:
        logger.info(f"Generating line charts of device io usage...")
        return self.plot_simple_chart(
            io_usage,
            "io_usage",
            "Device IO Usage (MB/sec)",
            "{value}MB/s",
            "Mean IO Usage",
        )

    def plot_frequency_chart(self, frequency: DiagnosticsDataCollection) -> Grid:
        logger.info("Generating line charts of message frequency...")
        return self.plot_simple_chart(
            frequency, "freq", "Channel Frequency (Hz)", "{value}Hz", "Mean Frequency"
        )

    def plot_simple_chart(
        self,
        data_collection: DiagnosticsDataCollection,
        chart_id: str,
        title: str,
        formatter: str,
        markerline_hint: str,
    ) -> Grid:

        chart = Line(
            init_opts=opts.InitOpts(
                chart_id=chart_id,
                animation_opts=opts.AnimationOpts(
                    animation_duration=300,
                ),
            )
        ).set_global_opts(
            title_opts=opts.TitleOpts(
                title=title,
                pos_left="left",
                pos_top="2%",
            ),
            xaxis_opts=opts.AxisOpts(
                type_="time",
                axistick_opts=opts.AxisTickOpts(is_align_with_label=True),
            ),
            yaxis_opts=opts.AxisOpts(
                is_scale=True,
                axislabel_opts=opts.LabelOpts(formatter=formatter),
            ),
            tooltip_opts=opts.TooltipOpts(trigger="axis", axis_pointer_type="cross"),
            legend_opts=opts.LegendOpts(
                type_="scroll",
                selected_mode="multiple",
                orient="vertical",
                legend_icon="pin",
                selector=True,
                pos_right="right",
                item_height=16,
                textstyle_opts=opts.TextStyleOpts(
                    font_size=16,
                ),
                is_page_animation=True,
                page_animation_duration_update=300,
                page_button_position="end",
                page_icon_size=15,
                padding=0,
            ),
            datazoom_opts=[
                opts.DataZoomOpts(type_="inside", range_start=0, range_end=100),
            ],
        )

        for key in sorted(data_collection.keys(), key=lambda x: self.node_sort_key(x)):
            timestamps, values = data_collection[key]
            temp_chart = (
                Line()
                .add_xaxis([datetime.fromtimestamp(ts) for ts in timestamps])
                .add_yaxis(
                    series_name=key,
                    y_axis=values,
                    is_connect_nones=False,
                    is_symbol_show=False,
                    markline_opts=opts.MarkLineOpts(
                        data=[opts.MarkLineItem(type_="average", name=markerline_hint)]
                    ),
                    color=ChartPainter.generate_hex_color_from_string(key),
                )
                .set_series_opts(label_opts=opts.LabelOpts(is_show=False))
            )
            chart.overlap(temp_chart)

        grid = (
            Grid(
                init_opts=opts.InitOpts(
                    width="95vw",
                    height="95vh",
                    renderer="svg",
                )
            )
            .add(
                chart,
                grid_opts=opts.GridOpts(
                    pos_right="20%",
                ),
            )
            .add_js_funcs(ChartPainter.scroll_by_wheel_js_function(chart_id))
        )

        return grid

    def plot_usage_chart(self, usage: SystemAndNodeResourceUsage) -> Grid:
        logger.info(f"Generating line charts of cpu/memory usage...")

        cpu_chart = self.plot_cpu_usage_chart(usage["system_cpu"], usage["node_cpu"])
        mem_chart = self.plot_mem_usage_chart(usage["system_mem"], usage["node_mem"])

        usage_grid = (
            Grid(
                init_opts=opts.InitOpts(
                    width="95vw",
                    height="95vh",
                    renderer="svg",
                )
            )
            .add(
                cpu_chart,
                grid_opts=opts.GridOpts(
                    pos_right="20%",
                    pos_bottom="52.5%",
                ),
            )
            .add(
                mem_chart,
                grid_opts=opts.GridOpts(
                    pos_right="20%",
                    pos_top="52.5%",
                ),
            )
            .add_js_funcs(ChartPainter.scroll_by_wheel_js_function("usage"))
        )

        return usage_grid

    def plot_statistics_table(
        self,
        title: str,
        colum_suffix: str,
        *data_collections: DiagnosticsDataCollection,
    ) -> Table:
        statistics_dict: Dict[str, Statistics] = {}
        for data_collection in data_collections:
            for key, value in data_collection.items():
                statistics_dict[key] = calculate_statistics(value)

        table = Table()

        rows: List[StatisticsRow] = []
        headers: List[str] = ["Name", "Mean", "Min", "Median", "P95", "P99", "Max"]
        if colum_suffix:
            for i in range(1, len(headers)):
                headers[i] = f"{headers[i]} {colum_suffix}"

        for name, statistics in statistics_dict.items():
            row: StatisticsRow = (
                name,
                statistics["Mean"],
                statistics["Min"],
                statistics["Median"],
                statistics["P95"],
                statistics["P99"],
                statistics["Max"],
            )
            rows.append(row)

        rows.sort(key=lambda row: self.node_sort_key2(row[0], row[1]))

        table.add(headers=headers, rows=rows)
        table.set_global_opts(title_opts=opts.ComponentTitleOpts(title=title))

        return table

    def merge_charts(self, titles: List[str], charts: List[Base]) -> Tab:
        tab = Tab(page_title="Performance Analysis")
        for i in range(0, min(len(titles), len(charts))):
            tab.add(charts[i], titles[i])

        return tab
