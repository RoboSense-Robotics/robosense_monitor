import re
from typing import Dict, List, Tuple, TypedDict

DiagnosticsData = Tuple[List[float], List[float]]
DiagnosticsDataCollection = Dict[str, DiagnosticsData]


class SystemAndNodeResourceUsage(TypedDict):
    node_cpu: DiagnosticsDataCollection
    node_mem: DiagnosticsDataCollection
    system_cpu: DiagnosticsDataCollection
    system_mem: DiagnosticsDataCollection
    node_io_read: DiagnosticsDataCollection
    node_io_write: DiagnosticsDataCollection


class NodeUsage(object):
    cpu: float
    mem: float
    io_read: float
    io_write: float
    pid: int
    nodes: List[str]

    @staticmethod
    def parse(data: str, splitter: str = "|") -> "NodeUsage":
        parts = data.split(splitter)
        parts = [part.strip() for part in parts if part]

        usage = NodeUsage()
        usage.cpu = 0.0
        usage.mem = 0.0
        usage.pid = 0
        usage.nodes = []

        CPU_PREFIX = "CPU: "
        MEM_PREFIX = "Memory: "
        PID_PREFIX = "PID: "
        NODES_PREFIX = "Nodes: "
        IO_READ_PREFIX = "Read: ("
        IO_WRITE_PREFIX = "Write: ("

        if len(parts) > 0 and parts[0].startswith(CPU_PREFIX):
            try:
                usage.cpu = float(parts[0][len(CPU_PREFIX) : -1])
            except ValueError:
                pass

        if len(parts) > 1 and parts[1].startswith(MEM_PREFIX):
            left_bracket_idx = parts[1].find("(")
            percentage_idx = parts[1].rfind("%")

            try:
                usage.mem = float(parts[1][left_bracket_idx + 1 : percentage_idx])
            except ValueError:
                pass

        if len(parts) > 2 and parts[2].startswith(PID_PREFIX):
            try:
                usage.pid = int(parts[2][len(PID_PREFIX) :])
            except ValueError:
                pass

        if len(parts) > 3 and parts[3].startswith(IO_READ_PREFIX):
            suffix_idx = parts[3].find("MB/s")

            try:
                usage.io_read = float(parts[3][len(IO_READ_PREFIX) : suffix_idx])
            except ValueError:
                pass

        if len(parts) > 4 and parts[4].startswith(IO_WRITE_PREFIX):
            suffix_idx = parts[4].find("MB/s")

            try:
                usage.io_write = float(parts[4][len(IO_WRITE_PREFIX) : suffix_idx])
            except ValueError:
                pass

        if len(parts) > 5 and parts[5].startswith(NODES_PREFIX):
            results = re.findall("<.*?>", parts[5])
            if results:
                for result in results:
                    usage.nodes.append(str(result)[1:-1])

        return usage
