#!/usr/bin/env python3

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

import os
from argparse import ArgumentParser
from typing import cast

from loguru import logger

from rs_monitor.parser import StatisticsResultParser


class Args(object):
    input_directory: str
    output_directory: str
    output_filename: str
    split_duration: str

    frequency_topic: str
    resource_usage_topic: str


def parse_arguments() -> Args:
    parser = ArgumentParser()
    parser.add_argument("input_directory", nargs=1, help="Path of input bag directory")
    parser.add_argument(
        "-d",
        "--output_directory",
        type=str,
        required=False,
        default=os.path.join(os.getcwd(), "output"),
    )
    parser.add_argument(
        "-f",
        "--output_filename",
        type=str,
        required=False,
        default="result.html",
        help="File name of output html charts",
    )
    parser.add_argument(
        "--resource_usage_topic",
        type=str,
        default="/diagnostics",
        help="Topic name containing resource usage records",
    )
    parser.add_argument(
        "--frequency_topic",
        type=str,
        default="/diagnostics",
        help="Topic name containing frequency records",
    )
    parser.add_argument(
        "--split_duration",
        type=str,
        default="",
        help="Time interval string for splitting the generated html charts, examples: 1h30m | 1d | 1m30s",
    )

    args = parser.parse_args()
    args.input_directory = args.input_directory[0]

    return cast(Args, args)


def main() -> None:
    args = parse_arguments()

    if not os.path.exists(args.output_directory):
        os.makedirs(args.output_directory, exist_ok=True)

    parser = StatisticsResultParser(
        args.input_directory,
        args.resource_usage_topic,
        args.frequency_topic,
        args.split_duration,
    )

    parser.parse(os.path.join(args.output_directory, args.output_filename))


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        logger.opt(exception=e).error("Failed to parse resource usage: {}".format(e))
