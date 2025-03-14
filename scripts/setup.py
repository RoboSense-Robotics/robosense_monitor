#!/usr/bin/env python3
import subprocess


def install_packages() -> None:
    packages = [
        "pyecharts==2.0.7",
        "loguru==0.7.3",
        "pyyaml==6.0.2",
        "numpy==1.21.5",
        "pycryptodomex",
        "python-gnupg",
    ]
    for package in packages:
        subprocess.run(
            [f"python3 -m pip install {package}"],
            check=True,
            shell=True,
        )


if __name__ == "__main__":
    install_packages()
