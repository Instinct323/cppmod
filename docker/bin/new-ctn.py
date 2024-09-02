import argparse
import json
import os
from pathlib import Path


def execute(cmd, check=True):
    ret = print(cmd) or os.system(cmd)
    if check and ret: raise OSError(f"Fail to execute: {cmd}")


class DockerCmd(dict):
    registered = {"image", "name", "port", "volume"}

    def __init__(self, file):
        with open(file, "r") as f:
            super().__init__(json.load(f))
        assert set(self.keys()).issubset(self.registered), f"Invalid keys: {set(self.keys()) - self.registered}"
        # name
        self.setdefault("name", Path(file).stem)
        # port
        port = str(self.get("port", "")).replace(" ", "").split(",")
        self["port"] = []
        for p in port:
            if "-" in p:
                s, e = map(int, p.split("-"))
                self["port"].extend(range(s, e + 1))
            else:
                self["port"].append(int(p))
        self["port"].sort()
        # volume
        self["volume"] = [f"{k}:{v}" for k, v in self.get("volume", {}).items()]
        self["volume"].sort()

    def export(self) -> str:
        cmd = f"docker run -d "
        if self.get("name"): cmd += f"--name {self['name']} "
        # port: <1st>:22
        ports = self["port"]
        if ports:
            cmd += f"-p {ports[0]}:22 "
            ports = ports[1:]
        for p in ports:
            cmd += f"-p {p}:{p} "
        # volume
        for v in self["volume"]: cmd += f"-v {v} "
        # image
        cmd += f"{self['image']}"
        return cmd


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("json", type=str, help="Docker container configuration file")
    args = parser.parse_args()

    # D:\Software\Anaconda3\envs\torch2\python.exe zjdocker.py D:\Workbench\cppmod\docker\lab-ubuntu.json
    dcmd = DockerCmd(args.json)
    execute(dcmd.export())
