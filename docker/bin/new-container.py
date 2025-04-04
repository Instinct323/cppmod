#!/usr/bin/python3

import argparse
import json
import os
from pathlib import Path


def execute(cmd, check=True):
    ret = print(cmd) or os.system(cmd)
    if check and ret: raise OSError(f"Fail to execute")


class DockerCmd(dict):
    registered = {"file", "image", "name", "port", "volume", "env", "workdir", "options"}

    def __init__(self, cfg: Path):
        cfg = cfg.absolute()
        os.chdir(cfg.parent)
        with cfg.open() as f:
            super().__init__(json.load(f))
        assert set(self.keys()).issubset(self.registered), f"Invalid keys: {set(self.keys()) - self.registered}"
        # build Dockerfile
        file = Path(self.get("file"))
        if file:
            assert file.is_file(), f"File not found: {file.absolute()}"
            execute(f"docker build -t {self['image']} -f {file} .")
        # name
        self.setdefault("name", cfg.stem)
        # port
        port = str(self.get("port", "")).replace(" ", "").split(",")
        self["port"] = []
        for p in filter(None, port):
            if "-" in p:
                s, e = map(int, p.split("-"))
                self["port"].extend(range(s, e + 1))
            else:
                self["port"].append(int(p))
        # volume
        self["volume"] = [f"{k}:{v}" for k, v in self.get("volume", {}).items()]
        self["volume"].sort()
        # other
        # self["cpus"] = float(self.get("cpus", os.cpu_count()))
        # self["gpus"] = self.get("gpus", "all")

    def export(self) -> str:
        cmd = f"docker run "
        # other
        if self.get("name"): cmd += f"--name={self['name']} "
        if self.get("workdir"): cmd += f"--workdir={self['workdir']} "
        # cmd += f"--cpus {self['cpus']} --gpus {self['gpus']} "
        if self.get("options"): cmd += f"{self['options']} "
        # port: <1st>:22
        ports = self["port"]
        if ports:
            cmd += f"-p {ports[0]}:22 "
            ports = ports[1:]
        for p in ports:
            cmd += f"-p {p}:{p} "
        # volume
        for v in self["volume"]: cmd += f"-v {v} "
        # env
        for k, v in self.get("env", {}).items(): cmd += f"-e {k}={v} "
        # image
        cmd += f"{self['image']}"
        return cmd


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("json", type=str, help="Docker container configuration file")
    args = parser.parse_args()

    # D:\Software\Anaconda3\envs\torch2\python bin\new-container.py my-jammy.json
    dcmd = DockerCmd(Path(args.json))
    execute(dcmd.export())
