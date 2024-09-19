#!/usr/bin/env python3

import argparse
import json
import os


def execute(cmd, check=True):
    ret = print(cmd) or os.system(cmd)
    if check and ret: raise OSError(f"Fail to execute: {cmd}")


class UserCfg(dict):
    registered = {"name", "passwd", "email"}

    def __init__(self, file):
        with open(file, "r") as f:
            super().__init__(json.load(f))

    def execute(self):
        execute(f"useradd -m {self['name']}")
        execute(f"echo {self['name']}:{self['passwd']} | chpasswd")
        # git
        execute(f"git config --global user.name {self['name']}")
        execute(f"git config --global user.email {self['email']}")


if __name__ == '__main__':
    print("WARNING: This script should be executed in the container!")

    parser = argparse.ArgumentParser()
    parser.add_argument("json", type=str, help="Docker container configuration file")
    args = parser.parse_args()

    # python3 bin\new-user.py user-tongzj.json
    ucfg = UserCfg(args.json)
    ucfg.execute()
