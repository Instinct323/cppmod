#!/usr/bin/env python3

import argparse
import json
import os


def execute(cmd, check=True):
    ret = print("\033[32m\033[1m" + cmd + "\033[0m") or os.system(cmd)
    if check and ret: raise OSError(f"Fail to execute: {cmd}")
    return ret


class UserCfg(dict):
    registered = {"name", "passwd", "email"}

    def __init__(self, file):
        with open(file, "r") as f:
            super().__init__(json.load(f))

    def generate(self):
        # Linux
        if os.name == "posix":
            execute(f"useradd -m {self['name']}")
            execute(f"echo {self['name']}:{self['passwd']} | chpasswd")
        # git
        execute("git config --global credential.helper store")
        execute(f"git config --global user.name {self['name']}")
        execute(f"git config --global user.email {self['email']}")
        execute(f"ssh-keygen -t rsa -C {self['email']} -N ''")


if __name__ == '__main__':
    print("\033[33m\033[1m" + "WARNING: This script should be executed in the container!" + "\033[0m")

    parser = argparse.ArgumentParser()
    parser.add_argument("json", type=str, help="Docker container configuration file")
    args = parser.parse_args()

    # python3 bin\new-user.py user-tongzj.json
    ucfg = UserCfg(args.json)
    ucfg.generate()
