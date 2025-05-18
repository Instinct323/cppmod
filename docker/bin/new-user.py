#!/usr/bin/python3

import argparse
import json
import os

from pathlib import Path


def execute(cmd, check=True):
    ret = print("\033[32m\033[1m" + cmd + "\033[0m") or os.system(cmd)
    if check and ret: raise OSError(f"Fail to execute")
    return ret


class UserCfg(dict):
    registered = {"name", "passwd", "email"}

    def __init__(self, file):
        with open(file, "r") as f:
            super().__init__(json.load(f))

    def generate(self):
        # git
        execute("git config --global credential.helper store")
        execute(f"git config --global user.name {self['name']}")
        execute(f"git config --global user.email {self['email']}")
        # ssh
        file = Path.home() / ".ssh/id_rsa"
        execute(f"ssh-keygen -t rsa -C {self['email']} -N {self['passwd']} -f {file}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("json", type=str, help="Docker container configuration file")
    args = parser.parse_args()

    # python3 bin\new-user.py user-tongzj.json
    ucfg = UserCfg(args.json)
    ucfg.generate()
