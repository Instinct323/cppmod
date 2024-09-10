set DST_PATH=D:\Software\docker-desktop-data
wsl -l -v
wsl --export docker-desktop-data %DST_PATH%.tar
wsl --unregister docker-desktop-data
wsl --import docker-desktop-data %DST_PATH% %DST_PATH%.tar --version 2
