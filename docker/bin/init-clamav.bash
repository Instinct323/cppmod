if [ $(id -u) -eq 0 ]; then

  apt install -y clamav clamav-daemon
  /etc/init.d/clamav-freshclam stop
  freshclam
  /etc/init.d/clamav-freshclam start
  # clamscan -r /

else
  echo "error: permission denied."
fi