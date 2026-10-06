journalctl -u navpy -f -n 100 --no-pager

systemctl status navpy.service

journalctl -b 0 -p alert --no-pager | grep -F "UNCLEAN SHUTDOWN" || echo "Last shutdown was clean."

journalctl -b -1 -u systemd-shutdown -o short-precise
journalctl -b -1 -p err..alert --no-pager

sudo ls -l /sys/fs/pstore
sudo ls -l /var/lib/systemd/pstore
sudo sed -n '1,200p' /var/lib/systemd/pstore/* 2>/dev/null

# edit arguments
sudo nano /etc/default/navpy
# restart service
sudo systemctl restart navpy.service

