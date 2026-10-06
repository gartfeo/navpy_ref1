sudo mkdir -p /var/log/journal
sudo sed -i 's/^#\?Storage=.*/Storage=persistent/' /etc/systemd/journald.conf
sudo systemctl restart systemd-journald

# 1) Write a tiny script that appends timestamp + get_throttled
sudo tee /usr/local/sbin/log-throttle.sh >/dev/null <<'SH'
#!/bin/sh
ts=$(date -Is)
val=$(vcgencmd get_throttled 2>/dev/null || echo "throttled=N/A")
echo "$ts $val" >> /var/log/throttle.log
SH
sudo chmod +x /usr/local/sbin/log-throttle.sh

# 2) One-shot service that runs the script
sudo tee /etc/systemd/system/log-throttle.service >/dev/null <<'UNIT'
[Unit]
Description=Log throttling/undervoltage snapshot
[Service]
Type=oneshot
ExecStart=/usr/local/sbin/log-throttle.sh
UNIT

# 3) Timer: run every minute
sudo tee /etc/systemd/system/log-throttle.timer >/dev/null <<'UNIT'
[Unit]
Description=Record vcgencmd get_throttled every minute
[Timer]
OnBootSec=60s
OnUnitActiveSec=60s
Unit=log-throttle.service
[Install]
WantedBy=timers.target
UNIT

# 4) Enable timer + write the first line immediately
sudo systemctl daemon-reload
sudo systemctl enable --now log-throttle.timer
sudo systemctl start log-throttle.service

# Reboot on panic after 10s (harmless if set already)
echo 'kernel.panic = 10' | sudo tee /etc/sysctl.d/99-panic.conf >/dev/null
sudo sysctl --system

# Enable pstore (ramoops). Path differs by image; this covers both.
CFG=/boot/firmware/config.txt; [ -f /boot/config.txt ] && CFG=/boot/config.txt
sudo grep -q '^dtoverlay=ramoops' "$CFG" || \
  echo 'dtoverlay=ramoops,ramoops.mem_size=0x100000,ramoops.record_size=0x10000,ramoops.console_size=0x20000' | sudo tee -a "$CFG" >/dev/null
sudo systemctl enable --now systemd-pstore.service