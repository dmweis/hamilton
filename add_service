cat <<EOT | sudo tee /etc/systemd/system/hamilton.service > /dev/null
[Unit]
Description=hamilton
[Service]
Type=simple
Restart=on-failure
RestartSec=5s
ExecStart=/home/$(whoami)/.cargo/bin/hamilton_controller --body_config /home/pi/.config/hamiltoncontroller/wheel_config.json
[Install]
WantedBy=default.target
EOT

sudo systemctl daemon-reload
sudo systemctl enable hamilton
sudo systemctl restart hamilton
