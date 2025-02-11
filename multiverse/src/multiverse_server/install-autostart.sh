#!/bin/bash
# This script installs and enables/starts a systemd service
# It also installs the service file
export NAME=MultiverseServer

cat >/etc/systemd/system/${NAME}.service <<EOF
[Unit]	

Description=${NAME}

[Service]
Type=simple
Restart=always
User=root
Group=root
WorkingDirectory=$(pwd)
ExecStart=$(pwd)/multiverse_server

[Install]
WantedBy=multi-user.target
EOF

# Enable and start service
systemctl enable --now ${NAME}.service
