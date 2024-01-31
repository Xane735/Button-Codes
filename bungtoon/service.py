[Unit]
Description=Return leg button
After=multi-user.target
StartLimitIntervalSec=10

[Service]
Type=simple
WorkingDirectory=/home/pi/button-mission-auto/
ExecStart=sudo /usr/bin/python /home/pi/button-mission-auto/automatic_upload.py
Restart=on-failure
RestartSec=3s


[Install]
WantedBy=multi-user.target
