[Unit]
Description=CURTmini 8BitDo Joystick Patch Autostart
PartOf=ipa-tmux-master.service
After=ipa-ros-autostart.service

Wants=multicast-lo.service
After=multicast-lo.service

[Service]
Type=oneshot
RemainAfterExit=yes
User=curt
ExecStart=/bin/bash /home/curt/Documents/Duarte/joy_8bitdo/start-joystick-patch.sh
ExecStop=/bin/killall -9 moltengamepad

[Install]
WantedBy=multi-user.target
