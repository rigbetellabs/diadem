#!/bin/bash

systemctl stop --user rbl_upstart.service && systemctl disable --user rbl_upstart.service > /dev/null 2>&1

if [ $? -eq 0 ]; then
    echo "Upstart service stopped and disabled successfully. Good to go in development mode."
else
    echo "Something went wrong stopping or disabling the service!"
    exit 1
fi

sudo rm -r /etc/systemd/user/rbl_upstart.service > /dev/null 2>&1
