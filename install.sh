#!/bin/bash

# Usage : source install.sh

# # Install udev rules
RULE_CONTENT=$(cat <<EOL
SUBSYSTEM=="tty", KERNELS=="1-4.1", SYMLINK+="esp"
SUBSYSTEM=="tty", KERNELS=="1-4.2", SYMLINK+="pixhawk"
KERNEL=="event*", SUBSYSTEM=="input", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="028e", SYMLINK+="/input/haptics"
KERNEL=="js*", SUBSYSTEM=="input", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="028e", SYMLINK+="input/joy"
EOL
)

RULE_FILE="/etc/udev/rules.d/rbl.rules"

echo "$RULE_CONTENT" | sudo tee "$RULE_FILE" > /dev/null

sudo udevadm control --reload-rules
sudo usermod -a -G dialout $USER
sudo usermod -a -G input $USER

# No error checking
echo "USB Ports configured!.."
sleep 3
# Package installation
echo -e "\nChecking for dependencies, installing if necessary..."

# Removes Sudo password prompt for these following commands
# [robonet, robod, systemctl, nmcli , lshw]
if ! sudo grep -q "/bin/robod" /etc/sudoers; then
    echo "%sudo	ALL=(ALL:ALL) NOPASSWD: /bin/robod, /usr/bin/systemctl,/bin/nmcli,/bin/lshw" | sudo tee -a /etc/sudoers
fi

sudo cp getip /usr/bin/

cat requirements.txt | xargs sudo apt-get install -y 

echo "Installation Successful"
