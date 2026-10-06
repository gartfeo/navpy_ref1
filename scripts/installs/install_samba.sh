#!/bin/bash
# nano install_samba.sh
# chmod +x install_samba.sh
# ./install_samba.sh

#set -e  # Stop the script if any command fails

echo "Updating system packages..."
sudo apt-get update

# Setup paths for the samba share
SHARE_NAME="peron"
USER_NAME="pi"
SHARE_PATH="/home/$USER_NAME"

echo "Installing samba..."
sudo apt-get install -y samba samba-common-bin

echo "Setting up samba share..."
# Ensure the directory exists before changing its permissions
[ -d "$SHARE_PATH" ] && chmod 777 "$SHARE_PATH"
echo -e "\n[$SHARE_NAME]\n   path = $SHARE_PATH\n   force user = $USER_NAME\n   browsable = yes\n   writeable = yes\n   public = yes\n   create mode = 0644\n   directory mode = 0755\n" | sudo tee -a "/etc/samba/smb.conf"

SMB_PASSWORD="izen2021"

echo -e "$SMB_PASSWORD\n$SMB_PASSWORD" | sudo smbpasswd -a $USER_NAME
sudo smbpasswd -e $USER_NAME

echo "Restarting samba..."
sudo systemctl restart smbd

echo "Samba setup completed successfully!"
