#!/bin/bash
# nano install.sh
# chmod +x install.sh
# ./install.sh

#set -e  # Stop the script if any command fails

## Update and upgrade system packages
echo "Updating system packages..."
sudo apt-get update

# Setup paths for the samba share
SHARE_NAME="peron"
USER_NAME="pi"
SHARE_PATH="/home/$USER_NAME"

if [ ! -d "$SHARE_PATH/navpy" ]
then
  # Hint about GitHub personal access token
  echo "Hint: If you don't have a GitHub personal access token, generate one by following these steps:"
  echo "1. Go to https://github.com/settings/tokens."
  echo "2. Click 'Generate new token'."
  echo "3. Select the necessary permissions for the token."
  echo "4. Click 'Generate token' at the bottom of the page."
  echo "5. Copy the generated token and use it when prompted in this script."
  echo

  read -rp "Enter your GitHub username: " github_username
  read -rsp "Enter your GitHub personal access token: " github_token
  echo

  # Install Git
  echo "Installing Git..."
  sudo apt-get install -y git

  # Clone the repository
  echo "Cloning the navpy repository..."
  mkdir -p "$SHARE_PATH"
  cd "$SHARE_PATH" || exit

  if git clone "https://$github_username:$github_token@github.com/gartfeo/navpy.git"; then
      echo "Repository cloned successfully!"
  else
      echo "Failed to clone the repository. Please check your token and permissions."
      exit 1
  fi

  # Change to the navpy directory
  cd navpy || { echo "Failed to enter the navpy directory"; exit 1; }

  # Pull latest changes
  git pull || { echo "Git pull failed"; exit 1; }

  # Check if the branch already exists
  if git rev-parse --verify 03Cent >/dev/null 2>&1; then
      echo "Branch 03Cent already exists. Checking out..."
      git checkout 03Cent || { echo "Failed to checkout branch 03Cent"; exit 1; }
  else
      echo "Creating and checking out branch 03Cent..."
      git checkout -b 03Cent origin/03Cent || { echo "Failed to create or checkout branch 03Cent"; exit 1; }
  fi

  git submodule update --init --recursive || { echo "Failed to update submodules"; exit 1; }

  # Pull submodules
  git pull --recurse-submodules || { echo "Failed to pull submodules"; exit 1; }

  # Return to the parent directory
  cd ..

  sudo apt-get install -y samba samba-common-bin

  echo "Setting up samba share..."
  # Ensure the directory exists before changing its permissions
  [ -d "$SHARE_PATH" ] && chmod 777 "$SHARE_PATH"
  echo -e "\n[$SHARE_NAME]\n   path = $SHARE_PATH\n   force user = $USER_NAME\n   browsable = yes\n   writeable = yes\n   public = yes\n   create mode = 0644\n   directory mode = 0755\n" | sudo tee -a "/etc/samba/smb.conf"

  SMB_PASSWORD="izen2021"

  echo -e "$SMB_PASSWORD\n$SMB_PASSWORD" | sudo smbpasswd -a $USER_NAME

  sudo smbpasswd -e $USER_NAME

  # Restart the samba service
  echo "Restarting samba..."
  sudo systemctl restart smbd

  # connect from windows using rpi1-1\pi and izen2021
fi

# Install Python3, venv, python-is-python3, and samba
echo "Installing Python and related packages..."
sudo apt-get install -y python3 python3-venv python3-pip
sudo apt-get install -y python-is-python3 || (sudo rm -f /usr/bin/python && sudo ln -s /usr/bin/python3 /usr/bin/python)

#
## Install virtualenv and virtualenvwrapper
#echo "Installing virtualenv and virtualenvwrapper..."
#sudo apt-get install -y virtualenv virtualenvwrapper
#
## Setup virtualenvwrapper environment variables
#export WORKON_HOME=$HOME/.virtualenvs
#export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3
#source /usr/share/virtualenvwrapper/virtualenvwrapper.sh
#
## Append variables for future shell sessions.
#{
#  echo "export WORKON_HOME=$HOME/.virtualenvs"
#  echo "export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3"
#  echo "source /usr/share/virtualenvwrapper/virtualenvwrapper.sh"
#} >> ~/.bashrc
#
## Create virtual environment
#echo "Creating virtual environment..."
#mkvirtualenv main
#
#workon main

#
## Ensure pip is in the PATH
#source "$WORKON_HOME"/main/bin/activate
#
## Check if pip is installed in the virtual environment
#if ! command -v pip &> /dev/null
#then
#    echo "pip could not be found in the virtual environment, installing pip..."
#    sudo apt-get install -y python3-pip
#fi

# enable uart
#if grep -q "^enable_uart=1" /boot/config.txt; then
#    echo "UART is already enabled."
#else
#    sudo cp /boot/config.txt /boot/config_backup.txt
#    echo "enable_uart=1" | sudo tee -a /boot/config.txt
#    echo "UART has been enabled. Please reboot your Raspberry Pi."
#fi

# change console baudrate
#sudo cp /boot/cmdline.txt /boot/cmdline_backup.txt
#sudo sed -i 's/^.*\(root=.*\)$/\1 console=serial0,57600/' /boot/cmdline.txt

# nano install2.sh
# chmod +x install2.sh
# ./instal2l.sh
# Install dependencies for some of the Python packages
echo "Installing Python dependencies..."
pip install numpy==1.25.2 --break-system-packages

# Install Python packages
echo "Installing Python packages..."
pip install \
    geopy==2.4.0 --break-system-packages\
    pymap3d==3.0.1 --break-system-packages\
    pymavlink==2.4.40 --break-system-packages\
    pyserial==3.5 --break-system-packages\
    pyzmq==25.1.1 --break-system-packages\
    requests==2.31.0 --break-system-packages \
    line_profiler --break-system-packages

# install helping soft
sudo apt-get install screen -y
# Print success message
echo "update system packages..."
sudo apt-get update
#sudo apt-get upgrade -y
echo "Setup completed successfully!"