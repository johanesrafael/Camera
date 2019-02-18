# MorpheusChair Setup RaspBerry Ubuntu Mate
sudo apt-get update
sudo apt install vim
sudo apt install git
sudo apt install openssh-server
sudo apt install terminator
sudo ufw allow 22
sudo systemctl enable ssh
sudo systemctl start ssh
whoami
ifconfig

# Install RDS systems
sudo apt install xrdp
echo " In your client computer -- sudo apt install remmina "


