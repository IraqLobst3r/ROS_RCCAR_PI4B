# install dependencies for xbox controller
sudo apt install dkms git linux-headers-`uname -r`

# install Xpadneo kernel module (permanently disable Ertm)
cd
git clone https://github.com/atar-axis/xpadneo.git
cd xpadneo
sudo ./install.sh

# nice gui to check and calibrate
sudo apt-get install jstest-gtk
