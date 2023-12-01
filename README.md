# BerthaPi

You need BusIO for this to work;  
https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/installing-circuitpython-on-raspberry-pi

Adafruit proviced a script (on that site) to automatically install the correct matching version for your Pi hardware and OS.

cd ~
pip3 install --upgrade adafruit-python-shell
wget https://raw.githubusercontent.com/adafruit/Raspberry-Pi-Installer-Scripts/master/raspi-blinka.py
sudo -E env PATH=$PATH python3 raspi-blinka.py

Also, the PCA9685 lpython libraries (the servokit has the Motor library, which includes servos.. duh)

https://github.com/adafruit/Adafruit_CircuitPython_PCA9685

sudo pip3 install adafruit-circuitpython-pca9685
sudo pip3 install adafruit-circuitpython-servokit

Then SSHKeyboard, which we control with

https://pypi.org/project/sshkeyboard/

pip install sshkeyboard
