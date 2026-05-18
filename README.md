## Ardupilot Fixed-wing with PCAC for Echopilot
This codebase builds on prior work of pcac.cpp and pcac.hpp to implement PCAC in ardupilot on a fixed-wing vehicle. 
This means that the core pcac code is designed to be implementation-independent. 
A wrapper, AP_PCAC, interfaces pcac.cpp with the implementation-specific details.
The functions for the elevator, throttle, and rudder/aileron lateral instances of pcac are scheduled in plane.cpp at a fixed rate on a single thread.
This is one of the limitation of Ardupilot- it is not designed to run as a multithreaded program, and once oneStep is called, it will run until completion.
This is markedly different from PX4, which is a multithreaded design where oneStep can be interrupted once an interval has been completed.

## Cloning the repository and install
https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux
```bash
  git clone git@github.com:DunkTheDoughnut/ardupilot.git
  Tools/environment_install/install-prereqs-ubuntu.sh -y
  . ~/.profile
```

## Setting up simulation environment
This respository has been tested on Ubuntu Jammy.

# Install qGroundControl
https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html

# Install gdb
https://ardupilot.org/dev/docs/debugging-with-gdb-on-linux.html

# Install python3
https://www.python.org/downloads/

# Update python libraries 
```bash
  pip install --upgrade numpy
  sudo apt install python3-tk
  export MPLBACKEND=Qt5Agg
```
# Rebuild MAVLink messages
Copy ardupilotmega.xml from ros2pcac repository
```bash
    cd ./modules/mavlink/pymavlink$
    python3 -m pip install Cython wheel setuptools future --user
    python3 setup.py install --user
```

# Build SITL
```bash
  ./waf clean #(not necessary every time)
  ./waf configure --board sitl #(or desired target)
  ./waf plane -j3 --debug
```
  
# Build and Flash Echopilot FMU
On a host computer
```bash
  ./waf clean #(not necessary every time)
  ./waf configure --board EchopilotAI --enable-custom-controller
  ./waf plane
```
Ensure the board is powered off
Connect one end of a USB cable to the FMU USB port
```bash
  ./waf plane --upload
```
Once the "waiting for bootloader..." screen appears, plug the other end of the USB cable into the host computer.
Ensure the cable is not unplugged during the flash process.
    
# GDB Commands	  
    r (run)
	break file:linenumber (optional)
    break class::function (optional)
    delete <breakpoint number>
    n (next line/step over))
    s (step into)
    u (step out)
    c (continue to next breakpoint)
    CTRL + c (break)
    q (quit)
    print *<Eigen Matrix>.m_storage.data()@(<size>)
    
    info threads (show threads)
    set scheduler-locking on (prevent changing threads)
# GDB debugging after failure
    bt (backtrace)
    frame <frame number>
    print <variable>

    
# Common Failures
(bind failed - Address already in use)
sudo lsof -i :<port>
sudo kill -9 <PID>

# Debugging console (without mavproxy or sitl setup)
```bash
gdb --args build/sitl/bin/arduplane --model plane --defaults Tools/autotest/models/plane.parm
```

## License ##

The ArduPilot project is licensed under the GNU General Public
License, version 3.

- [Overview of license](https://ardupilot.org/dev/docs/license-gplv3.html)

- [Full Text](https://github.com/ArduPilot/ardupilot/blob/master/COPYING.txt)
