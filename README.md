Note that this guide shows only one of infinite ways to do this, and that many components are not optimal, but has followed a "Whatever works" approach. If something seems stupid, it probably is, so find a way to improve it 😄

# Guide for the FishOtter
## Developing for the FishOtters
### Building/rebuilding DUNE - Getting the code
The repository for the toolchain contains a setup shell script that fetches the necessary files etc. 

    https://github.com/nikkone/otter-docker-toolchain

It rougly works by:
1. First clone http://github.com/LSTS/dune.git
2. Enter the DUNE folder
3. Clone this library to a folder named user
4. Update the IMCversion of DUNE to the otter version

:warning: For updating IMC, python needs to be installed, and usable through the python command. In ununtu 20.04+, python3 is default, so sudo apt install python-is-python3
needs to be installed. After that, the descriptions on the LSTS/dune wiki can be followed. 

### Deploying Changes
Once a change in DUNE has been made, upload it to the server:

    scp dune-2022.04.0--bit-linux-glibc-gcc94.tar.bz2 nikolal@otter.itk.ntnu.no:/var/www/html/software

There is an otaupdate.sh in the home folder /home/ubuntu/ on each otter. Running this will:
1. Download the latest software from the server, i.e. the .tar.bz2 file.
2. Rename the `/home/ubuntu/dune/` folder to dune-old and if there is a dune-old, it will rename that to dune-old-old. :warning: If there already is a dune-old-old, it gets deleted and overwritten.
3. Unpack the .tar.bz2 that was downloaded from the server to the `/home/ubuntu/dune/` folder

### Updating/modifying IMC messages

### Neptus
When building Neptus, IMC java bindings must be used. This is done thorugh the IMCJava tool by LSTS. Besides that, the standard LSTS approach can be used.

## Operating the FishOtters

### Executing DUNE on the FishOtters
The running of DUNE on the otters is managed by `systemd`, and set to automatically start at boot. It can be stopped, started or restarted with:

    sudo systemctl dune stop
    sudo systemctl dune start
    sudo systemctl dune restart

### Communication with DUNE on the FishOtters

### Using the field computer with OpenVPN, IMCProxy and Neptus

