# Manual for the FishOtter
## Building/rebuilding DUNE
### Getting the code
The repository for the toolchain contains a setup shell script that fetches the necessary files etc. 

    https://github.com/nikkone/otter-docker-toolchain

It rougly works by:
1. First clone http://github.com/LSTS/dune.git
2. Enter the DUNE folder
3. Clone this library to a folder named user
4. Update the IMCversion of DUNE to the otter version

:warning: For updating IMC, python needs to be installed, and usable through the python command. In ununtu 20.04+, python3 is default, so sudo apt install python-is-python3
needs to be installed. After that, the descriptions on the LSTS/dune wiki can be followed. 
