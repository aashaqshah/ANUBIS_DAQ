# Standalone script to run proANUBIS data-taking. 
This code is built to continously read data from the proANUBIS TDCs, periodically writing the output data to ROOT files. In addition to the main output, a secondary monitoring file is created every 30 minutes (configurable) which can be used to check the state of the setup. The code is designed to run as a linux background process, and reads /cfg/runFlag.txt periodically to determine whether to continue running. By writing a zero (or anything **other** than a 1) to this file the script knows to stop.

Due to the event formatting for root, the script only tolerates TDC data which exactly follows the format of header -> data words -> end of block word. Any variations from this pattern will put the reading state machine into "INVALID", the current TTree will be written, and the data-taking stopped.
## First Time Operation
I didn't write in automatic directory creation for the monitoring or config flags yet, so this needs to be done for new installs:

    mkdir cfg/
    echo 1 > cfg/runFlag.txt 
    mkdir data/
    mkdir data/monitor/
## Compiling
    source setup.sh
    make
## Running
First, make sure that runFlag.txt has a one in it:

    echo 1 > cfg/runFlag.txt
(Can skip if runFlag.txt is already set)
Then run the code 
    
    ./proANUBIS 1 #
where # is the number of seconds before creating a new output file, currently using 21600 for 6 hour runs.

To disown the process:

    ./proANUBIS 1 21600 &
    disown -a
To stop the disowned process:
    
    echo 0 > cfg/runFlag.txt

The output ROOT files will be written to the data/ directory in folders and files named after the date and time of their creation, and every 30 minutes the last 30 minutes of data will be written to data/monitor/proANUBISmonitor.root
