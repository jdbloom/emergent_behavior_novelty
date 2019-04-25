#!/usr/bin/env bash
#SBATCH -J emergent_behavior
#SBATCH -N 1
#SBATCH -n 16

# Stop execution after any error
set -e

# Cleanup function to be executed upon exit, for any reason
function cleanup() {
    rm -rf ${WORKDIR}
}



########################################
#
# Useful variables
#
########################################

# Your user name
# (Don't change this)
MYUSER=$(whoami)

# Path of the local storage on a node
# Use this to avoid sending data streams over the network
# (Don't change this)
LOCALDIR=/tmp

# ARGoS environment variables
# (Don't change this)
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:$HOME/argos3bundle/lib/argos3
export PATH=$PATH:$HOME/argos3bundle/bin

# Buzz environment variables
# (Don't change this)
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:$HOME/buzzbundle/lib
export PATH=$PATH:$HOME/buzzbundle/bin

# Folder where you want your data to be stored
# (Adapt this to your needs)
DATADIR=~/Experiment_Results

# Path to the file template.argos
# (Adapt this to your needs)
TEMPLATE=experiments/emergent_behavior.argos



########################################
#
# Job-related variables
#
########################################

# Parameters related to this job
RAND_SEED=$1

# Job id
# (Change this to reflect the above parameters)
THISJOB=${RAND_SEED}

# Job working directory
# (Don't change this)
WORKDIR=${LOCALDIR}/${MYUSER}/${THISJOB}



########################################
#
# Job directory
#
########################################

# Create work dir from scratch, enter it
# (Don't change this)
rm -rf ${WORKDIR} && mkdir -p ${WORKDIR} && cd ${WORKDIR}

# Make sure you cleanup upon exit
# (Don't change this)
trap cleanup EXIT SIGINT SIGTERM



########################################
#
# Actual job logic
#
########################################

mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ~/Swarms_Group_2
make -j4
cd ..
cp -r ~/Swarms_Group_2/experiments .
cp -r ~/Swarms_Group_2/buzz .
bzzc buzz/emergent_behavior.bzz
./build/embedding/mpga/mpga_emergent_behavior ${RAND_SEED}

# Transfer info back to my home directory
mkdir data_${THISJOB}
mv *.dat *.csv data_${THISJOB}/
zip data_${THISJOB}.zip data_${THISJOB}/*

cp -a data_${THISJOB}.zip ${DATADIR}
