#!/usr/bin/env bash

# Stop execution after any error
set -e

# Create data dir if not already there
mkdir -p Experiment_Results

for RAND_SEED in 12345 67890 13579 24680
do
    # Submit job
    sbatch run_job.sh ${RAND_SEED}
    # Sleep for 1 second to avoid overloading the machine
    sleep 1
done