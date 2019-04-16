#!/usr/bin/env bash

# Stop execution after any error
set -e

# Create data dir if not already there
mkdir -p data

# Go through all the values for PARAM1
for PARAM1 in value1 value2 ... valueN
  # Go through all the values for PARAM2
  for PARAM2 in value1 value2 ... valueM
    # Submit job
    sbatch run_job.sh $PARAM1 $PARAM2
    # Sleep for 1 second to avoid overloading the machine
    sleep 1
  done
done