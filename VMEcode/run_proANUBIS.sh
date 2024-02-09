#!/bin/bash

# Loop 15 times
for ((i=1; i<=15; i++))
do
    # Run the executable with the argument "100"
    ./proANUBIS 500

    # Add any additional commands or delays if needed
    # sleep 1
done

