#!/bin/bash

set -e  # Exit immediately if a command exits with a non-zero status

# # ----- STEP 0: Install necessary tools -----

# # ----- STEP 1: Go to NOCS directory -----
NOCS_DIR="$(pwd)"
echo "Navigating to NOCS directory: $NOCS_DIR"


# # ----- STEP 2: Compile NOCS and run simple test -----
echo "Compiling NOCS..."
cd "$NOCS_DIR/solver"
mkdir -p build
cd build
cmake .. && make -j6
cd $NOCS_DIR

# Move the compiled NOCS to the examples directory

#Create a directory for compiled examples if it doesn't exist
if [ ! -d "$NOCS_DIR/compiled_examples" ]; then
    mkdir -p "$NOCS_DIR/compiled_examples"
fi
mv $NOCS_DIR/solver/build/example/localCollocation/nocs_examples/RRPlanarRobot/planarRobot $NOCS_DIR/compiled_examples/
cd $NOCS_DIR/compiled_examples
echo "Running simple test..."
./planarRobot


