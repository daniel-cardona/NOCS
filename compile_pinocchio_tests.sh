#!/bin/bash

set -e  # Exit immediately if a command exits with a non-zero status

# # ----- STEP 1: Go to NOCS directory -----
NOCS_DIR="$(pwd)"
echo "Navigating to NOCS directory: $NOCS_DIR"


# # ----- STEP 2: Compile NOCS and run simple test -----
echo "Compiling NOCS..."
cd "$NOCS_DIR/solver"
mkdir -p build
cd build
cmake .. -DDYNLIBRARY=pinocchio && make -j6
cd $NOCS_DIR

# Move the compiled NOCS to the examples directory

#Create a directory for compiled examples if it doesn't exist
if [ ! -d "$NOCS_DIR/compiled_examples/pinocchio_examples" ]; then
    mkdir -p "$NOCS_DIR/compiled_examples/pinocchio_examples"
fi

pinocchio_examples_dir="$NOCS_DIR/solver/build/example/localCollocation/pinocchio_examples"

mv $pinocchio_examples_dir/kukaYoubot/kuka_robot $NOCS_DIR/compiled_examples/pinocchio_examples/
mv $pinocchio_examples_dir/NAO/nao_robot $NOCS_DIR/compiled_examples/pinocchio_examples/
mv $pinocchio_examples_dir/NAO_COM/nao_robot_com $NOCS_DIR/compiled_examples/pinocchio_examples/
mv $pinocchio_examples_dir/snake_robot/snakeRobot $NOCS_DIR/compiled_examples/pinocchio_examples/

