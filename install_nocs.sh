#!/bin/bash

set -e  # Exit immediately if a command exits with a non-zero status

# # ----- STEP 0: Install necessary tools -----
# echo "Installing essential development tools..."
# #sudo apt update
sudo apt install -y gcc g++ gfortran patch wget pkg-config liblapack-dev libmetis-dev

# # ----- STEP 1: Go to NOCS directory -----
NOCS_DIR="$(pwd)"
echo "Navigating to NOCS directory: $NOCS_DIR"

# # ----- STEP 2: Fetch, build and install Ipopt using coinbrew -----
COINBREW_PATH="$NOCS_DIR/coinbrew"

echo "Fetching Ipopt..."
$COINBREW_PATH fetch Ipopt --no-prompt

echo "Creating build directory for Ipopt..."
mkdir -p $NOCS_DIR/Ipopt/build

echo "Building Ipopt..."
$COINBREW_PATH build Ipopt --prefix="$NOCS_DIR/Ipopt/build" --test --no-prompt --verbosity=3

#echo "Installing Ipopt..."
$COINBREW_PATH install Ipopt --no-prompt

# ----- STEP 3: Add IPOPT to NOCS -----
echo "Copying Ipopt include and lib to ipopt directory..."
echo "$(pwd)"
mkdir -p ../ipopt
cp -r Ipopt/build/include ../ipopt/
cp -r Ipopt/build/lib ../ipopt/

# ----- STEP 4: Add Eigen library -----
echo "Downloading Eigen 3.3.4..."
wget https://gitlab.com/libeigen/eigen/-/archive/3.3.4/eigen-3.3.4.zip 
unzip eigen-3.3.4.zip

if [ -d "../eigen3" ]; then
    rm -r ../eigen3
fi

echo "Moving Eigen to NOCS directory..."
mv eigen-3.3.4 "$NOCS_DIR/../eigen3"
rm eigen-3.3.4.zip

# # ----- STEP 6: Compile NOCS and run simple test -----
echo "Compiling NOCS..."
cd "$NOCS_DIR/solver"
mkdir -p build
cd build
cmake .. && make -j6
cd $NOCS_DIR

# Move the compiled NOCS to the examples directory
mkdir $NOCS_DIR/compiled_examples
mv $NOCS_DIR/solver/build/example/localCollocation/nocs_examples/RRPlanarRobot/planarObstacle $NOCS_DIR/compiled_examples/
cd $NOCS_DIR/compiled_examples
echo "Running simple test..."
./planarObstacle


