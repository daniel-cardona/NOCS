#!/bin/bash

set -e  # Exit immediately if a command exits with a non-zero status

# ----- STEP 0: Install necessary tools -----
echo "Installing essential development tools..."
#sudo apt update
sudo apt install -y gcc g++ gfortran patch wget pkg-config liblapack-dev libmetis-dev

# ----- STEP 1: Go to NOCS directory -----
NOCS_DIR="."
cd "$NOCS_DIR" || { echo "NOCS directory not found!"; exit 1; }

# ----- STEP 2: Fetch, build and install Ipopt using coinbrew -----
COINBREW_PATH="./coinbrew"

echo "Fetching Ipopt..."
$COINBREW_PATH fetch Ipopt --no-prompt

echo "Creating build directory for Ipopt..."
mkdir -p Ipopt/build

echo "Building Ipopt..."
$COINBREW_PATH build Ipopt --prefix="$NOCS_DIR/Ipopt/build" --test --no-prompt --verbosity=3

#echo "Installing Ipopt..."
$COINBREW_PATH install Ipopt --no-prompt

# ----- STEP 3: Add IPOPT to NOCS -----
echo "Copying Ipopt include and lib to ipopt directory..."
mkdir -p ../ipopt
cp -r Ipopt/build/include ../ipopt/
cp -r Ipopt/build/lib ../ipopt/

# ----- STEP 4: Add Eigen library -----
cd "$NOCS_DIR/.."

echo "Downloading Eigen 3.3.4..."
wget https://gitlab.com/libeigen/eigen/-/archive/3.3.4/eigen-3.3.4.zip
unzip eigen-3.3.4.zip

echo "Moving Eigen to NOCS directory..."
mv eigen-3.3.4 "$NOCS_DIR/eigen3"

# ----- STEP 5: Install Python3 and NumPy -----
#echo "Installing Python3, NumPy and Matplotlib..."
#sudo apt install -y python3-matplotlib python3-numpy python3.7

# ----- STEP 6: Compile NOCS and run simple test -----
echo "Compiling NOCS..."
cd "NOCS/solver"
mkdir -p build
cd build
cmake .. && make

# ----- STEP 7: Run example -----
echo "Running example..."
cd "$NOCS_DIR/solver/build/example/localCollocation/nocs_examples/RRPlanarRobot"
./planarObstacle

echo "Installation and test complete."

