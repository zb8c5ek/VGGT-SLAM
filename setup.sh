#!/bin/bash
set -e  # Exit immediately if a command exits with a non-zero status

# 1. Install Python dependencies
echo "Installing base requirements..."
pip3 install -r requirements.txt

mkdir -p 3rdParty

# 2. Clone and install Salad
echo "Cloning and installing Salad..."
if [ ! -d "3rdParty/salad" ]; then
    git clone https://github.com/Dominic101/salad.git 3rdParty/salad
else
    echo "  3rdParty/salad already exists, skipping clone."
fi
pip install -e 3rdParty/salad

# 3. Clone and install our fork of VGGT
echo "Cloning and installing VGGT..."
if [ ! -d "3rdParty/vggt" ]; then
    git clone https://github.com/MIT-SPARK/VGGT_SPARK.git 3rdParty/vggt
else
    echo "  3rdParty/vggt already exists, skipping clone."
fi
pip install -e 3rdParty/vggt

# 4. Install Perception Encoder
echo "Cloning and installing Perception Encoder..."
if [ ! -d "3rdParty/perception_models" ]; then
    git clone https://github.com/facebookresearch/perception_models.git 3rdParty/perception_models
else
    echo "  3rdParty/perception_models already exists, skipping clone."
fi
pip install -e 3rdParty/perception_models

# 5. Install SAM 3
echo "Cloning and installing SAM 3..."
if [ ! -d "3rdParty/sam3" ]; then
    git clone https://github.com/facebookresearch/sam3.git 3rdParty/sam3
else
    echo "  3rdParty/sam3 already exists, skipping clone."
fi
pip install -e 3rdParty/sam3

# 6. Install current repo in editable mode
echo "Installing current repo..."
pip install -e .

echo "Installation Complete"
