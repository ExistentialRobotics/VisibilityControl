#!/bin/bash
set -e

ENV_NAME="erl_vc"
MODEL_URL="https://drive.google.com/uc?id=1kVWNmjN1OAg5i5M3fzSbBQ0pSwpU8CU5"
MODEL_DIR="pursuer/planner/models"
MODEL_ZIP="model.zip"

PYTHON_VERSION=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")

echo "Downloading OMPL official installation script..."
wget https://ompl.kavrakilab.org/install-ompl-ubuntu.sh -O install-ompl-ubuntu.sh
chmod +x install-ompl-ubuntu.sh

echo "Running OMPL installation with Python bindings..."
./install-ompl-ubuntu.sh --python

# 1. Check for conda
if ! command -v conda &> /dev/null; then
    echo "Conda is not installed. Please install Anaconda or Miniconda first."
    exit 1
fi

# 2. Create conda environment if it doesn't exist
if conda info --envs | grep -q "$ENV_NAME"; then
    echo "Conda environment '$ENV_NAME' already exists."
else
    echo "Creating conda environment '$ENV_NAME'..."
    conda create -y -n $ENV_NAME python=$PYTHON_VERSION
fi

# 3. Activate environment and install dependencies
echo "Activating environment and installing dependencies..."
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate $ENV_NAME

# Optional: install requirements if available
if [ -f "requirements.txt" ]; then
    pip install --upgrade pip
    pip install -r requirements.txt
else
    echo "No requirements.txt found. Skipping dependency installation."
fi

# 4. Install gdown if not installed
if ! pip show gdown > /dev/null; then
    pip install gdown
fi

# 5. Download model
mkdir -p "$MODEL_DIR"
echo "Downloading model file..."
gdown "$MODEL_URL" -O "$MODEL_ZIP"

# 6. Unzip model
echo "Extracting model to $MODEL_DIR..."
unzip -o "$MODEL_ZIP" -d "$MODEL_DIR"
rm "$MODEL_ZIP"

# Step 7: Copy OMPL shared libraries into Conda environment
echo "Copying OMPL shared libraries to Conda lib directory..."
OMPL_PY_SRC="/usr/local/lib/python/dist-packages/ompl"
if [ ! -d "$OMPL_PY_SRC" ]; then
    echo "OMPL Python bindings not found at $OMPL_PY_SRC"
    exit 1
fi
CONDA_SITE_PACKAGES=$(python -c "import site; print(site.getsitepackages()[0])")
echo "📁 Copying OMPL Python bindings to Conda environment..."
cp -r "$OMPL_PY_SRC" "$CONDA_SITE_PACKAGES/"

# Step 8: Verify installation
echo "Testing OMPL installation in Conda environment..."
python -c "import ompl.base as ob; print(ob.StateSpace.__name__)"
echo "OMPL has been successfully set up in Conda environment '$ENV_NAME'"

echo "Setup complete. You can activate the environment with: conda activate $ENV_NAME"

# Thanks GPT-4o for easily generating this script!