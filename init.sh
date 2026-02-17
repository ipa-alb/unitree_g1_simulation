#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"

echo "============================================"
echo " LeRobot Unitree G1 MuJoCo Sim Setup"
echo "============================================"
echo ""
echo "Repo: https://huggingface.co/lerobot/unitree-g1-mujoco"
echo ""

# ── 1. Check prerequisites ──────────────────────────────────────────
check_command() {
    if ! command -v "$1" &>/dev/null; then
        echo "ERROR: '$1' is not installed. Please install it first."
        exit 1
    fi
}

check_command python3
check_command git
check_command cmake

# ── 2. Create virtual environment ───────────────────────────────────
VENV_DIR="$PROJECT_DIR/venv"

if [ -d "$VENV_DIR" ]; then
    echo "[skip] Virtual environment already exists at $VENV_DIR"
else
    echo "[1/6] Creating virtual environment..."
    python3 -m venv "$VENV_DIR"
fi

# Activate venv
source "$VENV_DIR/bin/activate"

# ── 3. Download the repo from HuggingFace ───────────────────────────
SIM_DIR="$PROJECT_DIR/unitree-g1-mujoco"

echo "[2/6] Installing huggingface_hub..."
pip install --quiet huggingface_hub

if [ -d "$SIM_DIR" ] && [ -f "$SIM_DIR/run_sim.py" ]; then
    echo "[skip] unitree-g1-mujoco already downloaded at $SIM_DIR"
else
    echo "[3/6] Downloading lerobot/unitree-g1-mujoco from HuggingFace..."
    python3 -c "
from huggingface_hub import snapshot_download
snapshot_download(
    repo_id='lerobot/unitree-g1-mujoco',
    local_dir='$SIM_DIR',
)
"
fi

# ── 4. Build CycloneDDS from source (required by unitree_sdk2py) ────
CYCLONE_DIR="$PROJECT_DIR/cyclonedds"
CYCLONE_INSTALL="$CYCLONE_DIR/install"

if [ -d "$CYCLONE_INSTALL" ]; then
    echo "[skip] CycloneDDS already built at $CYCLONE_INSTALL"
else
    echo "[4/6] Building CycloneDDS 0.10.x from source..."
    git clone -b releases/0.10.x https://github.com/eclipse-cyclonedds/cyclonedds.git "$CYCLONE_DIR"
    mkdir -p "$CYCLONE_DIR/build"
    cmake -S "$CYCLONE_DIR" -B "$CYCLONE_DIR/build" -DCMAKE_INSTALL_PREFIX="$CYCLONE_INSTALL"
    cmake --build "$CYCLONE_DIR/build" --target install
fi

export CYCLONEDDS_HOME="$CYCLONE_INSTALL"

# ── 5. Install unitree_sdk2_python from GitHub ──────────────────────
SDK_DIR="$PROJECT_DIR/unitree_sdk2_python"

if [ -d "$SDK_DIR" ]; then
    echo "[skip] unitree_sdk2_python already cloned at $SDK_DIR"
else
    echo "[5/6] Cloning unitree_sdk2_python..."
    git clone https://github.com/unitreerobotics/unitree_sdk2_python.git "$SDK_DIR"
fi

echo "Installing unitree_sdk2_python (with CYCLONEDDS_HOME=$CYCLONEDDS_HOME)..."
pip install --quiet -e "$SDK_DIR"

# ── 6. Install remaining dependencies ────────────────────────────────
echo "[6/6] Installing Python dependencies from requirements.txt..."
# Install everything except unitree-sdk2py (already installed from source)
# Strip comments and blank lines before passing to pip
grep -v 'unitree-sdk2py' "$SIM_DIR/requirements.txt" | grep -v '^\s*#' | grep -v '^\s*$' | pip install --quiet -r /dev/stdin

# Install undeclared dependencies (used in code but missing from requirements.txt)
pip install --quiet gymnasium pygame scipy termcolor

echo ""
echo "============================================"
echo " Setup complete!"
echo "============================================"
echo ""
echo "Activate the environment:"
echo "  source $VENV_DIR/bin/activate"
echo ""
echo "Run the MuJoCo simulator:"
echo "  cd $SIM_DIR && python run_sim.py"
echo ""
echo "Use as a Gymnasium env (from Python):"
echo "  from env import make_env"
echo "  env = make_env()"
echo "  obs, info = env.reset()"
echo "  obs, reward, terminated, truncated, info = env.step(action)"
echo ""
echo "Config: $SIM_DIR/config.yaml"
echo "  - USE_JOYSTICK: 1     (set to 0 to disable gamepad)"
echo "  - JOYSTICK_TYPE: xbox  (or switch)"
echo "  - INTERFACE: lo        (loopback for sim, 192.168.123.x for real robot)"
echo ""
echo "View live cameras (while sim is running):"
echo "  cd $SIM_DIR && python view_cameras_live.py"
echo ""
echo "--- Optional: Full LeRobot framework ---"
echo "  conda create -y -n lerobot python=3.10 && conda activate lerobot"
echo "  git clone https://github.com/huggingface/lerobot.git"
echo "  cd lerobot && pip install -e '.[unitree_g1]'"
echo "  git clone https://github.com/unitreerobotics/unitree_sdk2_python.git"
echo "  cd unitree_sdk2_python && pip install -e ."
echo ""
echo "  # Run locomotion policy in sim:"
echo "  python examples/unitree_g1/gr00t_locomotion.py --repo-id nepyope/GR00T-WholeBodyControl_g1"
