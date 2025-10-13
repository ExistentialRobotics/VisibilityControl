#!/usr/bin/env bash
set -euo pipefail

CARLA_VERSION="0.9.14"
CARLA_URL="https://carla-releases.b-cdn.net/Linux/CARLA_0.9.14.tar.gz"
CARLA_TARBALL="CARLA_${CARLA_VERSION}.tar.gz"

# Install in current directory
CARLA_ROOT="$(pwd)/CARLA_${CARLA_VERSION}"
ENVZIP="Env02.zip"

echo "[0/6] Installing into: $CARLA_ROOT"
mkdir -p "$CARLA_ROOT"

echo "[1/6] Downloading CARLA ${CARLA_VERSION} from tiny link…"
curl --fail -L --progress-bar -C - -o "$CARLA_TARBALL" "$CARLA_URL"

echo "[2/6] Extracting CARLA archive…"
tar -xzf "$CARLA_TARBALL" -C "$CARLA_ROOT"

if [ ! -x "$CARLA_ROOT/CarlaUE4.sh" ]; then
  echo "Error: CarlaUE4.sh not found in $CARLA_ROOT" >&2
  exit 1
fi

echo "[3/6] Verifying Env02.zip presence…"
if [ ! -f "$ENVZIP" ]; then
  echo "Error: Env02.zip not in current working directory." >&2
  exit 1
fi

echo "[4/6] Unzipping Env02 into CarlaUE4/Content…"
unzip -o "$ENVZIP" -d "$CARLA_ROOT/CarlaUE4/Content" >/dev/null

CFG_FILE="$CARLA_ROOT/CarlaUE4/Config/DefaultEngine.ini"
mkdir -p "$(dirname "$CFG_FILE")"
touch "$CFG_FILE"

echo "[5/6] Updating DefaultEngine.ini map settings…"
sed -i '/^\s*EditorStartupMap\s*=/d' "$CFG_FILE" || true
sed -i '/^\s*GameDefaultMap\s*=/d'   "$CFG_FILE" || true

cat >> "$CFG_FILE" <<'EOF'

[/Script/EngineSettings.GameMapsSettings]
EditorStartupMap=/Game/Env02/Maps/Env02/Env02.Env02
GameDefaultMap=/Game/Env02/Maps/Env02/Env02.Env02
EOF

echo "[6/6] All done."
echo "To launch: cd \"$CARLA_ROOT\" && ./CarlaUE4.sh"

