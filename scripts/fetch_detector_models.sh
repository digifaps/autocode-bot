#!/bin/bash
# Download the YOLOX detection models (Apache-2.0, Megvii) and build their TensorRT engines for this Jetson.
#
#   scripts/fetch_detector_models.sh [model ...] [--gpu]
#     models: yolox_nano, yolox_tiny (default), yolox_s
#     --gpu: also build GPU engines (about 20 minutes each on the Xavier NX; DLA engines take about 1 minute)
#
# Files go to ~/.local/share/autocode-bot/models (object_detector's model_dir): <model>.onnx,
# <model>_dla.engine and, with --gpu, <model>_gpu.engine. Engines are tied to the TensorRT version and
# device, so rebuild them after a JetPack upgrade (delete the .engine files and run this again).
set -euo pipefail

DIR="${MODEL_DIR:-$HOME/.local/share/autocode-bot/models}"
URL=https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0
TRTEXEC=/usr/src/tensorrt/bin/trtexec
declare -A SHA256=(
  [yolox_nano]=c789161ed43c8269fcd4e67c67eeeb4e80c622da2eb296a20bc6007bd18a0b7d
  [yolox_tiny]=427cc366d34e27ff7a03e2899b5e3671425c262ea2291f88bb942bc1cc70b0f7
  [yolox_s]=c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063
)

models=()
gpu=false
for a in "$@"; do
  case "$a" in
    --gpu) gpu=true ;;
    -h|--help) sed -n '2,10p' "$0"; exit 0 ;;
    *) [[ -n "${SHA256[$a]:-}" ]] || { echo "unknown model: $a (${!SHA256[*]})" >&2; exit 1; }; models+=("$a") ;;
  esac
done
[[ ${#models[@]} -gt 0 ]] || models=(yolox_tiny)
[[ -x "$TRTEXEC" ]] || { echo "trtexec not found at $TRTEXEC (TensorRT from JetPack)" >&2; exit 1; }
mkdir -p "$DIR"

for m in "${models[@]}"; do
  onnx="$DIR/$m.onnx"
  if ! echo "${SHA256[$m]}  $onnx" | sha256sum -c --status 2>/dev/null; then
    echo "[$m] downloading"
    curl -fsSL -o "$onnx.part" "$URL/$m.onnx"
    echo "${SHA256[$m]}  $onnx.part" | sha256sum -c --status || { rm -f "$onnx.part"; echo "[$m] checksum mismatch" >&2; exit 1; }
    mv "$onnx.part" "$onnx"
  fi
  targets=(dla)
  $gpu && targets+=(gpu)
  for t in "${targets[@]}"; do
    engine="$DIR/${m}_$t.engine"
    if [[ -f "$engine" ]]; then
      echo "[$m] $t engine exists: $engine"
      continue
    fi
    extra=()
    [[ $t == dla ]] && extra=(--useDLACore=0 --allowGPUFallback)
    echo "[$m] building $t engine (FP16)..."
    "$TRTEXEC" --onnx="$onnx" --fp16 "${extra[@]}" --saveEngine="$engine.part" > "$DIR/${m}_$t.build.log" 2>&1 \
      || { echo "[$m] build failed, see $DIR/${m}_$t.build.log" >&2; rm -f "$engine.part"; exit 1; }
    mv "$engine.part" "$engine"
    echo "[$m] $t engine ready: $engine"
  done
done
