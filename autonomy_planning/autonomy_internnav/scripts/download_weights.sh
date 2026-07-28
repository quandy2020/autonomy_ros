#!/usr/bin/env bash
# Download NavDP / InternNav weights referenced by JdInternNav into autonomy_internnav/weights.
#
# JdInternNav 仓库本身不含 .ckpt（仅引用 checkpoints/），本脚本下载公开可获取的权重。
# navdp-cross-modal.ckpt 需通过官方表单申请，若已存在则跳过。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WEIGHTS_DIR="${PKG_ROOT}/weights"
TRAIN_CKPT_DIR="${PKG_ROOT}/checkpoints"

mkdir -p "${WEIGHTS_DIR}" "${TRAIN_CKPT_DIR}"

download_if_missing() {
  local url="$1"
  local dest="$2"
  if [[ -f "${dest}" ]] && [[ -s "${dest}" ]]; then
    echo "[skip] exists: ${dest}"
    return 0
  fi
  echo "[download] ${url}"
  echo "         -> ${dest}"
  curl -fL --retry 3 --connect-timeout 30 -o "${dest}.part" "${url}"
  mv "${dest}.part" "${dest}"
}

echo "=== autonomy_internnav weights -> ${WEIGHTS_DIR} ==="

# 1) NavDP 推理默认权重（表单申请；已有则跳过）
NAVDP_CKPT="${WEIGHTS_DIR}/navdp-cross-modal.ckpt"
if [[ -f "${NAVDP_CKPT}" ]] && [[ -s "${NAVDP_CKPT}" ]]; then
  echo "[skip] NavDP: ${NAVDP_CKPT} ($(du -h "${NAVDP_CKPT}" | cut -f1))"
else
  echo "[manual] navdp-cross-modal.ckpt 不在本地。"
  echo "  请从 NavDP 官方表单下载后放到:"
  echo "    ${NAVDP_CKPT}"
  echo "  表单: https://docs.google.com/forms/d/e/1FAIpQLSdl3RvajO5AohwWZL5C0yM-gkSqrNaLGp1OzN9oF24oNLfikw/viewform"
  # 若 JdInternNav 本地有 checkpoints/，可手动复制:
  JD_CKPT="/home/quandy/workspace/project/jdbot/src/JdInternNav/checkpoints/navdp-cross-modal.ckpt"
  if [[ -f "${JD_CKPT}" ]]; then
    cp -v "${JD_CKPT}" "${NAVDP_CKPT}"
  fi
fi

# 2) DepthAnything V2 ViT-S（训练 backbone / JdInternNav checkpoints/depth_anything_v2_vits.pth）
DA_VITS="${WEIGHTS_DIR}/depth_anything_v2_vits.pth"
download_if_missing \
  "https://huggingface.co/depth-anything/Depth-Anything-V2-Small/resolve/main/depth_anything_v2_vits.pth" \
  "${DA_VITS}"
ln -sf "../weights/depth_anything_v2_vits.pth" "${TRAIN_CKPT_DIR}/depth_anything_v2_vits.pth"

# 3) DepthAnything Metric Hypersim（InternVLA-N1 S1 / JdInternNav internvla）
DA_METRIC="${WEIGHTS_DIR}/depth_anything_v2_metric_hypersim_vits.pth"
download_if_missing \
  "https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-Hypersim-Small/resolve/main/depth_anything_v2_metric_hypersim_vits.pth" \
  "${DA_METRIC}"

echo ""
echo "Done. weights/:"
ls -lh "${WEIGHTS_DIR}"
echo ""
echo "训练时 DepthAnything 软链: ${TRAIN_CKPT_DIR}/depth_anything_v2_vits.pth"
