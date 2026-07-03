#!/usr/bin/env bash
# Download FBoW ORB vocabulary for autonomy_slam / atlas.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VOCAB_URL="https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow"
OUTPUT="${SCRIPT_DIR}/orb_vocab.fbow"

if [[ -f "${OUTPUT}" ]]; then
  echo "Vocabulary already exists: ${OUTPUT}"
  ls -lh "${OUTPUT}"
  exit 0
fi

echo "Downloading ORB vocabulary from GitHub..."
echo "  URL:    ${VOCAB_URL}"
echo "  Output: ${OUTPUT}"

curl -fSL "${VOCAB_URL}" -o "${OUTPUT}.part"
mv "${OUTPUT}.part" "${OUTPUT}"

echo "Done."
ls -lh "${OUTPUT}"
