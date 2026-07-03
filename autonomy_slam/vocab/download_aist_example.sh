#!/usr/bin/env bash
# Download stella_vslam AIST Living Lab sample datasets from Google Drive.
# Reference: https://stella-cv.readthedocs.io/en/latest/simple_tutorial.html
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_DIR="${1:-${SCRIPT_DIR}/../data/aist_living_lab}"

is_valid_zip() {
  local file="$1"
  [[ -f "${file}" ]] || return 1
  [[ "$(stat -c%s "${file}" 2>/dev/null || stat -f%z "${file}")" -gt 100000 ]] || return 1
  [[ "$(head -c 2 "${file}")" == $'PK' ]]
}

download_gdrive() {
  local file_id="$1"
  local output="$2"

  if is_valid_zip "${output}"; then
    echo "Already exists: ${output}"
    return 0
  fi

  if [[ -f "${output}" ]]; then
    echo "Removing invalid download: ${output}"
    rm -f "${output}"
  fi

  echo "Downloading ${output} ..."
  # Large files need confirm=t; older cookie-based flow often saves HTML instead of zip.
  if ! curl -fSL \
    "https://drive.usercontent.google.com/download?id=${file_id}&export=download&confirm=t" \
    -o "${output}.part"; then
    rm -f "${output}.part"
    echo "Download failed for ${output}" >&2
    return 1
  fi

  if ! is_valid_zip "${output}.part"; then
    rm -f "${output}.part"
    echo "Downloaded file is not a valid zip: ${output}" >&2
    return 1
  fi

  mv "${output}.part" "${output}"
  ls -lh "${output}"
}

mkdir -p "${OUTPUT_DIR}"
cd "${OUTPUT_DIR}"

download_gdrive "1d8kADKWBptEqTF7jEVhKatBEdN7g0ikY" "aist_living_lab_1.zip"
download_gdrive "1TVf2D2QvMZPHsFoTb7HNxbXclPoFMGLX" "aist_living_lab_2.zip"

for zip in aist_living_lab_1.zip aist_living_lab_2.zip; do
  dir="${zip%.zip}"
  if [[ -d "${dir}" ]]; then
    echo "Already extracted: ${dir}/"
    continue
  fi
  echo "Extracting ${zip} ..."
  unzip -q "${zip}"
done

echo "Datasets ready under: ${OUTPUT_DIR}"
ls -la "${OUTPUT_DIR}"
