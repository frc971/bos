#!/usr/bin/env bash
set -euo pipefail

IMAGE_FOLDER="${1:-/bos/frames/gamepiece_camera}"
FPS="${2:-30}"

if [[ ! -d "$IMAGE_FOLDER" ]]; then
  echo "Image folder does not exist: $IMAGE_FOLDER" >&2
  exit 1
fi

if ! awk -v fps="$FPS" 'BEGIN { exit !(fps > 0) }'; then
  echo "FPS must be greater than zero: $FPS" >&2
  exit 1
fi

mapfile -d '' files < <(
  find "$IMAGE_FOLDER" -maxdepth 1 -type f \
    \( -iname '*.jpg' -o -iname '*.jpeg' -o -iname '*.png' \) \
    -printf '%f\0' | sort -z -V
)

if (( ${#files[@]} == 0 )); then
  echo "No image files found in: $IMAGE_FOLDER" >&2
  exit 1
fi

temporary_folder="$(mktemp -d "$IMAGE_FOLDER/.rename-frames.XXXXXX")"
declare -a temporary_files=()

restore_on_failure() {
  local status=$?
  if [[ -d "$temporary_folder" ]]; then
    for i in "${!temporary_files[@]}"; do
      if [[ -e "${temporary_files[$i]}" ]]; then
        mv -- "${temporary_files[$i]}" "$IMAGE_FOLDER/${files[$i]}"
      fi
    done
    rmdir "$temporary_folder" 2>/dev/null || true
  fi
  exit "$status"
}
trap restore_on_failure EXIT

# Move everything aside first so neither the old names nor the new names collide.
for i in "${!files[@]}"; do
  temporary_files[$i]="$temporary_folder/$i"
  mv -- "$IMAGE_FOLDER/${files[$i]}" "${temporary_files[$i]}"
done

for i in "${!files[@]}"; do
  timestamp="$(awk -v frame="$i" -v fps="$FPS" \
    'BEGIN { printf "%.6f", frame / fps }')"
  mv -- "${temporary_files[$i]}" "$IMAGE_FOLDER/$timestamp.jpg"
done

rmdir "$temporary_folder"
trap - EXIT
echo "Renamed ${#files[@]} images in $IMAGE_FOLDER at ${FPS} FPS."
