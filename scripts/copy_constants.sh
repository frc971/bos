if [ "$(pwd)" != "/bos" ]; then
  mkdir -p /bos
  sudo cp -r constants /bos
fi
