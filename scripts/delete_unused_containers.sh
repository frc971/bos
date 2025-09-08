HOST="$1"

ssh -t "$HOST" "docker container prune -f"

