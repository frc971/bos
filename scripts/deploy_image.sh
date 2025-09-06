HOST="$1"

if [ -z "$HOST" ]; then
  echo "Usage: $0 <host>"
  exit 1
fi

rsync -avz image.tar "$HOST":/bos/image.tar
ssh -t "$HOST" 'docker load -i /bos/image.tar'
