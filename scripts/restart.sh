HOST="$1"

ssh -t "$HOST" 'docker stop $(docker ps -q)'
ssh -t "$HOST" 'docker run -v /bos:/bos --rm orin /bos/main'
