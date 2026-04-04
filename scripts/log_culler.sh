cd "$1"
for f in *.jpg; do
  timestamp=${f%%.*}
  if (($(echo "$timestamp < $2" | bc -l))) | (($(echo "$timestamp > $3" | bc -l))); then
    rm "$f"
  fi
done
