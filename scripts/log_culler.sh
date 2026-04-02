cd log44/left
for f in *.jpg; do
  timestamp=${f%%.*} # strip extension
  if (($(echo "$timestamp < 348" | bc -l))); then
    rm "$f"
  fi
done
