#!/bin/bash

BASHRC="$HOME/.bashrc"
BASH_ALIASES="$HOME/.bash_aliases"

ALIASES=(
    "alias gf='git fetch'"
    "alias gs='git status'"
    "alias ..='git switch main && git pull origin main'"
)

# Ensure .bashrc sources .bash_aliases
if ! grep -qF ".bash_aliases" "$BASHRC"; then
    echo "" >> "$BASHRC"
    echo "# Load custom aliases" >> "$BASHRC"
    echo "if [ -f ~/.bash_aliases ]; then" >> "$BASHRC"
    echo "    . ~/.bash_aliases" >> "$BASHRC"
    echo "fi" >> "$BASHRC"
    echo "Added .bash_aliases sourcing to .bashrc"
fi

# Add aliases to .bash_aliases if missing
touch "$BASH_ALIASES"
for alias_line in "${ALIASES[@]}"; do
    if ! grep -Fxq "$alias_line" "$BASH_ALIASES"; then
        echo "$alias_line" >> "$BASH_ALIASES"
        echo "Added: $alias_line"
    else
        echo "Already exists: $alias_line"
    fi
done

# Reload aliases now
source "$BASH_ALIASES"

