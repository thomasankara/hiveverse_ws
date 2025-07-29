#!/bin/bash
cd ~/.gz/fuel/fuel.gazebosim.org/openrobotics/models/
for m in *; do
  if [ -d "$m" ]; then
    last=$(ls "$m" | sort -n | tail -1)
    name=$(echo "$m" | tr ' ' '_')
    if [ ! -e "$name" ]; then
      cp -rs "$m/$last" "$name"
    fi
  fi
done
