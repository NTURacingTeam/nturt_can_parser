#!/bin/bash

if [[ $@ == '-h' || $@ == '--help' ]]; then
    echo 'Put your frames into doc/frames and enter the command "./combine.sh", then this script will combine all of them into "can.yaml"'.
fi

echo 'can:' > ../doc/can.yaml
cat ../doc/frames/*.yaml >> ../doc/can.yaml
echo "combine finished"
