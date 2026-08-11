#/bin/bash

docker cp ~/Documents/bos/constants/navgrid.json orin:/root/bos/constants/navgrid.json

./scripts/deploy.sh nvidia@10.99.71.11 false