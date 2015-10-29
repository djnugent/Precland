#!/bin/bash
mavproxy.py --master=udpout:127.0.0.1:14560 --cmd="module load droneapi.module.api; api start /home/root/precland/Precland/PrecisionLand.py"
