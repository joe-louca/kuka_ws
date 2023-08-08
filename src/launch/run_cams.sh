#!/bin/bash

# Start camera viewer and recorder
cd ~/kuka_ws/src/scripts/src/gui && python3 cam1_show.py &
cd ~/kuka_ws/src/scripts/src/gui && python3 cam2_show.py &
cd ~/kuka_ws/src/scripts/src/gui && python3 cam3_show.py
