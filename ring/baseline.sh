#!/bin/bash

# echo "flow,duration,cap,seed,result" > 'mp_baseline.txt' #0.33 0.33 2 4 | 0.2 0.25 0.33 

for flow in 0.2 0.25 0.33 0.5 1 2 3 4 5
do
	for duration in 0.2 0.25 0.33 0.5
    do
        for cap in 1 2 3 4 5 6 7 8 9 
        do
            for seed in 0 1 2 3 4
            do
                python main.py -f $flow -d $duration -c $cap -s $seed
            done
        done
    done
done