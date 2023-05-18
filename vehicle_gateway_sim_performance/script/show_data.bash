#!/bin/bash

gnuplot

plot './build/vehicle_gateway_sim_performance/test/system_collector_1.csv' using 1:2 with lines title 'CPU 1', \
     './build/vehicle_gateway_sim_performance/test/system_collector_3.csv' using 1:2 with lines title 'CPU 3', \
     './build/vehicle_gateway_sim_performance/test/system_collector_5.csv' using 1:2 with lines title 'CPU 5', \
     './build/vehicle_gateway_sim_performance/test/system_collector_7.csv' using 1:2 with lines title 'CPU 7', \
     './build/vehicle_gateway_sim_performance/test/system_collector_9.csv' using 1:2 with lines title 'CPU 9'
set term qt 2 size 500,300

plot './build/vehicle_gateway_sim_performance/test/system_collector_1.csv' using 1:3 with lines title 'Memory 1', \
     './build/vehicle_gateway_sim_performance/test/system_collector_3.csv' using 1:3 with lines title 'Memory 3', \
     './build/vehicle_gateway_sim_performance/test/system_collector_5.csv' using 1:3 with lines title 'Memory 5', \
     './build/vehicle_gateway_sim_performance/test/system_collector_7.csv' using 1:3 with lines title 'Memory 7', \
     './build/vehicle_gateway_sim_performance/test/system_collector_9.csv' using 1:3 with lines title 'Memory 9'
