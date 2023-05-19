#!/bin/bash

gnuplot

plot './build/vehicle_gateway_sim_performance/test/system_collector_1.csv' using 1:2 with lines title 'CPU - 1 vehicle vehicles', \
     './build/vehicle_gateway_sim_performance/test/system_collector_3.csv' using 1:2 with lines title 'CPU - 3 vehicles', \
     './build/vehicle_gateway_sim_performance/test/system_collector_5.csv' using 1:2 with lines title 'CPU - 5 vehicles', \
     './build/vehicle_gateway_sim_performance/test/system_collector_7.csv' using 1:2 with lines title 'CPU - 7 vehicles', \
     './build/vehicle_gateway_sim_performance/test/system_collector_9.csv' using 1:2 with lines title 'CPU - 9'
set term qt 2 size 500,300

plot './build/vehicle_gateway_sim_performance/test/system_collector_1.csv' using 1:3 with lines title 'Memory - 1 vehicles', \
     './build/vehicle_gateway_sim_performance/test/system_collector_3.csv' using 1:3 with lines title 'Memory - 3 vehicles', \
     './build/vehicle_gateway_sim_performance/test/system_collector_5.csv' using 1:3 with lines title 'Memory - 5 vehicles', \
     './build/vehicle_gateway_sim_performance/test/system_collector_7.csv' using 1:3 with lines title 'Memory - 7 vehicles', \
     './build/vehicle_gateway_sim_performance/test/system_collector_9.csv' using 1:3 with lines title 'Memory - 9 vehicles'


plot './build/vehicle_gateway_sim_performance/test/system_collector_1.csv' using 1:4 with lines title 'RTF - 1 vehicles', \
     './build/vehicle_gateway_sim_performance/test/system_collector_9.csv' using 1:4 with lines title 'RTF - 9 vehicles', \
     './build/vehicle_gateway_sim_performance/test/system_collector_13.csv' using 1:4 with lines title 'RTF - 13 vehicles', \
     './build/vehicle_gateway_sim_performance/test/system_collector_19.csv' using 1:4 with lines title 'RTF - 19 vehicles', \
     './build/vehicle_gateway_sim_performance/test/system_collector_31.csv' using 1:4 with lines title 'RTF - 31 vehicles'
