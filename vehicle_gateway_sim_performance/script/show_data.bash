#!/bin/bash

gnuplot

set term qt 1 size 640,480

plot './build/vehicle_gateway_sim_performance/test/system_collector_1.csv' using 1:2 with lines title 'CPU - 1 vehicle vehicles', \
     './build/vehicle_gateway_sim_performance/test/system_collector_3.csv' using 1:2 with lines title 'CPU - 3 vehicles', \
     './build/vehicle_gateway_sim_performance/test/system_collector_5.csv' using 1:2 with lines title 'CPU - 5 vehicles', \
     './build/vehicle_gateway_sim_performance/test/system_collector_7.csv' using 1:2 with lines title 'CPU - 7 vehicles', \
     './build/vehicle_gateway_sim_performance/test/system_collector_9.csv' using 1:2 with lines title 'CPU - 9'
set term qt 2 size 640,480

plot './build/vehicle_gateway_sim_performance/test/system_collector_1.csv' using 1:3 with lines title 'Memory - 1 vehicles', \
     './build/vehicle_gateway_sim_performance/test/system_collector_3.csv' using 1:3 with lines title 'Memory - 3 vehicles', \
     './build/vehicle_gateway_sim_performance/test/system_collector_5.csv' using 1:3 with lines title 'Memory - 5 vehicles', \
     './build/vehicle_gateway_sim_performance/test/system_collector_7.csv' using 1:3 with lines title 'Memory - 7 vehicles', \
     './build/vehicle_gateway_sim_performance/test/system_collector_9.csv' using 1:3 with lines title 'Memory - 9 vehicles'

set term qt 3 size 640,480

plot './build/vehicle_gateway_sim_performance/test/system_collector_1.csv' using 1:4 with lines title 'RFT - 1 vehicles', \
     './build/vehicle_gateway_sim_performance/test/system_collector_3.csv' using 1:4 with lines title 'RFT - 3 vehicles', \
     './build/vehicle_gateway_sim_performance/test/system_collector_5.csv' using 1:4 with lines title 'RFT - 5 vehicles', \
     './build/vehicle_gateway_sim_performance/test/system_collector_7.csv' using 1:4 with lines title 'RFT - 7 vehicles', \
     './build/vehicle_gateway_sim_performance/test/system_collector_9.csv' using 1:4 with lines title 'RFT - 9 vehicles'


set term qt 4 size 640,480

plot './build/vehicle_gateway_sim_performance/test/system_collector_1_multi_dds_domain.csv' using 1:2 with lines title 'Multi Domain ID - CPU - 1 vehicle', \
    './build/vehicle_gateway_sim_performance/test/system_collector_3_multi_dds_domain.csv' using 1:2 with lines title 'Multi Domain ID - CPU - 3 vehicles', \
    './build/vehicle_gateway_sim_performance/test/system_collector_5_multi_dds_domain.csv' using 1:2 with lines title 'Multi Domain ID - CPU - 5 vehicles', \
    './build/vehicle_gateway_sim_performance/test/system_collector_7_multi_dds_domain.csv' using 1:2 with lines title 'Multi Domain ID - CPU - 7 vehicles', \
    './build/vehicle_gateway_sim_performance/test/system_collector_9_multi_dds_domain.csv' using 1:2 with lines title 'Multi Domain ID - CPU - 9'
set term qt 5 size 640,480

plot './build/vehicle_gateway_sim_performance/test/system_collector_1_multi_dds_domain.csv' using 1:3 with lines title 'Multi Domain ID - Memory - 1 vehicles', \
    './build/vehicle_gateway_sim_performance/test/system_collector_3_multi_dds_domain.csv' using 1:3 with lines title 'Multi Domain ID - Memory - 3 vehicles', \
    './build/vehicle_gateway_sim_performance/test/system_collector_5_multi_dds_domain.csv' using 1:3 with lines title 'Multi Domain ID - Memory - 5 vehicles', \
    './build/vehicle_gateway_sim_performance/test/system_collector_7_multi_dds_domain.csv' using 1:3 with lines title 'Multi Domain ID - Memory - 7 vehicles', \
    './build/vehicle_gateway_sim_performance/test/system_collector_9_multi_dds_domain.csv' using 1:3 with lines title 'Multi Domain ID - Memory - 9 vehicles'

set term qt 6 size 640,480

plot './build/vehicle_gateway_sim_performance/test/system_collector_1_multi_dds_domain.csv' using 1:4 with lines title 'Multi Domain ID - RFT - 1 vehicles', \
    './build/vehicle_gateway_sim_performance/test/system_collector_3_multi_dds_domain.csv' using 1:4 with lines title 'Multi Domain ID - RFT - 3 vehicles', \
    './build/vehicle_gateway_sim_performance/test/system_collector_5_multi_dds_domain.csv' using 1:4 with lines title 'Multi Domain ID - RFT - 5 vehicles', \
    './build/vehicle_gateway_sim_performance/test/system_collector_7_multi_dds_domain.csv' using 1:4 with lines title 'Multi Domain ID - RFT - 7 vehicles', \
    './build/vehicle_gateway_sim_performance/test/system_collector_9_multi_dds_domain.csv' using 1:4 with lines title 'Multi Domain ID - RFT - 9 vehicles'
