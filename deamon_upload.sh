#!/bin/bash

fswatch -o STM32CubeExpansion_LRWAN_V1.1.4/Projects/Multi/Applications/LoRa/End_Node/SW4STM32/B-L072Z-LRWAN1/mlm32l07x01/Debug/mlm32l07x01.bin | xargs -n1 ./up-bin_stm32.sh
