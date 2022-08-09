#!/bin/bash

ipcrm -m `ipcs -m | grep 0x1d021326 | awk '{print $2}'`
