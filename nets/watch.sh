#!/bin/sh
nc -ulp 5001|  ffplay -fflags nobuffer -fflags discardcorrupt -flags low_delay -framedrop -probesize 32 -f mpegts  -
