#!/bin/bash

make && ./interactive_icp arts.ply ssms.ply 20 \
			  -0.8819646192208759 -0.36252718381389526 0.6798311024876594 -0.030573786909470213 \
			  0.013973396528450782 1.0256515474838328 0.5650677161230486 -0.2646142461505974 \
			  -0.7703252761678527 0.4336713278636562 -0.768105477624616 2.457226580844369 \
			  0.0 0.0 0.0 1.0
