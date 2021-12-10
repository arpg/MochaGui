#!/bin/bash
cd "$(dirname "$0")"; ./build/devel/lib/MochaGui/gui -params ./N02_params.csv -mesh ./disk.ply -localizer true -mode Simulation -logfile 1.log -car ./herbie/herbie.blend -wheel ./herbie/wheel.blend
