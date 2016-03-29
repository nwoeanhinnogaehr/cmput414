#!/bin/bash
../cmput414_bin ../screwdriver.off
compare before.png after.png -compose src diff.png
