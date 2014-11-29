#!/bin/bash

# Delete the /html folder
rm -R html/

# Create the /html folder
mkdir html

# Use the htlatex command to generate the HTML user manual from the .tex file
htlatex ReactPhysics3D-UserManual.tex "configHTLatex.cfg,html" "" -dhtml/

# Copy the images/ folder into the html/ folder
cp -R images/ html/images/
