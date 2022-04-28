#!/bin/bash
wget https://otter.itk.ntnu.no/software/dune-2020.01.0--bit-linux-glibc-gcc94.tar.bz2
rm -r dune-old
mv dune dune-old
tar -xvjf dune-2020.01.0--bit-linux-glibc-gcc94.tar.bz2
mv dune-2020.01.0--bit-linux-glibc-gcc94 dune
