mpc-walkgen
===========

This software provides ...

Setup
-----

To compile this package, it is recommended to create a separate build
directory:

    mkdir _build
    cd _build
    cmake [OPTIONS] ..
    make install

Please note that CMake produces a `CMakeCache.txt` file which should
be deleted to reconfigure a package from scratch.


### Dependencies

The package mpc-walkgen depends on several packages which
have to be available on your machine.

 - System tools:
   - CMake (>=2.6)
   - pkg-config
   - usual compilation tools (GCC/G++, make, etc.)

 - Libraries
   - boost (>=1.40)
   - eigen (>=3.0.0)
   - lssol.


### Installation of lssol.

Lssol is a software package for solving constrained linear least-squares problems and convex quadratic programs.
It is distributed by Standford Business Software Inc.
http://www.sbsi-sol-optimize.com/asp/sol_product_lssol.htm

In order to link the lssol libraries with this project, it is necessary
to install manually two extra files, located in share/lssol:
- The header lssol.h, that defines methods to call lssol routines.
  (the install should be {include_folder}/lssol/lssol.h

- The config file lssol.pc, that defines the compilation flags and link flags
   required. It is mandatory to customize it according to your installation.
   Please add the path to this file into your environment variable
   PKG_CONFIG_PATH

Once these steps done, you should be able to compile and link with lssol.
Please run the unit test test-lssol to verify that the solver has been installed
correctly.


