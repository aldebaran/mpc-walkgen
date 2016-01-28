mpc-walkgen
===========

This software provides...

Setup
-----

### Dependencies

The package mpc-walkgen depends on several packages which
have to be available on your machine.

 - System tools:
   - CMake (>=2.8)
   - pkg-config
   - usual compilation tools (GCC/G++, make, etc.)

 - Libraries
   - boost (>=1.40)
   - eigen (>=3.0.0)
   - qpoases (version 2.0, compiled twice for both float and double, with
     special care taken to avoid symbol collisions)

### Compile

The project is installed and built thanks to qibuild
(https://github.com/aldebaran/qibuild).

Once you have qibuild installed, simply run

    qibuild configure
    qibuild make

Design choices
--------------

### API

At some stage, the lib might be split in two in order to separate the humanoid
and zebulon parts (the public and currently secret parts).
The common pieces used by these two might go in a third "common" library.

In this spirit it was decided that most of the header could be kept public.


### Template instantiations

Just like Eigen, mpc-walkgen is templated by a scalar type. However unlike
Eigen, mpc-walkgen is not a head-only library. Its algorithms are not
implemented in headers the user includes, but in a shared library the
user links with.

This means:

* the algorithms are not implemented in (public) headers but in (private)
  cpp files.

* the templates are not instantiated in the user code but in the mpc-walkgen
  library (there are explicit instantiation for float and double in each cpp
  files)

* the user code will be compiled faster (no need to instantiate mpc-walkgen
  templates in it)

* the user code can be compile in debug while using mpc-walkgen compiled in
  release

* the user cannot extend mpc-walkgen to support scalar types besides float
  and double. (Note that we currently have no QP solver for other scalar types
  anyway.)

