next version
============

New Features/API changes
------------------------

* change the whole API to make mpc-walkgen templated. Instantiate the
  class templates for float and double. Both can be used at the same time.

* improve push recovery formulation. Push recovery needs to run in double
  precision. For regular moves float precisaion is enough.
