## Copyright (C) 2012 Aldebaran Robotics

clean(LSSOL)
fpath(LSSOL "lssol/lssol.h")
flib(LSSOL "liblssol_c.so")
flib(LSSOL "liblssol.so")
flib(LSSOL "libcblas.so" PATH_SUFFIXES "blas/reference")
flib(LSSOL "libblas.so" PATH_SUFFIXES "blas/reference")

flib(LSSOL "libf2c.a")
export_header(LSSOL)
export_lib(LSSOL)
