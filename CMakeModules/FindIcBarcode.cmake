
find_package(PkgConfig QUIET)

if( PKG_CONFIG_FOUND )
  pkg_check_modules( ic_barcode ic_barcode )
endif()

if( NOT ic_barcode_FOUND )
  message("ic_barcode could not be found by pkg-config. Trying to manually find ic_barcode.")
  find_path(ic_barcode_INCLUDE_DIRS ic_barcode.h
    PATHS
    "$ENV{IC_BARCODE_INCLUDE_PATH}"
    /usr/local/include/
    /usr/include/
  )
  find_library(IC_BARCODE_LIBRARIES aravis-0.6
    PATHS
    "$ENV{IC_BARCODE_LIBRARY}"
    /usr/local/lib
    /usr/lib
    /usr/lib/x86_64-linux-gnu
  )
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args( IC_BARCODE DEFAULT_MSG
    IC_BARCODE_INCLUDE_DIRS
    IC_BARCODE_LIBRARIES
  )
endif()
    
