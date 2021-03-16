
find_package(PkgConfig QUIET)

if( PKG_CONFIG_FOUND )
  pkg_check_modules( IcBarcode ic_barcode )
endif()

if( NOT IcBarcode_FOUND )
  message("ic_barcode could not be found by pkg-config. Trying to manually find ic_barcode.")
  find_path(IcBarcode_INCLUDE_DIRS ic_barcode.h
    PATHS
    "$ENV{IC_BARCODE_INCLUDE_PATH}"
    /usr/local/include/
    /usr/include/
  )
  find_library(IcBarcode_LIBRARIES ic_barcode
    PATHS
    "$ENV{IC_BARCODE_LIBRARY}"
    /usr/local/lib
    /usr/lib
    /usr/lib/x86_64-linux-gnu
  )
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args( IC_BARCODE DEFAULT_MSG
    IcBarcode_INCLUDE_DIRS
    IcBarcode_LIBRARIES
  )
endif()
    
