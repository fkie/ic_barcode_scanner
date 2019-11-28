IC Barcode Scanner
==================

Summary
-------

The `ic_barcode_scanner` provides a [ROS](http://www.ros.org) wrapper nodelet/node
to the IC-Barcode library of *The Imaging Source* (TIS).

See https://www.theimagingsource.com/support/documentation/ic-barcode/ for the
API documentation of IC-Barcode and 
https://www.theimagingsource.com/support/downloads-for-linux/install/tisicbarcode/
for the download.

Please note that IC-Barcode is proprietary and only runs with a TIS camera
connected to your computer (however it then accepts images from any source).

Note that you could run all modern cameras of TIS inside ROS using `camera_aravis`.

Requirements
------------

The `ic_barcode_scanner` node/nodelet requires C++11 or better. Obviously, it
depends on the proprietary IC-Barcode library, which can be obtained from TIS
for free. Also, it depends on `cv_bridge` and `OpenCV` for the conversion of input
images to *mono8* format and visualizing the detections.

IcBarcodeScannerNodelet
-----------------------

The wrapper is implemented in `IcBarcodeScannerNodelet`. There is also a standalone
node version `IcBarcodeScannerNode` with the same functionality. The nodelet/node
only subscribes to the image input on demand, i.e. if somebody listens to a barcode
topic.

###Parameters
*   `~barcode_formats` (`integer`, default `ICBarcode_Format::IC_BARCODEFORMAT_ALL`)

    A bitmask which defines what barcode formats we are looking for. See `enum ICBarcode_Format`
    of IC-Barcode for available options. For convenience, one could also (de-)select formats
    using `dynamic_reconfigure`.
*   `~max_barcodes` (`integer`, default `10`)

    Maximal number of reported barcodes per image.
*   `~image_transport` (`string`, default `raw`)

    Transport hints which image (compressed) format should be used.

*   all options of `enum ICBarcode_Param` from IC-Barcode

    e.g. ICBarcode_Param::IC_BARCODEPARAMS_ORIENTATION becomes `~orientation` (`integer`)

###Received Topics
*   `~image` (`sensor_msgs/Image`)

###Published Topics
*   `~barcodes` (`ic_barcode_scanner/IcBarcodeResults`)

    Detections of barcodes. It contains positions as well as the carried information.
    A vector of `IcBarcodeResult` with a `Header` corresponding to the original `Image`.
*   `~image_barcodes` (`sensor_msgs/Image`)

    Barcode positions drawn as boundaries on the original `Image`.

###Service
*   `find_barcodes`

    `Request`: `sensor_msgs/Image`

    `Response`: `ic_barcode_scanner/IcBarcodeResults`


