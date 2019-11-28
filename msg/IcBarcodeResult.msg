# A barcode detection within an image.
#
# This message corresponds to the content of a ICBarcode_Result
#
# struct ICBarcode_Result
# {
#     char Text[4096];
#     char RawBytes[4096];
#     int RawBytesLength;
#     float Frequency;
#     TIS_PointF ResultPoints[4];
#     int ResultPointsLength;
#     int Format;
# };

string                        Text
uint8[]                       RawBytes
float32                       Frequency
geometry_msgs/Point32[]       ResultPoints


# Definition of enum ICBarcode_Format
int32                         IC_BARCODEFORMAT_NONE               = 0
int32                         IC_BARCODEFORMAT_AZTEC              = 2
int32                         IC_BARCODEFORMAT_CODE_39            = 8
int32                         IC_BARCODEFORMAT_CODE_93            = 16
int32                         IC_BARCODEFORMAT_CODE_128           = 32
int32                         IC_BARCODEFORMAT_DATA_MATRIX        = 64
int32                         IC_BARCODEFORMAT_EAN_8              = 128
int32                         IC_BARCODEFORMAT_EAN_13             = 256
int32                         IC_BARCODEFORMAT_MAXICODE           = 1024
int32                         IC_BARCODEFORMAT_PDF_417            = 2048
int32                         IC_BARCODEFORMAT_QR_CODE            = 4096
int32                         IC_BARCODEFORMAT_UPC_A              = 32768
int32                         IC_BARCODEFORMAT_UPC_E              = 65536
int32                         IC_BARCODEFORMAT_UPC_EAN_EXTENSION  = 131072
int32                         IC_BARCODEFORMAT_INTERLEAVED_2_OF_5 = 262144
int32                         IC_BARCODEFORMAT_1D                 = 491960
int32                         IC_BARCODEFORMAT_2D                 = 7234
int32                         IC_BARCODEFORMAT_ALL                = 499194

int32                         Format
