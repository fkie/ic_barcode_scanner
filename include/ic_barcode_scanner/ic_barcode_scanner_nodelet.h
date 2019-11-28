/****************************************************************************
 *
 * ic_barcode_scanner
 *
 * Copyright © 2019 Fraunhofer FKIE
 * Authors: Dominik A. Klein,
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

#ifndef INCLUDE_IC_BARCODE_SCANNER_IC_BARCODE_SCANNER_NODELET_H_
#define INCLUDE_IC_BARCODE_SCANNER_IC_BARCODE_SCANNER_NODELET_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <ic_barcode_scanner/IcBarcodeResults.h>
#include <dynamic_reconfigure/server.h>
#include <ic_barcode_scanner/IcBarcodeConfig.h>
#include <ic_barcode_scanner/find_barcodes.h>

#include <ic_barcode.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <memory>
#include <string>
#include <functional>

namespace ic_barcode_scanner {

typedef IcBarcodeConfig Config;

static const std::map<std::string, ICBarcode_Param> IC_BARCODEPARAMS = {
		{ "check_char",				IC_BARCODEPARAMS_CHECK_CHAR },
		{ "num_scanlines",			IC_BARCODEPARAMS_NUM_SCANLINES },
		{ "element_size_min",		IC_BARCODEPARAMS_ELEMENT_SIZE_MIN },
		{ "element_size_max",		IC_BARCODEPARAMS_ELEMENT_SIZE_MAX },
		{ "orientation",			IC_BARCODEPARAMS_ORIENTATION },
		{ "orientation_tol",		IC_BARCODEPARAMS_ORIENTATION_TOL },
		{ "stop_after_num_results", IC_BARCODEPARAMS_STOP_AFTER_NUM_RESULTS },
		{ "timeout",				IC_BARCODEPARAMS_TIMEOUT },
		{ "min_width",				IC_BARCODEPARAMS_MIN_WIDTH },
		{ "min_height",				IC_BARCODEPARAMS_MIN_HEIGHT },
		{ "max_width",				IC_BARCODEPARAMS_MAX_WIDTH },
		{ "max_height",				IC_BARCODEPARAMS_MAX_HEIGHT },
		{ "preprocess",				IC_BARCODEPARAMS_PREPROCESS },
		{ "try_harder",				IC_BARCODEPARAMS_TRY_HARDER },
		{ "ic_only",				IC_BARCODEPARAMS_IC_ONLY },
		{ "fixed_data_length",		IC_BARCODEPARAMS_FIXED_DATA_LENGTH },
		{ "store_debug_info",		IC_BARCODEPARAMS_STORE_DEBUG_INFO },
		{ "use_multithreading",		IC_BARCODEPARAMS_USE_MULTITHREADING }
};

/**
 * \class IcBarcodeScannerNodelet
 * \brief Scans for different kinds of 1d and 2d barcodes in images.
 * This nodelet wraps around <a href="https://www.theimagingsource.com/support/downloads-for-linux/install/tisicbarcode/">IC-Barcode library</a>
 * provided by The Imaging Source together with their cameras in order
 * to detect different kinds of barcodes in a given image.
 * You can stream input images on a topic or call the service with
 * a single image. Output is provided as an IcBarcodeResults message
 * containing a vector of barcode locations and decoded information.
 * In addition, the streaming version provides an output image stream
 * with boundaries drawn around detections.
 */
class IcBarcodeScannerNodelet: public nodelet::Nodelet {
public:
	virtual ~IcBarcodeScannerNodelet();
	IcBarcodeScannerNodelet();

	/*!
	 * \brief Converts a IC-Barcode detection into an IcBarcodeResult message.
	 */
	static IcBarcodeResult createMsgFromResult(const ICBarcode_Result& icb_result);

private:
	virtual void onInit() override;

protected:
	/*!
	 * \brief Scans the given image for barcodes and publishs the results.
	 */
	void scanImageCallback(const sensor_msgs::ImageConstPtr& img_msg);

	/*!
	 * \brief Maintains the image subscription depending on our audience.
	 */
	void connectCallback();

	/*!
	 * \brief Use dynamic reconfigure to (de-)activate recognition of different barcode formats.
	 */
	void dynReconfigureCallback(Config &config, uint32_t level);

	/*!
	 * \brief read configuration from parameter server and apply it to IC-Barcode.
	 */
	void readICBarcodeParams();

	/*!
	 * \brief The service version to scan a single image for barcodes using IC-Barcode.
	 */
	bool findBarcodesSrv(find_barcodes::Request& req, find_barcodes::Response& res);



	ICBarcode_Scanner*										p_scanner_				=		NULL;

	Config 													dyn_config_;
	std::unique_ptr<dynamic_reconfigure::Server<Config> >	reconfigure_server_;
	boost::recursive_mutex									reconfigure_mutex_;

	std::unique_ptr<image_transport::ImageTransport> 		it_;
	std::string												transport_hints_		=		"raw";
	image_transport::Subscriber								image_sub_;
	image_transport::Publisher								image_pub_;
	ros::Publisher					 						barcodes_pub_;
	ros::ServiceServer										find_barcodes_srv_;

	int														barcode_formats_		=		IC_BARCODEFORMAT_ALL;
	int														max_barcodes_			=		10;

};

} /* namespace ic_barcode_scanner */

#endif /* INCLUDE_IC_BARCODE_SCANNER_IC_BARCODE_SCANNER_NODELET_H_ */
