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

#include "../include/ic_barcode_scanner/ic_barcode_scanner_nodelet.h"

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ic_barcode_scanner::IcBarcodeScannerNodelet, nodelet::Nodelet)

namespace ic_barcode_scanner {

namespace ph = std::placeholders;

IcBarcodeScannerNodelet::~IcBarcodeScannerNodelet() {
	ICBarcode_DestroyScanner(p_scanner_);
}

IcBarcodeScannerNodelet::IcBarcodeScannerNodelet() {
	p_scanner_ = ICBarcode_CreateScanner();
}

void IcBarcodeScannerNodelet::onInit() {

	if (!p_scanner_) {
		ROS_ERROR("Failed to initialize IC-Barcode-Scanner.");
		return;
	}

	readICBarcodeParams();

	ros::NodeHandle& nh = this->getNodeHandle();
	ros::NodeHandle& pnh = this->getPrivateNodeHandle();

	reconfigure_server_.reset(new dynamic_reconfigure::Server<Config>(reconfigure_mutex_, pnh));
	reconfigure_server_->updateConfig(dyn_config_);
	reconfigure_server_->setCallback(std::bind(&IcBarcodeScannerNodelet::dynReconfigureCallback, this, ph::_1, ph::_2));

	pnh.param<std::string>("image_transport", transport_hints_, transport_hints_);
	it_.reset(new image_transport::ImageTransport(nh));
	ros::SubscriberStatusCallback ros_sscb = [this](const ros::SingleSubscriberPublisher& ssp) { this->connectCallback(); };
	barcodes_pub_ = nh.advertise<ic_barcode_scanner::IcBarcodeResults>(ros::names::remap("barcodes"), 5, ros_sscb, ros_sscb);
	image_transport::SubscriberStatusCallback it_sscb = [this](const image_transport::SingleSubscriberPublisher& ssp) { this->connectCallback(); };
	image_pub_ = it_->advertise(ros::names::remap("image_barcodes"), 1, it_sscb, it_sscb);

	find_barcodes_srv_ = nh.advertiseService("find_barcodes", &IcBarcodeScannerNodelet::findBarcodesSrv, this);
}

void IcBarcodeScannerNodelet::scanImageCallback(const sensor_msgs::ImageConstPtr& img_msg) {

	cv_bridge::CvImagePtr image_mono = cv_bridge::toCvCopy(img_msg, "mono8");

	std::vector<ICBarcode_Result> icb_results(max_barcodes_);

	reconfigure_mutex_.lock();
	const int found_barcodes = ICBarcode_FindBarcodes(p_scanner_, image_mono->image.data, image_mono->image.cols, image_mono->image.rows, image_mono->image.step, icb_results.data(), max_barcodes_);
	reconfigure_mutex_.unlock();

	if (barcodes_pub_.getNumSubscribers() > 0) {
		IcBarcodeResultsPtr barcodes_msg(new IcBarcodeResults);
		barcodes_msg->header = img_msg->header;
		for (int i=0; i<found_barcodes; ++i) {
			barcodes_msg->results.emplace_back(createMsgFromResult(icb_results[i]));
		}
		barcodes_pub_.publish(barcodes_msg);
	}

	if (image_pub_.getNumSubscribers() > 0) {
		if (found_barcodes > 0) {
			sensor_msgs::ImagePtr barcode_img_msg(new sensor_msgs::Image(*img_msg));
			cv::Mat barcode_img(barcode_img_msg->height, barcode_img_msg->width, cv_bridge::getCvType(barcode_img_msg->encoding), barcode_img_msg->data.data(), barcode_img_msg->step);
			for (int i=0; i<found_barcodes; ++i) {
				const int n = icb_results[i].ResultPointsLength-1;
				for (int j=0; j<n; ++j) {
					cv::line(barcode_img, cv::Point2f(icb_results[i].ResultPoints[j].x, icb_results[i].ResultPoints[j].y),
							 cv::Point2f(icb_results[i].ResultPoints[j+1].x, icb_results[i].ResultPoints[j+1].y), cv::Scalar(255,0,0));
				}
				cv::line(barcode_img, cv::Point2f(icb_results[i].ResultPoints[n].x, icb_results[i].ResultPoints[n].y),
						 cv::Point2f(icb_results[i].ResultPoints[0].x, icb_results[i].ResultPoints[0].y), cv::Scalar(255,0,0));
			}
			image_pub_.publish(barcode_img_msg);
		}
		else {
			image_pub_.publish(img_msg);
		}
	}
}

void IcBarcodeScannerNodelet::connectCallback()
{
	if (barcodes_pub_.getNumSubscribers() == 0 && image_pub_ == 0) {
		image_sub_.shutdown();		// shutdown subscription if nobody listens to us
	}
	else {
		image_sub_ = it_->subscribe(ros::names::remap("image"), 1, &IcBarcodeScannerNodelet::scanImageCallback, this, image_transport::TransportHints(transport_hints_));
	}
}

void IcBarcodeScannerNodelet::dynReconfigureCallback(Config& config, uint32_t level) {

	reconfigure_mutex_.lock();

	// write booleans into bits
	int barcode_formats = 0;
	if (config.AZTEC) barcode_formats |= IC_BARCODEFORMAT_AZTEC;
	if (config.CODE39) barcode_formats |= IC_BARCODEFORMAT_CODE_39;
	if (config.CODE93) barcode_formats |= IC_BARCODEFORMAT_CODE_93;
	if (config.CODE128) barcode_formats |= IC_BARCODEFORMAT_CODE_128;
	if (config.DATA_MATRIX) barcode_formats |= IC_BARCODEFORMAT_DATA_MATRIX;
	if (config.EAN_8) barcode_formats |= IC_BARCODEFORMAT_EAN_8;
	if (config.EAN_13) barcode_formats |= IC_BARCODEFORMAT_EAN_13;
	if (config.MAXICODE) barcode_formats |= IC_BARCODEFORMAT_MAXICODE;
	if (config.PDF417) barcode_formats |= IC_BARCODEFORMAT_PDF_417;
	if (config.QR_CODE) barcode_formats |= IC_BARCODEFORMAT_QR_CODE;
	if (config.UPC_A) barcode_formats |= IC_BARCODEFORMAT_UPC_A;
	if (config.UPC_E) barcode_formats |= IC_BARCODEFORMAT_UPC_E;
	if (config.UPC_EAN_EXT) barcode_formats |= IC_BARCODEFORMAT_UPC_EAN_EXTENSION;
	if (config.INTERLEAVED_2_OF_5) barcode_formats |= IC_BARCODEFORMAT_INTERLEAVED_2_OF_5;

	if (barcode_formats != barcode_formats_) {
		barcode_formats_ = barcode_formats;
		ICBarcode_SetBarcodeFormats(p_scanner_, barcode_formats_);
	}

	dyn_config_ = config;

	reconfigure_mutex_.unlock();
}

IcBarcodeResult IcBarcodeScannerNodelet::createMsgFromResult(const ICBarcode_Result& icb_result) {

	IcBarcodeResult ros_result;
	ros_result.Text = icb_result.Text;
	ros_result.RawBytes.resize(icb_result.RawBytesLength);
	memcpy(ros_result.RawBytes.data(), icb_result.RawBytes, icb_result.RawBytesLength);
	ros_result.Frequency = icb_result.Frequency;
	ros_result.ResultPoints.resize(icb_result.ResultPointsLength);
	for (int i=0; i<icb_result.ResultPointsLength; ++i) {
		ros_result.ResultPoints[i].x = icb_result.ResultPoints[i].x;
		ros_result.ResultPoints[i].y = icb_result.ResultPoints[i].y;
		ros_result.ResultPoints[i].z = 0.f;
	}
	ros_result.Format = icb_result.Format;

	return ros_result;
}

void IcBarcodeScannerNodelet::readICBarcodeParams() {

	reconfigure_mutex_.lock();

	ros::NodeHandle& pnh = this->getPrivateNodeHandle();

	// eventually get parameters from parameter server
	int p;
	for (const auto& kv: IC_BARCODEPARAMS) {
		if (pnh.getParam(kv.first, p)) {
			ICBarcode_SetParam(p_scanner_, kv.second, p);
		}
	}

	pnh.param<int>("barcode_formats", barcode_formats_, barcode_formats_);
	// remove undefined bits
	barcode_formats_ &= IC_BARCODEFORMAT_ALL;
	ICBarcode_SetBarcodeFormats(p_scanner_, barcode_formats_);

	// separate bits into booleans
	dyn_config_.AZTEC = barcode_formats_ & IC_BARCODEFORMAT_AZTEC;
	dyn_config_.CODE39 = barcode_formats_ & IC_BARCODEFORMAT_CODE_39;
	dyn_config_.CODE93 = barcode_formats_ & IC_BARCODEFORMAT_CODE_93;
	dyn_config_.CODE128 = barcode_formats_ & IC_BARCODEFORMAT_CODE_128;
	dyn_config_.DATA_MATRIX = barcode_formats_ & IC_BARCODEFORMAT_DATA_MATRIX;
	dyn_config_.EAN_8 = barcode_formats_ & IC_BARCODEFORMAT_EAN_8;
	dyn_config_.EAN_13 = barcode_formats_ & IC_BARCODEFORMAT_EAN_13;
	dyn_config_.MAXICODE = barcode_formats_ & IC_BARCODEFORMAT_MAXICODE;
	dyn_config_.PDF417 = barcode_formats_ & IC_BARCODEFORMAT_PDF_417;
	dyn_config_.QR_CODE = barcode_formats_ & IC_BARCODEFORMAT_QR_CODE;
	dyn_config_.UPC_A = barcode_formats_ & IC_BARCODEFORMAT_UPC_A;
	dyn_config_.UPC_E = barcode_formats_ & IC_BARCODEFORMAT_UPC_E;
	dyn_config_.UPC_EAN_EXT = barcode_formats_ & IC_BARCODEFORMAT_UPC_EAN_EXTENSION;
	dyn_config_.INTERLEAVED_2_OF_5 = barcode_formats_ & IC_BARCODEFORMAT_INTERLEAVED_2_OF_5;

/*	@todo ICBarcode_GetParam reports nonsense values, so we skip this atm
	// sync current status of parameters
	ICBarcode_GetParam(p_scanner_, IC_BARCODEPARAMS_CHECK_CHAR, &dyn_config_.check_char);
	ICBarcode_GetParam(p_scanner_, IC_BARCODEPARAMS_NUM_SCANLINES, &dyn_config_.num_scanlines);
	ICBarcode_GetParam(p_scanner_, IC_BARCODEPARAMS_ELEMENT_SIZE_MIN, &dyn_config_.element_size_min);
	ICBarcode_GetParam(p_scanner_, IC_BARCODEPARAMS_ELEMENT_SIZE_MAX, &dyn_config_.element_size_max);
	ICBarcode_GetParam(p_scanner_, IC_BARCODEPARAMS_ORIENTATION, &dyn_config_.orientation);
	ICBarcode_GetParam(p_scanner_, IC_BARCODEPARAMS_ORIENTATION_TOL, &dyn_config_.orientation_tol);
	ICBarcode_GetParam(p_scanner_, IC_BARCODEPARAMS_STOP_AFTER_NUM_RESULTS, &dyn_config_.stop_after_num_results);
	ICBarcode_GetParam(p_scanner_, IC_BARCODEPARAMS_TIMEOUT, &dyn_config_.timeout);
	ICBarcode_GetParam(p_scanner_, IC_BARCODEPARAMS_MIN_WIDTH, &dyn_config_.min_width);
	ICBarcode_GetParam(p_scanner_, IC_BARCODEPARAMS_MIN_HEIGHT, &dyn_config_.min_height);
	ICBarcode_GetParam(p_scanner_, IC_BARCODEPARAMS_MAX_WIDTH, &dyn_config_.max_width);
	ICBarcode_GetParam(p_scanner_, IC_BARCODEPARAMS_MAX_HEIGHT, &dyn_config_.max_height);
	ICBarcode_GetParam(p_scanner_, IC_BARCODEPARAMS_PREPROCESS, &dyn_config_.preprocess);
	ICBarcode_GetParam(p_scanner_, IC_BARCODEPARAMS_TRY_HARDER, &dyn_config_.try_harder);
	ICBarcode_GetParam(p_scanner_, IC_BARCODEPARAMS_IC_ONLY, &dyn_config_.ic_only);
	ICBarcode_GetParam(p_scanner_, IC_BARCODEPARAMS_FIXED_DATA_LENGTH, &dyn_config_.fixed_data_length);
	ICBarcode_GetParam(p_scanner_, IC_BARCODEPARAMS_STORE_DEBUG_INFO, &dyn_config_.store_debug_info);
	ICBarcode_GetParam(p_scanner_, IC_BARCODEPARAMS_USE_MULTITHREADING, &dyn_config_.use_multithreading);
*/

	pnh.param<int>("max_barcodes", max_barcodes_, max_barcodes_);

	reconfigure_mutex_.unlock();
}

bool IcBarcodeScannerNodelet::findBarcodesSrv(find_barcodes::Request& req, find_barcodes::Response& res) {

	cv_bridge::CvImagePtr image_mono = cv_bridge::toCvCopy(req.image, "mono8");

	std::vector<ICBarcode_Result> icb_results(max_barcodes_);

	reconfigure_mutex_.lock();
	const int found_barcodes = ICBarcode_FindBarcodes(p_scanner_, image_mono->image.data, image_mono->image.cols, image_mono->image.rows, image_mono->image.step, icb_results.data(), max_barcodes_);
	reconfigure_mutex_.unlock();

	res.barcodes.header = req.image.header;
	res.barcodes.results.clear();
	for (int i=0; i<found_barcodes; ++i) {
		res.barcodes.results.emplace_back(createMsgFromResult(icb_results[i]));
	}

	return (found_barcodes > 0);
}

} /* namespace ic_barcode_scanner */
