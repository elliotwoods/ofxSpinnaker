#include "Spinnaker.h"

using namespace ofxMachineVision;

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

namespace ofxMachineVision {
	namespace Device {
		//----------
		::Spinnaker::SystemPtr Spinnaker::system;

		//----------
		Spinnaker::Spinnaker() {

		}

		//----------
		Spinnaker::~Spinnaker() {

		}

		//----------
		string Spinnaker::getTypeName() const {
			return "Spinnaker (FLIR)";
		}

		//----------
		void Spinnaker::initOnMainThread() {

		}

		//----------
		vector<Device::Base::ListedDevice> Spinnaker::listDevices() const {
			vector<Device::Base::ListedDevice> devices;

			try {
				auto system = this->getSystem();

				auto cameraList = system->GetCameras();
				auto cameraCount = cameraList.GetSize();

				devices.resize(cameraCount);
				for (int i = 0; i < cameraCount; i++) {
					auto initialisationSettings = static_pointer_cast<InitialisationSettings>(this->getDefaultSettings());

					auto camera = cameraList.GetByIndex(i);

					initialisationSettings->serialNumber = (string)camera->TLDevice.DeviceSerialNumber();
					initialisationSettings->useSerialNumber = true;

					ListedDevice listedDevice{
						initialisationSettings,
						camera->TLDevice.DeviceVendorName(),
						camera->TLDevice.DeviceModelName()
					};
					devices[i] = listedDevice;
				}

				cameraList.Clear();
			}
			catch (const std::exception & e) {
				ofLogError("ofxSpinnaker") << e.what();
			}

			return devices;
		}

		//----------
		Specification Spinnaker::open(shared_ptr<Base::InitialisationSettings> initialisationSettings) {
			auto settings = this->getTypedSettings<InitialisationSettings>(initialisationSettings);		
			auto cameraList = this->getSystem()->GetCameras();

			if (settings->useSerialNumber) {
				this->camera = cameraList.GetBySerial(settings->serialNumber);
			}
			else {
				this->camera = cameraList.GetByIndex(settings->deviceID);
			}

			this->camera->Init();

			Specification specification(CaptureSequenceType::Continuous
				, this->camera->Width()
				, this->camera->Height()
				, (string) this->camera->DeviceVendorName()
				, (string) this->camera->DeviceModelName()
				, (string) this->camera->DeviceSerialNumber());

			//turn off auto exposure
			this->camera->ExposureAuto.SetValue(::Spinnaker::ExposureAutoEnums::ExposureAuto_Off);
			this->camera->GainAuto.SetValue(::Spinnaker::GainAutoEnums::GainAuto_Off);

			//add basic exposure parameters
			this->setupFloatParameter(this->camera->AcquisitionFrameRate);
			this->setupFloatParameter(this->camera->ExposureTime);
			this->setupFloatParameter(this->camera->Gain);
			this->setupFloatParameter(this->camera->Gamma);

			//always capture the latest image from the camera (ideally this should be a parameter)
			this->camera->TLStream.StreamBufferHandlingMode.SetValue(::Spinnaker::StreamBufferHandlingMode_NewestFirstOverwrite);

			//build trigger parameter
			auto & spinnakerParameter = this->camera->TriggerMode;
			try {
				auto parameter = make_shared<Parameter<bool>>(ofParameter<bool>{(string) spinnakerParameter.GetName(), false});

				parameter->getDeviceValueFunction = [&spinnakerParameter]() {
					return spinnakerParameter.GetValue() == ::Spinnaker::TriggerModeEnums::TriggerMode_On;
				};
				parameter->setDeviceValueFunction = [&spinnakerParameter](const bool & value) {
					spinnakerParameter.SetValue(value ? ::Spinnaker::TriggerModeEnums::TriggerMode_On : ::Spinnaker::TriggerModeEnums::TriggerMode_Off);
				};
				this->parameters.push_back(parameter);
			}
			catch (const std::exception & e) {
				OFXMV_ERROR << "Cannot add parameter " << spinnakerParameter.GetName() << ". " << e.what();
			}

			this->setupFloatParameter(this->camera->TriggerDelay);

			//Flip parameter
			{
				auto parameter = make_shared<Parameter<bool>> (ofParameter<bool>{"Flip image", false});
				parameter->getDeviceValueFunction = [this]() {
					return this->flipCamera.load();
				};
				parameter->setDeviceValueFunction = [this](const bool & value) {
					this->flipCamera = value;
				};
				this->parameters.push_back(parameter);
			}
			return specification;
		}

		//----------
		void Spinnaker::close() {
			this->camera->DeInit();
		}

		//----------
		bool Spinnaker::startCapture() {
			//Referenced from:
			//http://softwareservices.ptgrey.com/Spinnaker/latest/_acquisition_8cpp-example.html

			try {
				this->camera->AcquisitionMode.SetValue(AcquisitionMode_Continuous);
				this->camera->BeginAcquisition();
				return true;
			}
			catch (const std::exception & e) {
				ofLogError("ofxSpinnaker") << e.what();
				return false;
			}
		}

		//----------
		void Spinnaker::stopCapture() {
			this->camera->EndAcquisition();
		}

		//----------
		shared_ptr<Frame> Spinnaker::getFrame() {
			shared_ptr<Frame> frame;
			try {
				// Pull the camera frame from the Spinnaker SDK
				auto cameraImage = this->camera->GetNextImage(1000);

				// This will change if conversion needs to happen - but this image does not need to be released
				auto image = cameraImage;

				if (image->IsIncomplete()) {
					stringstream ss;
					throw(ofxMachineVision::Exception("Incomplete image"));
				}
				else {
					// Handle Bayer images
					switch (image->GetPixelFormat()) {
					case ::Spinnaker::PixelFormat_BayerRG8:
					case ::Spinnaker::PixelFormat_BayerBG8:
						// Needs debayering conversion
						{
							auto cameraImage = image;
							image = cameraImage->Convert(::Spinnaker::PixelFormat_RGB8, ::Spinnaker::NEAREST_NEIGHBOR);
							break;
						}
					default:
						// No debayering conversion
						break;
					}

					auto ofFormat = Spinnaker::toOf(image->GetPixelFormat());

					if (ofFormat == ofPixelFormat::OF_PIXELS_UNKNOWN) {
						stringstream ss;
						ss << "Pixel format not supported : " << image->GetPixelFormatName();
						throw(ofxMachineVision::Exception(ss.str()));
					}
					else {
						// Fill a frame in the frame pool with the data from the Point Grey image

						frame = FramePool::X().getAvailableFrameFilledWith(
							(unsigned char *) image->GetData()
							, image->GetWidth()
							, image->GetHeight()
							, toOf(image->GetPixelFormat()));

						if (this->flipCamera) {
							frame->getPixels().rotate90(2);
						}

						frame->setTimestamp(chrono::nanoseconds(cameraImage->GetTimeStamp()));
						frame->setFrameIndex(cameraImage->GetFrameID());
					}
				}
				cameraImage->Release();

				return frame;
			}
			catch (const ::Spinnaker::Exception & e) {
				throw(ofxMachineVision::Exception(string(e.GetFunctionName()) + " : " + string(e.GetErrorMessage())));
			}
		}

		//----------
		ofPixelFormat Spinnaker::toOf(::Spinnaker::PixelFormatEnums pixelFormatSpinnaker) {
			switch (pixelFormatSpinnaker) {
			case ::Spinnaker::PixelFormat_Mono1p:
			case ::Spinnaker::PixelFormat_Mono2p:
			case ::Spinnaker::PixelFormat_Mono4p:
			case ::Spinnaker::PixelFormat_Mono8:
			case ::Spinnaker::PixelFormat_Mono8s:
			case ::Spinnaker::PixelFormat_Mono10:
			case ::Spinnaker::PixelFormat_Mono10p:
			case ::Spinnaker::PixelFormat_Mono12:
			case ::Spinnaker::PixelFormat_Mono12p:
			case ::Spinnaker::PixelFormat_Mono14:
			case ::Spinnaker::PixelFormat_Mono16:
				return ofPixelFormat::OF_PIXELS_GRAY;

			case ::Spinnaker::PixelFormat_RGBa8:
			case ::Spinnaker::PixelFormat_RGBa10:
			case ::Spinnaker::PixelFormat_RGBa10p:
			case ::Spinnaker::PixelFormat_RGBa12:
			case ::Spinnaker::PixelFormat_RGBa12p:
			case ::Spinnaker::PixelFormat_RGBa14:
			case ::Spinnaker::PixelFormat_RGBa16:
				return ofPixelFormat::OF_PIXELS_RGBA;

			case ::Spinnaker::PixelFormat_RGB8:
			case ::Spinnaker::PixelFormat_RGB10:
			case ::Spinnaker::PixelFormat_RGB10p:
			case ::Spinnaker::PixelFormat_RGB10p32:
			case ::Spinnaker::PixelFormat_RGB12:
			case ::Spinnaker::PixelFormat_RGB12p:
			case ::Spinnaker::PixelFormat_RGB14:
			case ::Spinnaker::PixelFormat_RGB16:
			case ::Spinnaker::PixelFormat_RGB565p:
				return ofPixelFormat::OF_PIXELS_RGB;

			case ::Spinnaker::PixelFormat_BGRa8:
			case ::Spinnaker::PixelFormat_BGRa10:
			case ::Spinnaker::PixelFormat_BGRa10p:
			case ::Spinnaker::PixelFormat_BGRa12:
			case ::Spinnaker::PixelFormat_BGRa12p:
			case ::Spinnaker::PixelFormat_BGRa14:
			case ::Spinnaker::PixelFormat_BGRa16:
				return ofPixelFormat::OF_PIXELS_BGRA;

			case ::Spinnaker::PixelFormat_BGR8:
			case ::Spinnaker::PixelFormat_BGR10:
			case ::Spinnaker::PixelFormat_BGR10p:
			case ::Spinnaker::PixelFormat_BGR12:
			case ::Spinnaker::PixelFormat_BGR12p:
			case ::Spinnaker::PixelFormat_BGR14:
			case ::Spinnaker::PixelFormat_BGR16:
			case ::Spinnaker::PixelFormat_BGR565p:
				return ofPixelFormat::OF_PIXELS_BGR;

			case ::Spinnaker::PixelFormat_R8:
			case ::Spinnaker::PixelFormat_R10:
			case ::Spinnaker::PixelFormat_R12:
			case ::Spinnaker::PixelFormat_R16:
			case ::Spinnaker::PixelFormat_G8:
			case ::Spinnaker::PixelFormat_G10:
			case ::Spinnaker::PixelFormat_G12:
			case ::Spinnaker::PixelFormat_G16:
			case ::Spinnaker::PixelFormat_B8:
			case ::Spinnaker::PixelFormat_B10:
			case ::Spinnaker::PixelFormat_B12:
			case ::Spinnaker::PixelFormat_B16:
				return ofPixelFormat::OF_PIXELS_GRAY;

			default:
				//there are some formats we'd like to handle but don't (e.g. Bayer, YUV, 3D coords)
				return ofPixelFormat::OF_PIXELS_UNKNOWN;
				break;
			}
		}

		//----------
		::Spinnaker::SystemPtr Spinnaker::getSystem() {
			if (!Spinnaker::system.IsValid()) {
				Spinnaker::system = ::Spinnaker::System::GetInstance();
			}
			return Spinnaker::system;
		}

		//----------
		void Spinnaker::setupFloatParameter(::Spinnaker::GenApi::IFloat & floatParameter) {
			try {
				auto parameter = make_shared<Parameter<float>>(ofParameter<float>{(string)floatParameter.GetName()
					, (float)floatParameter.GetValue()
					, (float)floatParameter.GetMin()
					, (float)floatParameter.GetMax() }
				, (string)floatParameter.GetUnit());

				parameter->getDeviceValueFunction = [&floatParameter]() {
					return floatParameter.GetValue();
				};
				parameter->getDeviceValueRangeFunction = [&floatParameter](float & min, float & max) {
					min = (float)floatParameter.GetMin();
					max = (float)floatParameter.GetMax();
				};
				parameter->setDeviceValueFunction = [&floatParameter](const float & value) {
					floatParameter.SetValue((double)value, true);
				};
				this->parameters.push_back(parameter);
			}
			catch (const std::exception & e) {
				OFXMV_ERROR << "Cannot add parameter " << floatParameter.GetName() << ". " << e.what();
			}
		}

		//----------
		void Spinnaker::setupBoolParameter(::Spinnaker::GenApi::IBoolean & spinnakerParameter) {
			try {
				auto parameter = make_shared<Parameter<bool>>(ofParameter<bool>{(string)spinnakerParameter.GetName()
					, spinnakerParameter.GetValue() }
				, "");

				parameter->getDeviceValueFunction = [&spinnakerParameter]() {
					return spinnakerParameter.GetValue();
				};
				parameter->setDeviceValueFunction = [&spinnakerParameter](const bool & value) {
					spinnakerParameter.SetValue((double)value, true);
				};
				this->parameters.push_back(parameter);
			}
			catch (const std::exception & e) {
				OFXMV_ERROR << "Cannot add parameter " << spinnakerParameter.GetName() << ". " << e.what();
			}
		}
	}
}
