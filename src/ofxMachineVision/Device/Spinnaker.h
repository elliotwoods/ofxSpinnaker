#pragma once

#define _HAS_STD_BYTE 0

#include "ofxMachineVision/Device/Blocking.h"

#include "spinnaker/Spinnaker.h"
#include "spinnaker/SpinGenApi/SpinnakerGenApi.h"

namespace ofxMachineVision {
	namespace Device {
		class Spinnaker : public ofxMachineVision::Device::Blocking {
		public:
			struct InitialisationSettings : Base::InitialisationSettings {
			public:
				InitialisationSettings() {
					add(useSerialNumber);
					add(serialNumber);
				}

				ofParameter<bool> useSerialNumber{ "Use Serial number", false };
				ofParameter<string> serialNumber{ "Serial number", "" };
			};

			Spinnaker();
			virtual ~Spinnaker();
			string getTypeName() const override;

			virtual void initOnMainThread() override;
			vector<ListedDevice> listDevices() const override;

			shared_ptr<Base::InitialisationSettings> getDefaultSettings() const override {
				return make_shared<InitialisationSettings>();
			}
			Specification open(shared_ptr<Base::InitialisationSettings> = nullptr) override;
			void close() override;
			bool startCapture() override;
			void stopCapture() override;

			shared_ptr<Frame> getFrame() override;

			static ofPixelFormat toOf(::Spinnaker::PixelFormatEnums);
			static ::Spinnaker::SystemPtr getSystem();

		protected:
			void setupFloatParameter(::Spinnaker::GenApi::IFloat & spinnakerParameter);
			void setupBoolParameter(::Spinnaker::GenApi::IBoolean & spinnakerParameter);

			static ::Spinnaker::SystemPtr system;
			::Spinnaker::CameraPtr camera;

			atomic<bool> flipCamera = false;
		};
	}
}