#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	auto deviceDescriptions = this->grabber.getDevice()->listDevices();
	for (auto deviceDescription : deviceDescriptions) {
		cout << deviceDescription.manufacturer << ", " << deviceDescription.model << endl;
	}
	this->grabber.open();
	this->grabber.startCapture();
}

//--------------------------------------------------------------
void ofApp::update(){
	this->grabber.update();
}

//--------------------------------------------------------------
void ofApp::draw(){
	this->grabber.draw(0, 0);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
