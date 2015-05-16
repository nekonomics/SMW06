#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	ofSetFrameRate(30);
	ofDisableArbTex();

	texture.loadImage("cookie.png");

	srcData.plane.set(200, 200, 20, 20);
	dstData.plane.set(200, 200, 20, 20);


	deformers.push_back(shared_ptr<MLS::IDeformer>(new MLS::AffineDeformer()));
	deformers.push_back(shared_ptr<MLS::IDeformer>(new MLS::SimilarityDeformer()));
	deformers.push_back(shared_ptr<MLS::IDeformer>(new MLS::RigidDeformer()));
	setDeformer(0);

	showMesh = false;
}

//--------------------------------------------------------------
void ofApp::exit() {
}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(10, 20, 30);
	ofSetColor(255, 255, 255);

	float w = ofGetWidth(), h = ofGetHeight();

	ofPushMatrix();

	ofTranslate(w / 4, h / 2);

	ofSetColor(255, 255, 255);
	drawPlane(srcData.plane);

	ofSetColor(255, 0, 0);
	drawPins(srcData.pins);

	ofTranslate(w / 2, 0);

	ofSetColor(255, 255, 255);
	drawPlane(dstData.plane);

	ofSetColor(0, 255, 0);
	drawPins(dstData.pins);

	ofPopMatrix();

	ofSetColor(255, 255, 255);
	ofLine(w / 2, 0, w / 2, h);

	ofDrawBitmapString("deformer: " + ofToString(deformerIndex), 10, 20);
}

//--------------------------------------------------------------
void ofApp::drawPlane(ofPlanePrimitive &plane) {
	// Texture
	texture.bind();
	plane.draw();
	texture.unbind();

	if(!showMesh) {
		return;
	}

	const ofMesh &mesh = plane.getMesh();

	ofPolyline line;
	int rows = plane.getNumRows(), cols = plane.getNumColumns();

	// Horizontal
	for(int iy = 0; iy < rows; ++ iy) {
		line.clear();
		for(int ix = 0; ix < cols; ++ ix) {
			int i = iy * cols + ix;
			line.addVertex(mesh.getVertex(i));
		}
		line.draw();
	}

	// Vertical
	for(int ix = 0; ix < cols; ++ ix) {
		line.clear();
		for(int iy = 0; iy < rows; ++ iy) {
			int i = iy * cols + ix;
			line.addVertex(mesh.getVertex(i));
		}
		line.draw();
	}

}

//--------------------------------------------------------------
void ofApp::drawPins(vector<ofVec3f> &pins) {
	for(vector<ofVec3f>::const_iterator it = pins.begin(); it != pins.end(); ++ it) {
		ofCircle(*it, 4);
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if('1' <= key && key <= '3') {
		setDeformer(key - '1');
		return;
	}

	if(key == 'h') {
		showMesh = !showMesh;
		return;
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
	if(selection.indices.size() == 0) {
		return;
	}

	ofVec3f p(x, y);
	ofVec3f d = p - selection.origin;
	selection.origin = p;

	vector<ofVec3f> &pins = selection.isSrc ? srcData.pins : dstData.pins;
	for(vector<int>::iterator it = selection.indices.begin(); it != selection.indices.end(); ++ it) {
		pins[*it] += d;
	}

	if(selection.isSrc) {
		deformer->compute(srcData.plane.getMesh().getVertices(), srcData.pins);
	}

	if(pins.size() > 0) {
		deform();
	}
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
	float w = ofGetWidth(), h = ofGetHeight();

	selection.origin = ofVec3f(x, y);
	selection.indices.clear();

	int numPins = srcData.pins.size();

	if(x < w / 2) {
		ofVec3f p(x - w / 4, y - h / 2);
		int index = getNearestPin(p, srcData.pins, 10);
		if(index == -1) {
			if(button == 0) {
				// Add pin
				srcData.pins.push_back(p);
				dstData.pins.push_back(p);
			}
		} else {
			if(button == 0) {
				selection.indices.push_back(index);
				selection.isSrc = true;
			} else if(button == 2) {
				// Remove pin
				srcData.pins.erase(srcData.pins.begin() + index);
				dstData.pins.erase(dstData.pins.begin() + index);
			}
		}
	} else {
		ofVec3f p(x - w / 2 - w / 4, y - h / 2);
		int index = getNearestPin(p, dstData.pins, 10);
		if(index == -1) {
		} else {
			if(button == 0) {
				selection.indices.push_back(index);
				selection.isSrc = false;
			}
		}
	}

	if(srcData.pins.size() != numPins) {
		deformer->compute(srcData.plane.getMesh().getVertices(), srcData.pins);
		deform();
	}

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

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

//--------------------------------------------------------------
void ofApp::setDeformer(int index) {
	deformerIndex = index;
	deformer = deformers[index];
	deformer->compute(srcData.plane.getMesh().getVertices(), srcData.pins);
	deform();
}

//--------------------------------------------------------------
int ofApp::getNearestPin(const ofVec3f &p, const vector<ofVec3f> &pins, float radius) {
	float min = -1;
	int index = -1;
	float r2 = radius * radius;
	for(vector<ofVec3f>::const_iterator it = pins.begin(); it != pins.end(); ++ it) {
		float d = (p - *it).lengthSquared();
		if(d <= r2 && (min < 0 || d < min)) {
			min = d;
			index = it - pins.begin();
		}
	}
	return index;
}

//--------------------------------------------------------------
void ofApp::deform() {
	vector<ofVec3f> vertices;
	deformer->deform(vertices, dstData.pins);
	ofMesh &dstMesh = dstData.plane.getMesh();
	for(int i = 0; i < vertices.size(); ++ i) {
		dstMesh.setVertex(i, vertices[i]);
	}
}
