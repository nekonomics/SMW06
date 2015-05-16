#pragma once

#include "ofMain.h"
#include <assert.h>

#include "ofMatrix2x2.h"
#include "MLS.h"

class ofApp : public ofBaseApp{
public:
	void setup();
	void exit();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y );
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	void drawPlane(ofPlanePrimitive &plane);
	void drawPins(vector<ofVec3f> &pins);

	void setDeformer(int index);

	void deform();

	int getNearestPin(const ofVec3f &p, const vector<ofVec3f> &pins, float radius);

	int deformerIndex;
	shared_ptr<MLS::IDeformer> deformer;
	vector< shared_ptr<MLS::IDeformer> > deformers;

	bool showMesh;
	ofImage texture;

	struct {
		ofPlanePrimitive plane;
		vector<ofVec3f> pins;
	} srcData, dstData;

	struct {
		ofVec3f origin;
		vector<int> indices;
		bool isSrc;
	} selection;
};
