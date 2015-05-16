#pragma once

#include "ofMain.h"
#include "ofMatrix2x2.h"

/**
* Scott Schaefer, Travis McPhail, and Joe Warren. 2006. Image deformation using moving least squares.
* In ACM SIGGRAPH 2006 Papers (SIGGRAPH '06). ACM, New York, NY, USA, 533-540. DOI=10.1145/1179352.1141920
* http://doi.acm.org/10.1145/1179352.1141920
*/

namespace MLS {
	class IDeformer {
	public:

		virtual void compute(const vector<ofVec3f> &srcVerts, const vector<ofVec3f> &srcPins) = 0;
		virtual void deform(vector<ofVec3f> &dstVerts, const vector<ofVec3f> &dstPins) = 0;

	};

	template<typename ResultClass>
	class BaseDeformer : public IDeformer {
	public:

		void compute(const vector<ofVec3f> &srcVerts, const vector<ofVec3f> &srcPins) {
			if(srcPins.size() < 3) { return; }
			int numVerts = srcVerts.size();
			results.resize(numVerts);
			for(int i = 0; i < numVerts; ++ i) {
				compute(srcVerts[i], srcPins, results[i]);
			}
		}

		void deform(vector<ofVec3f> &dstVerts, const vector<ofVec3f> &dstPins) {
			int numVerts = results.size();
			dstVerts.resize(numVerts);
			for(int i = 0; i < numVerts; ++ i) {
				deform(dstVerts[i], dstPins, results[i]);
			}
		}

	protected:

		virtual void compute(const ofVec3f &v, const vector<ofVec3f> &srcPins, ResultClass &res) = 0;
		virtual void deform(ofVec3f &dst, const vector<ofVec3f> &dstPins, const ResultClass &res) = 0;

		vector<ResultClass> results;

	};

	typedef struct {
		vector<float> As;
		vector<float> weights;
		float sumWeights;
	} AffineDeformerResult;

	class AffineDeformer : public BaseDeformer<AffineDeformerResult> {
	public:
		AffineDeformer() {}
		~AffineDeformer() {}

	protected:

		void compute(const ofVec3f &v, const vector<ofVec3f> &srcPins, AffineDeformerResult &res);
		void deform(ofVec3f &dst, const vector<ofVec3f> &dstPins, const AffineDeformerResult &res);
	};


	typedef struct {
		float mu;
		vector<ofMatrix2x2> As;
		vector<float> weights;
		float sumWeights;
	} SimilarityDeformerResult;

	class SimilarityDeformer : public BaseDeformer<SimilarityDeformerResult> {
	public:
		SimilarityDeformer() {}
		~SimilarityDeformer() {}

	protected:

		void compute(const ofVec3f &v, const vector<ofVec3f> &srcPins, SimilarityDeformerResult &res);
		void deform(ofVec3f &dst, const vector<ofVec3f> &dstPins, const SimilarityDeformerResult &res);

	};

	typedef struct {
		float length;
		float mu;
		vector<ofMatrix2x2> As;
		vector<float> weights;
		float sumWeights;
	} RigidDeformerResult;

	class RigidDeformer : public BaseDeformer<RigidDeformerResult> {
	public:
		RigidDeformer() {}
		~RigidDeformer() {}

	protected:

		void compute(const ofVec3f &v, const vector<ofVec3f> &srcPins, RigidDeformerResult &res);
		void deform(ofVec3f &dst, const vector<ofVec3f> &dstPins, const RigidDeformerResult &res);

	};
}