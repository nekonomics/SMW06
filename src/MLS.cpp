#include "MLS.h"
#include "ofMatrix2x2.h"

static const float INF = 99999;

float computeWeight(const ofVec3f &p, const ofVec3f &v, float alpha = 1.0) {
	float d = (p - v).lengthSquared();
	return d == 0 ? INF : 1.0 / powf(d, alpha);
}

void MLS::AffineDeformer::compute(const ofVec3f &v, const vector<ofVec3f> &srcPins, MLS::AffineDeformerResult &res) {
	int numPins = srcPins.size();

	// Compute weights, sum of weights and p*
	vector<float> &weights = res.weights;
	weights.resize(numPins);
	res.sumWeights = 0;
	ofVec3f p_a(0, 0, 0);
	for(int i = 0; i < numPins; ++ i) {
		float w = weights[i] = computeWeight(srcPins[i], v, 1.0);
		res.sumWeights += w;
		p_a += w * srcPins[i];
	}
	p_a /= res.sumWeights;

	// Compute array of p^
	vector<ofVec3f> p_hs;
	for(int i = 0; i < numPins; ++ i) {
		p_hs.push_back(srcPins[i] - p_a);
	}

	// Compute (sigma(p^_i^T w_i p^_i)^-1
	ofMatrix2x2 m(0, 0, 0, 0);
	for(int i = 0; i < numPins; ++ i) {
		ofMatrix2x2 n;
		n.makeFromMultiplicationOf(p_hs[i], res.weights[i] * p_hs[i]);
		m += n;
	}
	ofVec3f t = (v - p_a) * m.getInverse();

	// Compute A_j
	vector<float> &As = res.As;
	As.resize(srcPins.size());
	for(int j = 0; j < numPins; ++ j) {
		As[j] = t.dot(res.weights[j] * p_hs[j]);
	}
}

void MLS::AffineDeformer::deform(ofVec3f &dst, const vector<ofVec3f> &dstPins, const MLS::AffineDeformerResult &res) {
	int numPins = dstPins.size();

	// Compute q*
	ofVec3f q_a(0, 0, 0);
	for(int i = 0; i < dstPins.size(); ++ i) {
		q_a += res.weights[i] * dstPins[i];
	}
	q_a /= res.sumWeights;

	// Compute sum(A_j q^_j)
	dst.set(q_a);
	const vector<float> &As = res.As;
	for(int j = 0; j < As.size(); ++ j) {
		dst += As[j] * (dstPins[j] - q_a);
	}
}


void MLS::SimilarityDeformer::compute(const ofVec3f &v, const vector<ofVec3f> &srcPins, MLS::SimilarityDeformerResult &res) {
	int numPins = srcPins.size();

	// Compute weights, sum of weights and p*
	res.sumWeights = 0;
	res.weights.clear();
	ofVec3f p_a(0, 0, 0);
	for(int i = 0; i < numPins; ++ i) {
		float w = computeWeight(srcPins[i], v, 1.0);
		res.weights.push_back(w);
		res.sumWeights += w;
		p_a += w * srcPins[i];
	}
	p_a /= res.sumWeights;

	// Compute array of p^
	vector<ofVec3f> p_hs;
	for(int i = 0; i < numPins; ++ i) {
		p_hs.push_back(srcPins[i] - p_a);
	}

	// Compute mu_s
	res.mu = 0;
	for(int i = 0; i < numPins; ++ i) {
		res.mu += res.weights[i] * p_hs[i].dot(p_hs[i]);
	}

	// Compute A_j
	vector<ofMatrix2x2> &As = res.As;
	As.clear();
	ofVec3f vp_a = v - p_a;
	for(int i = 0; i < srcPins.size(); ++ i) {
		const ofVec3f &p_h = p_hs[i];
		// [( x, y)]
		// [(-y, x)]
		ofMatrix2x2 m0(p_h.x, p_h.y, -p_h.y, p_h.x);
		ofMatrix2x2 m1(vp_a.x, vp_a.y, -vp_a.y, vp_a.x);
		As.push_back(res.weights[i] * (m0 * m1.getTransposed()));
	}
}

void MLS::SimilarityDeformer::deform(ofVec3f &dst, const vector<ofVec3f> &dstPins, const MLS::SimilarityDeformerResult &res) {
	// Compute q*
	ofVec3f q_a(0, 0, 0);
	for(int i = 0; i < dstPins.size(); ++ i) {
		q_a += res.weights[i] * dstPins[i];
	}
	q_a /= res.sumWeights;

	// Compute sum(A_j q^_j)
	dst.set(q_a);
	const vector<ofMatrix2x2> &As = res.As;
	for(int i = 0; i < As.size(); ++ i) {
		dst += (dstPins[i] - q_a) * ((1.0 / res.mu) * As[i]);
	}
}


void MLS::RigidDeformer::compute(const ofVec3f &v, const vector<ofVec3f> &srcPins, MLS::RigidDeformerResult &res) {
	int numPins = srcPins.size();

    // Compute weights, sum of weights and p*
    res.sumWeights = 0;
    res.weights.clear();
    ofVec3f p_a(0, 0, 0);
    for(int i = 0; i < numPins; ++ i) {
        float w = computeWeight(srcPins[i], v, 1.0);
        res.weights.push_back(w);
        res.sumWeights += w;
        p_a += w * srcPins[i];
    }
    p_a /= res.sumWeights;

    // Compute length of (v - p*)
    res.length = (v - p_a).length();

    // Compute array of p^
    vector<ofVec3f> p_hs;
    for(int i = 0; i < numPins; ++ i) {
        p_hs.push_back(srcPins[i] - p_a);
    }

    // Compute mu_s
    res.mu = 0;
    for(int i = 0; i < numPins; ++ i) {
        res.mu += res.weights[i] * p_hs[i].dot(p_hs[i]);
    }

    // Compute A_j
    vector<ofMatrix2x2> &As = res.As;
    As.clear();
    ofVec3f vp_a = v - p_a;
    for(int i = 0; i < numPins; ++ i) {
        const ofVec3f &p_h = p_hs[i];
        // [( x, y)]
        // [(-y, x)]
        ofMatrix2x2 m0(p_h.x, p_h.y, -p_h.y, p_h.x);
        ofMatrix2x2 m1(vp_a.x, vp_a.y, -vp_a.y, vp_a.x);
        As.push_back(res.weights[i] * (m0 * m1.getTransposed()));
    }
}

void MLS::RigidDeformer::deform(ofVec3f &dst, const vector<ofVec3f> &dstPins, const MLS::RigidDeformerResult &res) {
	// Compute q*
	ofVec3f q_a(0, 0, 0);
	for(int i = 0; i < dstPins.size(); ++ i) {
		q_a += res.weights[i] * dstPins[i];
	}
	q_a /= res.sumWeights;

	// Compute sum(A_j q^_j)
	ofVec3f t(0, 0, 0);
	const vector<ofMatrix2x2> &As = res.As;
	for(int i = 0; i < As.size(); ++ i) {
		t += (dstPins[i] - q_a) * ((1.0 / res.mu) * As[i]);
	}
	dst.set(res.length * t / t.length() + q_a);
}