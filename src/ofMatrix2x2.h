#pragma once

class ofMatrix2x2 {
public:
	float a, b, c, d;

	ofMatrix2x2() {
		makeIdentityMatrix();
	}
	ofMatrix2x2(float a, float b, float c, float d) {
		set(a, b, c, d);
	}
	ofMatrix2x2(const ofMatrix2x2 &mat) {
		set(mat);
	}
	~ofMatrix2x2() {}

	void set(float a, float b, float c, float d) {
		this->a = a;
		this->b = b;
		this->c = c;
		this->d = d;
	}

	void set(const ofMatrix2x2 &mat) {
		this->a = mat.a;
		this->b = mat.b;
		this->c = mat.c;
		this->d = mat.d;
	}

	void makeIdentityMatrix() {
		a = c = 1;
		b = d = 0;
	}

	//---------------------------------------------
	// copy a matrix using = operator
	ofMatrix2x2& operator = (const ofMatrix2x2& rhs) {
		if ( &rhs == this ) return *this;
		set(rhs);
		return *this;
	}

	inline ofMatrix2x2 operator + (const ofMatrix2x2& rhs) {
		return ofMatrix2x2(this->a + rhs.a, this->b + rhs.b,
			this->c + rhs.c, this->d + rhs.d);
	}

	inline void operator += (const ofMatrix2x2& other) {
		this->a += other.a;
		this->b += other.b;
		this->c += other.c;
		this->d += other.d;
	}

	bool makeInvertOf(const ofMatrix2x2 & rhs){
		float d = rhs.a * rhs.d - rhs.b * rhs.c;
		if(d == 0) { return false; }
		set(rhs.d / d, -rhs.b / d, -rhs.c / d, rhs.a / d);
		return true;
	}

	ofMatrix2x2 getInverse() const {
		ofMatrix2x2 inverse;
		inverse.makeInvertOf(*this);
		return inverse;
	}

	void makeTransposeOf(const ofMatrix2x2 & rhs) {
		set(rhs.a, rhs.c, rhs.b, rhs.d);
	}

	ofMatrix2x2 getTransposed() const {
		ofMatrix2x2 transposed;
		transposed.makeTransposeOf(*this);
		return transposed;
	}

	// p^T q
	void makeFromMultiplicationOf(const ofVec3f &p, const ofVec3f &q) {
		set(p.x * q.x, p.x * q.y, p.y * q.x, p.y * q.y);
	}

	// v * M
	inline ofVec3f postMult(const ofVec3f& v) const {
		return ofVec3f(this->a * v.x + this->c * v.y, this->b * v.x + this->d * v.y, 0);
	}
	// M * v^T
	inline ofVec3f preMult(const ofVec3f& v) const {
		return ofVec3f(this->a * v.x + this->b * v.y, this->c * v.x + this->d * v.y, 0);
	}
	inline ofVec3f operator* (const ofVec3f& v) const {
		return postMult(v);
	}

	inline ofMatrix2x2 operator* (const ofMatrix2x2& m) const {
		return ofMatrix2x2(this->a * m.a + this->b * m.c, this->a * m.b + this->b * m.d,
			this->c * m.a + this->d * m.c, this->c * m.b + this->d * m.d);
	}

	// s * M
	inline ofMatrix2x2 postMult(float s) const {
		return ofMatrix2x2(s * this->a, s * this->b, s * this->c, s * this->d);
	}

	// M * s
	inline ofMatrix2x2 preMult(float s) const {
		return postMult(s);
	}

	inline ofMatrix2x2 operator* (float s) const {
		return postMult(s);
	}

	friend inline ostream& operator<<(ostream& os, const ofMatrix2x2& M) {
		int w = 8;
		os	<< setw(w)
			<< M.a << ", " << setw(w)
			<< M.b << std::endl;

		os	<< setw(w)
			<< M.c << ", " << setw(w)
			<< M.d;

		return os;
	}
};

inline ofVec3f operator* (const ofVec3f& v, const ofMatrix2x2& M) {
	return M.preMult(v);
}

inline ofMatrix2x2 operator* (float s, const ofMatrix2x2& M) {
	return M.preMult(s);
}