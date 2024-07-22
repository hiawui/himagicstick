#include "MPU6500.h"

const float lsbPerG = static_cast<float>(1 << 14);
const float lsbPerRadian = static_cast<float>(1 << 15) / 250 * 180 / pi;
const float MIN_NORM = 0.000001f;

Vector3f multiply(Vector3f a, Vector3f b) {
	return Vector3f(a.x() * b.x(), a.y() * b.y(), a.z() * b.z());
}

Matrix3f covarianceMatrix(Vector3f v) {
	return Matrix3f{
		{v.x(), 0, 0},
		{0, v.y(), 0},
		{0, 0, v.z()},
	};
}

Vector3f xyzFloat2Vector(xyzFloat v) { return Vector3f(v.x, v.y, v.z); }

xyzFloat vector2XyzFloat(Vector3f v) { return xyzFloat(v.x(), v.y(), v.z()); }

bool isZero(float x) { return x > -MIN_NORM && x < MIN_NORM; }

/**
卡尔曼滤波predict
x0: 重力相对芯片坐标的向量
x1: 水平x轴单位向量
P: x的协方差矩阵, 未知, 随机初始值
u: 陀螺仪的角速度, in radians
dt: 两次取样的时间差
Q: 角速度的协方差矩阵
 */
void karlmanPredict(Vector3f& x0, Vector3f& x1, Matrix3f& P, const Vector3f& u, float dt, const Matrix3f& Q) {
	Vector3f theta = Vector3f(u.x() * dt, u.y() * dt, u.z() * dt);
	float norm = theta.norm();
	if (norm < MIN_NORM) {
		return;
	}
	float halfNorm = norm / 2;
	float sinHalfNorm = sinf(halfNorm);
	Quaternionf q = Quaternionf(
		cosf(halfNorm),
		sinHalfNorm * theta.x() / norm,
		sinHalfNorm * theta.y() / norm,
		sinHalfNorm * theta.z() / norm
	);

	Matrix3f F = q.inverse().toRotationMatrix();
	x0 = F * x0;
	x1 = F * x1;
	P = F * P * F.transpose() + Q;
}
/**
卡尔曼滤波update
x0: 估计的重力单位向量
x1: 水平x轴单位向量
z: 加速度计单位向量, 假设加速度只有重力影响
P: x的协方差矩阵, 未知, 随机初始值
R: 加速度协方差
 */
void karlmanUpdate(Vector3f& x0, Vector3f& x1, const Vector3f& z, Matrix3f& P, const Matrix3f& R) {
	Vector3f x00 = x0;
	Matrix3f S = P + R;
	Matrix3f K = P * S.inverse();
	x0 = x0 + K * (z - x0);
	x0 /= x0.norm();
	P = P - K * P;

	// 修正x1
	Quaternionf q = Quaternionf::FromTwoVectors(x00, x0);
	x1 = q * x1;
	x1 /= x1.norm();
}

MPU6500::MPU6500(TwoWire& wire, uint8_t addr) : mpu6500(&wire, addr) {
	gyroLastTimeMs = millis();
	P = Matrix3f{
		{1.f, 0.f, 0.f},
		{0.f, 1.f, 0.f},
		{0.f, 0.f, 1.f},
	};
	unitAccAvg = Vector3f(0.f, 0.f, 0.f);
	unitAccVar = Vector3f(0.f, 0.f, 0.f);
	gyroAvg = Vector3f(0.f, 0.f, 0.f);
	gyroVar = Vector3f(0.f, 0.f, 0.f);
}

bool MPU6500::init() {
	if (!this->mpu6500.init()) {
		return false;
	}

	mpu6500.enableAccDLPF(true);
	mpu6500.setAccDLPF(MPU6500_DLPF_4);
	mpu6500.enableGyrDLPF();
	mpu6500.setGyrDLPF(MPU6500_DLPF_5);
	mpu6500.setAccRange(MPU6500_ACC_RANGE_2G);
	mpu6500.setGyrRange(MPU6500_GYRO_RANGE_250);
	delay(100);

	Vector3f rawAccAvg{ 0.f, 0.f, 0.f }, rawGyroAvg{ 0.f, 0.f, 0.f };
	for (int i = 0; i < 200; ++i) {
		Vector3f rawAccValue = xyzFloat2Vector(mpu6500.getAccRawValues());
		rawAccAvg = rawAccAvg * i / (i + 1) + rawAccValue / (i + 1);
		updateUnitAccVar(rawAccValue / rawAccValue.norm());

		Vector3f rawGyroValue = xyzFloat2Vector(mpu6500.getGyrRawValues());
		rawGyroAvg = rawGyroAvg * i / (i + 1) + rawGyroValue / (i + 1);
		updateGyroVar(rawGyroValue / lsbPerRadian);
		delay(1);
	}
	gyroLastTimeMs = millis();

	float accOffsetFactor = lsbPerG / rawAccAvg.norm();
	Vector3f accOffsets = rawAccAvg * (1 - accOffsetFactor);
	mpu6500.setAccOffsets(vector2XyzFloat(accOffsets));

	gUnitVector = rawAccAvg / rawAccAvg.norm();
	gHorizontalXUnitVector = initHorizontalXUnitVector();

	Serial.printf("init g: [%f, %f, %f] hx: [%f, %f, %f]\n",
		gUnitVector.x(), gUnitVector.y(), gUnitVector.z(),
		gHorizontalXUnitVector.x(), gHorizontalXUnitVector.y(), gHorizontalXUnitVector.z()
	);

	mpu6500.setGyrOffsets(vector2XyzFloat(rawGyroAvg));

	mpu6500.setSampleRateDivider(5);
	return true;
}

void MPU6500::setAccRange(MPU9250_accRange range) {
	mpu6500.setAccRange(range);
}

void MPU6500::setGyrRange(MPU9250_gyroRange range) {
	mpu6500.setGyrRange(range);
}

Vector3f MPU6500::getAccValues() {
	Vector3f rawAcc = xyzFloat2Vector(mpu6500.getGValues());
	float rawAccNorm = rawAcc.norm();
	// 只在加速度接近重力加速度时校正
	if (rawAccNorm >= 0.85 && rawAccNorm <= 1.15) {
		Vector3f unitAcc = rawAcc / rawAccNorm;
		updateUnitAccVar(unitAcc);
		Vector3f x0 = gUnitVector;
		Vector3f x1 = gHorizontalXUnitVector;
		Vector3f z = unitAcc;
		Matrix3f R = covarianceMatrix(unitAccVar);
		karlmanUpdate(x0, x1, z, P, R);
		gUnitVector = x0;
		gHorizontalXUnitVector = x1;
	}
	return rawAcc * g;
}

xyzFloat MPU6500::getGyroRawValues() {
	return mpu6500.getGyrValues();
}

Vector3f MPU6500::getGyroValues() {
	Vector3f gyro = xyzFloat2Vector(mpu6500.getGyrValues()) * pi / 180;
	updateGyroVar(gyro);
	Vector3f x0 = gUnitVector;
	Vector3f x1 = gHorizontalXUnitVector;
	Vector3f u = gyro;
	unsigned long currMs = millis();
	float dt = (currMs - gyroLastTimeMs) / 1000.0f;
	gyroLastTimeMs = currMs;
	Matrix3f Q = covarianceMatrix(gyroVar);
	karlmanPredict(x0, x1, P, u, dt, Q);
	gUnitVector = x0;
	gHorizontalXUnitVector = x1;
	return gyro;
}

float MPU6500::getTemperature() { return mpu6500.getTemperature(); }

Vector3f MPU6500::getGUnitVector() { return gUnitVector; }
Vector3f MPU6500::getHorizontalXUnitVector() { return gHorizontalXUnitVector; }

Vector3f MPU6500::getHorizontalYUnitVector() {
	Vector3f g = gUnitVector;
	Vector3f x = gHorizontalXUnitVector;
	Vector3f y = g.cross(x);
	y /= y.norm();
	return y;
}

Vector3f MPU6500::initHorizontalXUnitVector() {
	Vector3f g = gUnitVector;
	Vector3f y{ 0.f, 1.f, 0.f };
	Vector3f hx = y.cross(g);
	if (isZero(hx.x()) && isZero(hx.y()) && isZero(hx.z())) {
		hx = Vector3f(1.f, 0.f, 0.f);
	} else {
		hx /= hx.norm();
	}
	return hx;
}

void MPU6500::updateUnitAccVar(const Vector3f& unitAcc) {
	Vector3f newUnitAccAvg = (unitAccAvg * unitAccNum + unitAcc) / (unitAccNum + 1);
	unitAccVar = (unitAccVar * unitAccNum +
		multiply(unitAcc - newUnitAccAvg, unitAcc - unitAccAvg)) /
		(unitAccNum + 1);
	unitAccAvg = newUnitAccAvg;
	// 当样本数达到最大值时就取近似值, 相当于增加近期样本的权重
	if (unitAccNum < MAX_SAMPLE_NUM) {
		unitAccNum++;
	}
}

void MPU6500::updateGyroVar(const Vector3f& gyro) {
	Vector3f newGyroAvg = (gyroAvg * gyroNum + gyro) / (gyroNum + 1);
	gyroVar =
		(gyroVar * gyroNum + multiply(gyro - newGyroAvg, gyro - gyroAvg)) /
		(gyroNum + 1);
	gyroAvg = newGyroAvg;
	// 当样本数达到最大值时就取近似值, 相当于增加近期样本的权重
	if (gyroNum < MAX_SAMPLE_NUM) {
		gyroNum++;
	}
}

Vector3f MPU6500::getUnitAccVar() {
	return unitAccVar;
}

Vector3f MPU6500::getGyroVar() {
	return gyroVar;
}