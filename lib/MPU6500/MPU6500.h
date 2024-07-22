#ifndef __MPU6500_H__
#define __MPU6500_H__

#include <ArduinoEigen.h>
#include <MPU6500_WE.h>
#include <Wire.h>

using namespace Eigen;

const float g = 9.80665f;
const float pi = 3.141592654f;

class MPU6500 {
public:
	MPU6500(TwoWire& wire = Wire, uint8_t const addr = 0x68);
	bool init();

	void setAccRange(MPU9250_accRange accRange);
	void setGyrRange(MPU9250_gyroRange gyroRange);
	/**
	get acceleration in M/s^2
	 */
	Vector3f getAccValues();
	/**
	get angle speed in Radius/s
	 */
	Vector3f getGyroValues();
	xyzFloat getGyroRawValues();
	float getTemperature();

	Vector3f getGUnitVector();
	Vector3f getHorizontalXUnitVector();
	Vector3f getHorizontalYUnitVector();

	Vector3f getUnitAccVar();
	Vector3f getGyroVar();

private:
	void updateUnitAccVar(const Vector3f& unitAcc);
	void updateGyroVar(const Vector3f& gyro);
	Vector3f initHorizontalXUnitVector();

	MPU6500_WE mpu6500;
	Vector3f gUnitVector;
	Vector3f gHorizontalXUnitVector;
	static const int MAX_SAMPLE_NUM = 500;
	// 计算方差的样本数
	int unitAccNum;
	Vector3f unitAccVar;
	Vector3f unitAccAvg;
	// 计算方差的样本数
	int gyroNum;
	Vector3f gyroVar;
	Vector3f gyroAvg;
	unsigned long gyroLastTimeMs;
	/** karlman P */
	Matrix3f P;
};

#endif