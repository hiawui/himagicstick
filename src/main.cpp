#include <MPU6500.h>
#include <Wire.h>

MPU6500 myMPU6500;

void setup() {
	Serial.begin(115200);
	Wire.begin();
	if (!myMPU6500.init()) {
		Serial.println("MPU6500 does not respond");
	} else {
		Serial.println("MPU6500 is connected");
	}

	/*  MPU6500_GYRO_RANGE_250       250 degrees per second (default)
	 *  MPU6500_GYRO_RANGE_500       500 degrees per second
	 *  MPU6500_GYRO_RANGE_1000     1000 degrees per second
	 *  MPU6500_GYRO_RANGE_2000     2000 degrees per second
	 */
	myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);

	/*  MPU6500_ACC_RANGE_2G      2 g   (default)
	 *  MPU6500_ACC_RANGE_4G      4 g
	 *  MPU6500_ACC_RANGE_8G      8 g
	 *  MPU6500_ACC_RANGE_16G    16 g
	 */
	myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);

	delay(200);
}

const unsigned long sampleIntvMs = 20;

void loop() {
	unsigned long ms = millis();
	// xyzFloat gyrRaw = myMPU6500.getGyroRawValues();
	Vector3f gyr = myMPU6500.getGyroValues();
	Vector3f acc = myMPU6500.getAccValues();
	float temp = myMPU6500.getTemperature();

	Vector3f gyroVar = myMPU6500.getGyroVar();
	Vector3f accVar = myMPU6500.getUnitAccVar();

	Vector3f gv = myMPU6500.getGUnitVector();
	Vector3f hxv = myMPU6500.getHorizontalXUnitVector();
	Vector3f hyv = myMPU6500.getHorizontalYUnitVector();
	Vector3f x = Vector3f(1, 0, 0);
	Vector3f y = Vector3f(0, 1, 0);
	Vector3f z = Vector3f(0, 0, 1);
	// Serial.printf("@@ gv: [%f, %f, %f], hx: [%f, %f, %f], hy: [%f, %f, %f]\n",
	// 	gv.x(), gv.y(), gv.z(),
	// 	hxv.x(), hxv.y(), hxv.z(),
	// 	hyv.x(), hyv.y(), hyv.z());

	Serial.printf("gyro: [%f, %f, %f], gyroVar: [%f, %f, %f], acc: [%f, %f, %f], accVar: [%f, %f, %f]\n",
		gyr.x(), gyr.y(), gyr.z(),
		gyroVar.x(), gyroVar.y(), gyroVar.z(),
		acc.x(), acc.y(), acc.z(),
		accVar.x(), accVar.y(), accVar.z()
	);
	Serial.printf("##RR##|%f,%f,%f|%f,%f,%f|%f,%f,%f\n",
		x.dot(hxv), x.dot(hyv), x.dot(gv),
		y.dot(hxv), y.dot(hyv), y.dot(gv),
		z.dot(hxv), z.dot(hyv), z.dot(gv));

	unsigned long newMs = millis();
	uint32_t d = (newMs - ms) > sampleIntvMs ? 0 : ms + sampleIntvMs - newMs;
	delay(d);
}