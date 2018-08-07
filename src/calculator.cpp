#include <thu_data_glove/calculator.h>
#include <QtGui/QQuaternion>


Calculator::Calculator(QObject *parent) : QObject(parent)
{
	this->map["littlefinger bot"] = 0;
	this->map["littlefinger mid"] = 1;
	this->map["littlefinger top"] = 2;

	this->map["ringfinger bot"] = 3;
	this->map["ringfinger mid"] = 4;
	this->map["ringfinger top"] = 5;

	this->map["middlefinger bot"] = 6;
	this->map["middlefinger mid"] = 7;
	this->map["middlefinger top"] = 8;

	this->map["forefinger bot"] = 15;
	this->map["forefinger mid"] = 16;
	this->map["forefinger top"] = 17;

	this->map["thumb bot"] = 12;
	this->map["thumb mid"] = 13;
	this->map["thumb top"] = 14;

	this->map["palm"] = 9;
	this->map["forearm"] = 10;
	this->map["arm"] = 11;
}

void Calculator::sensorData_received(SensorData sd)
{
	GloveData gd;
	gd.timestemp = sd.timestemp;
	gd.gloveID = sd.gloveID;

	QQuaternion base = QQuaternion::fromEulerAngles(sd.sensor_data[11].pitch, sd.sensor_data[11].yaw, sd.sensor_data[11].roll);
	// littlefinger
	{
		QQuaternion q0 = QQuaternion::fromEulerAngles(sd.sensor_data[0].pitch, sd.sensor_data[0].yaw, sd.sensor_data[0].roll);
		QQuaternion q1 = QQuaternion::fromEulerAngles(sd.sensor_data[1].pitch, sd.sensor_data[1].yaw, sd.sensor_data[1].roll);
		QQuaternion q2 = QQuaternion::fromEulerAngles(sd.sensor_data[2].pitch, sd.sensor_data[2].yaw, sd.sensor_data[2].roll);

		gd.littlefinger.l00 = (base.inverted()*q2).inverted().toEulerAngles()[2]/100.0;
		gd.littlefinger.l01 = (base.inverted()*q0).inverted().toEulerAngles()[1]/100.0;
		gd.littlefinger.l1 = (q0.inverted()*q1).inverted().toEulerAngles()[1]/100.0;
		gd.littlefinger.l2 = (q1.inverted()*q2).inverted().toEulerAngles()[1]/100.0;
	}

	// ringfinger
	{
		QQuaternion q0 = QQuaternion::fromEulerAngles(sd.sensor_data[3].pitch, sd.sensor_data[3].yaw, sd.sensor_data[3].roll);
		QQuaternion q1 = QQuaternion::fromEulerAngles(sd.sensor_data[4].pitch, sd.sensor_data[4].yaw, sd.sensor_data[4].roll);
		QQuaternion q2 = QQuaternion::fromEulerAngles(sd.sensor_data[5].pitch, sd.sensor_data[5].yaw, sd.sensor_data[5].roll);

		gd.ringfinger.r00 = (base.inverted()*q2).inverted().toEulerAngles()[2]/100.0;
		gd.ringfinger.r01 = (base.inverted()*q0).inverted().toEulerAngles()[1]/100.0;
		gd.ringfinger.r1 = (q0.inverted()*q1).inverted().toEulerAngles()[1]/100.0;
		gd.ringfinger.r2 = (q1.inverted()*q2).inverted().toEulerAngles()[1]/100.0;
	}
	// midfinger
	{
		QQuaternion q0 = QQuaternion::fromEulerAngles(sd.sensor_data[6].pitch, sd.sensor_data[6].yaw, sd.sensor_data[6].roll);
		QQuaternion q1 = QQuaternion::fromEulerAngles(sd.sensor_data[7].pitch, sd.sensor_data[7].yaw, sd.sensor_data[7].roll);
		QQuaternion q2 = QQuaternion::fromEulerAngles(sd.sensor_data[8].pitch, sd.sensor_data[8].yaw, sd.sensor_data[8].roll);

		gd.middlefinger.m00 = (base.inverted()*q2).inverted().toEulerAngles()[2]/100.0;
		gd.middlefinger.m01 = (base.inverted()*q0).inverted().toEulerAngles()[1]/100.0;
		gd.middlefinger.m1 = (q0.inverted()*q1).inverted().toEulerAngles()[1]/100.0;
		gd.middlefinger.m2 = (q1.inverted()*q2).inverted().toEulerAngles()[1]/100.0;
	}
	// forefinger
	{
		QQuaternion q0 = QQuaternion::fromEulerAngles(sd.sensor_data[15].pitch, sd.sensor_data[15].yaw, sd.sensor_data[15].roll);
		QQuaternion q1 = QQuaternion::fromEulerAngles(sd.sensor_data[16].pitch, sd.sensor_data[16].yaw, sd.sensor_data[16].roll);
		QQuaternion q2 = QQuaternion::fromEulerAngles(sd.sensor_data[17].pitch, sd.sensor_data[17].yaw, sd.sensor_data[17].roll);

		gd.forefinger.f00 = (base.inverted()*q2).inverted().toEulerAngles()[2]/100.0;
		gd.forefinger.f01 = (base.inverted()*q0).inverted().toEulerAngles()[1]/100.0;
		gd.forefinger.f1 = (q0.inverted()*q1).inverted().toEulerAngles()[1]/100.0;
		gd.forefinger.f2 = (q1.inverted()*q2).inverted().toEulerAngles()[1]/100.0;
	}
	// thumb
	{
		QQuaternion q0 = QQuaternion::fromEulerAngles(sd.sensor_data[12].pitch, sd.sensor_data[12].yaw, sd.sensor_data[12].roll);
		QQuaternion q1 = QQuaternion::fromEulerAngles(sd.sensor_data[13].pitch, sd.sensor_data[13].yaw, sd.sensor_data[13].roll);
		QQuaternion q2 = QQuaternion::fromEulerAngles(sd.sensor_data[14].pitch, sd.sensor_data[14].yaw, sd.sensor_data[14].roll);

		gd.thumb.t00 = -(base.inverted()*q2).inverted().toEulerAngles()[0]/100.0;
		gd.thumb.t01 = -(base.inverted()*q0).inverted().toEulerAngles()[2]/100.0;
		gd.thumb.t1 = -(q0.inverted()*q1).inverted().toEulerAngles()[2]/100.0;
		gd.thumb.t2 = -(q1.inverted()*q2).inverted().toEulerAngles()[2]/100.0;
	}

	emit this->gloveData_received(gd);
}
