#include <QtCore/QCoreApplication>
#include "glovemanager.h"
#include "calculator.h"
#include "rosbridge.h"
#include <QtCore/QThread>


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
	GloveManager mgr;
	rosBridge bridge(argc, argv);
	Calculator calc;

	QObject::connect(&mgr, SIGNAL(sensorData_received(SensorData)), &calc, SLOT(sensorData_received(SensorData)));
	QObject::connect(&mgr, SIGNAL(sensorData_received(SensorData)), &bridge, SLOT(sensorData_received(SensorData)));
	QObject::connect(&calc, SIGNAL(gloveData_received(GloveData)), &bridge, SLOT(gloveData_received(GloveData)));

	mgr.close();

	mgr.open();

	return a.exec();
}
