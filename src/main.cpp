#include <QtCore/QCoreApplication>
#include <thu_data_glove/glovemanager.h>
#include <thu_data_glove/calculator.h>
#include <thu_data_glove/rosbridge.h>
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
