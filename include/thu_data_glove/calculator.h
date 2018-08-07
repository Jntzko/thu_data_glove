#ifndef CALCULATOR_H
#define CALCULATOR_H

#include <QtCore/QObject>
#include <thu_data_glove/datastruct.h>
#include <QtCore/QMap>


class Calculator : public QObject
{
	Q_OBJECT

	QMap<QString, int> map;

public:
	explicit Calculator(QObject *parent=nullptr);

public slots:
	void sensorData_received(SensorData sd);

signals:
	void gloveData_received(GloveData);
};

#endif // CALCULATOR_H
