#ifndef GLOVEMANAGER_H
#define GLOVEMANAGER_H

#include <QtCore/QObject>
#include <QtNetwork/QUdpSocket>
#include <thu_data_glove/protocol.h>
#include <QtCore/QList>
#include <QtCore/QPair>
#include <thu_data_glove/datastruct.h>

class GloveManager : public QObject
{
    Q_OBJECT

private:
	QUdpSocket * client;
	QByteArray buf;
	ip_addr ip;
	QList<QPair<ip_addr, uint32_t>> dev_ip_id;
	bool connected;

private:
	void response_dataflow(const QByteArray &dataload, const QHostAddress &addr);
	void response_search(const QByteArray &dataload, const QHostAddress &addr);
	void response_connect(const QByteArray &dataload, const QHostAddress &addr);

public:
    explicit GloveManager(QObject *parent = nullptr);
	~GloveManager();

public:
	bool isOpen() const;

public slots:
	void open();
	void close();

signals:
	void sensorData_received(SensorData);

private slots:
	void readPendingDatagrams();
};

#endif // GLOVEMANAGER_H
