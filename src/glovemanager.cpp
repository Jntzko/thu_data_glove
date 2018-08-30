#include <thu_data_glove/glovemanager.h>
#include <QtNetwork/QNetworkDatagram>
#include <QtCore/QThread>
#include <QtNetwork/QNetworkInterface>

#define NEW_LH 1192974848
#define NEW_RH 1193564786
#define OLD_LH 1192778367
#define OLD_RH 1192974877

void GloveManager::response_dataflow(const QByteArray &dataload, const QHostAddress &addr)
{
	SensorData sd;
	bool is_connected_dev = false;
	for (int i = 0 ; i < this->dev_ip_id.length() ; i++){
		if (this->dev_ip_id[i].first.ip == addr.toIPv4Address()){
			sd.gloveID = this->dev_ip_id[i].second;
			is_connected_dev = true;
			break;
		}
	}
	if (!is_connected_dev)
		return;

	const response_glove_data * ptr = reinterpret_cast<const response_glove_data *>(dataload.constData());
	// fill in the structure
	for(int i = 0; i < 18; i++){
		sd.sensor_data[i].sensor_id = i;

		sd.sensor_data[i].yaw = (float)ptr->sensor_data[i*12] * 0.01f;
		sd.sensor_data[i].pitch = (float)ptr->sensor_data[i*12+1] * 0.01f;
		sd.sensor_data[i].roll = (float)ptr->sensor_data[i*12+2] * 0.01f;

		sd.sensor_data[i].Wx = (float)ptr->sensor_data[i*12+3] * 0.0002f;
		sd.sensor_data[i].Wy = (float)ptr->sensor_data[i*12+4] * 0.0002f;
		sd.sensor_data[i].Wz = (float)ptr->sensor_data[i*12+5] * 0.0002f;

		sd.sensor_data[i].Ax = (float)ptr->sensor_data[i*12+6] * 0.001f;
		sd.sensor_data[i].Ay = (float)ptr->sensor_data[i*12+7] * 0.001f;
		sd.sensor_data[i].Az = (float)ptr->sensor_data[i*12+8] * 0.001f;

		sd.sensor_data[i].Mx = (float)ptr->sensor_data[i*12+9];
		sd.sensor_data[i].My = (float)ptr->sensor_data[i*12+10];
		sd.sensor_data[i].Mz = (float)ptr->sensor_data[i*12+11];
	}
	sd.timestemp = (float)ptr->time * 0.001f;
	emit this->sensorData_received(sd);
}

void GloveManager::response_search(const QByteArray &dataload, const QHostAddress &addr)
{
	const response_glove_search * ptr = reinterpret_cast<const response_glove_search *>(dataload.constData());
	qDebug() << QString("new dev detected: %1 # %2 %3 %4 %5 %6 %7 %8 %9 %10 %11 %12 %13").arg(ptr->name).arg((ushort)ptr->id.id[0], 2, 16, QChar('0'))
			.arg((ushort)ptr->id.id[1], 2, 16, QChar('0')).arg((ushort)ptr->id.id[2], 2, 16, QChar('0')).arg((ushort)ptr->id.id[3], 2, 16, QChar('0'))
			.arg((ushort)ptr->id.id[4], 2, 16, QChar('0')).arg((ushort)ptr->id.id[5], 2, 16, QChar('0')).arg((ushort)ptr->id.id[6], 2, 16, QChar('0'))
			.arg((ushort)ptr->id.id[7], 2, 16, QChar('0')).arg((ushort)ptr->id.id[8], 2, 16, QChar('0')).arg((ushort)ptr->id.id[9], 2, 16, QChar('0'))
			.arg((ushort)ptr->id.id[10], 2, 16, QChar('0')).arg((ushort)ptr->id.id[11], 2, 16, QChar('0'));

	ip_addr ip;
	ip.ip = addr.toIPv4Address();
	this->dev_ip_id.append(QPair<ip_addr, uint32_t>(ip, reinterpret_cast<const uint32_t *>(ptr->id.id)[0]));

	this->client->writeDatagram(cmd_glove_connect_packer(ptr->id), QHostAddress(ip.ip), GLOVE_UDP_PORT);
}

void GloveManager::response_connect(const QByteArray &dataload, const QHostAddress &addr)
{
	Q_UNUSED(dataload);

	for (int i = 0 ; i < this->dev_ip_id.length() ; i++){
		if (this->dev_ip_id[i].first.ip == addr.toIPv4Address()){
			qDebug() << QString("Connected with shorter glove_id # %1").arg(this->dev_ip_id[i].second, 8, 16, QChar('0'));
			this->connected = true;
		}
    switch (this->dev_ip_id[i].second) {
      case NEW_RH:
		    qDebug() << "Connected with new right glove";
        break;
      case NEW_LH:
		    qDebug() << "Connected with new left glove";
        break;
      case OLD_RH:
		    qDebug() << "Connected with old right glove";
        break;
      case OLD_LH:
		    qDebug() << "Connected with old left glove";
        break;
      default:
        break;
    }
	}
}

GloveManager::GloveManager(QObject *parent) : QObject(parent)
{
	this->client = new QUdpSocket();
	this->client->bind(GLOVE_UDP_PORT);
	this->connected = false;

	connect(this->client, SIGNAL(readyRead()), this, SLOT(readPendingDatagrams()));
}

GloveManager::~GloveManager()
{
	this->close();
	delete this->client;
}

void GloveManager::open()
{
	if (this->connected){
		qDebug() << "Cannot send command when connected to device, call disconnect first!";
	}

	foreach (const QHostAddress &addr, QNetworkInterface::allAddresses()){
		if (addr.protocol() == QAbstractSocket::IPv4Protocol && !addr.isLoopback()){
			this->ip.ip = addr.toIPv4Address();
			break;
		}
	}

	qDebug() << QString("this ip is %1.%2.%3.%4").arg(this->ip.byte[3]).arg(this->ip.byte[2]).arg(this->ip.byte[1]).arg(this->ip.byte[0]);
	this->dev_ip_id.clear();
	ip_addr broadcast = this->ip;
	broadcast.byte[0] = 255u;
	this->client->writeDatagram(cmd_glove_search_packer(this->ip), QHostAddress(broadcast.ip), GLOVE_UDP_PORT);
}

bool GloveManager::isOpen() const
{
	return this->connected;
}

void GloveManager::close()
{
	ip_addr broadcast = this->ip;
	broadcast.byte[0] = 255u;
	this->client->writeDatagram(cmd_glove_disconnect_packer(), QHostAddress(broadcast.ip), GLOVE_UDP_PORT);
	this->dev_ip_id.clear();
	this->connected = false;
}

void GloveManager::readPendingDatagrams()
{
	while (this->client->hasPendingDatagrams()) {
		QNetworkDatagram datagram = this->client->receiveDatagram();
		QByteArray dataload = datagram.data();
		dataload = unpack(dataload);
		if (dataload.length() > 0){
			if (dataload[0] == GLOVE_DATALOAD_INDICATOR_CONTROL){
				if (dataload[1] == GLOVE_DATALOAD_DATATYPE_SEARCH_RET)
					this->response_search(dataload, datagram.senderAddress());
				else if (dataload[1] == GLOVE_DATALOAD_DATATYPE_CONNECT_RET)
					this->response_connect(dataload, datagram.senderAddress());
			}else if (dataload[0] == GLOVE_DATALOAD_INDICATOR_DATA && dataload[1] == GLOVE_DATALOAD_DATATYPE_DATA)
				this->response_dataflow(dataload, datagram.senderAddress());
		}
	}
}
