#include <thu_data_glove/protocol.h>

QByteArray pack(const uint8_t *data, size_t len)
{
	return pack(QByteArray::fromRawData(reinterpret_cast<const char*>(data), (int)len));
}

QByteArray pack(const QByteArray &data)
{
	package_frame frame;
	frame.length = data.length();
	QByteArray p = QByteArray::fromRawData(reinterpret_cast<const char *>(&frame), (int)sizeof(package_frame));

	uint8_t chksum = 0;
	for (int i = 0 ; i < data.length() ; i++)
		chksum += data[i];
	p.append(data);
	p.append(chksum); // chksum
	p.append(0xEDu);  // eop

	return p;
}

QByteArray unpack(QByteArray &data, bool *complete_package)
{
	for (int i = 0 ; i + (int)sizeof(package_frame) + 2 < data.length() ; i++){
		const package_frame * frame = reinterpret_cast<const package_frame *>(data.constData() + i);
		// if data[i] to data[-1] contains a complete package
		if (frame->header == GLOVE_PACKAGE_HEADER && frame->length <= data.length() - i - (int)sizeof(package_frame) - 2){
			uint8_t chksum = 0;
			for (int j = 0 ; j < frame->length; j++)
				chksum += frame->dataload[j];
			if (chksum == frame->dataload[frame->length] && frame->dataload[frame->length+1] == GLOVE_PACKAGE_EOP){
				QByteArray a(reinterpret_cast<const char *>(frame->dataload), frame->length);
				if (complete_package != nullptr)
					*complete_package = true;
				data.remove(0, i+sizeof(package_frame)+frame->length+2);
				return a;
			}
		}
		i++;
	}
	return QByteArray();
}

QByteArray cmd_glove_search_packer(ip_addr self_ip)
{
	cmd_glove_search p;

	p.ip.byte[0] = self_ip.byte[3];
	p.ip.byte[1] = self_ip.byte[2];
	p.ip.byte[2] = self_ip.byte[1];
	p.ip.byte[3] = self_ip.byte[0];
	p.udp_port = GLOVE_UDP_PORT;

	return pack(reinterpret_cast<const uint8_t *>(&p), sizeof(cmd_glove_search));
}

QByteArray cmd_glove_connect_packer(dev_id id)
{
	cmd_glove_connect p;

	p.id = id;

	return pack(reinterpret_cast<const uint8_t *>(&p), sizeof(cmd_glove_connect));
}

QByteArray cmd_glove_disconnect_packer()
{
	cmd_glove_disconnect p;

	return pack(reinterpret_cast<const uint8_t *>(&p), sizeof(cmd_glove_disconnect));
}
