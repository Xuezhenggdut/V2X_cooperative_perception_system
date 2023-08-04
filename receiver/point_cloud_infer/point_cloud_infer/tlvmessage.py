SEND = 0
RECEIVE = 1


class TLVMessage:
    def __init__(self, msg: bytes, message_type, new_msg=False, config: tuple = (b'\x00\x00\x00\x11',  # aid
                                                                                 b'\x00\x00\x00\x01',  # traffic_period
                                                                                 b'\x00\x00\x00\x00',  # priority
                                                                                 b'\x00\x00\xff\xff')):  # traffic_id
        """
        Initialization function.

        :param msg: Bytes TLV message or bytes payload.
        :param message_type: SEND or RECEIVE TLV message type.
        :param new_msg: Whether to generate a new TLV message.
        """
        self.message_type = message_type  # 消息类型，发送或接收
        self.unresolved = False
        if new_msg:
            self.send_tlv_message = {'aid': config[0], 'traffic_period': config[1],
                                     'network_protocol_type': b'\x00\x00\x00\x04', 'priority': config[2],
                                     'app_layer_id_changed': b'\x00\x00\x00\x00', 'traffic_id': config[3],
                                     'source_address': b'\x00\x00\x00\x00\x00\x00\x00\x00'
                                                       b'\x00\x00\x00\x00\x00\x00\x00\x00',
                                     'destination_address': b'\x00\x00\x00\x00\x00\x00\x00\x00'
                                                            b'\x00\x00\x00\x00\x00\x00\x00\x00',
                                     'payload': msg, 'payload_len': len(msg)}
            self.tlv_raw_message = b'\x00\x04\x00\x04' + self.send_tlv_message['aid']
            self.tlv_raw_message += b'\x00\x05\x00\x04' + self.send_tlv_message['traffic_period']
            self.tlv_raw_message += b'\x00\x06\x00\x04' + self.send_tlv_message['network_protocol_type']
            self.tlv_raw_message += b'\x00\x07\x00\x04' + self.send_tlv_message['priority']
            self.tlv_raw_message += b'\x00\x08\x00\x04' + self.send_tlv_message['app_layer_id_changed']
            self.tlv_raw_message += b'\x00\x09\x00\x04' + self.send_tlv_message['traffic_id']
            self.tlv_raw_message += b'\x00\x0a\x00\x10' + self.send_tlv_message['source_address']
            self.tlv_raw_message += b'\x00\x0b\x00\x10' + self.send_tlv_message['destination_address']
            s = hex(self.send_tlv_message['payload_len'])
            s = s[2:]
            if len(s) < 4:
                s = (4 - len(s)) * '0' + s

            self.tlv_raw_message += b'\x00\x01' + bytes.fromhex(s) + self.send_tlv_message['payload']
        else:
            self.tlv_raw_message = msg
            self.message_len = len(msg)
            if message_type is SEND:
                self.send_tlv_message = {'aid': b'', 'traffic_period': b'', 'network_protocol_type': b'',
                                         'priority': b'', 'app_layer_id_changed': b'', 'traffic_id': b'',
                                         'source_address': b'', 'destination_address': b'', 'payload': b'',
                                         'payload_len': -1}
                index = 0
                while index < self.message_len - 1:
                    tag = self.tlv_raw_message[index:index + 2]
                    if tag == b'\x00\x04':  # AID
                        self.send_tlv_message['aid'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x05':  # Traffic Period
                        self.send_tlv_message['traffic_period'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x06':  # Network Protocol Type
                        self.send_tlv_message['network_protocol_type'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x07':  # Priority
                        self.send_tlv_message['priority'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x08':  # Application Layer ID Changed
                        self.send_tlv_message['app_layer_id_changed'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x09':  # Traffic ID
                        self.send_tlv_message['traffic_id'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x0a':  # Source Address
                        self.send_tlv_message['source_address'] = self.tlv_raw_message[index + 4:index + 20]
                        index += 20
                    elif tag == b'\x00\x0b':  # Destination Address
                        self.send_tlv_message['destination_address'] = self.tlv_raw_message[index + 4:index + 20]
                        index += 20
                    elif tag == b'\x00\x01':  # Payload
                        self.send_tlv_message['payload_len'] = int.from_bytes(self.tlv_raw_message[index + 2:index + 4],
                                                                              byteorder='big', signed=False)
                        self.send_tlv_message['payload'] = \
                            self.tlv_raw_message[index + 4:index + 4 + self.send_tlv_message['payload_len']]
                        index += 4 + self.send_tlv_message['payload_len']
                    else:
                        self.unresolved = True
                        break
            elif message_type is RECEIVE:
                self.receive_tlv_message = {'aid': b'', 'network_protocol_type': b'', 'priority': b'',
                                            'source_address': b'', 'destination_address': b'',
                                            'payload': b'', 'payload_len': -1, 'rsrp': b'', 'sinr': b'',
                                            'rx_total_power': b'', 'res_pool1_crb': b'', 'res_pool2_crb': b'',
                                            'reserved1': b''}
                index = 0
                while index < self.message_len - 1:
                    tag = self.tlv_raw_message[index:index + 2]
                    # print(tag.hex())
                    if tag == b'\x00\x05':  # AID
                        self.receive_tlv_message['aid'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x06':  # Source Address
                        self.receive_tlv_message['source_address'] = self.tlv_raw_message[index + 4:index + 20]
                        index += 20
                    elif tag == b'\x00\x07':  # Destination Address
                        self.receive_tlv_message['destination_address'] = self.tlv_raw_message[index + 4:index + 20]
                        index += 20
                    elif tag == b'\x00\x08':  # Network Protocol Type
                        self.receive_tlv_message['network_protocol_type'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x09':  # Priority
                        self.receive_tlv_message['priority'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x0a':  # RSRP dBm
                        self.receive_tlv_message['rsrp'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x0b':  # SINR dB
                        self.receive_tlv_message['sinr'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x0c':  # RX Total Power
                        self.receive_tlv_message['rx_total_power'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x0d':  # Resource Pool 1 CBR
                        self.receive_tlv_message['res_pool1_crb'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x0e':  # Resource Pool 2 CBR
                        self.receive_tlv_message['res_pool2_crb'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x01':  # Payload
                        self.receive_tlv_message['payload_len'] = \
                            int.from_bytes(self.tlv_raw_message[index + 2:index + 4], byteorder='big', signed=False)
                        self.receive_tlv_message['payload'] = \
                            self.tlv_raw_message[index + 4:index + 4 + self.receive_tlv_message['payload_len']]
                        index += 4 + self.receive_tlv_message['payload_len']
                    elif tag == b'\x00\x02':  # Reserved1
                        length = int.from_bytes(self.tlv_raw_message[index + 2:index + 4],
                                                byteorder='big', signed=False)
                        self.receive_tlv_message['reserved1'] = self.tlv_raw_message[index + 4:index + 4 + length]
                        index += 4 + length
                    else:
                        self.unresolved = True
                        break
            else:
                print('Unknown message type!')
                raise ValueError

    def get_tlv_raw_message(self) -> bytes:
        return self.tlv_raw_message

    def get_payload(self) -> bytes:
        if self.unresolved:
            print('Can not resolve this TLV message!')
            raise ValueError
        else:
            if self.message_type is SEND:
                return self.send_tlv_message['payload']
            else:
                return self.receive_tlv_message['payload']

    def __str__(self):
        if self.unresolved:
            return 'Unresolved message: 0x' + self.tlv_raw_message.hex()
        else:
            if self.message_type is SEND:
                return 'AID: %d, ' % int.from_bytes(self.send_tlv_message['aid'], byteorder='big', signed=False) + \
                       'Traffic Period: %d, ' % \
                       int.from_bytes(self.send_tlv_message['traffic_period'], byteorder='big', signed=False) + \
                       'Network Protocol Type: %d, ' % \
                       int.from_bytes(self.send_tlv_message['network_protocol_type'], byteorder='big', signed=False) + \
                       'Priority: %d, ' % \
                       int.from_bytes(self.send_tlv_message['priority'], byteorder='big', signed=False) + \
                       'Application Layer ID Changed: %d, ' % \
                       int.from_bytes(self.send_tlv_message['app_layer_id_changed'], byteorder='big', signed=False) + \
                       'Traffic ID: %d\n' % \
                       int.from_bytes(self.send_tlv_message['traffic_id'], byteorder='big', signed=False) + \
                       'Source Address: %d, ' % \
                       int.from_bytes(self.send_tlv_message['source_address'], byteorder='big', signed=False) + \
                       'Destination Address: %d, ' \
                       % int.from_bytes(self.send_tlv_message['destination_address'], byteorder='big', signed=False) + \
                       'Payload length: %d\n' % self.send_tlv_message['payload_len'] + \
                       'Payload: 0x' + self.send_tlv_message['payload'].hex()
            else:
                return 'AID: %d, ' % int.from_bytes(self.receive_tlv_message['aid'], byteorder='big', signed=False) + \
                       'Source Address: ' + self.receive_tlv_message['source_address'].hex() + ', ' + \
                       'Destination Address: ' + self.receive_tlv_message['destination_address'].hex() + '\n' + \
                       'Network Protocol Type: %d, ' % \
                       int.from_bytes(self.receive_tlv_message['network_protocol_type'],
                                      byteorder='big', signed=False) + \
                       'Priority: %d, ' % \
                       int.from_bytes(self.receive_tlv_message['priority'], byteorder='big', signed=False) + \
                       'RSRP dBm: %ddBm, ' % \
                       int.from_bytes(self.receive_tlv_message['rsrp'], byteorder='big', signed=True) + \
                       'SINR dB: %ddB, ' % \
                       int.from_bytes(self.receive_tlv_message['sinr'], byteorder='big', signed=True) + \
                       'RX Total Power: %ddBm, ' % \
                       int.from_bytes(self.receive_tlv_message['rx_total_power'], byteorder='big', signed=True) + \
                       'Resource Pool 1 CBR: %d%%, ' % \
                       int.from_bytes(self.receive_tlv_message['res_pool1_crb'], byteorder='big', signed=False) + \
                       'Resource Pool 2 CBR: %d%%, ' % \
                       int.from_bytes(self.receive_tlv_message['res_pool2_crb'], byteorder='big', signed=False) + \
                       'Payload length: %d\n' % self.receive_tlv_message['payload_len'] + \
                       'Payload: 0x' + self.receive_tlv_message['payload'].hex()


def test():
    send_bytes_message = bytes.fromhex('00 04 00 04 00 00 00 11 '
                                       '00 05 00 04 00 00 00 01'
                                       '00 06 00 04 00 00 00 04'
                                       '00 07 00 04 00 00 00 00'
                                       '00 08 00 04 00 00 00 00'
                                       '00 09 00 04 00 00 FF FF'
                                       '00 0A 00 10 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00'
                                       '00 0B 00 10 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00'
                                       '00 01 00 06 00 11 00 02 11 22')
    receiver_bytes_message = bytes.fromhex('00 06 00 10 00 00 00 FF FF 00 00 64 00 00 00 00 00 00 00 00'
                                           '00 07 00 10 00 00 00 FF FF 00 00 11 00 00 00 00 00 00 00 00'
                                           '00 08 00 04 00 00 00 04'
                                           '00 0A 00 04 FF FF FF CE'
                                           '00 0B 00 04 00 00 00 0E'
                                           '00 0C 00 04 FF FF FF D0'
                                           '00 0D 00 04 00 00 00 00'
                                           '00 0E 00 04 00 00 00 00'
                                           '00 05 00 04 00 00 00 11'
                                           '00 01 00 06 00 11 00 02 11 22')
    # tlv_message = TLVMessage(receiver_bytes_message, RECEIVE)
    tlv_message = TLVMessage(send_bytes_message, SEND)
    print(tlv_message)


if __name__ == '__main__':
    test()
