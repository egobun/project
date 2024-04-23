#include "SerialPort.h"
QByteArray  dann;
SerialPort::SerialPort(QObject *parent)
    : QObject{parent}
    ,_serialPort(nullptr)
{}

bool SerialPort::connect(QString portName)
{
    if(_serialPort != nullptr){
        _serialPort->close();
        delete _serialPort;
    }
    _serialPort = new QSerialPort(this);
    _serialPort->setPortName(portName);
    _serialPort->setBaudRate(QSerialPort::Baud115200);
    _serialPort->setDataBits(QSerialPort::Data8);
    _serialPort->setParity(QSerialPort::NoParity);
    _serialPort->setStopBits(QSerialPort::OneStop);
    if(_serialPort->open(QIODevice::ReadWrite)){

        QObject::connect(_serialPort,&QSerialPort::readyRead,this, &SerialPort::dataReady);
    }
    return _serialPort->isOpen();
}

quint64 SerialPort::write(QByteArray data)
{
    if(_serialPort == nullptr || !_serialPort->isOpen()){
        return -1;
    }
    return _serialPort->write(data);
}

void SerialPort::dataReady()
{
    //if(_serialPort->isOpen()){
    while (_serialPort->waitForReadyRead(10)){
        dann = _serialPort->readAll();
        emit dataReceived(dann);
    }
        //requestData += serial.readAll();

        //return;
    //}
}

SerialPort::~SerialPort()
{
    if(_serialPort != nullptr){
        _serialPort->close();
        delete _serialPort;
    }

}
