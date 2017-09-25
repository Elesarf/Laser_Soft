#include "my_tcpsocket.h"
#include <QDebug>

#define SHIFT_X 20

My_TCPSocket::My_TCPSocket(QObject * parent) : QObject(parent)
{
    _server	= new QTcpServer(this);
    flag_	= false;
}

void My_TCPSocket::Connect(quint16 port)
{
    if ( !state_ ) {
        timer = new QTimer;
        timer->setSingleShot(true);

        connect(_server, SIGNAL(newConnection()), this, SLOT(incommingConnect()));
        connect(timer, &QTimer::timeout, [this](){
            for ( int i = 0; i < _sockets.length(); ++i ) {
                if ( _sockets[i]->state() != QAbstractSocket::ConnectedState )
                    _sockets.erase(_sockets.begin() + i);
            }
            timer->start(1000);
        });
        emit talking(QString("Message: server up on port %1 , listen %2 ").arg(port).arg(state_ = _server->listen(QHostAddress::AnyIPv4, port) ) );
    }
}

void My_TCPSocket::Connect()
{
    if ( !state_ ) {
        timer = new QTimer;
        timer->setSingleShot(true);

        connect(_server, SIGNAL(newConnection()), this, SLOT(incommingConnect()));

        connect(timer, &QTimer::timeout, [this](){
            for ( int i = 0; i < _sockets.length(); ++i ) {
                if ( _sockets[i]->state() != QAbstractSocket::ConnectedState )
                    _sockets.erase(_sockets.begin() + i);
            }
            timer->start(100);
        });
        emit talking(QString("Message: server up on default port (1236) , listen %1 ").arg(state_ =  _server->listen(QHostAddress::AnyIPv4, 1236) ) );
    }
}

My_TCPSocket::~My_TCPSocket()
{
    _server->close();
    delete  _server;
    _sockets.clear();
    delete timer;
}

void My_TCPSocket::incommingConnect()
{
    QTcpSocket * _socket = _server->nextPendingConnection();

    connect(_socket, SIGNAL(stateChanged(QAbstractSocket::SocketState)), this, SLOT(stateChanged(QAbstractSocket::SocketState)));

    connect(_socket, SIGNAL(readyRead()), this, SLOT(readyRead()));
    emit talking(QString("Connect %1").arg(_socket->socketDescriptor() ) );
    _sockets.append(_socket);
    emit incommingConnectSignal();
    timer->start(1000);
}

void My_TCPSocket::readyRead()
{
    QObject * object = QObject::sender();

    if ( !object )
        emit talking("Warning: something wrong");

    QTcpSocket * _socket = static_cast<QTcpSocket *>(object);
    emit talking(QString("ReadyRead %1").arg(_socket->socketDescriptor() ) );
    QByteArray buffer = _socket->readAll();
    //    QByteArray def = "KRYA\r\n";
    if ( (buffer == "KRYA\r\n") && flag_ ) {
        emit nextPacket();
    }
    if ( buffer.length() >= 9 ) {
        if ( (buffer[0] == 'x') && (buffer[5] == 'y') ) {
            QString s;
            s = QString::fromLatin1(buffer);
            s.prepend("Now coordinates ");
            emit talking(s);
        }
    }

    buffer.clear();
}

void My_TCPSocket::stateChanged(QAbstractSocket::SocketState stat)
{
    emit talking(QString("Message: state change %1").arg(stat) );
}

// My_TCPSocket::Write_X01

void My_TCPSocket::sendCommand(QString str)
{
    for ( auto &s:_sockets ) {
        qDebug() << s->write(str.toLocal8Bit().data() ) << str;
    }
}

void My_TCPSocket::setSendingFlag(bool f)
{
    flag_ = f;
}   // My_TCPSocket::Send_Vector

//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
