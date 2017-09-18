#include "my_tcpsocket.h"
#include <QDebug>

#define SHIFT_X 20

My_TCPSocket::My_TCPSocket(QObject * parent) : QObject(parent)
{
    _server		= new QTcpServer(this);
    _shift_x	= 0;
    _shift_y	= 0;
    _sizePost	= 200;
    _cons		= 0;
    _speed		= 42;
    _power		= 0;
    _laser_on	= false;
}

void My_TCPSocket::Connect(quint16 port)
{
    if ( !_state ) {
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
        emit talking(QString("Message: server up on port %1 , listen %2 ").arg(port).arg(_state = _server->listen(QHostAddress::AnyIPv4, port) ) );
    }
}

void My_TCPSocket::Connect()
{
    if ( !_state ) {
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
        emit talking(QString("Message: server up on default port (1236) , listen %1 ").arg(_state =  _server->listen(QHostAddress::AnyIPv4, 1236) ) );
    }
}

My_TCPSocket::~My_TCPSocket()
{
    _server->close();
    delete  _server;
    _sockets.clear();
    delete timer;
    //    delete  sendTimer;
}

void My_TCPSocket::incommingConnect()
{
    QTcpSocket * _socket = _server->nextPendingConnection();

    connect(_socket, SIGNAL(stateChanged(QAbstractSocket::SocketState)), this, SLOT(stateChanged(QAbstractSocket::SocketState)));

    connect(_socket, SIGNAL(readyRead()), this, SLOT(readyRead()));
    emit talking(QString("Connect %1").arg(_socket->socketDescriptor() ) );
    _sockets.append(_socket);
    Write_X02(99);
    Write_X01(120, 0, 0, 0);
    Write_X00();
    Write_X05(false);
    Write_X04(false);

    _flagReadyGo = true;
    timer->start(1000);
}

void My_TCPSocket::readyRead()
{
    QObject * object = QObject::sender();

    if ( !object )
        emit talking("Warning: something wrong");

    QTcpSocket * _socket = static_cast<QTcpSocket *>(object);
    emit talking(QString("ReadyRead %1").arg(_socket->socketDescriptor() ) );
    _buffer = _socket->readAll();
    QByteArray def = "KRYA\r\n";
    Write_X04(false);
    if ( (_buffer == def) && (_cons > 0) ) {
        emit talking(QString("count = %1").arg(_cons) );
        _flagReadyGo = true;
        Send_Vector();
        //        qDebug() << _buffer;
    }
    if ( _buffer.length() >= 9 ) {
        if ( (_buffer[0] == 'x') && (_buffer[5] == 'y') ) {
            QString s;
            s = QString::fromLatin1(_buffer);
            s.prepend("Now coordinates ");
            //            qDebug() << _buffer;
            emit talking(s);
        }
    }

    _buffer.clear();
}

void My_TCPSocket::stateChanged(QAbstractSocket::SocketState stat)
{
    emit talking(QString("Message: state change %1").arg(stat) );

    _contour.clear();
}

bool My_TCPSocket::Write_X01(double xt, double yt, int on = 0, int power = 0)
{
    QString s = "X01";

    xt	*= 10;
    yt	*= 10;
    int x	= static_cast<int>(xt);
    int y	= static_cast<int>(yt);

    if ( ( (x < 20000) && ( y < 20000) ) && ( (x >= 0) && ( y >= 0) ) ) {
        s.append(QString("%1%2%3%4").arg(x, 4, 10, QChar('0') ).arg(y, 4, 10, QChar('0') ).arg(on, 1, 10, QChar('0') ).arg(power, 2, 10, QChar('0') ) );
        QByteArray ba;
        ba.append(QByteArray::fromStdString(s.toStdString() ) );
        ba.append(0x0D);
        ba.append(0x0A);
        for ( auto &s:_sockets ) {
            qDebug() << s->write(ba) << ba;
        }
        return true;
    } else {
        return false;
    }
}   // My_TCPSocket::Write_X01

bool My_TCPSocket::Write_X00()
{
    QString s;
    QByteArray ba;

    s.clear();
    s = "X00";
    ba.clear();
    ba.append(QByteArray::fromStdString(s.toStdString() ) );
    ba.append(0x0D);
    ba.append(0x0A);
    for ( int i = 0; i < _sockets.length(); ++i ) {
        qDebug() << "Write to socket " <<  _sockets[i]->write(ba) << ba;
    }
    return true;
}

bool My_TCPSocket::Write_X02(int speed)
{
    if ( (speed > 350) || (speed < 0) )
        return false;

    QString s;
    QByteArray ba;

    s.clear();
    s.append("X02");
    s.append(QString("%1%2%3").arg(speed / 100).arg( (speed - (speed / 100) * 100) / 10 ).arg(speed % 10) );
    ba.clear();
    ba.append(QByteArray::fromStdString(s.toStdString() ) );
    ba.append(0x0D);
    ba.append(0x0A);
    for ( int i = 0; i < _sockets.length(); ++i ) {
        qDebug() << "Write to socket " <<  _sockets[i]->write(ba) << ba;
    }
    return true;
}

bool My_TCPSocket::Write_X42()
{
    QString s;
    QByteArray ba;

    s.clear();
    s = "X42";
    ba.clear();
    ba.append(QByteArray::fromStdString(s.toStdString() ) );
    ba.append(0x0D);
    ba.append(0x0A);
    for ( int i = 0; i < _sockets.length(); ++i ) {
        qDebug() << "Write to socket " <<  _sockets[i]->write(ba) << ba;
    }
    return true;
}

bool My_TCPSocket::Write_X03()  // give now coordinate
{
    for ( auto &s:_sockets ) {
        Write_X04(false);
        qDebug() << s->write(QByteArray("X03").append(0x0D).append(0X0A) ) << "X03";
    }
    return true;
}

bool My_TCPSocket::Write_X04(bool on)   // laser piupiu on:power
{
    if ( on ) {
        for ( auto &s:_sockets ) {
            qDebug()
                << s->write(QByteArray("X04").append(QString("%1%2").arg(1, 1, 10, QChar('0') ).arg(_power, 2, 10, QChar('0') ) ).append(0x0D).append(0X0A) ) << "X04:1";
        }
        //        _laser_on = 1;
        return true;
    } else {
        for ( auto &s:_sockets ) {
            qDebug()
                << s->write(QByteArray("X04").append(QString("%1%2").arg(0, 1, 10, QChar('0') ).arg(_power, 2, 10, QChar('0') ) ).append(0x0D).append(0X0A) ) << "X04:0";
        }
        //        _laser_on = 0;
        return false;
    }
}

bool My_TCPSocket::Write_X05(bool on)
{
    if ( !on ) {
        for ( auto &s:_sockets ) {
            qDebug() << s->write(QByteArray("X051").append(0x0D).append(0X0A) ) << "X051";
        }
    } else {
        for ( auto &s:_sockets ) {
            qDebug() << s->write(QByteArray("X050").append(0x0D).append(0X0A) ) << "X050";
        }
    }
    return true;
}

bool My_TCPSocket::Write_String(QString)
{
    return true;
}

void My_TCPSocket::Send_Vector(vector<vector<cv::Point> > contour, double cx, double cy)
{
    _d_coord.clear();

    d_Coordinate tmp;
    auto end = contour.end();
    for ( auto it = contour.begin(); it != end; ++it ) {
        for ( auto itt = it->begin(); itt != it->end(); ++itt ) {
            if ( ( (itt->x < 4500) && (itt->x > 0) ) && ( (itt->y < 4500) && (itt->y > 0) ) ) {
                tmp.x	= itt->x / cx;
                tmp.y	= itt->y / cy;
                _d_coord.push_back(tmp);
            }
        }
    }

    _flagReadySend	= false;
    _cons			= static_cast<unsigned>( _d_coord.size() - 1);
    Write_X02(_speed);
    Send_Vector();
}   // My_TCPSocket::Send_Vector

void My_TCPSocket::Send_Vector()
{
    _flagReadySend = false;
    if ( _flagReadyGo ) {
        qDebug() << "Send Vector " << "size : " << _d_coord.size();
        int count = 0;
        for (; _cons > 0; --_cons ) {
            if ( count >= _sizePost ) {
                count = 0;
                _flagReadyGo	= true;
                _flagReadySend	= true;
            }
            if ( _flagReadySend )
                break;

            if ( _cons % 2 == 0 ) {
                if ( count == 0 || count == 1 ) {
                    Write_X01(940 - ( (_d_coord[_cons].x - _shift_x ) + (static_cast<double>(_d_coord[_cons].x) / 940 * 18 + 8) ),
                      _d_coord[_cons].y + _shift_y + ( (_d_coord[_cons].y / 600) * 3 ), _laser_on, _power);
                } else {
                    Write_X01(940 - ( (_d_coord[_cons].x - _shift_x) + (static_cast<double>(_d_coord[_cons].x) / 940 * 18 + 8) ),
                      _d_coord[_cons].y + _shift_y + ( (_d_coord[_cons].y / 600) * 3 ), _laser_on, _power);
                }
            }
            //            qDebug() << "x: " << (_d_coord[_cons].x / 100)  << "y: " <<  (_d_coord[_cons].y / 100) * 0.4;
            count++;
        }
    }
    qDebug() << "end post";
    Write_X00();
}   // My_TCPSocket::Send_Vector

void My_TCPSocket::SetShift(int x, int y)
{
    if ( ( x > (-100) ) && ( x < (100) ) )
        _shift_x = x;
    if ( ( y > (-100) ) && ( y < (100) ) )
        _shift_y = y;
}

void My_TCPSocket::SetSize(int s)
{
    if ( ( s > (10) ) && ( s < (1000) ) )
        _sizePost = s;
    talking(QString("setting size %1").arg(_sizePost) );
}

void My_TCPSocket::SetSpeed(int s)
{
    if ( ( s > (10) ) && ( s < (250) ) )
        _speed = s;
    talking(QString("setting speed %1").arg(_speed) );
    Write_X02(_speed);
}

void My_TCPSocket::SetPower(int p)
{
    if ( (p > 2) && (p < 98) )
        _power = p;
    talking(QString("set power %1").arg(_power) );
}

void My_TCPSocket::Reset()
{
    _contour.clear();
    _d_coord.clear();
    _flagReadySend = false;
    _cons = 0;
}

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
