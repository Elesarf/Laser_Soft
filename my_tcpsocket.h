#ifndef MY_TCPSOCKET_H
#define MY_TCPSOCKET_H

#ifndef QT_WARNING_DISABLE_DEPRECATED
# define QT_WARNING_DISABLE_DEPRECATED
#endif

#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>

#include <vector>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace  std;

class My_TCPSocket : public QObject
{
    Q_OBJECT
public:
    explicit My_TCPSocket(QObject * parent = NULL);
    ~My_TCPSocket();
signals:
    void talking(const QString &);
    void nextPacket();
    void incommingConnectSignal();
public slots:
    void Connect(quint16);
    void Connect();
    bool GetState(){ return state_; }

    void incommingConnect();
    void readyRead();
    void stateChanged(QAbstractSocket::SocketState stat);

    void sendCommand(QString);
    void setSendingFlag(bool);

private:
    QTcpServer * _server;
    QList<QTcpSocket *> _sockets;
    QTimer * timer;
    QString buffer_;

    bool state_	= false;
    bool flag_	= false;
};

#endif  // MY_TCPSOCKET_H
