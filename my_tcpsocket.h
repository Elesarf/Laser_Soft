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
    enum STATE {
        S_START,
        S_READ
    };

    struct d_Coordinate {
        double	x;
        double	y;
    };

    struct Coordinate {
        int x;
        int y;
    };

signals:
    void imgComplete(const QByteArray &, char, char, uint16_t);
    void talking(const QString &);
public slots:
    void Connect(quint16);
    void Connect();
    bool GetState(){ return _state; }

    bool Get_Laser_State(){ return _laser_on; }

    void incommingConnect();
    void readyRead();
    void stateChanged(QAbstractSocket::SocketState stat);

    bool Write_X01(double, double, int, int);
    bool Write_X00();
    bool Write_X02(int);
    bool Write_X42();
    bool Write_X03();
    bool Write_X04(bool);
    bool Write_X05(bool);
    bool Write_String(QString);

    void Send_Vector(vector<vector<cv::Point> >, double, double);
    void Send_Vector();

    void SetShift(int, int);
    void SetSize(int);
    void SetSpeed(int);
    void SetPower(int);
    void SetLaserState(bool l){ _laser_on = l; }

    void Reset();

private:
    QTcpServer * _server;
    QList<QTcpSocket *> _sockets;
    QTimer * timer;
    QTimer * sendTimer;

    QByteArray _buffer;
    vector<cv::Point> _contour;
    vector<d_Coordinate> _d_coord;

    Coordinate _now_coordinate;

    unsigned _cons, con;

    int _shift_x, _shift_y;
    int _sizePost, _speed;
    double _cx, _cy;

    int _power;
    bool _laser_on;

    bool _state			= false;
    bool flag			= false;
    bool _flagReadyGo	= false;
    bool _flagReadySend = false;
};

#endif  // MY_TCPSOCKET_H
