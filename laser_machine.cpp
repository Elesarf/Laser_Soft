#include "laser_machine.h"

#include <QString>
#include <QDebug>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

Q_DECLARE_METATYPE(cv::Point)
Q_DECLARE_METATYPE(vector<vector<cv::Point> > )

Laser_Machine::Laser_Machine(QObject * parent) : QObject(parent)
{
    qRegisterMetaType<cv::Point>("cv::Point");
    qRegisterMetaType<vector<vector<cv::Point> > >("vector<vector<cv::Point>>");
    dotsToBurn_.clear();
    flagReadyGo_		= true;
    power_				= 5;
    burnyState_			= false;
    shiftX_				= 0;
    shiftY_				= 0;
    shift_				= 1;
    sizePacket_			= 200;
    cons_				= 0;
    stateLaserMachine_	= WAIT;
}

Laser_Machine::~Laser_Machine()
{
    flagReadyGo_	= false;
    cons_			= 0;
}

void Laser_Machine::command_X00()   // command GO
{
    QString s = "X00";

    s.append(0x0D);
    s.append(0X0A);
    emit sendCommand(s);
}

void Laser_Machine::command_X01(DotCoordinateState d)   // command send setup from coordinates, power, laser state
{
    QString s = "X01";

    d.x *= 10;
    d.x += shiftX_;
    d.x = 4000 - d.x;
    d.y *= 10;
    d.y += shiftY_;

    qDebug() << d << ": speed =" <<  speed_;

    s.append(QString("%1").arg(static_cast<int>(d.x), 4, 10, QChar('0') ) );
    s.append(QString("%1").arg(static_cast<int>(d.y), 4, 10, QChar('0') ) );
    s.append(QString("%1").arg(d.laserOn, 1, 10, QChar('0') ) );
    s.append(QString("%1").arg(d.power, 2, 10, QChar('0') ) );
    s.append(0x0D);
    s.append(0x0A);
    emit sendCommand(s);
}

void Laser_Machine::command_X01(double x, double y, bool p, unsigned int s)
{
    DotCoordinateState d = { 400 - x, y, p, s };

    command_X01(d);
}

void Laser_Machine::command_X02()   // command send speed move
{
    QString s = "X02";

    s.append(QString("%1").arg(speed_, 3, 10, QChar('0') ) );
    s.append(0x0D);
    s.append(0x0A);
    emit sendCommand(s);
}

void Laser_Machine::command_X02(unsigned int speed)
{
    QString s = "X02";

    s.append(QString("%1").arg(speed, 3, 10, QChar('0') ) );
    s.append(0x0D);
    s.append(0x0A);
    emit sendCommand(s);
}

void Laser_Machine::command_X03()   // command request now coordinates
{
    QString s = "X03";

    s.append(0x0D);
    s.append(0x0A);
    emit sendCommand(s);
}

void Laser_Machine::command_X04()   // command to on laser for 3 seconds
{
    QString s = "X04";

    s.append(QString("%1%2").arg(burnyState_).arg(power_, 2, 10, QChar('0') ) );
    s.append(0x0D);
    s.append(0x0A);
    emit sendCommand(s);
}

void Laser_Machine::command_X04(bool ls)
{
    QString s = "X04";

    s.append(QString("%1%2").arg(ls).arg(0, 1, 10, QChar('0') ) );
    s.append(0x0D);
    s.append(0x0A);
    emit sendCommand(s);
}

void Laser_Machine::command_X05()   // command to on/off cooling
{
    QString s = "X05";

    s.append(QString("%1").arg(coolingState_, 1, 10, QChar('0') ) );
    s.append(0x0D);
    s.append(0x0A);
    emit sendCommand(s);
}

void Laser_Machine::command_X05(bool c)
{
    QString s = "X05";

    s.append(QString("%1").arg(c, 1, 10, QChar('0') ) );
    s.append(0x0D);
    s.append(0x0A);
    emit sendCommand(s);
}

void Laser_Machine::command_X42()
{
    QString s = "X42";

    s.append(0x0D);
    s.append(0x0A);
    emit sendCommand(s);
}

void Laser_Machine::prepareContour(vector<vector<cv::Point> > c, double cx, double cy)
{
    if ( c.size() > 0 && stateLaserMachine_ == WAIT ) {
        stateLaserMachine_ = WORK;
        DotCoordinateState tmp;

        for ( auto it = c.begin(); it != c.end(); ++it ) {
            for ( unsigned int i = 0; i < it->size(); ++i ) {
                tmp.x		= it->at(i).x / cx;
                tmp.y		= it->at(i).y / cy;
                tmp.power	= power_;
                tmp.laserOn = (i == 0 || i == it->size() - 1) ? false : burnyState_;
                dotsToBurn_.push_back(tmp);
            }
        }

        qDebug() << "dots to burn" << dotsToBurn_.size();

        cons_ = static_cast<unsigned>(dotsToBurn_.size() - 1);
        command_X02();
        sendToLaser();
    }
}   // Laser_Machine::prepareContour

void Laser_Machine::sendToLaser()
{
    flagReadySend_ = false;
    if ( flagReadyGo_ ) {
        qDebug() << "Send vector size: " << dotsToBurn_.size();
        int count = 0;
        for (; cons_ > 0; --cons_ ) {
            if ( count >= sizePacket_ ) {
                count =  0;
                flagReadyGo_	= true;
                flagReadySend_	= true;
            }
            if ( flagReadySend_ )
                break;
            command_X01(dotsToBurn_[cons_]);
            count++;
        }
    }

    emit setSendingFlag( (cons_ > 0) ? true : false );

    if ( cons_ <= 0 ) {
        command_X02(150);
        command_X01(home_);
        command_X02();
        stateLaserMachine_ = WAIT;
        dotsToBurn_.clear();
    }
    qDebug() << "end post " << cons_;
    command_X00();
}

void Laser_Machine::nextPacket()
{
    flagReadyGo_ = true;
    sendToLaser();
}

void Laser_Machine::initMoves()
{
    command_X02(140);
    command_X01(home_);
    command_X00();
    command_X05(false);
    command_X04(false);
}

void Laser_Machine::resetState()
{
    dotsToBurn_.clear();
    emit sendCommand("X050");
    command_X01(home_);
    command_X00();
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
