#ifndef LASER_MACHINE_H
#define LASER_MACHINE_H

#include <QObject>
#include "vector"
#include "opencv2/core/core.hpp"
#include <QDebug>

using namespace std;
class Laser_Machine : public QObject
{
    Q_OBJECT

    // Structures:

    enum StateLaserMachine {
        WAIT,
        WORK,
    };

    struct DotCoordinateState {
        double			x;
        double			y;
        bool			laserOn;
        unsigned int	power;

        DotCoordinateState(double x_, double y_, bool laserOn_, unsigned int power_) : x(x_), y(y_), laserOn(laserOn_), power(power_){ }

        DotCoordinateState() : x(0), y(0), laserOn(0), power(0){ }

        friend QDebug operator << (QDebug qd, DotCoordinateState &dcs)
        {
            qd << "x=" << dcs.x << ": y=" << dcs.y << ": power=" << dcs.power << ": laser state=" << dcs.laserOn;
            return qd.maybeSpace();
        }
    };

public:
    explicit Laser_Machine(QObject * parent = NULL);
    ~Laser_Machine();
signals:
    bool sendCommand(QString);
    void talking(const QString &);
    void setSendingFlag(bool);

public slots:

    // Commands:
    void command_X00();
    void command_X01(DotCoordinateState);
    void command_X01(double, double, bool, unsigned int);
    void command_X02();
    void command_X02(unsigned int);
    void command_X03();
    void command_X04();
    void command_X04(bool);
    void command_X05();
    void command_X05(bool);
    void command_X42();

    // Recieve part
    void prepareContour(vector<vector<cv::Point> >, double, double);
    void sendToLaser();
    void nextPacket();


    // Seters:
    void setShift(int x, int y)
    {
        shiftX_ = x;
        shiftY_ = y;
    }

    void setSizePacket(int sp){ sizePacket_ = sp; }

    void setPowerLaser(unsigned int p){ power_ = p; }

    void setSpeed(int s){ speed_ = s; }

    void setBurnyState(bool s){ burnyState_ = s; }

    void setShift(double i){ shift_ = i; }

    int GetStateMachine(){ return stateLaserMachine_; }

    // Other:
    void initMoves();
    void resetState();

private:
    vector<DotCoordinateState> dotsToBurn_;
    DotCoordinateState home_ = { 0, 0, 0, 0 };
    StateLaserMachine stateLaserMachine_;
    int shiftX_, shiftY_;
    int speed_;
    double shift_;
    unsigned int power_;
    int sizePacket_;
    bool coolingState_, burnyState_;

    unsigned int cons_;
    bool flagReadyGo_	= false;
    bool flagReadySend_ = false;
};

#endif  // LASER_MACHINE_H
