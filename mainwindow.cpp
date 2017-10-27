#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDebug>
#include <QThread>
#include <QTimer>
#include <QTime>
#include <QFile>

#include "vector"

Q_DECLARE_METATYPE(cv::Point)
Q_DECLARE_METATYPE(vector<vector<cv::Point> > )

MainWindow::MainWindow(QWidget * parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    qRegisterMetaType<cv::Point>("cv::Point");
    qRegisterMetaType<vector<vector<cv::Point> > >("vector<vector<cv::Point>>");

    ui->setupUi(this);

    scene	= new QGraphicsScene;
    itm		= new QGraphicsPixmapItem;
    outputCorners_	= new vector<Point2f>(4);
    pointsFile		= new QFile("./settings.txt");
    if ( !pointsFile->exists() ) {
        qDebug() << "File not exist";
    }
    scene->addItem(itm);
    ui->graphicsView->setScene(scene);
}

MainWindow::~MainWindow()
{
    if ( _socketState ) {
        delete _socket;
        delete  laserMachine_;
    }
    delete scene;
    delete ui;
    delete  pointsFile;
    delete  outputCorners_;
}

void MainWindow::talking(const QString &s)
{
    ui->textBrowser->append(s);
}

void MainWindow::talking2(const QString &s)
{
    ui->textBrowser_2->append(s);
}

void MainWindow::tr_Take_Contour(vector<vector<Point> > v, double a, double b)
{
    emit Take_Contour(v, a, b);
}

void MainWindow::GetCalibrationPoints(vector<Point2f> ov)
{
    if ( !pointsFile->open(QIODevice::WriteOnly | QIODevice::Text) ) {
        qDebug() << "Error: open file";
    } else {
        QTextStream os(pointsFile);
        for ( auto i:ov )
            os << "x" <<  i.x << "\n"  << "y" <<  i.y << "\n";
        os << "sx" << ui->lineEdit_3->text() << "\nsy" << ui->lineEdit_4->text() << "\nsp" << ui->lineEdit_5->text();
        pointsFile->close();
    }
}

void MainWindow::on_pushButton_24_clicked()
{
    if ( !pointsFile->open(QIODevice::ReadOnly | QIODevice::Text) ) {
        qDebug() << "Error: open file";
    } else {
        vector<Point2f> ov;
        QTextStream is(pointsFile);
        QString l;
        float x, y;
        while ( !is.atEnd() ) {
            if ( ov.size() < 4 ) {
                l = is.readLine();
                if ( l[0] == 'x' )
                    x = l.remove(0, 1).toFloat();
                l = is.readLine();
                if ( l[0] == 'y' )
                    y = l.remove(0, 1).toFloat();
                ov.push_back(Point2f(x, y) );
            } else {
                l = is.readLine();
                if ( l[0] == 's' ) {
                    if ( l[1] == 'x' ) {
                        x = l.remove(0, 2).toInt();
                        ui->lineEdit_3->setText(QString::number(x) );
                    }
                    if ( l[1] == 'y' ) {
                        y = l.remove(0, 2).toInt();
                        ui->lineEdit_4->setText(QString::number(y) );
                    }
                    if ( l[1] == 'p' ) {
                        int s = 0;
                        s = l.remove(0, 2).toInt();
                        ui->lineEdit_5->setText(QString::number(s) );
                    }
                }
            }
        }
        emit SetCalibrationPoints(ov);
        pointsFile->close();
    }
}   // MainWindow::on_pushButton_24_clicked

void MainWindow::on_pushButton_4_clicked()
{
    laserMachine_ = new Laser_Machine;

    connect(laserMachine_, &Laser_Machine::talking, [ = ](QString s){
        ui->textBrowser_2->append(s);
    });
    connect(this, SIGNAL(Take_Contour(vector<vector<cv::Point> >,double,double)),
      laserMachine_, SLOT(prepareContour(vector<vector<cv::Point> >,double,double)));

    if ( !_socketState ) {
        _socket = new My_TCPSocket();
        QObject::connect(_socket, &My_TCPSocket::talking, [ = ](QString s){
            ui->textBrowser_2->append(s);
        });

        _socket->Connect(ui->lineEdit->text().toUShort() );
        _socketState = _socket->GetState();
        connect(laserMachine_, SIGNAL(sendCommand(QString)), _socket, SLOT(sendCommand(QString)));
        connect(laserMachine_, SIGNAL(setSendingFlag(bool)), _socket, SLOT(setSendingFlag(bool)));
        connect(_socket, SIGNAL(nextPacket()), laserMachine_, SLOT(nextPacket()));
        connect(_socket, SIGNAL(incommingConnectSignal()), laserMachine_, SLOT(initMoves()));

        ui->pushButton_4->setText("close socket");
    } else {
        delete _socket;
        _socketState = false;
        ui->pushButton_4->setText("open socket");
    }
}

void MainWindow::on_pushButton_2_clicked()
{
    emit Quit();
}

void MainWindow::imgShow(const Mat &frame, int i)
{
    Mat tmp;

    frame.copyTo(tmp);
    cvtColor(tmp, tmp, CV_BGR2RGB);
    dst = new QImage(static_cast<uchar *>(tmp.data), tmp.cols, tmp.rows, static_cast<int>(tmp.step), QImage::Format_RGB888);
    dst->bits();

    if ( ui->radioButton->isChecked() && (i == 1) ) {
        itm->setPixmap(QPixmap::fromImage(*dst) );
    } else if ( ui->radioButton_2->isChecked() && (i == 2) ) {
        itm->setPixmap(QPixmap::fromImage(*dst) );
    } else if ( ui->radioButton_3->isChecked() && (i == 3) ) {
        itm->setPixmap(QPixmap::fromImage(*dst) );
    } else if ( ui->radioButton_4->isChecked() && (i == 4) ) {
        itm->setPixmap(QPixmap::fromImage(*dst) );
    }
    ui->graphicsView->fitInView(itm, Qt::AspectRatioMode::IgnoreAspectRatio);
    ui->graphicsView->update();

    delete  dst;
}   // MainWindow::imgShow

void MainWindow::on_pushButton_3_clicked()
{
    emit NextFrame();
}

void MainWindow::on_verticalSlider_sliderReleased()
{ }

void MainWindow::on_verticalSlider_valueChanged(int value)
{
    emit SetCoeff(value);

    ui->label_3->setText(QString::number(value) );
}

void MainWindow::on_verticalSlider_2_valueChanged(int value)
{
    emit SetCoeff(-1, value);

    ui->label_4->setText(QString::number(value) );
}

void MainWindow::on_verticalSlider_5_valueChanged(int value)
{
    emit SetThVal(value);
}

void MainWindow::on_verticalSlider_3_valueChanged(int value)
{
    emit SetCoeff(-1, -1, value);

    ui->label_5->setText(QString::number(value) );
}

void MainWindow::on_verticalSlider_4_valueChanged(int value)
{
    emit SetScale(value);

    ui->label_9->setText(QString::number(value) );
}

void MainWindow::on_pushButton_clicked()
{
    if ( _socketState ) {
        laserMachine_->setBurnyState(ui->checkBox->checkState() );
        emit SetSendFlag();
    }
}

void MainWindow::on_pushButton_5_clicked()
{
    if ( _socketState ) {
        laserMachine_->setShift(ui->lineEdit_3->text().toInt(), ui->lineEdit_4->text().toInt() );
    }
}

void MainWindow::on_lineEdit_2_editingFinished()
{
    if ( _socketState ) {
        laserMachine_->setSizePacket(ui->lineEdit_2->text().toInt() );
    }
}

void MainWindow::on_pushButton_6_clicked()
{
    if ( _socketState ) {
        laserMachine_->command_X42();
        laserMachine_->resetState();
    }
}

void MainWindow::on_pushButton_7_clicked()
{
    if ( _socketState ) {
        laserMachine_->command_X00();
    }
}

void MainWindow::on_pushButton_8_clicked()
{
    if ( _socketState ) {
        laserMachine_->setSpeed(ui->lineEdit_5->text().toInt() );
        laserMachine_->command_X02();
    }
}

void MainWindow::on_pushButton_9_clicked()
{
    if ( _socketState ) {
        laserMachine_->command_X04(true);
    }
}

void MainWindow::on_pushButton_10_clicked()
{
    if ( _socketState ) {
        laserMachine_->setPowerLaser(ui->lineEdit_6->text().toUInt() );
    }
}

void MainWindow::on_pushButton_15_clicked()
{
    double x, y;

    x	= ui->lineEdit_7->text().toDouble();
    y	= ui->lineEdit_8->text().toDouble();
    if ( _socketState ) {
        laserMachine_->command_X01(x, y, 0, 0);
        laserMachine_->command_X00();
    }
}

void MainWindow::on_pushButton_16_clicked()
{
    if ( _socketState ) {
        laserMachine_->command_X03();
    }
}

void MainWindow::on_pushButton_17_clicked() // calibrate
{
    emit setCalibrateFlag();
}

void MainWindow::on_pushButton_18_clicked()
{
    if ( _socketState )
        laserMachine_->setShift(ui->lineEdit_9->text().toDouble() );
}

void MainWindow::on_pushButton_19_clicked()
{
    uint p;

    if ( ui->radioButton_5->isChecked() )
        p = 1;
    if ( ui->radioButton_6->isChecked() )
        p = 4;
    if ( ui->radioButton_8->isChecked() )
        p = 2;
    if ( ui->radioButton_7->isChecked() )
        p = 3;

    emit SetCalibrationPoints(p, true, false);
}

void MainWindow::on_pushButton_20_clicked()
{
    uint p;

    if ( ui->radioButton_5->isChecked() )
        p = 1;
    if ( ui->radioButton_6->isChecked() )
        p = 4;
    if ( ui->radioButton_8->isChecked() )
        p = 2;
    if ( ui->radioButton_7->isChecked() )
        p = 3;

    emit SetCalibrationPoints(p, true, true);
}

void MainWindow::on_pushButton_21_clicked()
{
    uint p;

    if ( ui->radioButton_5->isChecked() )
        p = 1;
    if ( ui->radioButton_6->isChecked() )
        p = 4;
    if ( ui->radioButton_8->isChecked() )
        p = 2;
    if ( ui->radioButton_7->isChecked() )
        p = 3;

    emit SetCalibrationPoints(p, false, false);
}

void MainWindow::on_pushButton_22_clicked()
{
    uint p;

    if ( ui->radioButton_5->isChecked() )
        p = 1;
    if ( ui->radioButton_6->isChecked() )
        p = 4;
    if ( ui->radioButton_8->isChecked() )
        p = 2;
    if ( ui->radioButton_7->isChecked() )
        p = 3;

    emit SetCalibrationPoints(p, false, true);
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
//
//
//
