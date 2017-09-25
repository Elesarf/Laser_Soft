#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDebug>
#include <QThread>
#include <QTimer>
#include <QTime>

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

void MainWindow::on_verticalSlider_3_valueChanged(int value)
{
    emit SetCoeff(-1, -1, value);

    ui->label_5->setText(QString::number(value) );
}

void MainWindow::on_pushButton_clicked()
{
    if ( _socketState ) {
        laserMachine_->setBurnyState(ui->checkBox->checkState() );
    }

    emit SetSendFlag();
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
