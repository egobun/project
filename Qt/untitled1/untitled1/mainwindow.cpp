#include "mainwindow.h"
#include "ui_mainwindow.h"
extern QByteArray dann;
QString arr = "";
QString help = "";

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->plot->addGraph();
    ui->plot->graph(0)->setScatterStyle(QCPScatterStyle::ssCircle);
    ui->plot->graph(0)->setLineStyle(QCPGraph::lsNone);
    ui->plot->xAxis->setRange(0, 10);
    ui->plot->xAxis->setLabel("Время, [с]");
    ui->plot->yAxis->setLabel("Высота, [м]");
    ui->plot->graph()->setName("Высота полета");
    if(arr != "")
        ui->plot->yAxis->setRange(0, arr.toFloat()+1);
    //customPlot->yAxis->setRange(0, 1);
    //ui->plot->graph(0)->set

    loadPorts();
    connect(&_port, &SerialPort::dataReceived,this,&MainWindow::readData);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::addPoint(double x, double y)
{
    qDebug()<<y;
    qv_x.append(x);
    qv_y.append(y);
}

void MainWindow::clearData()
{
    qv_x.clear();
    qv_y.clear();
}

void MainWindow::plot()
{   
    //qv_x.remove(1);
    //qv_y.remove(1);
    ui->plot->graph(0)->setData(qv_x,qv_y);
    //qDebug()<<qv_x;
    ui->plot->replot();
    ui->plot->update();
}

void MainWindow::on_btn_add_clicked()
{
    addPoint(ui->bx_x->value(),ui->bx_y->value());
    plot();
}


void MainWindow::on_btn_clear_clicked()
{
    clearData();
    plot();
}

void MainWindow::loadPorts()
{
    foreach (auto &port, QSerialPortInfo::availablePorts()) {
        ui->cmbPorts->addItem(port.portName());
    }
}

void MainWindow::on_btnOpenPorts_clicked()
{
    auto isConnected = _port.connect(ui->cmbPorts->currentText());
    if (!isConnected){
        QMessageBox::critical(this,"Error","There is a problem connecting");
    }
}

void MainWindow::on_btnSend_clicked()
{
    auto numBytes = _port.write(ui->lnMessage->text().toUtf8());
    if(numBytes == -1){
        QMessageBox::critical(this,"Error","Something went wrong");
    }
}


void MainWindow::readData(QByteArray data)
{
    ui->lstMessages->addItem(QString(data));
    //qDebug() << data;
    arr = QString(data);
    qsizetype index1 = arr.indexOf(";",3);
    qsizetype index2 = arr.indexOf(";",(int8_t)index1+6);
    qsizetype index3 = arr.indexOf(";",9);

    //setlocale(LC_ALL, "");

    //ui->lstMessages->addItem(arr);

    //arr.replace(QString(""), QString(""));
    //arr.indexOf(QRegExp("[\\n]"), 0);
    //ui->lstMessages->addItem(QString(arr.indexOf(QRegExp("0"), 0)));
    //qDebug() << arr;
    qsizetype len = index2-index3-1;
    QString y_data = arr.mid(index2-len, len);
    QString x_data = arr.mid(3, index1-3);
    //ui->lstMessages->addItem((x_data));
    //qDebug() << x_data;
    //qDebug() << y_data;
    //QString X_data = QString(&x_data);
    //x_data = QString(x_data);
    //y_data = QString(y_data);
    /*
    for(int i = 14; i<18;i++){
        y_data.append(arr[i]);
    }
    for(int i = 3; i<8; i++){
        x_data.append(arr[i]);
    }*/
    //ui->lstMessages->addItem(x_data);
    if(x_data != "" && y_data != ""){
        ui->plot->xAxis->setRange(0,x_data.toFloat()/1000 + 1);
        ui->plot->yAxis->setRange(0,y_data.toFloat() + 1);
        addPoint(x_data.toFloat()/1000,y_data.toFloat());
    }

    //qDebug() << x_data;
    //qDebug() << y_data;
    plot();

}
