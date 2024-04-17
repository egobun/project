#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->plot->addGraph();
    ui->plot->graph(0)->setScatterStyle(QCPScatterStyle::ssCircle);
    ui->plot->graph(0)->setLineStyle(QCPGraph::lsNone);
}

MainWindow::~MainWindow()
{
    delete ui;
}



void MainWindow::clearData()
{

}

void MainWindow::plot()
{
    ui->plot->graph(0)->setData(qv_x,qv_y);
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

}

