#include "pumpkinqt.h"
#include "ui_pumpkinqt.h"

PumpkinQT::PumpkinQT(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PumpkinQT)
{
    ui->setupUi(this);
}

PumpkinQT::~PumpkinQT()
{
    delete ui;
}
