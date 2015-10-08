#ifndef PUMPKINQT_H
#define PUMPKINQT_H

#include <QMainWindow>

namespace Ui {
class PumpkinQT;
}

class PumpkinQT : public QMainWindow
{
    Q_OBJECT

public:
    explicit PumpkinQT(QWidget *parent = 0);
    ~PumpkinQT();

private:
    Ui::PumpkinQT *ui;
};

#endif // PUMPKINQT_H
