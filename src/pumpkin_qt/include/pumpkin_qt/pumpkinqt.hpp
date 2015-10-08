#ifndef PUMPKINQT_H
#define PUMPKINQT_H

#include "ui_pumpkinqt.h"
#include "qnode.hpp"

#include <QMainWindow>
#include <QStringListModel>
#include <vector>
#include "pumpkin_messages/Files.h"
#include "pumpkin_messages/PlaybackAction.h"
#include "pumpkin_messages/RecordAction.h"

namespace pumpkin_qt {

class FolderItem {
private:
    FolderItem *_parent;
    QList<FolderItem *> _children;
    int id;
    QString folder;
public:
    explicit FolderItem(int index, const QString &name, FolderItem *parent);
    ~FolderItem();


};


class FolderViewModel : public QAbstractItemModel {
private:

};


class PumpkinQT : public QMainWindow
{
    Q_OBJECT
public:
    explicit PumpkinQT(int argc, char *argv[], QWidget *parent = 0);
    ~PumpkinQT();

public Q_SLOTS:
    void fillTable(QString base_path, std::vector<pumpkin_messages::FileList> file_list);

private:
    Ui::PumpkinQTDesign ui;
    QNode node;
    QString _base_path;
    std::vector<QStringListModel> _model_list;
};

}
#endif // PUMPKINQT_H
