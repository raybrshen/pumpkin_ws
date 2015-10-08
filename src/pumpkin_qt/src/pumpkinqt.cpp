#include "../include/pumpkin_qt/pumpkinqt.hpp"
#include <QTreeWidget>

namespace pumpkin_qt {

using namespace Qt;

PumpkinQT::PumpkinQT(int argc, char *argv[], QWidget *parent) :
    QMainWindow(parent), node(argc, argv)
{
    ui.setupUi(this);
}

PumpkinQT::~PumpkinQT()
{}

void PumpkinQT::fillTable(QString base_path, std::vector<pumpkin_messages::FileList> file_list)
{
    _base_path = base_path;
    _model_list.clear();
    _model_list.reserve(file_list.size());
    for (int i = 0; i < file_list.size(); ++i) {
        pumpkin_messages::FileList &files = file_list[i];
        QStringList list;
        for (auto it = files.filenames.begin; it != files.filenames.end(); ++it) {
            list << QString(it->c_str());
        }
        //First of all, add all files to respective list model
        _model_list.push_bach(QStringListModel(list, this));
    }

}

}
