#include "../include/pumpkin_qt/pumpkinqt.hpp"
#include <QTreeView>
#include <QListView>
#include <QTabWidget>
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>
#include <QProgressBar>
#include <QSpinBox>
#include <vector>

namespace pumpkin_qt {

using namespace Qt;

PumpkinQT::PumpkinQT(int argc, char *argv[], QWidget *parent) :
    QMainWindow(parent), node(argc, argv, &_filename)
{
    ui.setupUi(this);
    _folder_model = nullptr;


    //Link signals
    QObject::connect(&node, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&node, SIGNAL(filesReady(QString,std::vector<pumpkin_messages::FileList>)),
                     this, SLOT(fillTable(QString,std::vector<pumpkin_messages::FileList>)));
    QObject::connect(ui.folderTree, SIGNAL(clicked(QModelIndex)), this, SLOT(folderSelected(QModelIndex)));
    QObject::connect(ui.fileView, SIGNAL(clicked(QModelIndex)), this, SLOT(fileSelected(QModelIndex)));
    QObject::connect(this, SIGNAL(changePlaybackFileName(QString)), ui.playbackFileName, SLOT(setText(QString)));
    QObject::connect(this, SIGNAL(changeRecordFileName(QString)), ui.recordFileName, SLOT(setText(QString)));

    QObject::connect(ui.playButton, SIGNAL(clicked()), &node, SLOT(playbackFile()));
    QObject::connect(ui.stopPlayButton, SIGNAL(clicked()), &node, SLOT(playbackStop()));

    QObject::connect(&node, SIGNAL(playbackPercentage(int)), ui.playbackProgress, SLOT(setValue(int)));
    QObject::connect(&node, SIGNAL(recordMinuteFeedback(int)), ui.minuteSpin, SLOT(setValue(int)));
    QObject::connect(&node, SIGNAL(recordSecondFeedback(int)), ui.secondSpin, SLOT(setValue(int)));
    QObject::connect(&node, SIGNAL(lockTab(bool)), ui.mainBlock, SLOT(setDisabled(bool)));

    node.init();

    node.callFiles();
}

PumpkinQT::~PumpkinQT()
{
    qDeleteAll(_model_list);
    delete _folder_model;
}

void PumpkinQT::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);
    if (!_filename.isEmpty()) {
        QFontMetrics metric = ui.playbackFileName->fontMetrics();
        if (metric.width(_filename) > ui.playbackFileName->width()) {
            ui.playbackFileName->setText(metric.elidedText(_filename, Qt::ElideLeft, ui.playbackFileName->width()));
        } else {
            ui.playbackFileName->setText(_filename);
        }
    }
}

void PumpkinQT::fillTable(const QString &base_path, const std::vector<pumpkin_messages::FileList> &file_list)
{
    if (_folder_model)
        delete _folder_model;
    _base_path = base_path;
    _model_list.clear();
    _model_list.reserve(file_list.size());
    FolderListType folders;
    for (int i = 0; i < file_list.size(); ++i) {
        const pumpkin_messages::FileList &files = file_list[i];
        QStringList list;
        for (std::vector<std::string>::const_iterator it = files.filenames.begin(); it != files.filenames.end(); ++it) {
            list << QString::fromStdString(*it);
        }
        ROS_INFO("New path");
        //First of all, add all files to respective list model
        _model_list.append(new QStringListModel(list, this));
        folders.append(QPair<int, QString>(files.parent_folder, QString(files.folder.c_str())));
    }
    ROS_INFO("Loaded structure.");
    _folder_model = new FolderModel(folders, this);
    ui.folderTree->setModel(_folder_model);
    ui.folderTree->setColumnHidden(1, true);
}

void PumpkinQT::folderSelected(const QModelIndex &index) {
    const QModelIndex & it =_folder_model->index(index.row(), 1, index.parent());
    int data = it.data().toInt();
    ROS_INFO("Selected folder %d.", data);
    ui.fileView->setModel(_model_list.at(data));
    QStringList rel;
    rel << index.data().toString();
    QModelIndex parent = index.parent();
    while (parent.isValid()) {
        rel.push_front(parent.data().toString());
        parent = parent.parent();
    }
    _relative_path = rel.join("/");
}

void PumpkinQT::fileSelected(const QModelIndex &index) {
    QString filename = index.data().toString();
    _filename = _base_path + "/" + _relative_path + "/" + filename;
    if (ui.mainBlock->currentWidget()->objectName().contains("playback")) {
        QFontMetrics metric = ui.playbackFileName->fontMetrics();
        if (metric.width(_filename) > ui.playbackFileName->width()) {
            Q_EMIT(changePlaybackFileName(metric.elidedText(_filename, Qt::ElideLeft, ui.playbackFileName->width())));
        } else {
            Q_EMIT(changePlaybackFileName(_filename));
        }
    } else {
        Q_EMIT(changeRecordFileName(filename.remove(filename.size()-5, 5)));
    }
}


/********************************
 * IMPLEMENTATION OF FOLDERITEM *
 ********************************/

FolderItem::FolderItem(const QString &name, int index, FolderItem *parent) {
    _id = index;
    _folder = name;
    _parent = parent;
    if (_parent)
        _parent->appendChild(this);
}

FolderItem::~FolderItem() {
    qDeleteAll(_children);
}

QVariant FolderItem::data(int column) const {
    switch (column) {
    case 1:
        return QVariant(_id);
    case 0:
        return QVariant(_folder);
    default:
        return QVariant();
    }
}

int FolderItem::row() const {
    if (_parent)
        return _parent->_children.indexOf(const_cast<FolderItem *>(this));
    return 0;
}

/*********************************
 * IMPLEMENTATION OF FOLDERMODEL *
 *********************************/

FolderModel::FolderModel(const FolderListType &folders, QObject *parent) : QAbstractItemModel(parent) {
    _root = new FolderItem(QString("Folder"), -1);
    QList<FolderItem *> stack;
    stack.reserve(folders.size() + 1);
    stack.append(_root);
    FolderItem *parentItem = nullptr;
    int i = 0;
    ROS_INFO("Started filling folder model with %d elements.", folders.count());
    for (FolderListType::const_iterator it = folders.begin(); it != folders.end(); ++it, ++i) {
        int parent_index = it->first - 1;
        while (!stack.isEmpty()) {
            if (stack.back()->data(1).toInt() == parent_index) {
                ROS_INFO("Parent %d", stack.back()->data(1).toInt());
                break;
            }
            stack.removeLast();
        }
        if (stack.isEmpty())
            throw QString("Error file structure.");
        ROS_INFO("Found element");
        parentItem = stack.back();
        stack.append(new FolderItem(it->second, i, parentItem));
    }
}

FolderModel::~FolderModel() {
    delete _root;
}

QModelIndex FolderModel::index(int row, int column, const QModelIndex &parent) const {
    if (!hasIndex(row, column, parent))
        return QModelIndex();

    FolderItem *parentItem;

    if (!parent.isValid())
        parentItem = _root;
    else
        parentItem = static_cast<FolderItem *>(parent.internalPointer());

    FolderItem *childItem = parentItem->child(row);
    if (childItem)
        return createIndex(row, column, childItem);
    else
        return QModelIndex();
}

QModelIndex FolderModel::parent(const QModelIndex &index) const {
    if (!index.isValid())
        return QModelIndex();

    FolderItem *childItem = static_cast<FolderItem *>(index.internalPointer());
    FolderItem *parentItem = childItem->parent();

    if (parentItem == _root)
        return QModelIndex();

    return createIndex(parentItem->row(), 0, parentItem);
}

int FolderModel::rowCount(const QModelIndex &parent) const {
    FolderItem *parentItem;
    if (parent.column() > 0)
        return 0;

    if (!parent.isValid())
        parentItem = _root;
    else
        parentItem = static_cast<FolderItem *>(parent.internalPointer());

    return parentItem->childCount();
}

int FolderModel::columnCount(const QModelIndex &parent) const {
    if (!parent.isValid())
        return _root->columnCount();
    else
        return static_cast<FolderItem *>(parent.internalPointer())->columnCount();
}

QVariant FolderModel::data(const QModelIndex &index, int role) const {
    if (!index.isValid())
        return QVariant();

    if (role != Qt::ItemDataRole::DisplayRole)
        return QVariant();

    FolderItem *item = static_cast<FolderItem *>(index.internalPointer());

    return item->data(index.column());
}

Qt::ItemFlags FolderModel::flags(const QModelIndex &index) const {
    if (!index.isValid())
        return 0;

    return Qt::ItemIsEnabled | Qt::ItemIsEditable;
}

QVariant FolderModel::headerData(int section, Orientation orientation, int role) const {
    if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
        return _root->data(section);

    return QVariant();
}

}
