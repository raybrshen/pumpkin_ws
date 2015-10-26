#include "../include/pumpkin_qt/pumpkinqt.hpp"
#include <QLineEdit>
#include <QProgressBar>
#include <QMessageBox>
#include <QLabel>
#include <QListWidget>
#include <QListWidgetItem>

namespace pumpkin_qt {

using namespace Qt;

PumpkinQT::PumpkinQT(int argc, char *argv[], QWidget *parent) :
	QMainWindow(parent), _node(argc, argv, this), _playback(this), _record(this),
	_config_dialog(nullptr), _folder_model(nullptr), _move_dialog(nullptr)
{
	_ui.setupUi(this);

    //Link signals
	QObject::connect(&_node, SIGNAL(rosShutdown()), this, SLOT(close()));
	QObject::connect(&_node, SIGNAL(filesReady(QString,std::vector<pumpkin_messages::FileList>)),
					 this, SLOT(fillTable(QString,std::vector<pumpkin_messages::FileList>)));
	QObject::connect(&_node, SIGNAL(started()), &_playback, SLOT(init()));
	QObject::connect(&_node, SIGNAL(started()), &_record, SLOT(init()));

	QObject::connect(_ui.actionSSC, SIGNAL(triggered()), this, SLOT(showSSCMoveDialog()));
	QObject::connect(_ui.actionAbout, SIGNAL(triggered()), this, SLOT(showAboutDialog()));

	QObject::connect(_ui.folderTree, SIGNAL(clicked(QModelIndex)), this, SLOT(folderSelected(QModelIndex)));
	QObject::connect(_ui.fileView, SIGNAL(clicked(QModelIndex)), this, SLOT(fileSelected(QModelIndex)));

	//Link the files signal for posterior playback, record and scene execution
	//QObject::connect(this, SIGNAL(changePlaybackFilename(QString)), _ui.playbackFileName, SLOT(setText(QString)));
	QObject::connect(this, SIGNAL(changePlaybackFilename(QString)), &_playback, SLOT(setPlaybackFilename(QString)));
	QObject::connect(this, SIGNAL(changeRecordFilename(QString)), &_record, SLOT(setRecordFilename(QString)));
	QObject::connect(this, SIGNAL(setSceneFilenames(std::vector<std::string>)), &_playback, SLOT(setSceneFilenames(std::vector<std::string>)));

	QObject::connect(_ui.recordFileName, SIGNAL(textEdited(QString)), this, SLOT(changeFilename(QString)));
	QObject::connect(_ui.minuteSpin, SIGNAL(valueChanged(int)), &_record, SLOT(setRecordTimeMinutes(int)));
	QObject::connect(_ui.secondSpin, SIGNAL(valueChanged(int)), &_record, SLOT(setRecordTimeSeconds(int)));

	//Link buttons signals
	QObject::connect(_ui.playButton, SIGNAL(clicked()), this, SLOT(runPlayback()));
	QObject::connect(_ui.stopPlayButton, SIGNAL(clicked()), &_playback, SLOT(playbackStop()));
	QObject::connect(_ui.recButton, SIGNAL(clicked()), this, SLOT(runRecord()));
	QObject::connect(_ui.stopRecButton, SIGNAL(clicked()), &_record, SLOT(recordStop()));
	QObject::connect(_ui.playSceneButton, SIGNAL(clicked()), this, SLOT(runScene()));
	QObject::connect(_ui.stopSceneButton, SIGNAL(clicked()), &_playback, SLOT(stopScene()));

	QObject::connect(_ui.addSceneMoveButton, SIGNAL(clicked()), this, SLOT(addSceneFile()));
	QObject::connect(_ui.removeSceneMoveButton, SIGNAL(clicked()), this, SLOT(removeSceneFile()));

	//Link feedbacks signals
	QObject::connect(&_playback, SIGNAL(playbackPercentage(int)), _ui.playbackProgress, SLOT(setValue(int)));
	QObject::connect(&_playback, SIGNAL(scenePercentage(int,int,int)), this, SLOT(updateSceneFeedback(int,int,int)));
	QObject::connect(&_record, SIGNAL(recordMinuteFeedback(int)), _ui.minuteSpin, SLOT(setValue(int)));
	QObject::connect(&_record, SIGNAL(recordSecondFeedback(int)), _ui.secondSpin, SLOT(setValue(int)));

	//Link signals for blocking on playback
	QObject::connect(&_playback, SIGNAL(blockOnPlayback(bool)), _ui.recordTab, SLOT(setDisabled(bool)));
	QObject::connect(&_playback, SIGNAL(blockOnPlayback(bool)), _ui.sceneTab, SLOT(setDisabled(bool)));
	QObject::connect(&_playback, SIGNAL(blockOnPlayback(bool)), _ui.playButton, SLOT(setDisabled(bool)));
	QObject::connect(&_playback, SIGNAL(blockOnPlayback(bool)), _ui.stopPlayButton, SLOT(setEnabled(bool)));
	//Link signals for blocking on record
	QObject::connect(&_record, SIGNAL(blockOnRecord(bool)), _ui.playbackTab, SLOT(setDisabled(bool)));
	QObject::connect(&_record, SIGNAL(blockOnRecord(bool)), _ui.sceneTab, SLOT(setDisabled(bool)));
	QObject::connect(&_record, SIGNAL(blockOnRecord(bool)), _ui.recButton, SLOT(setDisabled(bool)));
	QObject::connect(&_record, SIGNAL(blockOnRecord(bool)), _ui.minuteSpin, SLOT(setDisabled(bool)));
	QObject::connect(&_record, SIGNAL(blockOnRecord(bool)), _ui.secondSpin, SLOT(setDisabled(bool)));
	QObject::connect(&_record, SIGNAL(blockOnRecord(bool)), _ui.stopRecButton, SLOT(setEnabled(bool)));
	//Linkg signals for blocking on scene
	QObject::connect(&_playback, SIGNAL(blockOnScene(bool)), _ui.recordTab, SLOT(setDisabled(bool)));
	QObject::connect(&_playback, SIGNAL(blockOnScene(bool)), _ui.playbackTab, SLOT(setDisabled(bool)));
	QObject::connect(&_playback, SIGNAL(blockOnScene(bool)), _ui.playSceneButton, SLOT(setDisabled(bool)));
	QObject::connect(&_playback, SIGNAL(blockOnScene(bool)), _ui.addSceneMoveButton, SLOT(setDisabled(bool)));
	QObject::connect(&_playback, SIGNAL(blockOnScene(bool)), _ui.removeSceneMoveButton, SLOT(setDisabled(bool)));
	QObject::connect(&_playback, SIGNAL(blockOnScene(bool)), _ui.stopSceneButton, SLOT(setEnabled(bool)));

	//Link for respond on finishing playback or record (for now, it only reloads the file tree
	QObject::connect(&_playback, SIGNAL(playbackFinished(int)), this, SLOT(actionFinished(int)));
	QObject::connect(&_record, SIGNAL(recordFinished(int)), this, SLOT(actionFinished(int)));

	//Link status message signals
	QObject::connect(&_node, SIGNAL(sendStatusMessage(QString,int)), _ui.statusbar, SLOT(showMessage(QString,int)));
	QObject::connect(&_playback, SIGNAL(sendStatusMessage(QString,int)), _ui.statusbar, SLOT(showMessage(QString,int)));
	QObject::connect(&_record, SIGNAL(sendStatusMessage(QString,int)), _ui.statusbar, SLOT(showMessage(QString,int)));

	_node.init();
	_node.callFiles();

	if (!ros::param::has("/pumpkin/config")) {
		_config_dialog = new LoadConfig(this);
		QObject::connect(&_node, SIGNAL(rosShutdown()), _config_dialog, SLOT(close()));
		QObject::connect(&_node, SIGNAL(configFilesReady(QString,std::vector<std::string>)), _config_dialog, SLOT(load(QString,std::vector<std::string>)));
		_node.callConfigFiles();
		int result = _config_dialog->exec();
		if (result == QDialog::Rejected) {
			QApplication::closeAllWindows();
			_node.terminate();
		}
	}

	_files_dialog = new FilesDialog(this);

	QObject::connect(this, SIGNAL(selectFolder(QString)), _files_dialog, SLOT(setSelectedFolder(QString)));
	QObject::connect(this, SIGNAL(selectFile(QString)), _files_dialog, SLOT(setSelectedFile(QString)));
	QObject::connect(_ui.actionFiles, SIGNAL(triggered()), _files_dialog, SLOT(show()));

	//Theses signals (results from the FilesDialog) below can be handled
	QObject::connect(_files_dialog, SIGNAL(accepted()), &_node, SLOT(callFiles()));
	QObject::connect(_files_dialog, SIGNAL(rejected()), &_node, SLOT(callFiles()));
}

PumpkinQT::~PumpkinQT()
{
    qDeleteAll(_model_list);
	delete _folder_model;
	delete _files_dialog;
	if (_config_dialog)
		delete _config_dialog;
	if (_move_dialog)
		delete _move_dialog;
}

void PumpkinQT::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);
    if (!_filename.isEmpty()) {
		QFontMetrics metric = _ui.playbackFileName->fontMetrics();
		if (metric.width(_filename) > _ui.playbackFileName->width()) {
			_ui.playbackFileName->setText(metric.elidedText(_filename, Qt::ElideLeft, _ui.playbackFileName->width()));
		} /*else {
			_ui.playbackFileName->setText(_filename);
		}*/
    }
}

void PumpkinQT::fillTable(const QString &base_path, const std::vector<pumpkin_messages::FileList> &file_list)
{
	_filename = QString();
	while(_ui.sceneList->count() > 0) {
		_ui.sceneList->removeItemWidget(_ui.sceneList->item(0));
	}
    if (_folder_model)
        delete _folder_model;
    _base_path = base_path;
    _model_list.clear();
	_model_list.reserve(file_list.size());
    FolderListType folders;
    for (int i = 0; i < file_list.size(); ++i) {
		const pumpkin_messages::FileList &files = file_list[i];
		if (files.filenames.size() > 0) {
			QStringList list;
			for (std::vector<std::string>::const_iterator it = files.filenames.begin(); it != files.filenames.end(); ++it) {
				list << QString::fromStdString(*it);
			}
			//ROS_INFO("New path");
			//First of all, add all files to respective list model
			_model_list.append(new QStringListModel(list, this));
		} else {
			//ROS_INFO("New void path");
			_model_list.append(new QStringListModel(this));
		}
		folders.append(QPair<int, QString>(files.parent_folder, QString::fromStdString(files.folder)));
    }
	//ROS_INFO("Loaded structure.");
	_folder_model = new FolderModel(folders, this);
	_ui.folderTree->setModel(_folder_model);
	_ui.folderTree->setColumnHidden(1, true);
	Q_EMIT(selectFolder(QString()));
}

void PumpkinQT::folderSelected(const QModelIndex &index) {
	_filename = QString();
    const QModelIndex & it =_folder_model->index(index.row(), 1, index.parent());
	int data = it.data().toInt();
	//ROS_INFO("Selected folder %d.", data);
	_ui.fileView->setModel(_model_list.at(data));
    QStringList rel;
    rel << index.data().toString();
    QModelIndex parent = index.parent();
    while (parent.isValid()) {
        rel.push_front(parent.data().toString());
        parent = parent.parent();
    }
    _relative_path = rel.join("/");
	_ui.playbackFileName->setText("Playback file");
	Q_EMIT(selectFolder(_base_path + "/" + _relative_path));
	//Q_EMIT(selectFile(QString()));
}

void PumpkinQT::fileSelected(const QModelIndex &index) {
    QString filename = index.data().toString();
	if (!filename.contains(".yaml"))
		filename = filename + ".yaml";
	_filename = _base_path + "/" + _relative_path + "/" + filename;
	if (_ui.mainBlock->currentWidget()->objectName().contains("playback")) {
		QFontMetrics metric = _ui.playbackFileName->fontMetrics();
		if (metric.width(_filename) > _ui.playbackFileName->width()) {
			_ui.playbackFileName->setText(metric.elidedText(_filename, Qt::ElideLeft, _ui.playbackFileName->width()));
		} else {
			_ui.playbackFileName->setText(_filename);
		}
	} else {
		_ui.recordFileName->setText(filename.remove(filename.size()-5, 5));
	}
	Q_EMIT(selectFile(filename));
}

void PumpkinQT::runPlayback()
{
	Q_EMIT(changePlaybackFilename(_filename));
	_playback.playbackFile();
}

void PumpkinQT::runRecord()
{
	Q_EMIT(changeRecordFilename(_filename));
	_record.recordFile();
}

void PumpkinQT::runScene()
{
	if (_ui.sceneList->count() == 0)
		return;
	std::vector<std::string> files;
	for (int i = 0; i < _ui.sceneList->count(); ++i) {
		files.push_back(_ui.sceneList->item(i)->text().toStdString());
	}
	Q_EMIT(setSceneFilenames(files));
	_playback.playScene();
}

void PumpkinQT::addSceneFile()
{
	if (_filename.isEmpty())
		return;
	if (_ui.sceneList->findItems(_filename, Qt::MatchContains).size() > 0)
		return;
	_ui.sceneList->addItem(_filename);
}

void PumpkinQT::removeSceneFile()
{
	auto list = _ui.sceneList->selectedItems();
	if (list.size() == 0) {
		ROS_WARN("No selected file on the list");
		return;
	}
	_ui.sceneList->removeItemWidget(list[0]);
}

void PumpkinQT::changeFilename(const QString &filename)
{
	QString file = filename + (bool(filename.contains(".yaml")) ? "" : ".yaml");
	_filename = _base_path + "/" + _relative_path + "/" + file;
}

void PumpkinQT::actionFinished(int state)
{
	_node.callFiles();
}

void PumpkinQT::showSSCMoveDialog()
{
	if (!_move_dialog) {
		_move_dialog = new SSCMoveCommand(this);
		QObject::connect(&_node, SIGNAL(rosShutdown()), _move_dialog, SLOT(close()));
	}
	if (_playback.isRunning())
		_playback.playbackStop();
	if (_record.isRunning())
		_record.recordStop();
	_move_dialog->exec();
}

void PumpkinQT::showAboutDialog()
{
	QMessageBox *box = new QMessageBox(this);
	box->setText("Dr. Robot Pumpkin. University of Ottawa");
	box->exec();
	delete box;
}

void PumpkinQT::updateSceneFeedback(int step, int total, int percentage)
{
	_ui.sceneProgress->setValue(percentage);
	int move = step >> 1;
	if (step & 1) {
		//If step is even, it means it is between tho movements
		_ui.sceneLabel->setText(QString("Playing planned from %0 to %1.").arg(move + 1).arg(move + 2));
	} else {
		_ui.sceneLabel->setText(QString("Playing scene movement: %0 of %1.").arg(move+1).arg(total));
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
	//ROS_INFO("Started filling folder model with %d elements.", folders.count());
    for (FolderListType::const_iterator it = folders.begin(); it != folders.end(); ++it, ++i) {
		int parent_index = it->first - 1;
		//ROS_INFO("Folder %d - %s, parent: %d", i, it->second.toStdString().c_str(), parent_index);
        while (!stack.isEmpty()) {
            if (stack.back()->data(1).toInt() == parent_index) {
				//ROS_INFO("Parent %d", stack.back()->data(1).toInt());
                break;
            }
            stack.removeLast();
        }
		if (stack.isEmpty()) {
			ROS_FATAL("Error in file structure");
            throw QString("Error file structure.");
		}
		//ROS_INFO("Found element");
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
