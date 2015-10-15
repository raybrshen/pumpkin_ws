#include "../include/pumpkin_qt/filesdialog.hpp"
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>

namespace pumpkin_qt {

using namespace Qt;

namespace pmsg = pumpkin_messages;

FilesDialog::FilesDialog(QWidget *parent) :
	QDialog(parent), _box(new QMessageBox(this)), _success(false)
{
	_ui.setupUi(this);
	ros::NodeHandle nh;

	_client = nh.serviceClient<pmsg::FilesHandler>("file_handler");
	_client.waitForExistence();
	QObject::connect(_ui.createFolderButton, SIGNAL(clicked()), this, SLOT(createFolder()));
	QObject::connect(_ui.deleteFileButton, SIGNAL(clicked()), this, SLOT(deleteFile()));
	QObject::connect(_ui.deleteFolderButton, SIGNAL(clicked()), this, SLOT(deleteFolder()));
	QObject::connect(_ui.okButton, SIGNAL(clicked()), this, SLOT(accept()));

	_ui.createFolderButton->setEnabled(false);
	_ui.deleteFileButton->setEnabled(false);
	_ui.deleteFolderButton->setEnabled(false);
}

FilesDialog::~FilesDialog()
{
	delete _box;
}

void FilesDialog::resizeEvent(QResizeEvent *event)
{
	QDialog::resizeEvent(event);

	if (!_path.isEmpty()) {
		QFontMetrics metrics = _ui.currentFolderLabel->fontMetrics();
		if (metrics.width(_path) > _ui.currentFolderLabel->width()) {
			_ui.currentFolderLabel->setText(metrics.elidedText(_path, Qt::ElideLeft, _ui.currentFolderLabel->width()));
		} else {
			_ui.currentFolderLabel->setText(_path);
		}
	}

	if (!_file.isEmpty()) {
		QFontMetrics metrics = _ui.currentFileLabel->fontMetrics();
		if (metrics.width(_file) > _ui.currentFileLabel->width()) {
			_ui.currentFileLabel->setText(metrics.elidedText(_file, Qt::ElideLeft, _ui.currentFileLabel->width()));
		} else {
			_ui.currentFileLabel->setText(_file);
		}
	}
}

void FilesDialog::deleteFile()
{
	_success = false;
	_msg.request.folder = _path.toStdString();
	_msg.request.filename = _file.toStdString();
	_msg.request.service = static_cast<uint8_t>(pmsg::FHST::DeleteFile);
	if (_client.call(_msg)) {
		switch (static_cast<pmsg::IOState>(_msg.response.result)) {
			case pmsg::IOState::OK:
				_box->setText("File deleted successfully");
				_success = true;
			break;
			default:
				_box->setText("An error ocurred.");
		}
		_box->exec();
	}
	if (_success)
		this->accept();
	else
		this->reject();
}

void FilesDialog::deleteFolder()
{
	_success = false;
	_msg.request.folder = _path.toStdString();
	//_msg.request.filename = _file.toStdString();
	_msg.request.service = static_cast<uint8_t>(pmsg::FHST::DeleteFolder);
	if (_client.call(_msg)) {
		switch (static_cast<pmsg::IOState>(_msg.response.result)) {
			case pmsg::IOState::OK:
				_box->setText("Folder deleted successfully");
				_success = true;
			break;
			default:
				_box->setText("An error ocurred.");
		}
		_box->exec();
	}
	if (_success)
		this->accept();
	else
		this->reject();
}

void FilesDialog::createFolder()
{
	_success = false;
	_msg.request.folder = _path.toStdString();
	_msg.request.filename = _ui.newFolderLine->text().toStdString();
	_msg.request.service = static_cast<uint8_t>(pmsg::FHST::CreateFolder);
	if (_client.call(_msg)) {
		switch (static_cast<pmsg::IOState>(_msg.response.result)) {
			case pmsg::IOState::OK:
				_box->setText("Folder created successfully");
				_success = true;
			break;
			default:
				_box->setText("An error ocurred.");
		}
		_box->exec();
	}
	if (_success)
		this->accept();
	else
		this->reject();
}

void FilesDialog::setSelectedFolder(const QString &path)
{
	//ROS_INFO("Selected folder: %s", path.toStdString().c_str());
	_path = path;
	_file = QString();
	if (path.isEmpty()) {
		/* If path is empty, it means that the folder tree was restarted...
		 * So, all the buttons should be disabled
		 * (this should be like an reseted state)
		 */
		_ui.createFolderButton->setEnabled(false);
		_ui.deleteFileButton->setEnabled(false);
		_ui.deleteFolderButton->setEnabled(false);

		_ui.currentFileLabel->setText("No file selected.");
		_ui.currentFolderLabel->setText("No folder selected.");
	} else {
		/* But, when the folder is selected, theorically, the file is still not selected...
		 * So, we cannot allow yet the user to delete a file
		 */
		_file = QString();
		_ui.deleteFolderButton->setEnabled(true);
		_ui.createFolderButton->setEnabled(true);
		_ui.deleteFileButton->setEnabled(false);

		_ui.currentFileLabel->setText("No file selected.");

		//And now... for the elipsis
		QFontMetrics metrics = _ui.currentFolderLabel->fontMetrics();
		if (metrics.width(_path) > _ui.currentFolderLabel->width()) {
			_ui.currentFolderLabel->setText(metrics.elidedText(_path, Qt::ElideLeft, _ui.currentFolderLabel->width()));
		} else {
			_ui.currentFolderLabel->setText(_path);
		}
	}
}

void FilesDialog::setSelectedFile(const QString &file)
{
	//ROS_INFO("Selected file: %s", file.toStdString().c_str());

	//I should do something when the path is empty, but, the way the signals are emmitted,
	//that sould not be an problem
	if (!_path.isEmpty()) {
		_file = file;
		if (_file.isEmpty()) {
			_ui.deleteFolderButton->setEnabled(true);
			_ui.createFolderButton->setEnabled(true);
			_ui.deleteFileButton->setEnabled(false);

			_ui.currentFileLabel->setText("No file selected.");

			//And now... for the elipsis
			QFontMetrics metrics = _ui.currentFolderLabel->fontMetrics();
			if (metrics.width(_file) > _ui.currentFolderLabel->width()) {
				_ui.currentFolderLabel->setText(metrics.elidedText(_file, Qt::ElideLeft, _ui.currentFolderLabel->width()));
			} else {
				_ui.currentFolderLabel->setText(_file);
			}
		} else {
			_ui.deleteFolderButton->setEnabled(true);
			_ui.createFolderButton->setEnabled(true);
			_ui.deleteFileButton->setEnabled(true);

			//And now... for the elipsis
			QFontMetrics metrics = _ui.currentFolderLabel->fontMetrics();
			if (metrics.width(_path) > _ui.currentFolderLabel->width()) {
				_ui.currentFolderLabel->setText(metrics.elidedText(_path, Qt::ElideLeft, _ui.currentFolderLabel->width()));
			} else {
				_ui.currentFolderLabel->setText(_path);
			}

			metrics = _ui.currentFileLabel->fontMetrics();
			if (metrics.width(_file) > _ui.currentFileLabel->width()) {
				_ui.currentFileLabel->setText(metrics.elidedText(_file, Qt::ElideLeft, _ui.currentFileLabel->width()));
			} else {
				_ui.currentFileLabel->setText(_file);
			}
		}
	}
}

}
