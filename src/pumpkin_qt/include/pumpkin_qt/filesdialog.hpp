#ifndef FILESDIALOG_HPP
#define FILESDIALOG_HPP

#include <QDialog>
#include <QMessageBox>
#include "ui_filesdialog.h"

#include <ros/ros.h>
#include "pumpkin_messages/FilesHandler.h"
#include "file_type.h"

namespace pumpkin_qt {

class FilesDialog : public QDialog
{
	Q_OBJECT

public:
	explicit FilesDialog(QWidget *parent = 0);
	~FilesDialog();

protected:
	void resizeEvent(QResizeEvent *event);

public Q_SLOTS:
	void deleteFile();
	void deleteFolder();
	void createFolder();
	void setSelectedFolder(const QString &path);
	void setSelectedFile(const QString &file);

Q_SIGNALS:
	void sendStatusMessage(const QString &msg, int timeout);

private:
	Ui::FilesDialogDesign _ui;
	ros::ServiceClient _client;
	QString _path, _file;

	bool _success;
	pumpkin_messages::FilesHandler _msg;
	QMessageBox *_box;
};

}

#endif // FILESDIALOG_HPP
