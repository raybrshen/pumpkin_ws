#ifndef FILESDIALOG_HPP
#define FILESDIALOG_HPP

#include <QDialog>
#include <QMessageBox>
#include "ui_filesdialog.h"

#include <ros/ros.h>
#include "pumpkin_messages/FilesHandler.h"
#include "pumpkin_messages/file_type.h"

namespace pumpkin_qt {

/*!
 * \brief The FilesDialog is a dialog do handle the files.
 *
 * With it, you should be able to create, and delete, a folder. And delete movement files.
 *
 * To create a new movement, try to record it.
 */
class FilesDialog : public QDialog
{
	Q_OBJECT

public:
	/*!
	 * \brief Constructs the FilesDialog. Load layout and connect signals.
	 * \param parent	The parent widget. It should be the main window.
	 */
	explicit FilesDialog(QWidget *parent = 0);
	~FilesDialog();

protected:
	/*!
	 * \brief This overloaded resize event is most to ellipsis big strings, as the name of files and folders.
	 * \param event	The event handle.
	 */
	void resizeEvent(QResizeEvent *event);

public Q_SLOTS:
	/*!
	 * \brief Try to delete a file, sending a request to `file_server`.
	 */
	void deleteFile();
	/*!
	 * \brief Try to delete a folder, sending a request to `file_server`.
	 */
	void deleteFolder();
	/*!
	 * \brief Try to create a folder, sending a request to `file_server`.
	 */
	void createFolder();
	/*!
	 * \brief Set the selected folder. Its request should come from selecting the folder in the Folder Tree.
	 * \param path	The full path.
	 */
	void setSelectedFolder(const QString &path);
	/*!
	 * \brief Set the selected file. Its request should come from selecting the file in the File List.
	 * \param file	The file name.
	 */
	void setSelectedFile(const QString &file);

Q_SIGNALS:
	/*!
	 * \brief Send a message to the status bar.
	 * \param msg		The message string.
	 * \param timeout	The time that the message is shown.
	 */
	void sendStatusMessage(const QString &msg, int timeout);

private:
	Ui::FilesDialogDesign _ui;		//!< The dialog design.
	ros::ServiceClient _client;		//!< The handle to `file_handle` service.
	QString _path, _file;

	bool _success;							//!< This identifies if the last service call was success, or not.
	pumpkin_messages::FilesHandler _msg;	//!< The service message
	QMessageBox *_box;						//!< This is a confirm box.
};

}

#endif // FILESDIALOG_HPP
