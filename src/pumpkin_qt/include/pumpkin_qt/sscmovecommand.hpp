#ifndef SSCMOVECOMMAND_HPP
#define SSCMOVECOMMAND_HPP

#include <QDialog>
#include <QList>
#include "ui_sscmovecommand.h"
#include "ui_sscmoveblock.h"

#include <ros/ros.h>
#include "pumpkin_messages/SSCMoveCommand.h"

namespace pumpkin_qt {

/*!
 * \brief The SSCMoveCommand class is a dialog to help sending direct commands to SSC.
 */
class SSCMoveCommand : public QDialog
{
	Q_OBJECT

public:
	/*!
	 * \brief Constructs the dialog window, loading the robot parts that are commanded by SSC.
	 * \param parent	The parent window. It should be the main window.
	 */
	explicit SSCMoveCommand(QWidget *parent = 0);
	/*!
	 * \brief Destroys the window.
	 */
	virtual ~SSCMoveCommand();

Q_SIGNALS:
	/*!
	 * \brief Send a message to the status bar.
	 * \param msg		The message string.
	 * \param timeout	The time that the message is shown.
	 */
	void sendStatusMessage(const QString &msg, int timeout);

public Q_SLOTS:
	/*!
	 * \brief Send a command to SSC, based on the values set on the widgets of this dialog.
	 */
	void sendCommand();

private:
	Ui::SSCMoveCommandDesign _ui;				//!< The UI Design
	ros::ServiceClient _client;					//!< The `move_ssc` service client.
	QList<Ui::SSCMoveBlocksDesign *> _blocks;	//!< The list of parts widgets.
};

}

#endif // SSCMOVECOMMAND_HPP
