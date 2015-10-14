#ifndef SSCMOVECOMMAND_HPP
#define SSCMOVECOMMAND_HPP

#include <QDialog>
#include <QList>
#include "ui_sscmovecommand.h"
#include "ui_sscmoveblock.h"

#include <ros/ros.h>
#include "pumpkin_messages/SSCMoveCommand.h"

namespace pumpkin_qt {

class SSCMoveCommand : public QDialog
{
	Q_OBJECT

public:
	explicit SSCMoveCommand(QWidget *parent = 0);
	~SSCMoveCommand();

Q_SIGNALS:
	void sendStatusMessage(const QString &msg, int timeout);

public Q_SLOTS:
	void sendCommand();

private:
	Ui::SSCMoveCommandDesign _ui;
	ros::ServiceClient _client;
	QList<Ui::SSCMoveBlocksDesign *> _blocks;
};

}

#endif // SSCMOVECOMMAND_HPP
