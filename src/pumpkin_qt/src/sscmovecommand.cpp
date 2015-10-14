#include "../include/pumpkin_qt/sscmovecommand.hpp"
#include <XmlRpcValue.h>
#include <QScrollArea>
#include <QTabWidget>
#include <QBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QSlider>

namespace pumpkin_qt {

using namespace Qt;

SSCMoveCommand::SSCMoveCommand(QWidget *parent) :
	QDialog(parent)
{
	_ui.setupUi(this);
	ros::NodeHandle nh;
	_client = nh.serviceClient<pumpkin_messages::SSCMoveCommand>("move_ssc");

	_blocks.reserve(32);
	for (int i = 0; i < 32; ++i)
		_blocks.insert(i, nullptr);
	XmlRpc::XmlRpcValue config;
	ros::param::get("/pumpkin/config/ssc", config);

	for (XmlRpc::XmlRpcValue::iterator it = config.begin(); it != config.end(); ++it) {
		QScrollArea *tab = new QScrollArea(_ui.moveTab);
		QBoxLayout *box = new QBoxLayout(QBoxLayout::TopToBottom, tab);

		for (XmlRpc::XmlRpcValue::iterator part_it = it->second.begin(); part_it != it->second.end(); ++part_it) {
			int pin, min, max, def;
			pin = int(it->second["pin"]);
			min = int(it->second["pulse_min"]);
			max = int(it->second["pulse_max"]);
			def = int(it->second["pulse_rest"]);
			Ui::SSCMoveBlocksDesign *block = new Ui::SSCMoveBlocksDesign;
			QWidget *block_widget = new QWidget(tab);
			block->setupUi(block_widget);
			block->blockLabel->setText(QString::fromStdString(part_it->first));
			block->activeButton->setText(QString("Active ") + QString::fromStdString(part_it->first));
			block->pulseSpin->setRange(min, max);
			block->pulseSpin->setValue(def);
			block->pulseSlider->setRange(min, max);
			block->pulseSlider->setValue(def);
			block->activeButton->setChecked(true);
			box->addWidget(block_widget);
			_blocks.insert(pin, block);
		}

		tab->setLayout(box);
		_ui.moveTab->addTab(tab, QString::fromStdString(it->first));
	}
}

SSCMoveCommand::~SSCMoveCommand()
{
	qDeleteAll(_blocks);
}

void SSCMoveCommand::sendCommand()
{
	pumpkin_messages::SSCMoveCommand msg;
	for (int i = 0; i < _blocks.size(); ++i) {
		Ui::SSCMoveBlocksDesign *design = _blocks[i];
		if (design == nullptr)
			continue;
		if (!design->activeButton->isChecked())
			continue;
		pumpkin_messages::SSCMove move;
		move.channel = i;
		move.pulse = design->pulseSpin->value();
		move.speed = design->speedSpin->value();
		msg.request.move.list.push_back(move);
	}
	if (msg.request.move.list.size() > 0) {
		msg.request.move.time = _ui.timeSpin->value();
		if (_client.call(msg)) {
			Q_EMIT(sendStatusMessage(QString("Command sent to SSC successfully."), 0));
		} else {
			Q_EMIT(sendStatusMessage(QString("Error sending command."), 0));
		}
	} else {
		Q_EMIT(sendStatusMessage(QString("How do you want to send a void command?"), 0));
	}
}

}
