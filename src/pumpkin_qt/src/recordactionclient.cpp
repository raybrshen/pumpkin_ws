#include "../include/pumpkin_qt/recordactionclient.hpp"

namespace pumpkin_qt {

RecordActionClient::RecordActionClient(QObject *parent) :
	QObject(parent), _running(false)
{
	_minutes = _seconds = 0;
	_goal.filename = std::string();
	_goal.max_time = 0.0;
	_running = false;
}

RecordActionClient::~RecordActionClient()
{
	_record_client->cancelAllGoals();
	delete _record_client;
}

void RecordActionClient::init()
{
	_record_client = new actionlib::SimpleActionClient<pumpkin_messages::RecordAction>("recorder_action", false);
}

void RecordActionClient::setRecordFilename(const QString &filename)
{
	_goal.filename = filename.toStdString();
}

void RecordActionClient::setRecordTimeMinutes(int minutes)
{
	if (!_running)
		_minutes = minutes;
}

void RecordActionClient::setRecordTimeSeconds(int seconds)
{
	if (!_running)
		_seconds = seconds;
}

void RecordActionClient::recordFile() {
	if (_goal.filename.empty()) {
		Q_EMIT(sendStatusMessage(QString("Error: empty filename."), 1000));
		return;
	} else if (_minutes == 0 && _seconds == 0) {
		Q_EMIT(sendStatusMessage(QString("Error: Time was set to zero."), 1000));
		return;
	}
	_goal.max_time = double(60 * _minutes + _seconds);
	ROS_INFO("Starting record file %s", _goal.filename.c_str());
	_record_client->sendGoal(_goal, boost::bind(&RecordActionClient::recordDoneCallback, this, _1, _2),
							boost::bind(&RecordActionClient::recordActiveCallback, this),
							boost::bind(&RecordActionClient::recordFeedbackCallback, this, _1));
}

void RecordActionClient::recordStop() {
	Q_EMIT(blockPlayTab(false));
	if (_running)
		_record_client->cancelGoal();
	Q_EMIT(sendStatusMessage(QString("Stopped recording"), 1000));
	_running = false;
}

void RecordActionClient::recordActiveCallback() {
	_running = true;
	Q_EMIT(blockPlayTab(true));
	Q_EMIT(sendStatusMessage(QString("Start recording file %0.").arg(QString::fromStdString(_goal.filename)), 1000));
}

void RecordActionClient::recordDoneCallback(const actionlib::SimpleClientGoalState &goal,
								 const pumpkin_messages::RecordResultConstPtr &result) {
	_running = false;
	Q_EMIT(blockPlayTab(false));
	bool aborted = (goal == actionlib::SimpleClientGoalState::ABORTED);
	switch (static_cast<pumpkin_messages::IOState>(result->state)) {
		case pumpkin_messages::IOState::BadFile:
			Q_EMIT(sendStatusMessage(QString("Error: BAD. The record file should be corrupted."), 1000));
			ROS_ERROR("BAD FILE");
		break;
		case pumpkin_messages::IOState::EndOfFile:
			Q_EMIT(sendStatusMessage(QString("Error: EOF. There is an error saving record file on disk. Maybe the disk is full."), 1000));
			ROS_ERROR("END OF FILE");
		break;
		case pumpkin_messages::IOState::ErrorOpening:
			Q_EMIT(sendStatusMessage(QString("Error: OPEN. It was not possible to create record file."), 1000));
			ROS_ERROR("ERROR OPENING FILE");
		break;
		case pumpkin_messages::IOState::OK:
			Q_EMIT(sendStatusMessage(QString("Record terminated successfully%0.")
									 .arg(QString(aborted ? ", but it was aborted before the end" : " and with full time")), 1000));
			ROS_INFO("Playback executed with success.");
		break;
		default:
			Q_EMIT(sendStatusMessage(QString("Fatal: UNKNOWN ERROR!"), 1000));
			ROS_FATAL("UNKNOWN ERROR");
	}
	Q_EMIT(recordMinuteFeedback(_minutes));
	Q_EMIT(recordSecondFeedback(_seconds));
	Q_EMIT(recordFinished(result->state));
}

void RecordActionClient::recordFeedbackCallback(const pumpkin_messages::RecordFeedbackConstPtr &feedback) {
	ros::Time elapsed(feedback->time_elapsed);
	ROS_INFO("Passed %lf time", feedback->time_elapsed);
	Q_EMIT(recordMinuteFeedback(elapsed.sec / 60));
	Q_EMIT(recordSecondFeedback(elapsed.sec % 60));
}

}
