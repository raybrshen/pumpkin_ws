#include "../include/pumpkin_qt/playbackactionclient.hpp"

namespace pumpkin_qt {

PlaybackActionClient::PlaybackActionClient(QObject *parent) :
	QObject(parent)
{
	_goal.filename = std::string();
}

PlaybackActionClient::~PlaybackActionClient()
{
	_playback_client->cancelAllGoals();
	delete _playback_client;
}

void PlaybackActionClient::init()
{
	_playback_client = new actionlib::SimpleActionClient<pumpkin_messages::PlaybackAction>("playback_action", false);
}

void PlaybackActionClient::setPlaybackFilename(const QString &filename)
{
	_goal.filename = filename.toStdString();
}

void PlaybackActionClient::playbackFile() {
	if (_goal.filename.empty()) {
		return;
	}
	ROS_INFO("Starting playback file %s", _goal.filename.c_str());
	_playback_client->sendGoal(_goal, boost::bind(&PlaybackActionClient::playbackDoneCallback, this, _1, _2),
							   boost::bind(&PlaybackActionClient::playbackActiveCallback, this),
							   boost::bind(&PlaybackActionClient::playbackFeedbackCallback, this, _1));
}

void PlaybackActionClient::playbackStop() {
	Q_EMIT(blockRecTab(false));
	_playback_client->cancelGoal();
}

void PlaybackActionClient::playbackActiveCallback() {
	Q_EMIT(blockRecTab(true));
	Q_EMIT(sendStatusMessage(QString("Start playback file %0.").arg(QString::fromStdString(_goal.filename)), 1000));
}

void PlaybackActionClient::playbackDoneCallback(const actionlib::SimpleClientGoalState &goal,
								 const pumpkin_messages::PlaybackResultConstPtr &result) {
	Q_EMIT(blockRecTab(false));
	switch (static_cast<pumpkin_messages::IOState>(result->state)) {
		case pumpkin_messages::IOState::BadFile:
			Q_EMIT(sendStatusMessage(QString("Error: BAD. The playback file should be corrupted."), 1000));
			ROS_ERROR("BAD FILE");
		break;
		case pumpkin_messages::IOState::EndOfFile:
			Q_EMIT(sendStatusMessage(QString("Error: EOF. Reached End Of File before finishing the playback. Maybe ocurred some problems in recording."), 1000));
			ROS_ERROR("END OF FILE");
		break;
		case pumpkin_messages::IOState::ErrorOpening:
			Q_EMIT(sendStatusMessage(QString("Error: OPEN. There was an error opening file."), 1000));
			ROS_ERROR("ERROR OPENING FILE");
		break;
		case pumpkin_messages::IOState::OK:
			Q_EMIT(sendStatusMessage(QString("Playback terminated successfully."), 1000));
			ROS_INFO("Playback executed with success.");
		break;
		default:
			Q_EMIT(sendStatusMessage(QString("Fatal: UNKNOWN ERROR!"), 1000));
			ROS_FATAL("UNKNOWN ERROR");
	}
	Q_EMIT(playbackPercentage(0));
	Q_EMIT(playbackFinished(result->state));
}

void PlaybackActionClient::playbackFeedbackCallback(const pumpkin_messages::PlaybackFeedbackConstPtr &feedback) {
	double perc = feedback->percentage;
	perc*100;
	Q_EMIT(playbackPercentage(int(round(perc))));
}

}
