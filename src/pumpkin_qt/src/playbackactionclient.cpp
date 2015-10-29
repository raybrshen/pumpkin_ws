#include <pumpkin_messages/PlaybackGoal.h>

#include "playbackactionclient.hpp"

namespace pumpkin_qt {

PlaybackActionClient::PlaybackActionClient(QObject *parent) :
	QObject(parent)
{
	_running = false;
}

PlaybackActionClient::~PlaybackActionClient()
{
	_playback_client->cancelAllGoals();
	delete _playback_client;
}

void PlaybackActionClient::init()
{
	_playback_client = new actionlib::SimpleActionClient<pumpkin_messages::PlaybackAction>("/pumpkin/playback_action", false);
}

void PlaybackActionClient::setPlaybackFilename(const QString &filename)
{
	_goal.filenames.clear();
	_goal.filenames.push_back(filename.toStdString());
}

void PlaybackActionClient::setSceneFilenames(const std::vector<std::string> &filenames)
{
	_goal.filenames.clear();
	_goal.filenames = filenames;
}

void PlaybackActionClient::playbackFile() {
	if (_goal.filenames.empty()) {
		return;
	}
	ROS_INFO("Starting playback file %s", _goal.filenames[0].c_str());
	_playback_client->sendGoal(_goal, boost::bind(&PlaybackActionClient::playbackDoneCallback, this, _1, _2),
							   boost::bind(&PlaybackActionClient::playbackActiveCallback, this),
							   boost::bind(&PlaybackActionClient::playbackFeedbackCallback, this, _1));
}

void PlaybackActionClient::playScene()
{
	if (_goal.filenames.empty())
		return;
	ROS_INFO("Start playing scene.");
	_playback_client->sendGoal(_goal, boost::bind(&PlaybackActionClient::playbackDoneCallback, this, _1, _2),
							   boost::bind(&PlaybackActionClient::sceneActiveCallback, this),
							   boost::bind(&PlaybackActionClient::sceneFeedbackCallback, this, _1));
}

void PlaybackActionClient::playbackStop() {
	Q_EMIT(blockOnPlayback(false));
	if (_running)
		_playback_client->cancelGoal();
	_running = false;
}

void PlaybackActionClient::stopScene()
{
	Q_EMIT(blockOnScene(false));
	if (_running)
		_playback_client->cancelGoal();
	_running = false;
}

void PlaybackActionClient::playbackActiveCallback() {
	_running = true;
	_scene = false;
	Q_EMIT(blockOnPlayback(true));
	Q_EMIT(sendStatusMessage(QString("Start playback file %0.").arg(QString::fromStdString(_goal.filenames[0])), 1000));
}

void PlaybackActionClient::playbackDoneCallback(const actionlib::SimpleClientGoalState &goal,
								 const pumpkin_messages::PlaybackResultConstPtr &result) {
	_running = false;
	if (_scene)
		Q_EMIT(blockOnScene(false));
	else
		Q_EMIT(blockOnPlayback(false));

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
	if (_scene)
		Q_EMIT(scenePercentage(0, 0, 0));
	else
		Q_EMIT(playbackPercentage(0));
	Q_EMIT(playbackFinished(result->state));
}

void PlaybackActionClient::playbackFeedbackCallback(const pumpkin_messages::PlaybackFeedbackConstPtr &feedback) {
	double perc = feedback->percentage;
	perc*100;
	Q_EMIT(playbackPercentage(int(round(perc))));
}

void PlaybackActionClient::sceneActiveCallback()
{
	_running = true;
	_scene = true;
	Q_EMIT(blockOnScene(true));
	Q_EMIT(sendStatusMessage(QString("Start playing scene with %0 movements.").arg(_goal.filenames.size()), 0));
}

void PlaybackActionClient::sceneFeedbackCallback(const pumpkin_messages::PlaybackFeedbackConstPtr &feedback)
{
	double perc = feedback->percentage;
	perc*100;
	Q_EMIT(scenePercentage(feedback->movement_index, _goal.filenames.size(), int(round(perc))));
}

} //namespace end
