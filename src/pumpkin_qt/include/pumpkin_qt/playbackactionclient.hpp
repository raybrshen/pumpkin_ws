#ifndef PLAYBACKACTIONCLIENT_H
#define PLAYBACKACTIONCLIENT_H

#include <QObject>

#include <ros/ros.h>
#include "file_type.h"
#include "pumpkin_messages/PlaybackAction.h"
#include <actionlib/client/simple_action_client.h>

namespace pumpkin_qt {

class PlaybackActionClient : public QObject
{
    Q_OBJECT
public:
	explicit PlaybackActionClient(QObject *parent = 0);
	virtual ~PlaybackActionClient();
	bool isRunning() const {return _running;}

Q_SIGNALS:
	void playbackPercentage(int percentage);
	void playbackFinished(int state);
	void blockRecTab(bool block);
	void sendStatusMessage(const QString &msg, int timeout);

public Q_SLOTS:
	void init();
	void setPlaybackFilename(const QString &filename);
	void playbackFile();
	void playbackStop();

private:
	actionlib::SimpleActionClient<pumpkin_messages::PlaybackAction> *_playback_client;
	pumpkin_messages::PlaybackGoal _goal;
	bool _running;

	void playbackDoneCallback(const actionlib::SimpleClientGoalState &goal, const pumpkin_messages::PlaybackResultConstPtr &result);
	void playbackActiveCallback();
	void playbackFeedbackCallback(const pumpkin_messages::PlaybackFeedbackConstPtr &feedback);
};

}

#endif // PLAYBACKACTIONCLIENT_H
