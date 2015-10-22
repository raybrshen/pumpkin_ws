#ifndef RECORDACTIONCLIENT_HPP
#define RECORDACTIONCLIENT_HPP

#include <QObject>

#include <ros/ros.h>
#include "file_type.h"
#include "pumpkin_messages/RecordAction.h"
#include <actionlib/client/simple_action_client.h>

namespace pumpkin_qt {

class RecordActionClient : public QObject
{
    Q_OBJECT
public:
	explicit RecordActionClient(QObject *parent = 0);
	virtual ~RecordActionClient();
	bool isRunning() const {return _running;}

Q_SIGNALS:
	void recordMinuteFeedback(int minutes);
	void recordSecondFeedback(int seconds);
	void recordFinished(int state);
	void blockOnRecord(bool block);
	void sendStatusMessage(const QString &msg, int timeout);

public Q_SLOTS:
	void init();
	void setRecordFilename(const QString &filename);
	void setRecordTimeMinutes(int minutes);
	void setRecordTimeSeconds(int seconds);
	void recordFile();
	void recordStop();

private:
	actionlib::SimpleActionClient<pumpkin_messages::RecordAction> *_record_client;
	pumpkin_messages::RecordGoal _goal;
	int _minutes, _seconds;
	bool _running;

	void recordDoneCallback(const actionlib::SimpleClientGoalState &goal, const pumpkin_messages::RecordResultConstPtr &result);
	void recordActiveCallback();
	void recordFeedbackCallback(const pumpkin_messages::RecordFeedbackConstPtr &feedback);
};

}

#endif // RECORDACTIONCLIENT_HPP
