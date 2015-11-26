#ifndef RECORDACTIONCLIENT_HPP
#define RECORDACTIONCLIENT_HPP

#include <QObject>

#include <ros/ros.h>
#include "pumpkin_messages/file_type.h"
#include "pumpkin_messages/RecordAction.h"
#include <actionlib/client/simple_action_client.h>

namespace pumpkin_qt {

/*!
 * \brief The RecordActionClient class handles the action client to record movements.
 */
class RecordActionClient : public QObject
{
    Q_OBJECT
public:
	/*!
	 * \brief Simple constructor. Sets some members.
	 * \param parent	The parent QObject. It should be the main window.
	 */
	explicit RecordActionClient(QObject *parent = 0);
	/*!
	 * \brief Destructor. Stops service.
	 */
	virtual ~RecordActionClient();
	/*!
	 * \brief Check if the service is running, or not.
	 * \return True if running, False if not.
	 */
	inline bool isRunning() const {return _running;}

Q_SIGNALS:
	/*!
	 * \brief Sends how many minutes passed since the start of the recording.
	 * \param minutes	The ammount of minutes.
	 */
	void recordMinuteFeedback(int minutes);
	/*!
	 * \brief Sends how many seconds passed since the start of the recording.
	 * \param seconds	The ammount of seconds.
	 */
	void recordSecondFeedback(int seconds);
	/*!
	 * \brief Send the state of the finished record.
	 * \param state The state.
	 *
	 * \sa IOState
	 */
	void recordFinished(int state);
	/*!
	 * \brief Send a request to block the window on recording.
	 * \param block True to block, False to unlock.
	 */
	void blockOnRecord(bool block);
	/*!
	 * \brief Send a message to the status bar.
	 * \param msg		The message string.
	 * \param timeout	The time that the message is shown.
	 */
	void sendStatusMessage(const QString &msg, int timeout);

public Q_SLOTS:
	/*!
	 * \brief Setup the action client.
	 *
	 * This should be called after the QNode started all his stuffs.
	 */
	void init();
	/*!
	 * \brief Set the file to record the new movement.
	 * \param filename The filename, with full path.
	 */
	void setRecordFilename(const QString &filename);
	/*!
	 * \brief Set the record maximum time. (Minutes part)
	 * \param minutes The ammount of minutes.
	 */
	void setRecordTimeMinutes(int minutes);
	/*!
	 * \brief Set the record maximum time. (Seconds part)
	 * \param seconds The ammount of seconds.
	 */
	void setRecordTimeSeconds(int seconds);
	/*!
	 * \brief Request to start recording.
	 *
	 * The record may start a bit later calling this method. The record will only starts when the server accept the request.
	 */
	void recordFile();
	/*!
	 * \brief Request to stop recording.
	 *
	 * The record may stops a bit later, this is because the record will only stop when the server accepts the request.
	 */
	void recordStop();

private:
	actionlib::SimpleActionClient<pumpkin_messages::RecordAction> *_record_client;	//!< The record action client
	pumpkin_messages::RecordGoal _goal;												//!< The goal message
	int _minutes, _seconds;
	bool _running;

	/*!
	 * \brief Action client callback method. It is called when the client receives the finish of the action by the server.
	 *
	 * It will stops the current record, and send a message to the main window with some info.
	 *
	 * \param goal		The action record state.
	 * \param result	The result message.
	 */
	void recordDoneCallback(const actionlib::SimpleClientGoalState &goal, const pumpkin_messages::RecordResultConstPtr &result);
	/*!
	 * \brief Action client callback method. It is called when the client receives that the record is just started.
	 *
	 * Sets the inner members to check the service is `_running`.
	 */
	void recordActiveCallback();
	/*!
	 * \brief Action client callback method. It is called everytime the client receives a feedback from the server.
	 *
	 * It will send a message to the main window.
	 *
	 * \param feedback	The feedback message
	 */
	void recordFeedbackCallback(const pumpkin_messages::RecordFeedbackConstPtr &feedback);
};

}

#endif // RECORDACTIONCLIENT_HPP
