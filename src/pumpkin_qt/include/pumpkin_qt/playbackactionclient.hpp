#ifndef PLAYBACKACTIONCLIENT_H
#define PLAYBACKACTIONCLIENT_H

#include <QObject>

#include <ros/ros.h>
#include "pumpkin_messages/file_type.h"
#include "pumpkin_messages/PlaybackAction.h"
#include <actionlib/client/simple_action_client.h>

namespace pumpkin_qt {

/*!
 * \brief The PlaybackActionClient is a class to handle the playback and scene request fom GUI.
 */
class PlaybackActionClient : public QObject
{
    Q_OBJECT
public:
	explicit PlaybackActionClient(QObject *parent = 0);
	virtual ~PlaybackActionClient();
	bool isRunning() const {return _running;}

Q_SIGNALS:
	/*!
	 * \brief Send the atual playback complete percentage.
	 * \param percentage The percentage value.
	 */
	void playbackPercentage(int percentage);
	/*!
	 * \brief Send the status of the scene.
	 * \param step			The atual movement, or planned transition.
	 * \param total			The total ammount of movements.
	 * \param percentage	The complete percentage of current step.
	 */
	void scenePercentage(int step, int total, int percentage);
	/*!
	 * \brief Send the status of the finished playback.
	 * \param state	The state.
	 *
	 * \sa IOState
	 */
	void playbackFinished(int state);
	/*!
	 * \brief Send a request to main window to block on playback.
	 * \param block	True to block. False to unlock.
	 */
	void blockOnPlayback(bool block);
	/*!
	 * \brief Send a request to main window to block on scene.
	 * \param block	True to block. False to unlock.
	 */
	void blockOnScene(bool block);
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
	 * \brief Set the file to run the playback. Call this before start playback.
	 * \param filename	The filename, with full path.
	 */
	void setPlaybackFilename(const QString &filename);
	/*!
	 * \brief Set the list of movements to play a scene. Call this before start scene.
	 *
	 * For consistency, it only accepts lists with two, or more, files.
	 *
	 * \param filenames The list of files, each with respective full path.
	 */
	void setSceneFilenames(const std::vector<std::string>& filenames);
	/*!
	 * \brief Starts playback.
	 *
	 * The request may occur a bit later. The playback will only starts when the server accept the request.
	 *
	 * Only runs if there is only one playback file. In other words, calling `setPlaybackFilename` before.
	 *
	 * It is highly recommended, to maintain consistence, to call the `setPlaybackFilename`,
	 *  not the `setSceneFilenames` with only one movement file.
	 * (For now, this method doesn't accept list with only one movement)
	 */
	void playbackFile();
	/*!
	 * \brief Start playing the scene.
	 *
	 * The request may occur a bit later. The scene will only starts when the server accept the request.
	 *
	 * Only runs if there is at least two movements. In other words, it is necessary to call `setSceneFilenames` before.
	 */
	void playScene();
	/*!
	 * \brief Send a request to stop the playback (if there is any happening).
	 *
	 * The request may occur a bit later. The playback will only stops when the server accept the request.
	 *
	 * For consistency, it only runs if there is any playback (not scene) running.
	 */
	void playbackStop();
	/*!
	 * \brief Send a request to stop playing the scene.
	 *
	 * The request may occur a bit later. The scene will only stops when the server accept the request.
	 *
	 * For consistency, it only runs if there is a scene (not only playback) running.
	 */
	void stopScene();

private:
	actionlib::SimpleActionClient<pumpkin_messages::PlaybackAction> *_playback_client;	//!< The Playback Action Client
	pumpkin_messages::PlaybackGoal _goal;												//!< The goal message
	bool _running, _scene;

	/*!
	 * \brief Callback method for the action client. Called everytime a playback __or scene__ finishes.
	 *
	 * **This is the only callback that is the same for both.**
	 *
	 * It stops the service and send a message to the main window telling some info.
	 *
	 * \param goal		The state of the current action.
	 * \param result	The result message.
	 */
	void playbackDoneCallback(const actionlib::SimpleClientGoalState &goal, const pumpkin_messages::PlaybackResultConstPtr &result);
	/*!
	 * \brief Callback method for the action client. Called when the playback service just started.
	 *
	 * It sets the members that controls the client state, and send a message to the main window.
	 */
	void playbackActiveCallback();
	/*!
	 * \brief Callback method for the action client. Called everytime the client receives a feedback from playback.
	 * \param feedback	The feedback message.
	 */
	void playbackFeedbackCallback(const pumpkin_messages::PlaybackFeedbackConstPtr &feedback);
	/*!
	 * \brief Callback method for the action client. Called when the scene service just started.
	 *
	 * It sets the members that controls the client state, and send a message to the main window.
	 */
	void sceneActiveCallback();
	/*!
	 * \brief Callback method for the action client. Called everytime the client receives a feedback from scene.
	 * \param feedback	The feedback message.
	 */
	void sceneFeedbackCallback(const pumpkin_messages::PlaybackFeedbackConstPtr &feedback);
};

}

#endif // PLAYBACKACTIONCLIENT_H
