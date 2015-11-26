#ifndef __PUMPKIN_FILETYPE_CODES__
#define __PUMPKIN_FILETYPE_CODES__

/*
 * Helper enumerations
 * ===================
 *
 * This is just an helpter include file to use enums to identify some signals
 */

namespace pumpkin_messages {

/*!
 * \brief This enum identify the file type. It's used for the `file_lister` server.
 */
enum class FileType : uint8_t {
	Error = 0,          //< Just an error identifier
	ConfigFile,         //< Configuration files
	RobotDescription,   //< URDF Robot Description
	PlaybackFile,       //< Playback files
};

/*!
 * \brief This is for the response from the playback and record
 *
 * It is also used to Scene.
 */
enum class IOState : uint8_t {
	OK = 0,             //< When Murphy hasn't see you yet
	EndOfFile,          //< EOF Error
	ErrorOpening,       //< Some other weird error opening the file
	ErrorCreating,      //< Some weird error creating the file (as permission, or disk full)
	FileAlreadyExists,  //< You're trying to overwrite an existing file?
	FileNotExists,      //< When trying to open an file that doesn't exists
	ErrorDeleting,      //< Some weird error deleting the file
	BadFile,            //< Bad File error.
	Other               //< Other errors
};

/*!
 * \brief *File Handle Service File*. For use in the reques to create or delete a file or folder.
 */
enum class FHST : uint8_t {
	CreateFile,     //< Request to create a file
	CreateFolder,   //< Request to create a folder
	DeleteFile,     //< Request to delete a file
	DeleteFolder,   //< Request to delete a folder
};

}

#endif
