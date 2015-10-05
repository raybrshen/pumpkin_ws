#ifndef __PUMPKIN_FILETYPE_CODES__
#define __PUMPKIN_FILETYPE_CODES__

namespace pumpkin_messages {

enum class FileType : uint8_t {
	Error = 0,
	ConfigFile,
	RobotDescription,
	PlaybackFile,
};

enum class IOState : uint8_t {
	OK = 0,
	EndOfFile,
	ErrorOpening,
	BadFile
};

}

#endif
