#pragma once

#include <map>
#include <string>

#include "CalibrationPatternMethod.h"

class UICommon
{
public:
	UICommon() = delete;
	~UICommon() = delete;

	inline const static std::map<std::string, Pattern> string_to_clib_pattern_map {
		{ "Chessboard", Chessboard },
		{ "CirclesArray", CirclesArray }
	};
};
