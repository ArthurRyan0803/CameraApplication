#pragma once

#include <QDebug>
#include <string>

class Logger
{
public:

	static void log(const std::string& message)
	{
		qDebug() << message.c_str();
	}


	static void log(const char* c_str)
	{
		qDebug() << c_str;
	}
};
