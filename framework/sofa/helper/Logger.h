/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                              SOFA :: Framework                              *
*                                                                             *
* Authors: The SOFA Team (see Authors.txt)                                    *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_HELPER_LOGGER_H
#define SOFA_HELPER_LOGGER_H

#include <sofa/SofaFramework.h>
#include <sofa/helper/system/console.h>
#include <string>
#include <iostream>
#include <boost/shared_ptr.hpp>


namespace sofa
{

namespace helper
{


/// Logger class, that provides a centralized way to process messages.
class SOFA_HELPER_API Logger
{
public:
    typedef boost::shared_ptr<Logger> SPtr;
    enum Level {All, Debug, Info, Warning, Error, Off, LevelCount};

    Logger();
    virtual ~Logger();

    /// @brief Log a message if its level is higher than getLevel().
    ///
    /// @param location An indication of where the message comes from, if relevant. (Component, function...)
    virtual void log(Level level, const std::string& message, const std::string& location = "") = 0;

    /// @brief Set the minimal level of logging.
    void setLevel(Level level);
    /// @brief Get the level above which message will be logged.
    Level getLevel();

protected:
    Level m_currentLevel;

public:
    /// @brief Get the logger used <b>internally</b> to log messages in the Sofa libraries.
    ///
    static Logger& getMainLogger();
    /// @brief Change the logger used internally to log messages in the Sofa libraries.
    static void setMainLogger(boost::shared_ptr<Logger> logger);

private:
    static Logger::SPtr s_mainLogger;
};


/// Simple Logger that outputs to stdout and stderr.
class SOFA_HELPER_API TTYLogger: public Logger
{
public:
    TTYLogger();
    virtual void log(Level level, const std::string& message, const std::string& location = "");
protected:
    std::string m_prefixes[Logger::LevelCount];
    Console::ColorType m_colors[Logger::LevelCount];
};


} // namespace helper

} // namespace sofa

#endif