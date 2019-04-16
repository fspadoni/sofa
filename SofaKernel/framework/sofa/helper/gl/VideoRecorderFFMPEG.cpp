/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2018 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <sofa/helper/gl/VideoRecorderFFMPEG.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>      //popen
#include <errno.h>
#include <cstdio>		// sprintf and friends
#include <sstream>
#include <sofa/helper/logging/Messaging.h>

namespace sofa
{

namespace helper
{

namespace gl
{

VideoRecorderFFMPEG::VideoRecorderFFMPEG()
    : m_framerate(25)
    , m_prefix("sofa_video")
    , m_counter(-1)
    , m_ffmpeg(nullptr)
    , m_buffer(nullptr)
    , m_invalidParam(false)
{

}

VideoRecorderFFMPEG::~VideoRecorderFFMPEG()
{

}


bool VideoRecorderFFMPEG::init(const std::string& filename, int width, int height, unsigned int framerate, unsigned int bitrate, const std::string& codec )
{
    msg_error_when(codec.empty(), "VideoRecorderFFMPEG") << "No codec specified";
    msg_error_when(width & 1, "VideoRecorderFFMPEG")  << "Width  not divisible by 2 ("  << width << "x" << height << ").  Resize the viewport";
    msg_error_when(height & 1, "VideoRecorderFFMPEG") << "Height not divisible by 2 ("  << width << "x" << height << ").  Resize the viewport";
    
    if ( codec.empty() || (width & 1) || ( height & 1) )
    {
        m_invalidParam = true;        
        return false;
    }

    m_invalidParam = false;
    
    //std::string filename = findFilename();
    m_filename = filename;
    m_framerate = framerate;

    //GLint viewport[4];
    //glGetIntegerv(GL_VIEWPORT,viewport);
    m_Width = width;// viewport[2];
    m_Height = height;// viewport[3];

    m_FrameCount = 0;

    m_buffer = new unsigned char [4*m_Width*m_Height];

    std::stringstream ss;
    ss << FFMPEG_EXEC_FILE
       << " -r " << m_framerate
        << " -f rawvideo -pix_fmt rgba "
        << " -s " << m_Width << "x" << m_Height
        << " -i - -threads 0  -y"
        << " -preset fast "
        << " -pix_fmt " << codec // yuv420p " // " yuv444p "
        << " -crf 17 "
        << " -vf vflip "
        << "\"" << m_filename << "\""; // @TODO C++14 : replace with std::quoted

    const std::string& command_line = ss.str();

#ifdef WIN32
    m_ffmpeg = _popen(command_line.c_str(), "wb");
#else
    m_ffmpeg = popen(command_line.c_str(), "w");
#endif
    if (m_ffmpeg == nullptr) {
        msg_error("VideoRecorderFFMPEG") << "ffmpeg process failed to open (error " << errno << "). Command line : " << command_line;
        return false;
    }
    msg_info("VideoRecorderFFMPEG") << "Start recording to " << filename
        << " ( " <<  codec << ", "
        << framerate << " FPS, "
        << bitrate << " b/s)";
    return true;
}

void VideoRecorderFFMPEG::addFrame()
{
    if (m_invalidParam)
    {
        return;
    }
        
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
   
    if ((viewport[2] != m_Width) || (viewport[3] != m_Height))
    {
        std::cout << "WARNING viewport changed during video capture from " << m_Width << "x" << m_Height << "  to  " << viewport[2] << "x" << viewport[3] << std::endl;
    }

    //glReadPixels(viewport[0], viewport[1], viewport[2], viewport[3], GL_RGBA, GL_UNSIGNED_BYTE, m_buffer);

    glReadPixels(0, 0, m_Width, m_Height, GL_RGBA, GL_UNSIGNED_BYTE, (void*)m_buffer);

    fwrite(m_buffer, sizeof(unsigned char)*4*m_Width*m_Height, 1, m_ffmpeg);
    
    return;
}

void VideoRecorderFFMPEG::finishVideo()
{    
    if (m_invalidParam)
    {
        return;
    } 
    
#ifdef WIN32
    _pclose(m_ffmpeg);
#else
    pclose(m_ffmpeg);
#endif
    
    delete m_buffer;
    std::cout << m_filename << " written" << std::endl;
}

std::string VideoRecorderFFMPEG::findFilename(const unsigned int framerate, const unsigned int bitrate, const std::string& extension)
{
    SOFA_UNUSED(bitrate);
    std::string filename;
    char buf[32];
    int c = 0;
    struct stat st;
    do
    {
        ++c;
        sprintf(buf, "%04d", c);
        filename = m_prefix;
        filename += "_r" + std::to_string(framerate) + "_";
        //filename += +"_b" + std::to_string(bitrate) + "_";
        filename += buf;
        filename += ".";
        filename += extension;
    } while (stat(filename.c_str(), &st) == 0);
    m_counter = c + 1;
    return filename;
}

} // namespace gl

} // namespace helper

} // namespace sofa

