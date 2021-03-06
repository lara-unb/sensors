/*
 * Copyright (C) 2012
 * Laboratório de Automação e Robótica, Universidade de Brasília (LARA/UnB)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the names of Laboratório de Automação e Robótica or Universidade
 *     de Brasília nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific prior
 *     written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file     image_display.h
 * @author   George Andrew Brindeiro
 * @date     08/08/2012
 *
 * @brief Simple class enabling simple display of images
 *
 * This class separates all image display functions from videre_camera
 *
 * Contact: georgebrindeiro@lara.unb.br
 *
 * Revisions:
 * [08/08/2012] Created
 */

#ifndef IMAGE_DISPLAY_H
#define IMAGE_DISPLAY_H

// OpenCV library
#include <opencv2/core/core.hpp>

class ImageDisplay
{
    public:
        ImageDisplay()
        {
            Init();
        }

        ~ImageDisplay();

        void Init();
        void Display(const cv::Mat& left_image, const cv::Mat& right_image);
        void Close();

    private:
};

#endif //IMAGE_DISPLAY_H
