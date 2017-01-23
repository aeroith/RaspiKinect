/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

#include <algorithm>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <libfreenect.hpp>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <vector>

#include <iostream>
#include <stdio.h>

class Mutex {
public:
/**
* Default Mutex Constructor
*/
  Mutex() { pthread_mutex_init(&m_mutex, NULL); }
/**
* If the mutex isn't currently locked by any thread, 
* the calling thread locks it (from this point, and
* until its member unlock is called, the thread owns the mutex).
*/
  void lock() { pthread_mutex_lock(&m_mutex); }

/**
* Unlocks the mutex, releasing ownership over it.
* If other threads are currently blocked attempting
* to lock this same mutex, one of them acquires ownership
* over it and continues its execution.
*/
  void unlock() { pthread_mutex_unlock(&m_mutex); }

private:
  pthread_mutex_t m_mutex;
};

class MyFreenectDevice : public Freenect::FreenectDevice {
public:
  std::vector<uint16_t> m_buffer_depth;
  std::vector<uint8_t> m_buffer_video;
  std::vector<uint16_t> m_gamma;
  Mutex m_rgb_mutex;
  Mutex m_depth_mutex;
  bool m_new_rgb_frame;
  bool m_new_depth_frame;

/**
* Halve the integer depth buffer
*/
  uint16_t getDepthBufferSize16() { return getDepthBufferSize() / 2; }

  MyFreenectDevice(freenect_context *_ctx, int _index)
      : Freenect::FreenectDevice(_ctx, _index),
        m_buffer_depth(getDepthBufferSize()),
        m_buffer_video(getVideoBufferSize()), m_gamma(2048),
        m_new_rgb_frame(false), m_new_depth_frame(false) {

    for (unsigned int i = 0; i < 2048; i++) {
      float v = i / 2048.0;
      v = std::pow(v, 3) * 6;
      m_gamma[i] = v * 6 * 256;
    }
  }
/**
* Pass this as an argument to getRGB.
* Do not call directly even in child.
*/
  void VideoCallback(void *_rgb, uint32_t timestamp) {
    // std::cout << "RGB callback" << std::endl;
    m_rgb_mutex.lock();
    uint8_t *rgb = static_cast<uint8_t *>(_rgb);
    std::copy(rgb, rgb + getVideoBufferSize(), m_buffer_video.begin());
    m_new_rgb_frame = true;
    m_rgb_mutex.unlock();
  };
/**
* Pass this as an argument to getRGB.
* Do not call directly even in child.
*/
  void DepthCallback(void *_depth, uint32_t timestamp) {
    m_depth_mutex.lock();
    uint16_t *depth = static_cast<uint16_t *>(_depth);
    std::copy(depth, depth + getDepthBufferSize(), m_buffer_depth.begin());
    m_new_depth_frame = true;
    m_depth_mutex.unlock();
  }

/**
* Return an RGB vector from the buffer. Returns true if succeeds.
* @param[out] buffer an RGB buffer as a std::vector type
*/
  bool getRGB(std::vector<uint8_t> &buffer) {
    m_rgb_mutex.lock();
    if (m_new_rgb_frame) {
      buffer.swap(m_buffer_video);
      m_new_rgb_frame = false;
      m_rgb_mutex.unlock();
      return true;
    } else {
      m_rgb_mutex.unlock();
      return false;
    }
  }

/**
* Return a depth vector from the buffer. Returns true if succeeds.
* @param[out] buffer a depth buffer as a std::vector type
*/
  bool getDepth(std::vector<uint16_t> &buffer) {
    m_depth_mutex.lock();
    if (m_new_depth_frame) {
      buffer.swap(m_buffer_depth);
      m_new_depth_frame = false;
      m_depth_mutex.unlock();
      return true;
    } else {
      m_depth_mutex.unlock();
      return false;
    }
  }
};

Freenect::Freenect freenect;
MyFreenectDevice *device;
freenect_video_format requested_format(FREENECT_VIDEO_RGB);

double freenect_angle(0);
int got_frames(0), window(0);
int g_argc;
char **g_argv;

/**
* Get RGB and depth vectors using the Kinect and serialize
* the data.
* @param[out] depthfile serialized depth file name
* @param[out] rgbfile serialized rgb file name
*/
void KinectStream(std::string depthfile, std::string rgbfile) {
  static std::vector<uint16_t> depth(640 * 480 * 4);
  static std::vector<uint8_t> rgb(640 * 480 * 4);

  device->updateState();
  printf("\r demanded tilt angle: %+4.2f device tilt angle: %+4.2f",
         freenect_angle, device->getState().getTiltDegs());

  while (!device->getDepth(depth)) {
  }
  while (!device->getRGB(rgb)) {
  }

  fflush(stdout);
  {
    std::ofstream ofs("./" + depthfile);
    boost::archive::text_oarchive oa(ofs);
    oa &depth;
  }
  {
    std::ofstream ofs("./" + rgbfile);
    boost::archive::text_oarchive oa(ofs);
    oa &rgb;
  }

  got_frames = 0;
}

/**
* print warning message if no arguments provided
*/
void usage() {
  std::cout << "Usage: ./kinectRun [depthfile] [rgbfile]" << std::endl;
}

int main(int argc, char **argv) {
  if (argc != 3) {
    usage();
    exit(-1);
  }

  device = &freenect.createDevice<MyFreenectDevice>(0);
  device->setDepthFormat(FREENECT_DEPTH_REGISTERED);
  device->setVideoFormat(requested_format);
  device->setTiltDegrees(freenect_angle);
  device->setLed(LED_RED);
  device->startDepth();
  device->startVideo();
  std::string depthfile(argv[1]);
  std::string rgbfile(argv[2]);
  KinectStream(depthfile, rgbfile);
  device->setLed(LED_GREEN);
  device->stopVideo();
  device->stopDepth();
  return 0;
}
