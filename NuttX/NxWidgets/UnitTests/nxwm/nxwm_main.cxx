/////////////////////////////////////////////////////////////////////////////
// NxWidgets/UnitTests/nxwm/nxwm_main.cxx
//
//   Copyright (C) 2012 Gregory Nutt. All rights reserved.
//   Author: Gregory Nutt <gnutt@nuttx.org>
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
// 3. Neither the name NuttX, NxWidgets, nor the names of its contributors
//    me be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
// OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
//////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// Included Files
/////////////////////////////////////////////////////////////////////////////

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <cstdio>
#include <cstdlib>
#include <cunistd>

#ifdef CONFIG_NXWM_TOUCHSCREEN_CONFIGDATA
#  include "platform/configdata.hr"
#endif

#include "ctaskbar.hxx"
#include "cstartwindow.hxx"

#ifdef CONFIG_NXWM_NXTERM
#  include "cnxterm.hxx"
#endif

#include "chexcalculator.hxx"

#ifdef CONFIG_NXWM_MEDIAPLAYER
#  include "cmediaplayer.hxx"
#endif

#ifdef CONFIG_NXWM_TOUCHSCREEN
#  include "ctouchscreen.hxx"
#  include "ccalibration.hxx"
#endif

#ifdef CONFIG_NXWM_KEYBOARD
#  include "ckeyboard.hxx"
#endif

/////////////////////////////////////////////////////////////////////////////
// Pre-processor Definitions
/////////////////////////////////////////////////////////////////////////////

#ifdef CONFIG_HAVE_FILENAME
#  define showTestStepMemory(msg) \
     _showTestStepMemory((FAR const char*)__FILE__, (int)__LINE__, msg)
#endif

#ifdef CONFIG_NXWIDGET_MEMMONITOR
#  ifdef CONFIG_HAVE_FILENAME
#    define showTestCaseMemory(msg) \
       _showTestCaseMemory((FAR const char*)__FILE__, (int)__LINE__, msg)
#    define showTestMemory(msg) \
       _showTestMemory((FAR const char*)__FILE__, (int)__LINE__, msg)
#  endif
#else
#    define initMemoryUsage()
#    define showTestCaseMemory(msg)
#    define showTestMemory(msg)
#endif

/////////////////////////////////////////////////////////////////////////////
// Private Types
/////////////////////////////////////////////////////////////////////////////

struct SNxWmTest
{
  NxWM::CTaskbar     *taskbar;             // The task bar
  NxWM::CStartWindow *startwindow;         // The start window
#ifdef CONFIG_NXWM_TOUCHSCREEN
  NxWM::CTouchscreen *touchscreen;         // The touchscreen
  struct NxWM::SCalibrationData calibData; // Calibration data
  bool                calibrated;          // True: Touchscreen has been calibrated
#endif
#ifdef CONFIG_NXWIDGET_MEMMONITOR
  unsigned int        mmInitial;           // Initial memory usage
  unsigned int        mmStep;              // Memory Usage at beginning of test step
  unsigned int        mmSubStep;           // Memory Usage at beginning of test sub-step
#endif
};

/////////////////////////////////////////////////////////////////////////////
// Private Data
/////////////////////////////////////////////////////////////////////////////

static struct SNxWmTest g_nxwmtest;

/////////////////////////////////////////////////////////////////////////////
// Public Function Prototypes
/////////////////////////////////////////////////////////////////////////////

// Suppress name-mangling

extern "C" int nxwm_main(int argc, char *argv[]);

/////////////////////////////////////////////////////////////////////////////
// Public Functions
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// Name: updateMemoryUsage
/////////////////////////////////////////////////////////////////////////////

#ifdef CONFIG_NXWIDGET_MEMMONITOR
#ifdef CONFIG_HAVE_FILENAME
static void updateMemoryUsage(unsigned int *previous,
                              FAR const char *file, int line,
                              FAR const char *msg)
#else
static void updateMemoryUsage(unsigned int *previous,
                              FAR const char *msg)
#endif
{
  struct mallinfo mmcurrent;

  /* Get the current memory usage */

#ifdef CONFIG_CAN_PASS_STRUCTS
  mmcurrent = mallinfo();
#else
  (void)mallinfo(&mmcurrent);
#endif

  /* Show the change from the previous time */

#ifdef CONFIG_HAVE_FILENAME
  printf("File: %s Line: %d : %s\n", file, line, msg);
#else
  printf("\n%s:\n", msg);
#endif
  printf("  Before: %8u After: %8u Change: %8d\n",
         *previous, mmcurrent.uordblks, (int)mmcurrent.uordblks - (int)*previous);

  /* Set up for the next test */

  *previous =  mmcurrent.uordblks;
}
#endif

/////////////////////////////////////////////////////////////////////////////
// Name: showTestCaseMemory
/////////////////////////////////////////////////////////////////////////////

#ifdef CONFIG_NXWIDGET_MEMMONITOR
#ifdef CONFIG_HAVE_FILENAME
static void _showTestCaseMemory(FAR const char *file, int line, FAR const char *msg)
{
  updateMemoryUsage(&g_nxwmtest.mmStep, file, line, msg);
  g_nxwmtest.mmSubStep = g_nxwmtest.mmInitial;
}
#else
static void showTestCaseMemory(FAR const char *msg)
{
  updateMemoryUsage(&g_nxwmtest.mmStep, msg);
  g_nxwmtest.mmSubStep = g_nxwmtest.mmInitial;
}
#endif
#endif

/////////////////////////////////////////////////////////////////////////////
// Name: showTestMemory
/////////////////////////////////////////////////////////////////////////////

#ifdef CONFIG_NXWIDGET_MEMMONITOR
#ifdef CONFIG_HAVE_FILENAME
static void _showTestMemory(FAR const char *file, int line, FAR const char *msg)
{
  updateMemoryUsage(&g_nxwmtest.mmInitial, file, line, msg);
}
#else
static void showTestMemory(FAR const char *msg)
{
  updateMemoryUsage(&g_nxwmtest.mmInitial, msg);
}
#endif
#endif

/////////////////////////////////////////////////////////////////////////////
// Name: initMemoryUsage
/////////////////////////////////////////////////////////////////////////////

#ifdef CONFIG_NXWIDGET_MEMMONITOR
static void initMemoryUsage(void)
{
  struct mallinfo mmcurrent;

#ifdef CONFIG_CAN_PASS_STRUCTS
  mmcurrent = mallinfo();
#else
  (void)mallinfo(&mmcurrent);
#endif

  g_nxwmtest.mmInitial = mmcurrent.uordblks;
  g_nxwmtest.mmStep    = mmcurrent.uordblks;
  g_nxwmtest.mmSubStep = mmcurrent.uordblks;
}
#endif

/////////////////////////////////////////////////////////////////////////////
// Name: cleanup
/////////////////////////////////////////////////////////////////////////////

static void testCleanUpAndExit(int exitCode)
{
#ifdef CONFIG_NXWM_TOUCHSCREEN
  if (g_nxwmtest.touchscreen)
    {
      delete g_nxwmtest.touchscreen;
    }
#endif

  // Delete the task bar then the start window.  the order is important because
  // we must bet all of the application references out of the task bar before
  // deleting the start window.  When the start window is deleted, it will
  // also delete of of the resouces contained within the start window.

  if (g_nxwmtest.taskbar)
    {
      delete g_nxwmtest.taskbar;
    }

  if (g_nxwmtest.startwindow)
    {
      delete g_nxwmtest.startwindow;
    }

  // And exit

  exit(exitCode);
}

/////////////////////////////////////////////////////////////////////////////
// Name: createTaskbar
/////////////////////////////////////////////////////////////////////////////

static bool createTaskbar(void)
{
  // Create an instance of the Task Bar.
  //
  // The general sequence for initializing the task bar is:
  //
  // 1. Create the CTaskbar instance,
  // 2. Call the CTaskbar::connect() method to connect to the NX server (CTaskbar
  //    inherits the connect method from CNxServer),
  // 3. Call the CTaskbar::initWindowManager() method to initialize the task bar.
  // 3. Call CTaskBar::startApplication repeatedly to add applications to the task bar
  // 4. Call CTaskBar::startWindowManager to start the display with applications in place

  printf("createTaskbar: Create CTaskbar instance\n");
  g_nxwmtest.taskbar = new NxWM::CTaskbar();
  if (!g_nxwmtest.taskbar)
    {
      printf("createTaskbar: ERROR: Failed to instantiate CTaskbar\n");
      return false;
    }
  showTestCaseMemory("createTaskbar: After create taskbar");

  // Connect to the NX server

  printf("createTaskbar: Connect CTaskbar instance to the NX server\n");
  if (!g_nxwmtest.taskbar->connect())
    {
      printf("createTaskbar: ERROR: Failed to connect CTaskbar instance to the NX server\n");
      return false;
    }
  showTestCaseMemory("createTaskbar: After connecting to the server");

  // Initialize the task bar
  //
  // Taskbar::initWindowManager() prepares the task bar to receive applications.
  // CTaskBar::startWindowManager() brings the window manager up with those applications
  // in place.

  printf("createTaskbar: Initialize CTaskbar instance\n");
  if (!g_nxwmtest.taskbar->initWindowManager())
    {
      printf("createTaskbar: ERROR: Failed to intialize CTaskbar instance\n");
      return false;
    }

  showTestCaseMemory("createTaskbar: After initializing window manager");
  return true;
}

/////////////////////////////////////////////////////////////////////////////
// Name: createStartWindow
/////////////////////////////////////////////////////////////////////////////

static bool createStartWindow(void)
{
  // Create the start window.  The start window is unique among applications
  // because it has no factory.  The general sequence for setting up the
  // start window is:
  //
  // 1. Create and open a CApplicationWindow
  // 2. Use the window to create the CStartWindow the start window application
  // 2. Call Cstartwindow::addApplication numerous times to install applications
  //    in the start window.
  // 3. Call CTaskBar::startApplication (initially minimized) to start the start
  //    window application.
  //
  // NOTE: that the start window should not have a stop button.

  NxWM::CApplicationWindow *window = g_nxwmtest.taskbar->openApplicationWindow(NxWM::CApplicationWindow::WINDOW_PERSISTENT);
  if (!window)
    {
      printf("createStartWindow: ERROR: Failed to create CApplicationWindow\n");
      return false;
    }
  showTestCaseMemory("createStartWindow: After creating CApplicationWindow");

  // Open the window (it is hot in here)

  if (!window->open())
    {
      printf("createStartWindow: ERROR: Failed to open CApplicationWindow \n");
      delete window;
      return false;
    }
  showTestCaseMemory("createStartWindow: After opening CApplicationWindow");

  // Instantiate the application, providing the window to the application's
  // constructor

  g_nxwmtest.startwindow = new NxWM::CStartWindow(g_nxwmtest.taskbar, window);
  if (!g_nxwmtest.startwindow)
    {
      gerr("ERROR: Failed to instantiate CStartWindow\n");
      delete window;
      return false;
    }
  showTestCaseMemory("createStartWindow: After creating CStartWindow");

  // Add the CStartWindow application to the task bar (minimized)

  printf("createStartWindow: Start the start window application\n");
  if (!g_nxwmtest.taskbar->startApplication(g_nxwmtest.startwindow, true))
    {
      printf("createStartWindow: ERROR: Failed to start the start window application\n");
      return false;
    }
  showTestCaseMemory("createStartWindow: After starting the start window application");
  return true;
}

/////////////////////////////////////////////////////////////////////////////
// Name: startWindowManager
/////////////////////////////////////////////////////////////////////////////

static bool startWindowManager(void)
{
  // Start the window manager

  printf("startWindowManager: Start the window manager\n");
  if (!g_nxwmtest.taskbar->startWindowManager())
    {
      printf("startWindowManager: ERROR: Failed to start the window manager\n");
      return false;
    }

  showTestCaseMemory("startWindowManager: After starting the window manager");
  return true;
}

/////////////////////////////////////////////////////////////////////////////
// Name: createTouchScreen
/////////////////////////////////////////////////////////////////////////////

#ifdef CONFIG_NXWM_TOUCHSCREEN
static bool createTouchScreen(void)
{
  // Get the physical size of the display in pixels

  struct nxgl_size_s displaySize;
  (void)g_nxwmtest.taskbar->getDisplaySize(displaySize);

    // Create the touchscreen device

  printf("createTouchScreen: Creating CTouchscreen\n");
  g_nxwmtest.touchscreen = new NxWM::CTouchscreen(g_nxwmtest.taskbar, &displaySize);
  if (!g_nxwmtest.touchscreen)
    {
      printf("createTouchScreen: ERROR: Failed to create CTouchscreen\n");
      return false;
    }
  showTestCaseMemory("createTouchScreen: After creating CTouchscreen");

  printf("createTouchScreen: Start touchscreen listener\n");
  if (!g_nxwmtest.touchscreen->start())
    {
      printf("createTouchScreen: ERROR: Failed start the touchscreen listener\n");
      delete g_nxwmtest.touchscreen;
      return false;
    }

  showTestCaseMemory("createTouchScreen: After starting the touchscreen listener");
  return true;
}
#endif

/////////////////////////////////////////////////////////////////////////////
// Name: createKeyboard
/////////////////////////////////////////////////////////////////////////////

#ifdef CONFIG_NXWM_KEYBOARD
static bool createKeyboard(void)
{
  printf("createKeyboard: Creating CKeyboard\n");
  NxWM::CKeyboard *keyboard = new NxWM::CKeyboard(g_nxwmtest.taskbar);
  if (!keyboard)
    {
      printf("createKeyboard: ERROR: Failed to create CKeyboard\n");
      return false;
    }
  showTestCaseMemory("createKeyboard After creating CKeyboard");

  printf("createKeyboard: Start keyboard listener\n");
  if (!keyboard->start())
    {
      printf("createKeyboard: ERROR: Failed start the keyboard listener\n");
      delete keyboard;
      return false;
    }

  showTestCaseMemory("createKeyboard: After starting the keyboard listener");
  return true;
}
#endif

/////////////////////////////////////////////////////////////////////////////
// Name: createCalibration
/////////////////////////////////////////////////////////////////////////////

#ifdef CONFIG_NXWM_TOUCHSCREEN
static bool createCalibration(void)
{
  // 1Create the CCalibrationFactory application factory

  printf("createCalibration: Creating CCalibrationFactory\n");
  NxWM::CCalibrationFactory *factory = new NxWM::CCalibrationFactory(g_nxwmtest.taskbar, g_nxwmtest.touchscreen);
  if (!factory)
    {
      printf("createCalibration: ERROR: Failed to create CCalibrationFactory\n");
      return false;
    }
  showTestCaseMemory("createCalibration: After creating CCalibrationFactory");

  // Add the calibration application to the start window.

  printf("createCalibration: Adding CCalibration to the start window\n");
  if (!g_nxwmtest.startwindow->addApplication(factory))
    {
      printf("createCalibration: ERROR: Failed to add CCalibrationto the start window\n");
      delete factory;
      return false;
    }
  showTestCaseMemory("createCalibration: After adding CCalibration");

  // Call StartWindowFactory::create to to create the start window application

  printf("createCalibration: Creating CCalibration\n");
  NxWM::IApplication *calibration = factory->create();
  if (!calibration)
    {
      printf("createCalibration: ERROR: Failed to create CCalibration\n");
      return false;
    }
  showTestCaseMemory("createCalibration: After creating CCalibration");

#ifdef CONFIG_NXWM_TOUCHSCREEN_CONFIGDATA
  // Check if we have previously stored calibration data

  int ret = platform_getconfig(CONFIGDATA_TSCALIBRATION, 0,
                               (FAR uint8_t *)&g_nxwmtest.calibData,
                               sizeof(struct NxWM::SCalibrationData));
  if (ret == OK)
    {
      // We successfully got the calibration data from the platform-specific
      // logic.  This might fail if (1) calibration data was never saved, or
      // (2) if some failure occurred while trying to obtain the configuration
      // data.  In either event, the appropriate thing to do is to perform
      // the calibration.

      // Provide the calibration data to the touchscreen thread

      g_nxwmtest.touchscreen->setCalibrationData(g_nxwmtest.calibData);
      g_nxwmtest.touchscreen->setEnabled(true);
      g_nxwmtest.calibrated = true;
    }
  else
#endif
    {
      // Call CTaskBar::startApplication to start the Calibration application.
      // Nothing will be displayed because the window manager has not yet been
      // started.

      printf("createCalibration: Start the calibration application\n");
      g_nxwmtest.calibrated = false;

      if (!g_nxwmtest.taskbar->startApplication(calibration, false))
        {
          printf("createCalibration ERROR: Failed to start the calibration application\n");
          delete calibration;
          return false;
        }

      showTestCaseMemory("createCalibration: After starting the start window application");
    }

  return true;
}
#endif

/////////////////////////////////////////////////////////////////////////////
// Name: createNxTerm
/////////////////////////////////////////////////////////////////////////////

#ifdef CONFIG_NXWM_NXTERM
static bool createNxTerm(void)
{
  // Add the NxTerm application to the start window

  printf("createNxTerm: Creating the NxTerm application\n");
  NxWM::CNxTermFactory *console = new  NxWM::CNxTermFactory(g_nxwmtest.taskbar);
  if (!console)
    {
      printf("createNxTerm: ERROR: Failed to instantiate CNxTermFactory\n");
      return false;
    }
  showTestCaseMemory("createNxTerm: After creating the NxTerm application");

  printf("createNxTerm: Adding the NxTerm application to the start window\n");
  if (!g_nxwmtest.startwindow->addApplication(console))
    {
      printf("createNxTerm: ERROR: Failed to add CNxTermFactory to the start window\n");
      delete console;
      return false;
    }

  showTestCaseMemory("createNxTerm: After adding the NxTerm application");
  return true;
}
#endif

/////////////////////////////////////////////////////////////////////////////
// Name: createHexCalculator
/////////////////////////////////////////////////////////////////////////////

static bool createHexCalculator(void)
{
  // Add the hex calculator application to the start window

  printf("createHexCalculator: Creating the hex calculator application\n");
  NxWM::CHexCalculatorFactory *calculator = new  NxWM::CHexCalculatorFactory(g_nxwmtest.taskbar);
  if (!calculator)
    {
      printf("createHexCalculator: ERROR: Failed to instantiate CHexCalculatorFactory\n");
      return false;
    }
  showTestCaseMemory("createHexCalculator: After creating the hex calculator application");

  printf("createHexCalculator: Adding the hex calculator application to the start window\n");
  if (!g_nxwmtest.startwindow->addApplication(calculator))
    {
      printf("createHexCalculator: ERROR: Failed to add CHexCalculatorFactory to the start window\n");
      delete calculator;
      return false;
    }

  showTestCaseMemory("createHexCalculator: After adding the hex calculator application");
  return true;
}

/////////////////////////////////////////////////////////////////////////////
// Name: createMediaPlayer
/////////////////////////////////////////////////////////////////////////////

#ifdef CONFIG_NXWM_MEDIAPLAYER
static bool createMediaPlayer(void)
{
  // Add the hex calculator application to the start window

  printf("createHexCalculator: Creating the hex calculator application\n");
  NxWM::CMediaPlayerFactory *mediaplayer = new  NxWM::CMediaPlayerFactory(g_nxwmtest.taskbar);
  if (!mediaplayer)
    {
      printf("createMediaPlayer: ERROR: Failed to instantiate CMediaPlayerFactory\n");
      return false;
    }
  showTestCaseMemory("createMediaPlayer: After creating the media player application");

  printf("createMediaPlayer: Adding the hex calculator application to the start window\n");
  if (!g_nxwmtest.startwindow->addApplication(mediaplayer))
    {
      printf("createMediaPlayer: ERROR: Failed to add CMediaPlayerFactory to the start window\n");
      delete mediaplayer;
      return false;
    }

  showTestCaseMemory("createMediaPlayer: After adding the media player application");
  return true;
}
#endif

/////////////////////////////////////////////////////////////////////////////
// Public Functions
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// Name: showTestStepMemory
/////////////////////////////////////////////////////////////////////////////
// Called by ad hoc instrumentation in the NxWM/NxWidgets code

#ifdef CONFIG_HAVE_FILENAME
void _showTestStepMemory(FAR const char *file, int line, FAR const char *msg)
{
#ifdef CONFIG_NXWIDGET_MEMMONITOR
  updateMemoryUsage(&g_nxwmtest.mmSubStep, file, line, msg);
#endif
}
#else
void showTestStepMemory(FAR const char *msg)
{
#ifdef CONFIG_NXWIDGET_MEMMONITOR
  updateMemoryUsage(&g_nxwmtest.mmSubStep, msg);
#endif
}
#endif

/////////////////////////////////////////////////////////////////////////////
// nxwm_main
/////////////////////////////////////////////////////////////////////////////

int nxwm_main(int argc, char *argv[])
{
  // Call all C++ static constructors

#if defined(CONFIG_HAVE_CXX) && defined(CONFIG_HAVE_CXXINITIALIZE)
  up_cxxinitialize();
#endif

  // Initialize memory monitor logic

  initMemoryUsage();

  // Initialize the NSH library

#ifdef CONFIG_NXWM_NXTERM
  printf("nxwm_main: Initialize the NSH library\n");
  if (!NxWM::nshlibInitialize())
    {
      printf("nxwm_main: ERROR: Failed to initialize the NSH library\n");
      return EXIT_FAILURE;
    }
  showTestCaseMemory("nxwm_main: After initializing the NSH library");
#endif

  // Create the task bar.

  if (!createTaskbar())
    {
      printf("nxwm_main: ERROR: Failed to create the task bar\n");
      testCleanUpAndExit(EXIT_FAILURE);
    }

  // Create the start window.

  if (!createStartWindow())
    {
      printf("nxwm_main: ERROR: Failed to create the start window\n");
      testCleanUpAndExit(EXIT_FAILURE);
    }

  // Create the keyboard device

#ifdef CONFIG_NXWM_KEYBOARD
  if (!createKeyboard())
    {
      printf("nxwm_main: ERROR: Failed to create the keyboard\n");
      testCleanUpAndExit(EXIT_FAILURE);
    }
#endif

  // Create the touchscreen device

#ifdef CONFIG_NXWM_TOUCHSCREEN
  if (!createTouchScreen())
    {
      printf("nxwm_main ERROR: Failed to create the touchscreen\n");
      testCleanUpAndExit(EXIT_FAILURE);
    }
#endif

  // Create the calibration application and add it to the start window

#ifdef CONFIG_NXWM_TOUCHSCREEN
  if (!createCalibration())
    {
      printf("nxwm_main ERROR: Failed to create the calibration application\n");
      testCleanUpAndExit(EXIT_FAILURE);
    }
#endif

  // Create the NxTerm application and add it to the start window

#ifdef CONFIG_NXWM_NXTERM
  if (!createNxTerm())
    {
      printf("nxwm_main: ERROR: Failed to create the NxTerm application\n");
      testCleanUpAndExit(EXIT_FAILURE);
    }
#endif

  // Create the hex calculator application and add it to the start window

  if (!createHexCalculator())
    {
      printf("nxwm_main: ERROR: Failed to create the hex calculator application\n");
      testCleanUpAndExit(EXIT_FAILURE);
    }

  // Create the media player application and add it to the start window

#ifdef CONFIG_NXWM_MEDIAPLAYER
  if (!createMediaPlayer())
    {
      printf("nxwm_main: ERROR: Failed to create the media player application\n");
      testCleanUpAndExit(EXIT_FAILURE);
    }
#endif

  // Call CTaskBar::startWindowManager to start the display with applications in place.

  if (!startWindowManager())
    {
      printf("nxwm_main: ERROR: Failed to start the window manager\n");
      testCleanUpAndExit(EXIT_FAILURE);
    }

#ifdef CONFIG_NXWM_TOUCHSCREEN
#ifdef CONFIG_NXWM_TOUCHSCREEN_CONFIGDATA
  // There are two possibilities:  (1) We started the calibration earlier and now
  // need to obtain the calibration data from the calibration process, or (2)
  // We have already obtained stored calibration data in which case, the calibration
  // process never ran.

  if (!g_nxwmtest.calibrated)
#endif
    {
      // Since we started the touchscreen calibration program maximized, it will
      // run immediately when we start the window manager.  There is no positive
      // handshake to know when the touchscreen has been calibrated.  If we really
      // want to know, we have to poll

      printf("nxwm_main: Waiting for touchscreen calibration\n");
      while (!g_nxwmtest.touchscreen->isCalibrated())
        {
          std::sleep(2);
        }

      // This is how we would then recover the calibration data.  After the
      // calibration application creates the calibration data, it hands it to
      // the touchscreen driver.  After the touchscreen driver gets it, it will
      // report isCalibrated() == true and then we can read the calibration data
      // from the touchscreen driver.

      printf("nxwm_main: Getting calibration data from the touchscreen\n");
      if (!g_nxwmtest.touchscreen->getCalibrationData(g_nxwmtest.calibData))
        {
          printf("nxwm_main: ERROR: Failed to get calibration data from the touchscreen\n");
        }
      else
        {
#if 0 // ifdef CONFIG_NXWM_TOUCHSCREEN_CONFIGDATA.  Done in CCalibration
          // Save the new calibration data so that we do not have to do this
          // again the next time we start up.

          int ret = platform_setconfig(CONFIGDATA_TSCALIBRATION, 0,
                                       (FAR const uint8_t *)&g_nxwmtest.calibData,
                                       sizeof(struct NxWM::SCalibrationData));
          if (ret != 0)
            {
              printf("nxwm_main: ERROR: Failed to save calibration data\n");
            }
#endif
          g_nxwmtest.calibrated = true;
        }
    }
#endif

  // Wait a little bit for the display to stabilize.  Then simulate pressing of
  // the 'start window' icon in the task bar

#ifndef CONFIG_NXWM_TOUCHSCREEN
  sleep(2);
  g_nxwmtest.taskbar->clickIcon(0, true);
  usleep(500*1000);
  g_nxwmtest.taskbar->clickIcon(0, false);
  showTestCaseMemory("nxwm_main: After clicking the start window icon");

  // Wait bit to see the result of the button press.  Then press the first icon
  // in the start menu.  That should be the NxTerm icon (if the NxTerm
  // is not disabled).

  sleep(2);
  g_nxwmtest.startwindow->clickIcon(0, true);
  usleep(500*1000);
  g_nxwmtest.startwindow->clickIcon(0, false);
  showTestCaseMemory("nxwm_main: After clicking the NxTerm icon");
#endif

  // Wait bit to see the result of the button press.

  sleep(2);
  showTestMemory("nxwm_main: Final memory usage");
  return EXIT_SUCCESS;
}

