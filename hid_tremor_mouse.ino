/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2023 Bill Binko for Adafruit Industries
 Based on tremor_filter example by Thach Ha
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/


/* This example demonstrates use of both device and host, where
 * - Device run on native usb controller (controller0)
 * - Host run on bit-banging 2 GPIOs with the help of Pico-PIO-USB library (controller1)
 *
 * Example sketch receive mouse report from host interface (from e.g consumer mouse)
 * and reduce large motions due to tremors by applying the natural log function.
 * It handles negative values and a dead zone where small values will not be adjusted.
 * Adjusted mouse movement are send via device interface (to PC).
 *
 * Requirements:
 * - [Pico-PIO-USB](https://github.com/sekigon-gonnoc/Pico-PIO-USB) library
 * - 2 consecutive GPIOs: D+ is defined by PIN_PIO_USB_HOST_DP, D- = D+ +1
 * - Provide VBus (5v) and GND for peripheral
 * - CPU Speed must be either 120 or 240 Mhz. Selected via "Menu -> CPU Speed"
 */

// pio-usb is required for rp2040 host
#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"
#include <math.h>

// Pin D+ for host, D- = D+ + 1
#ifndef PIN_USB_HOST_DP
#define PIN_USB_HOST_DP  16
#endif

// Pin for enabling Host VBUS. comment out if not used
#ifndef PIN_5V_EN
#define PIN_5V_EN        18
#endif

#ifndef PIN_5V_EN_STATE
#define PIN_5V_EN_STATE  1
#endif

// Language ID: English
#define LANGUAGE_ID 0x0409

// USB Host object
Adafruit_USBH_Host USBHost;

// HID report descriptor using TinyUSB's template
// Single Report (no ID) descriptor
uint8_t const desc_hid_report[] =
{
  TUD_HID_REPORT_DESC_MOUSE()
};

// USB HID object. For ESP32 these values cannot be changed after this declaration
// desc report, desc len, protocol, interval, use out endpoint
Adafruit_USBD_HID usb_hid(desc_hid_report, sizeof(desc_hid_report), HID_ITF_PROTOCOL_MOUSE, 2, false);

/* Adjustable parameters for the log_filter() method.  Adjust for each user (would be ideal to have this 
 *  adjustable w/o recompiling
 */
#define PRESCALE 8.0  // Must be > 0, Higher numbers increase rate of attenuation
#define POSTSCALE 1.5 // Must be > 0, Higher numbers compensate for PRESCALE attenuation
#define DEADZONE 1.0  // Must be > 1, Movements < this magnitude will not be filtered


//--------------------------------------------------------------------+
// Setup and Loop on Core0
//--------------------------------------------------------------------+

void setup()
{
  Serial.begin(115200);
  usb_hid.begin();

  //while ( !Serial ) delay(10);   // wait for native usb
  Serial.println("ATMakers Logarithm Tremor Filter Example");
}

void loop()
{
  Serial.flush();
}

//--------------------------------------------------------------------+
// Setup and Loop on Core1
//--------------------------------------------------------------------+

void setup1() {
  //while ( !Serial ) delay(10);   // wait for native usb
  Serial.println("Core1 setup to run TinyUSB host with pio-usb");

  // Check for CPU frequency, must be multiple of 120Mhz for bit-banging USB
  uint32_t cpu_hz = clock_get_hz(clk_sys);
  if ( cpu_hz != 120000000UL && cpu_hz != 240000000UL ) {
    while ( !Serial ) delay(10);   // wait for native usb
    Serial.printf("Error: CPU Clock = %lu, PIO USB require CPU clock must be multiple of 120 Mhz\r\n", cpu_hz);
    Serial.printf("Change your CPU Clock to either 120 or 240 Mhz in Menu->CPU Speed \r\n");
    while(1) delay(1);
  }

#ifdef PIN_5V_EN
  pinMode(PIN_5V_EN, OUTPUT);
  digitalWrite(PIN_5V_EN, PIN_5V_EN_STATE);
#endif

  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = PIN_USB_HOST_DP;
  USBHost.configure_pio_usb(1, &pio_cfg);

  // run host stack on controller (rhport) 1
  // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
  // host bit-banging processing works done in core1 to free up core0 for other works
  USBHost.begin(1);
}

void loop1()
{
  USBHost.task();
}


extern "C"
{

// Invoked when device with hid interface is mounted
// Report descriptor is also available for use.
// tuh_hid_parse_report_descriptor() can be used to parse common/simple enough
// descriptor. Note: if report descriptor length > CFG_TUH_ENUMERATION_BUFSIZE,
// it will be skipped therefore report_desc = NULL, desc_len = 0
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *desc_report, uint16_t desc_len) {
  (void) desc_report;
  (void) desc_len;
  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);

  Serial.printf("HID device address = %d, instance = %d is mounted\r\n", dev_addr, instance);
  Serial.printf("VID = %04x, PID = %04x\r\n", vid, pid);

  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
  if (itf_protocol == HID_ITF_PROTOCOL_MOUSE) {
    Serial.printf("HID Mouse\r\n");
    if (!tuh_hid_receive_report(dev_addr, instance)) {
      Serial.printf("Error: cannot request to receive report\r\n");
    }
  }
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
  Serial.printf("HID device address = %d, instance = %d is unmounted\r\n", dev_addr, instance);
}


// Invoked when received report from device via interrupt endpoint
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *report, uint16_t len) {
  filter_report((hid_mouse_report_t const *) report);

  // continue to request to receive report
  if (!tuh_hid_receive_report(dev_addr, instance)) {
    Serial.printf("Error: cannot request to receive report\r\n");
  }
}

} // extern C

/*--------------------------------------------------------------------+
Here begins the TremorMouse code

Copyright (C) 2023  J.L. Kissel

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>

    Based on original code from Adafruit_tinyUSB_Arduino
--------------------------------------------------------------------*/
#define HIGH_MEDIUM_LEVEL 32  // the high speed range if from 127 to H_M_L (abs)
#define MEDIUM_LOW_LEVEL 12   // the medium speed range is between H_M_L and M_L_L
                              // the slow speed range is < M_L_L

const unsigned long Button_Timeout = 125;  // 125 miliseconds
hid_mouse_report_t filtered_report;

#define Xindex 0
#define Yindex 1
#define NUMBEROFMEASUREMENTS 2
#define MOVINGAVERAGEWINDOWSIZE 4
float CurrentMovingAverage[NUMBEROFMEASUREMENTS]; // the current moving average
float Remainder[NUMBEROFMEASUREMENTS] = {0,0}; // the current moving average
int MovingAverageWindow[NUMBEROFMEASUREMENTS][MOVINGAVERAGEWINDOWSIZE]; // array for storing mouse movement values to be averaged
int newData[2]; // we only smoth the X and Y data
int indexMAW = 0;   // index into MAW[] 

void UpdateMovingAverages() {
  float CalcMA;
  int i,j;
  newData[Xindex] = filtered_report.x;
  newData[Yindex] = filtered_report.y;
  
  for(j = 0; j < NUMBEROFMEASUREMENTS; j++) {
//    PreviousMovingAverage[j] = CurrentMovingAverage[j];
    MovingAverageWindow[j] [indexMAW] = newData[j];
    CalcMA = 0;
    for( i = 0; i < NUMBEROFMEASUREMENTS; i++) {
      CalcMA += MovingAverageWindow[j][i];
    }
    CurrentMovingAverage[j] = (int)( (CalcMA/MOVINGAVERAGEWINDOWSIZE) + Remainder[j] );
    Remainder[j] = ( CalcMA/MOVINGAVERAGEWINDOWSIZE ) - CurrentMovingAverage[j];
  }
  indexMAW++;
  if( indexMAW == MOVINGAVERAGEWINDOWSIZE ) // we need to wrap around our circular buffer
  {
    indexMAW = 0;
  }
}


/*
 * Cursor Speed attenuation
 */
void speed_filter(int8_t xy[])
{
  static float remainder[2];
  float tmp;

  for( int i=0; i<2; i++) {
    if( xy[i] == 0) {  // no mouse movement
      continue;
    }
    if( abs(xy[i]) >= HIGH_MEDIUM_LEVEL) {
      continue; // this is the high movement range, so no attenuation is required
    }
    else if( (abs(xy[i]) + remainder[i]) >= MEDIUM_LOW_LEVEL ) {
      // we are starting to approach the target so do a bit of attenuation
      tmp = (xy[i] + remainder[i]) / 2;  //make them move the mouse 2 pixels to move 1 pixel on the screen
      remainder[i] = tmp - int(tmp);  // save it for next mouse movement
      xy[i] = (int8_t)tmp;
    }
    else {  // we are verry close to the target, attenuate even more
      tmp = (xy[i] + remainder[i]) / 4;  //make them move the mouse 2 pixels to move 1 pixel on the screen
      remainder[i] = tmp - int(tmp);  // save it for next mouse movement
      xy[i] = (int8_t)tmp;
    }
  }  
}

/*
 * Adjust HID report by applying log_filterF
 */
void filter_report(hid_mouse_report_t const* report) {
  int8_t old_xy[2];

  old_xy[0] = report->x;
  old_xy[1] = report->y;

  hid_mouse_report_t filtered_report = *report;
  speed_filter(old_xy);
  filtered_report.x = old_xy[0];
  filtered_report.y = old_xy[1];
  //Serial.printf("%d,%d,%d,%d\n", old_x, filtered_report.x, old_y, filtered_report.y);

  UpdateMovingAverages();
/*--------------------------------------------------------------------+
This section decides if the Middle mouse button has been pressed and if
it has switched to X-Y mode.  Only horizontial or only verital movement
----------------------------------------------------------------------*/
  unsigned long currentMillis;  
  static unsigned long PreviousMillis = 0;
  static uint8_t previousButton = 0;
  static bool Middle_Button_Pressed = false;

  if( filtered_report.buttons & MOUSE_BUTTON_MIDDLE) {
    // the middle button has been pressed
    if( previousButton & MOUSE_BUTTON_MIDDLE ) { // we are waiting for the time this button has been pressed to exceed 
      currentMillis = millis();
      if( currentMillis - PreviousMillis >= Button_Timeout) { // we have a valid Middle button press
//        filtered_report.buttons = report->buttons; // send the mouse button
        if( Middle_Button_Pressed ) {  // the middle Button is a toggle into and out of X-Y mode
          Middle_Button_Pressed = false;
        }
        else {
          Middle_Button_Pressed = true;
        }
      }
      else {
        filtered_report.buttons = 0;
      }
    } else {
      PreviousMillis = millis(); // the start time
    }
  }

  /* If Middle_Button_Pressed, the we are in X-Y mode.  Adjust the x,y accordingly */
  if( Middle_Button_Pressed ) {
 //   if( filtered_report.x > filtered_report.y ) { // only allow X or Y movement
    if( CurrentMovingAverage[Xindex] > CurrentMovingAverage[Yindex] ) {
      filtered_report.y = 0;
    } else {
      filtered_report.x = 0;
    }
  }


  usb_hid.sendReport(0, &filtered_report, sizeof(filtered_report));
  
}
