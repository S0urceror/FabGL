/*
  Created by Fabrizio Di Vittorio (fdivitto2013@gmail.com) - <http://www.fabgl.com>
  Copyright (c) 2019-2022 Fabrizio Di Vittorio.
  All rights reserved.


* Please contact fdivitto2013@gmail.com if you need a commercial license.


* This library and related software is available under GPL v3.

  FabGL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  FabGL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with FabGL.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include "fabgl.h"
#include "fabui.h"

#include "App.h"



fabgl::VGAControllerS3 DisplayController;
fabgl::PS2Controller   PS2Controller;

const PinConfig pins(-1,-1,-1,5,4,  -1,-1,-1,-1,7,6,  -1,-1,-1,12,11,  14,13);


void setup()
{
  PS2Controller.begin(PS2Preset::KeyboardPort0_MousePort1, KbdMode::GenerateVirtualKeys);

  DisplayController.begin(pins);
  DisplayController.setResolution(VGA_640x480_60Hz);
}


void loop()
{
  // If the app crashes on InputBox replace 3500 by 4096
  MyApp().runAsync(&DisplayController, 3500).joinAsyncRun();  // why this? Just to use a larger stack!
}






