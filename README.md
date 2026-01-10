<!--
SPDX-FileCopyrightText: 2026 Sam Hanes <sam@maltera.com>
SPDX-License-Identifier: GPL-3.0-or-later
-->

# CoCo Chameleon Firmware

The CoCo Chameleon is a cartridge for the Tandy TRS-80 Color Computer
line of home computers from the 1980s which emulates other cartridges.
It supports all three versions of the CoCo and should be able to
replicate the computer-facing behavior of nearly any other cartridge.

This repository contains the firmware which runs on the CoCo Chameleon.

## Copying

Copyright 2026 Sam Hanes <sam@maltera.com>

This program is free software: you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the
Free Software Foundation, either version 3 of the License,
or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program. If not, see <https://www.gnu.org/licenses/>.

### Espressif System Libraries

When this application is compiled into an executable image, depending
on which features are enabled at compile time, that image may include
object code for certain non-free libraries maintained by Espressif and
distributed in object code form only as part of ESP-IDF (the software
development kit for the ESP32-S3 system-on-module). Those libraries are
currently necessary to use the module's RF hardware.

It is our view as the authors of this application that such non-free
libraries which are part of ESP-IDF constitute System Libraries under
the terms of the GPL (version 3), and therefore their inclusion in
the object code form of this application does not create a conflict
with the terms of the GPL when that object code form is conveyed.
