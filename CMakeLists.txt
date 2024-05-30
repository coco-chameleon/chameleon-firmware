# SPDX-FileCopyrightText: 2024 Sam Hanes <sam@maltera.com>
# SPDX-License-Identifier: GPL-3.0-or-later

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS "$ENV{ZEPHYR_ROOT}")

project(chameleon C)

target_sources(app PRIVATE src/main.c)
