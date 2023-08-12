#!/bin/bash

beginswith() { case $1 in "$2"*) true;; *) false;; esac; }

if beginswith "$OSTYPE" darwin; then #macos
	clang++ main.cpp -g -Iglad/include -ISDL/include -Ieigen glad.a libSDL2.a libSDL2main.a -framework CoreAudio -framework AudioToolbox -framework CoreFoundation -framework CoreGraphics -framework CoreHaptics -framework CoreVideo -framework ForceFeedback -framework GameController -framework IOKit -framework Carbon -framework Metal -framework Cocoa -liconv -std=c++17
else
	c++ main.cpp -g -Iglad/include -ISDL/include glad.a -Ieigen libSDL2.a libSDL2main.a -lpthread -ldl -lm -std=c++17
fi
