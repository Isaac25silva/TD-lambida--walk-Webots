# Webots Makefile system 
#
# You may add some variable definitions hereafter to customize the build process
# See documentation in $(WEBOTS_HOME_PATH)/resources/Makefile.include

CFLAGS = -std=c++11
WEBOTS_PATH_INCLUDE = /home/fei/webots/include/controller
RESOURCES_PATH = ../../
INCLUDE = -I"$(RESOURCES_PATH)/libraries/managers/include" -I"$(RESOURCES_PATH)/libraries/darwin/darwin/Framework/include" -I"$(RESOURCES_PATH)/Blackboard/include" -I"$(RESOURCES_PATH)/QL/include"  
LIBRARIES = -L"$(RESOURCES_PATH)/libraries/darwin/build/release" -ldarwin -L"$(RESOURCES_PATH)/libraries/managers/build/release" -lmanagers -L"../../QL/lib" -lQL
CXX_SOURCES =  \
	    $(RESOURCES_PATH)/Blackboard/src/blackboard.cpp \
	    $(wildcard *.cpp)


### Do not modify: this includes Webots global Makefile.include
space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
