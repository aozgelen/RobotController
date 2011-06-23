# Toolchain/Environment
export SHELL 	= /bin/bash
export CPP    	= g++

PROJ_DIR	= ../..
PROJ_LIB_DIR	= $(PROJ_DIR)/lib
LIB_DIR 	= ../lib

# Dependency: MetroUtil
METROUTIL     	= $(PROJ_LIB_DIR)/libMetrobotics.a
METROUTIL_DIR	= $(PROJ_DIR)/utils/MetroUtil
METROUTIL_LIB 	= $(PROJ_LIB_DIR)
METROUTIL_INC 	= $(METROUTIL_DIR)/include

#Dependency: Localization
LOCALIZE	= $(LIB_DIR)/libLocalization.a
LOCALIZE_DIR	= ./Localization
LOCALIZE_LIB	= $(LIB_DIR)
LOCALIZE_INC	= $(LOCALIZE_DIR)/include

#Dependency: PathPlanner
PATHPLAN	= $(LIB_DIR)/libPathPlanner.a
PATHPLAN_DIR	= ./PathPlanner
PATHPLAN_LIB	= $(LIB_DIR)
PATHPLAN_INC	= $(PATHPLAN_DIR)/include

CONTROLLER	= ./Controller
CONTROLLER_SRC 	= $(CONTROLLER)/src
CONTROLLER_INC	= $(CONTROLLER)/include
VISDEBUG	= ./VisualDebugger
VISDEBUG_SRC	= $(VISDEBUG)/src
VISDEBUG_INC	= $(VISDEBUG)/include
PERCEPTION	= ./Perception
PERCEPTION_SRC	= $(PERCEPTION)/src
PERCEPTION_INC	= $(PERCEPTION)/include

VPATH 		= $(VISDEBUG_SRC) $(CONTROLLER_SRC) $(PERCEPTION_SRC)

# Options
INC_FLAGS	= -I$(CONTROLLER_INC) -I$(VISDEBUG_INC) -I$(PERCEPTION_INC) -I$(METROUTIL_INC) -I$(LOCALIZE_INC) -I$(PATHPLAN_INC)
CPPFLAGS 	= $(INC_FLAGS) `pkg-config --cflags playerc++`
CXXFLAGS	= -c -g #-Wall
LIBS   		= -L$(METROUTIL_LIB) -lMetrobotics -L$(LOCALIZE_LIB) -lLocalization -L$(PATHPLAN_LIB) -lPathPlanner -lboost_system-mt -lglut -lGLU `pkg-config --libs playerc++`

# Files
OUTPUT_DIR	= ..
OUTPUT    	= controller

SOURCE_PATHS 	= $(shell find $(VISDEBUG_SRC) -name "*cpp") $(shell find $(CONTROLLER_SRC) -name "*cpp") $(wildcard *.cpp)
SRCS 		:= $(foreach source, $(SOURCE_PATHS), $(subst $(VISDEBUG_SRC)/, , $(source) ))
SOURCES 	:= $(foreach src, $(SRCS), $(subst $(CONTROLLER_SRC)/, , $(src) ))
SOURCE_DIRS 	= $(dir $(SOURCE_PATHS))

# Object files to link with no path
OBJECTS 	= $(notdir $(SOURCES:.cpp=.o))

################### General targets #########################
.PHONY: $(OUTPUT) all clean purge docs

all: $(OUTPUT) metroutil localization pathplanner
	mv $(OUTPUT) $(OUTPUT_DIR)

metroutil: $(METROUTIL)


# targets for localization
localization: $(LOCALIZE)

# rebuild localization library and re-link the executable
r-localization: c-localization $(LOCALIZE) $(OUTPUT)

# clean localization library and the executable
c-localization:
	rm -rf $(LOCALIZE)
	rm -rf $(OUTPUT)
	rm -rf $(OBJECTS)
	@$(MAKE) --directory=$(LOCALIZE_DIR) clean


# targets for 
pathplanner: $(PATHPLAN)

#rebuild pathplanner library and re-link the executable
r-pathplanner: c-pathplanner $(PATHPLAN) $(OUTPUT)

# clean pathplanner library and the executable
c-pathplanner:
	rm -rf $(PATHPLAN)
	rm -rf $(OUTPUT)
	rm -rf $(OBJECTS)
	@$(MAKE) --directory=$(PATHPLAN_DIR) clean

########################################################

# build metroutil, localization, pathplanner and the controller
$(OUTPUT): $(METROUTIL) $(LOCALIZE) $(PATHPLAN) $(OBJECTS)
	$(CPP) -o $@ $(OBJECTS) $(LIBS)

# Dependency: MetroUtil
$(METROUTIL):
	@echo Building MetroUtil
	@$(MAKE) -e --directory=$(METROUTIL_DIR)
	@if [ "$$?" != 0 ]; then \
		echo "MetroUtil failed to build"; \
		exit 1; \
	fi
	@echo Deploying library
	@$(MAKE) -e --directory=$(METROUTIL_DIR) deploy

# Dependence: Localization
$(LOCALIZE): 
	@echo Building Localization library
	@$(MAKE) --directory=$(LOCALIZE_DIR)
	@if [ "$$?" != 0 ]; then \
		echo "Localization failed to build"; \
		exit 1; \
	fi
#	@echo Deploying library
#	@$(MAKE) --directory=$(LOCALIZE_DIR) deploy

# Dependence: PathPlanner
$(PATHPLAN): 
	@echo Building PathPlanner library
	@$(MAKE) --directory=$(PATHPLAN_DIR)
	@if [ "$$?" != 0 ]; then \
		echo "PathPlanner failed to build"; \
		exit 1; \
	fi

# Remove unnecessary output files
clean:
	@$(MAKE) -e --directory=$(METROUTIL_DIR) clean
	@$(MAKE) --directory=$(LOCALIZE_DIR) clean
	@$(MAKE) --directory=$(PATHPLAN_DIR) clean
	rm -rf $(OUTPUT_DIR)/$(OUTPUT)
	rm -rf $(OBJECTS)

# Remove all output files
purge:
	@$(MAKE) --directory=$(LOCALIZE_DIR) clean
	@$(MAKE) --directory=$(PATHPLAN_DIR) clean
	@$(MAKE) -e clean
	rm -rf $(OUTPUT_DIR)/$(OUTPUT)
	rm -rf $(LIB_DIR)/*
	rm -rf $(PROJ_LIB_DIR)/*

