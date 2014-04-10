.SUFFIXES: .cpp .hpp

# Programs
SHELL 	= bash
CC     	= g++
LD	= ld
RM 	= rm
ECHO	= /bin/echo
CAT	= cat
PRINTF	= printf
SED	= sed
DOXYGEN = doxygen
######################################
# Project Name (generate executable with this name)
TARGET = cs296_08_exe

# Project Paths
PROJECT_ROOT=$(CURDIR)
EXTERNAL_ROOT=$(PROJECT_ROOT)/external
SRCDIR = $(PROJECT_ROOT)/src
OBJDIR = $(PROJECT_ROOT)/obj
BINDIR = $(PROJECT_ROOT)/bin
DOCDIR = $(PROJECT_ROOT)/doc
LIBDIR = $(PROJECT_ROOT)/lib
INST = ../

# Library Paths
BOX2D_ROOT=$(EXTERNAL_ROOT)
GLUI_ROOT=/usr
GL_ROOT=/usr/include/

#Libraries
LIBS = -lglui -lglut -lGLU -lGL -lBox2D

# Compiler and Linker flags
CPPFLAGS =-g -O3 -Wall -fno-strict-aliasing -Werror -fPIC
CPPFLAGS+=-I $(BOX2D_ROOT)/include -I $(GLUI_ROOT)/include
LDFLAGS+=-L $(BOX2D_ROOT)/lib -L $(GLUI_ROOT)/lib

######################################

NO_COLOR=\e[0m 
OK_COLOR=\e[1;32m
ERR_COLOR=\e[1;31m
WARN_COLOR=\e[1;33m
MESG_COLOR=\e[1;34m	
FILE_COLOR=\e[1;37m

OK_STRING="[OK]"
ERR_STRING="[ERRORS]"
WARN_STRING="[WARNINGS]"
OK_FMT="${OK_COLOR}%30s\n${NO_COLOR}"
ERR_FMT="${ERR_COLOR}%30s\n${NO_COLOR}"
WARN_FMT="${WARN_COLOR}%30s\n${NO_COLOR}"
######################################

SRCS := $(wildcard $(SRCDIR)/*.cpp)
INCS := $(wildcard $(SRCDIR)/*.hpp)
OBJS := $(SRCS:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
OBJS_WITHOUT_MAIN := $(filter-out .//obj/main.o, $(OBJS)) 


SHARED_LIB = FALSE

.PHONY: exe setup doc clean distclean

exe: setup $(BINDIR)/$(TARGET) static_lib exelib 

static_lib:
	@if test $(SHARED_LIB) = FALSE; \
	then ar -cvq lib/libCS296test.a $(OBJS_WITHOUT_MAIN); \
	else make dyna_lib; \
	fi;

dyna_lib :
	@$(CC)  -shared -Wall $(CPPFLAGS) -o $(LIBDIR)/libCS296test.so -fPIC $(SRCDIR)/*[!main].cpp; 

exelib:
	@if test $(SHARED_LIB) = TRUE; \
	then g++ -o bin/cs296_08_exelib $(LDFLAGS)  $(OBJDIR)/main.o  $(PROJECT_ROOT)/lib/libCS296test.so $(LIBS); \
	else g++ -o bin/cs296_08_exelib $(LDFLAGS)  $(OBJDIR)/main.o  $(PROJECT_ROOT)/lib/libCS296test.a $(LIBS); \
	fi;

setup:
	@$(ECHO) "Setting up compilation..."
	@mkdir -p obj
	@mkdir -p bin
	@mkdir -p lib
	@test -f $(EXTERNAL_ROOT)/include/Box2D/Box2D.h -a -f $(EXTERNAL_ROOT)/lib/libBox2D.a || { echo "Box2D is not installed. Installing..."; make installBox2D; }

dist:
	@make distclean
	@{ cd ../../; tar -cvzf cs296-g08-project.tar.gz cs296-g08-project;}

install:
	@make
	@make doc
	@rm -rf $(INST)/Pascaline
	@mkdir -p $(INST)/Pascaline
	@mkdir -p $(INST)/Pascaline/doc
	@cp -Rf images $(INST)/Pascaline
	@cp -Rf bin/cs296_08_exe $(INST)/Pascaline
	@cp -Rf doc/html $(INST)/Pascaline/doc
	@cp -Rf doc/ProjectReport.html $(INST)/Pascaline/doc
	@cp -Rf doc/ProjectReport.tex $(INST)/Pascaline/doc
	@cp -Rf doc/bibliographies.bib $(INST)/Pascaline/doc
	@cp -Rf doc/ProjectReport.pdf $(INST)/Pascaline/doc
	@make distclean
	@$(ECHO) *******************Pascaline has been installed!*******************

installBox2D:
	@{ cd  ./external/src; tar xvzf Box2D.tgz; cd ./Box2D; mkdir build296; cd build296; cmake ../; \
	make; make install;}
	@patch external/src/Box2D/Box2D/Common/b2Timer.cpp < ./patch/b2Timer_cpp.patch
	@patch external/src/Box2D/Box2D/Common/b2Timer.h < ./patch/b2Timer_h.patch;
	
$(BINDIR)/$(TARGET): $(OBJS)
	@$(PRINTF) "$(MESG_COLOR)Building executable:$(NO_COLOR) $(FILE_COLOR) %16s$(NO_COLOR)" "$(notdir $@)"
	@$(CC) -o $@ $(LDFLAGS) $(OBJS) $(LIBS) 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else $(PRINTF) $(OK_FMT) $(OK_STRING); \
	fi;
	@$(RM) -f temp.log temp.err

-include $(OBJS:.o=.d)

$(OBJS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	@$(PRINTF) "$(MESG_COLOR)Compiling: $(NO_COLOR) $(FILE_COLOR) %25s$(NO_COLOR)" "$(notdir $<)"
	@$(CC) $(CPPFLAGS) -c $< -o $@ -MD 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else printf "${OK_COLOR}%30s\n${NO_COLOR}" "[OK]"; \
	fi;
	@$(RM) -f temp.log temp.err

doc:
	@$(ECHO) -n "Generating Doxygen Documentation ...  "
	@$(RM) -rf doc/html
	@$(DOXYGEN) $(DOCDIR)/Doxyfile 2 > /dev/null
	@python3 scripts/gen_html.py
	@{ cd  ./doc; pdflatex ProjectReport.tex; bibtex ProjectReport; bibtex ProjectReport; bibtex ProjectReport; pdflatex ProjectReport.tex; bibtex ProjectReport; pdflatex ProjectReport.tex;}>tempo.log
	@rm -rf tempo.log
	@rm -rf ./doc/*.aux ./doc/*.blg ./doc/*.bbl ./doc/*.log

clean:
	@$(ECHO) -n "Cleaning up..."
	@$(RM) -rf lib obj bin doc/html doc/ProjectReport.html doc/ProjectReport.pdf
	@$(ECHO) "Done"

distclean: clean
	@make clean
	@$(RM) -rf external/include/Box2D external/src/Box2D external/lib/*
